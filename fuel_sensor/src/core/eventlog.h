// Device event log (DESIGN.md §9.2): fixed 16-byte records, appended to small numbered segments (a LittleFS file each
// on the device, memory in tests). Appends never rewrite older records, and a segment is small enough (512 B) to live
// inline in LittleFS metadata, so an event costs one metadata commit instead of rewriting an 8 KB file.
// EventBudget bounds how many events per day can reach flash at all (faults and unauthenticated clients can repeat).
#pragma once
#include <cstddef>
#include <cstdint>

namespace fs {

enum class Ev : uint16_t {
  Boot = 1, LoginOk = 2, LoginFail = 3, Lockout = 4, TankChanged = 5, OtaAccepted = 6, OtaRejected = 7,
  TrialPassed = 8, RolledBack = 9, SafeMode = 10, RadarLost = 11, RadarBack = 12, Migrated = 13, CoredumpFound = 14,
  LevelSaveError = 15, Restart = 16, NvsRestored = 17, EventsDropped = 18,
  // fuel ledger (DESIGN.md §13): value = level um; FuelGap is signed (int32 bits)
  FuelFill = 19, FuelDrain = 20, FuelSlow = 21, FuelGap = 22, FuelReset = 23,
  // a step of the level absorbed as not fuel (diagnostics, at most one per 30 min: arg = steps since the last, value = the
  // last one's signed level um)
  LevelJump = 24,
  ResetToDefaults = 25,                         // arg = reset items (core/reset.h), value 0 = requested, 1 = finished at boot
  // a tracker cycle correction followed the FUEL_FILL (arg 1) / FUEL_DRAIN (arg 2) of value level um logged last (within 5 min
  // of its close): that event may be a whole cycle off
  FuelUnverified = 26,
  // a signed fill report was issued (DESIGN.md §14): arg = its flags, value = its lifetime serial
  FillReport = 27,
};
constexpr uint16_t kEvTypeCount = 28;           // codes 0..27 (at most 64: 6 bits on flash)
const char* event_name(uint16_t type);

// On flash the type field also carries how many events of the same type the budget dropped just before this one:
// bits 0-5 = type, bits 6-15 = dropped (saturating at 1023).
constexpr uint16_t kEvTypeMask = 0x3F;
constexpr uint16_t kEvDroppedShift = 6, kEvDroppedMax = 1023;

struct EventRecord {
  uint32_t boot = 0;          // boot counter
  uint32_t uptime_s = 0;
  uint16_t type = 0;          // Ev code (low 6 bits) | dropped << 6
  uint16_t arg = 0;
  uint32_t value = 0;
  uint16_t code() const { return uint16_t(type & kEvTypeMask); }
  uint16_t dropped() const { return uint16_t(type >> kEvDroppedShift); }
};

// Numbered append-only segments.
class SegmentStore {
 public:
  virtual ~SegmentStore() = default;
  // Lowest and highest segment number present; false if there is none (or on error).
  virtual bool scan(uint32_t* oldest, uint32_t* newest) = 0;
  virtual uint32_t bytes(uint32_t id) = 0;                                     // 0 if missing
  virtual bool append(uint32_t id, const uint8_t* data, size_t len) = 0;       // creates; durable when it returns
  virtual bool read(uint32_t id, uint32_t offset, uint8_t* out, size_t len) = 0;
  virtual bool remove(uint32_t id) = 0;
};

class EventLog {
 public:
  static constexpr size_t kRecordBytes = 16;
  static constexpr uint32_t kSegRecords = 32;                      // 512 B: LittleFS keeps it inline (<= cache_size)
  static constexpr uint32_t kSegments = 17;                        // kept on flash
  static constexpr uint16_t kCapacity = uint16_t(kSegRecords * (kSegments - 1));   // newest records always kept: 512

  bool begin(SegmentStore* store);
  bool add(const EventRecord& r);
  uint16_t count() const { return count_; }
  uint32_t oldest_id() const { return oldest_; }
  uint32_t newest_id() const { return newest_; }
  // i = 0 is the newest record.
  bool get_newest(uint16_t i, EventRecord* r);

 private:
  bool drop_oldest();
  SegmentStore* store_ = nullptr;
  uint32_t oldest_ = 0, newest_ = 0, in_newest_ = 0;
  uint16_t count_ = 0;
  bool any_ = false, ok_ = false;
};

// Per-class token buckets in front of the log. The state is plain data so the platform can keep it in RTC memory:
// warm resets (crash, watchdog, software restart) keep the buckets, so a reboot loop cannot refill them.
enum class EvClass : uint8_t { Always = 0, Device = 1, Remote = 2, Fuel = 3 };
constexpr int kEvBudgetClasses = 3;             // Device, Remote, Fuel
EvClass event_class(uint16_t code);

struct EventBudgetState {
  uint32_t magic;
  uint32_t milli_tokens[kEvBudgetClasses];      // Device, Remote, Fuel: tokens x 1000
  uint16_t dropped[kEvTypeCount];
  uint32_t summary_milli;                       // EVENTS_DROPPED summaries x 1000 (1 per hour of uptime, warm resets kept)
  uint32_t check;
};

class EventBudget {
 public:
  static constexpr uint32_t kBurst = 16;                 // tokens per class
  static constexpr uint32_t kRefillMs = 5u * 60 * 1000;  // one token per 5 min of uptime: <= 288 a day per class
  // Keeps a valid state (warm reset), otherwise starts with full buckets.
  void begin(EventBudgetState* st);
  // True: write now; *dropped = events of this type dropped since its last written record.
  bool allow(uint16_t code, uint64_t uptime_ms, uint16_t* dropped);
  // Call about once a second: refills keep accruing in RTC memory between events, so a warm reboot loop shorter than
  // kRefillMs still refills (tokens accrued only inside allow() were lost at every reset).
  void tick(uint64_t uptime_ms);
  // At most once per hour of uptime (across warm resets): true when events were dropped since the last summary;
  // *mask bit (code-1) for each dropped code 1..16, *total their sum; the pending counts are then cleared.
  bool take_summary(uint64_t uptime_ms, uint16_t* mask, uint32_t* total);

 private:
  void seal();
  void refill(uint64_t uptime_ms);
  EventBudgetState* st_ = nullptr;
  uint64_t last_ms_ = 0;
};

// One-time import of the fw 1.0.0 ring file (/events.bin: "MRME", u16 next, u16 count, 512 x 16 B), oldest first.
class ByteStore {
 public:
  virtual ~ByteStore() = default;
  virtual bool read(uint32_t offset, uint8_t* out, size_t len) = 0;
};
using EventSink = bool (*)(const EventRecord& r, void* ctx);
uint16_t import_legacy_ring(ByteStore* old, EventSink sink, void* ctx);   // returns records imported

}  // namespace fs
