// Device event log (DESIGN.md §9.2).
#include "eventlog.h"

#include <cstring>

namespace fs {

namespace {
constexpr uint8_t kMagic[4] = {'M', 'R', 'M', 'E'};
constexpr uint32_t kBudgetMagic = 0x45564236;      // "EVB6" (3 classes, dropped[28]: fuel codes 19-24, 26; 25 RESET; 27 FILL)
constexpr uint32_t kSummaryMilli = 12000;          // 12 refill steps of 5 min = 1 hour
void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
void put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i)); }
uint16_t get16(const uint8_t* p) { return uint16_t(p[0] | p[1] << 8); }
uint32_t get32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
void pack(const EventRecord& r, uint8_t* b) {
  put32(b, r.boot);
  put32(b + 4, r.uptime_s);
  put16(b + 8, r.type);
  put16(b + 10, r.arg);
  put32(b + 12, r.value);
}
void unpack(const uint8_t* b, EventRecord* r) {
  r->boot = get32(b);
  r->uptime_s = get32(b + 4);
  r->type = get16(b + 8);
  r->arg = get16(b + 10);
  r->value = get32(b + 12);
}
uint32_t budget_check(const EventBudgetState& s) {
  uint32_t h = 0x811C9DC5u;
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&s);
  for (size_t i = 0; i < offsetof(EventBudgetState, check); ++i) h = (h ^ p[i]) * 16777619u;
  return h;
}
}  // namespace

const char* event_name(uint16_t type) {
  switch (Ev(type & kEvTypeMask)) {
    case Ev::Boot: return "boot";
    case Ev::LoginOk: return "login_ok";
    case Ev::LoginFail: return "login_fail";
    case Ev::Lockout: return "lockout";
    case Ev::TankChanged: return "tank_changed";
    case Ev::OtaAccepted: return "ota_accepted";
    case Ev::OtaRejected: return "ota_rejected";
    case Ev::TrialPassed: return "trial_passed";
    case Ev::RolledBack: return "rolled_back";
    case Ev::SafeMode: return "safe_mode";
    case Ev::RadarLost: return "radar_lost";
    case Ev::RadarBack: return "radar_back";
    case Ev::Migrated: return "migrated";
    case Ev::CoredumpFound: return "coredump_found";
    case Ev::LevelSaveError: return "level_save_error";
    case Ev::Restart: return "restart";
    case Ev::NvsRestored: return "nvs_restored";
    case Ev::EventsDropped: return "events_dropped";
    case Ev::FuelFill: return "fuel_fill";
    case Ev::FuelDrain: return "fuel_drain";
    case Ev::FuelSlow: return "fuel_slow";
    case Ev::FuelGap: return "fuel_gap";
    case Ev::FuelReset: return "fuel_reset";
    case Ev::LevelJump: return "level_jump";
    case Ev::ResetToDefaults: return "reset_to_defaults";
    case Ev::FuelUnverified: return "fuel_unverified";
    case Ev::FillReport: return "fill_report";
  }
  return "unknown";
}

// ---------- log ----------

bool EventLog::begin(SegmentStore* store) {
  store_ = store;
  ok_ = store_ != nullptr;
  any_ = ok_ && store_->scan(&oldest_, &newest_);
  if (!any_) oldest_ = newest_ = 0;
  // A power cut between append and remove leaves one segment too many. Prune by what the directory really holds
  // (re-scan), never by walking the id range: a corrupt or wrapped id (e.g. "0" next to "4294967295") would
  // otherwise mean billions of unlink() calls in setup -> task watchdog -> reboot loop -> safe mode for good.
  for (uint32_t guard = 0; any_ && newest_ - oldest_ >= kSegments && guard < 4 * kSegments; ++guard) {
    if (!store_->remove(oldest_)) break;
    any_ = store_->scan(&oldest_, &newest_);
  }
  if (any_ && newest_ - oldest_ >= kSegments) oldest_ = newest_ - (kSegments - 1);   // still odd: read only the window
  in_newest_ = any_ ? store_->bytes(newest_) / kRecordBytes : 0;
  uint32_t n = 0;
  if (any_)
    for (uint32_t id = newest_;; --id) {
      n += id == newest_ ? in_newest_ : store_->bytes(id) / kRecordBytes;
      if (id == oldest_) break;
    }
  count_ = uint16_t(n < kCapacity ? n : kCapacity);
  return ok_;
}

bool EventLog::add(const EventRecord& r) {
  if (!ok_) return false;
  if (any_ && in_newest_ >= kSegRecords) {                // the newest segment is full: start the next one
    ++newest_;
    in_newest_ = 0;
  }
  uint8_t rec[kRecordBytes];
  pack(r, rec);
  if (!store_->append(newest_, rec, sizeof rec)) return false;
  any_ = true;
  ++in_newest_;
  while (newest_ - oldest_ >= kSegments && drop_oldest()) {}   // after the append: never fewer than 512 kept
  if (count_ < kCapacity) ++count_;
  return true;
}

// Removes the oldest segment. A segment that cannot be removed stays the oldest (retried at the next add and boot):
// skipping it would leave a gap of ids that every later boot walks one stat() at a time (3,000 ids ~ 9 s in setup).
bool EventLog::drop_oldest() {
  if (!store_->remove(oldest_) && store_->bytes(oldest_) != 0) return false;
  ++oldest_;
  return true;
}

bool EventLog::get_newest(uint16_t i, EventRecord* r) {
  if (!ok_ || !any_ || i >= count_) return false;
  uint32_t skip = i;
  for (uint32_t id = newest_;; --id) {
    uint32_t n = id == newest_ ? in_newest_ : store_->bytes(id) / kRecordBytes;
    if (skip < n) {
      uint8_t rec[kRecordBytes];
      if (!store_->read(id, (n - 1 - skip) * kRecordBytes, rec, sizeof rec)) return false;
      unpack(rec, r);
      return true;
    }
    skip -= n;
    if (id == oldest_) return false;
  }
}

// ---------- budget ----------

EvClass event_class(uint16_t code) {
  switch (Ev(code)) {
    case Ev::LoginFail: case Ev::Lockout: case Ev::OtaRejected: case Ev::FillReport:
      return EvClass::Remote;                                 // anyone on the Wi-Fi can repeat these
    case Ev::Boot: case Ev::SafeMode: case Ev::RadarLost: case Ev::RadarBack: case Ev::CoredumpFound:
    case Ev::LevelSaveError: case Ev::Restart: case Ev::LevelJump:
      return EvClass::Device;                                 // faults, reboot loops, radar diagnostics
    case Ev::FuelFill: case Ev::FuelDrain: case Ev::FuelSlow: case Ev::FuelGap: case Ev::FuelUnverified:
      return EvClass::Fuel;                                   // the fuel itself: a vibration loop must not flood the flash
    default:
      return EvClass::Always;                                 // signed logins, settings, updates: rare and important
  }
}

void EventBudget::seal() { st_->check = budget_check(*st_); }

void EventBudget::begin(EventBudgetState* st) {
  st_ = st;
  last_ms_ = 0;
  if (st_->magic == kBudgetMagic && st_->check == budget_check(*st_)) return;
  std::memset(st_, 0, sizeof *st_);
  st_->magic = kBudgetMagic;
  for (uint32_t& t : st_->milli_tokens) t = kBurst * 1000;
  st_->summary_milli = kSummaryMilli;
  seal();
}

void EventBudget::refill(uint64_t uptime_ms) {
  uint64_t dt = uptime_ms > last_ms_ ? uptime_ms - last_ms_ : 0;
  uint64_t add = dt * 1000 / kRefillMs;                       // milli-tokens; the remainder stays for the next call
  last_ms_ += add * kRefillMs / 1000;
  for (uint32_t& t : st_->milli_tokens) t = uint32_t(t + add > kBurst * 1000 ? kBurst * 1000 : t + add);
  st_->summary_milli = uint32_t(st_->summary_milli + add > kSummaryMilli ? kSummaryMilli : st_->summary_milli + add);
}

void EventBudget::tick(uint64_t uptime_ms) {
  refill(uptime_ms);
  seal();
}

bool EventBudget::take_summary(uint64_t uptime_ms, uint16_t* mask, uint32_t* total) {
  refill(uptime_ms);
  *mask = 0;
  *total = 0;
  for (uint16_t code = 1; code < kEvTypeCount; ++code) {
    if (!st_->dropped[code]) continue;
    if (code <= 16) *mask = uint16_t(*mask | 1u << (code - 1));
    *total += st_->dropped[code];
  }
  bool ok = *total && st_->summary_milli >= kSummaryMilli;
  if (ok) {
    st_->summary_milli -= kSummaryMilli;
    for (uint16_t& d : st_->dropped) d = 0;
  }
  seal();
  return ok;
}

bool EventBudget::allow(uint16_t code, uint64_t uptime_ms, uint16_t* dropped) {
  *dropped = 0;
  if (code >= kEvTypeCount) code = 0;
  EvClass c = event_class(code);
  refill(uptime_ms);
  bool ok = true;
  if (c != EvClass::Always) {
    uint32_t& t = st_->milli_tokens[int(c) - 1];
    if (t >= 1000) t -= 1000;
    else ok = false;
  }
  if (ok) {
    *dropped = st_->dropped[code] > kEvDroppedMax ? kEvDroppedMax : st_->dropped[code];
    st_->dropped[code] = uint16_t(st_->dropped[code] - *dropped);   // beyond the 1023 a record holds: left for the summary
  } else if (st_->dropped[code] < 0xFFFF) {
    ++st_->dropped[code];
  }
  seal();
  return ok;
}

// ---------- fw 1.0.0 ring import ----------

uint16_t import_legacy_ring(ByteStore* old, EventSink sink, void* ctx) {
  constexpr uint16_t kOldCap = 512;
  uint8_t h[8];
  if (!old || !old->read(0, h, sizeof h) || std::memcmp(h, kMagic, 4) != 0) return 0;
  uint16_t next = get16(h + 4), count = get16(h + 6);
  if (next >= kOldCap || count > kOldCap) return 0;
  uint16_t done = 0;
  for (uint16_t k = count; k > 0; --k) {                     // oldest first
    uint16_t idx = uint16_t((next + kOldCap - k) % kOldCap);
    uint8_t rec[16];
    if (!old->read(8 + uint32_t(idx) * 16, rec, sizeof rec)) break;
    EventRecord r;
    unpack(rec, &r);
    if (!sink(r, ctx)) break;
    ++done;
  }
  return done;
}

}  // namespace fs
