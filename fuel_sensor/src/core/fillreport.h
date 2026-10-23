// Fill reports (DESIGN.md §14): the operator's Start / End / litres on the dashboard, the sensor's litres from the fuel
// ledger over the same window, signed with the fill-report key (ECDSA P-256) into a code the customer's IT verifies.
// Pure C++ (host-tested); the device keeps FillNv in NVS (fs_fill/rec), the report list in LittleFS (platform/fill.cpp).
#pragma once
#include <cstddef>
#include <cstdint>

namespace fs {

constexpr uint8_t kFillFormat = 1;
constexpr size_t kFillPayloadBytes = 62;
constexpr size_t kFillSigBytes = 64;
constexpr size_t kFillCodeBytes = kFillPayloadBytes + kFillSigBytes;        // 126 -> 202 base32 symbols
constexpr size_t kFillTextMax = 256;                                        // "FR-" + 202 symbols + 40 dashes + NUL
constexpr size_t kFillLineMax = 200;
// Signed message: SHA-256 over these 16 bytes (the NUL included) then the payload. No other key or domain signs it.
constexpr char kFillDomain[] = "MRM-FUEL-FILL-1";
constexpr size_t kFillDomainLen = sizeof(kFillDomain);                     // 16, with the NUL
static_assert(kFillDomainLen == 16, "domain is 15 characters + NUL");
constexpr uint32_t kFillOperatorNone = 0xFFFFFFFFu;                        // operator litres not entered (auto-closed)

enum FillFlag : uint16_t {
  kFillFlagRestart = 1u << 0,       // the device restarted while the report was open
  kFillFlagUnsettled = 1u << 1,     // the level had not settled 5 min after End (the ledger still had an event open)
  kFillFlagShortLookback = 1u << 2, // the 10-min look-back before Start was cut by the boot (the device had just started)
  kFillFlagTankChanged = 1u << 3,   // the tank size was changed during the window
  kFillFlagUncertain = 1u << 4,     // the level was not confirmed (tracker uncertain / holding / no radar) during the window
  kFillFlagUnverified = 1u << 5,    // the fuel ledger marked an event of the window FUEL_UNVERIFIED (may be a step off)
  kFillFlagAutoClosed = 1u << 6,    // closed by the device 2 h after Start (no End) or after End (no litres entered)
};
constexpr uint16_t kFillFlagsKnown = 0x7F;

// The signed fields (payload layout: DESIGN.md §14.2, big-endian).
struct FillReport {
  uint8_t format = kFillFormat;
  uint8_t key_id = 0;
  uint8_t mac[6] = {};
  uint16_t fw_version = 0;          // challenge form: major(5) minor(6) patch(5)
  uint32_t serial = 0;              // lifetime report serial: never reset, +1 per report
  uint16_t since_reset = 0;         // report number since the last Reset to defaults
  uint16_t gen = 0;                 // the fuel ledger's reset generation
  uint16_t settings_changes = 0;    // tank changes + resets to defaults (never reset; saturating)
  uint32_t phone_start = 0;         // unix s from the phone at Start (0 = unknown)
  uint32_t phone_end = 0;           // unix s from the phone at End (0 = not ended by the operator)
  int16_t tz_min = 0;               // the phone's local offset from UTC, minutes (IST +330)
  uint32_t duration_s = 0;          // device clock, Start -> End
  uint16_t lookback_s = 0;          // how far before Start the window begins (<= 600)
  uint32_t operator_cl = kFillOperatorNone;   // litres x 100 the operator entered
  uint32_t sensor_cl = 0;           // litres x 100 the ledger booked as filled in the window
  uint32_t drained_cl = 0;          // litres x 100 the ledger booked as drained in the window
  int32_t level_before = 0;         // 0.01 mm at the window start
  int32_t level_after = 0;          // 0.01 mm when the level had settled after End
  uint32_t lpmm_ul = 0;             // the tank's litres per mm x 1e6 used for the litres
  uint16_t flags = 0;
};

void fill_pack(const FillReport& r, uint8_t out[kFillPayloadBytes]);
bool fill_unpack(const uint8_t* in, size_t n, FillReport* r);     // format 1 and n >= kFillPayloadBytes
void fill_digest(const uint8_t payload[kFillPayloadBytes], uint8_t digest[32]);   // SHA-256(domain || payload)
// "FR-XXXXX-XXXXX-...": returns the length (0 if cap is too small).
size_t fill_code_text(const uint8_t code[kFillCodeBytes], char* out, size_t cap);
// Lenient: optional "FR" prefix, case-insensitive, ignores '-' and white space (base32 rules of challenge.h).
bool fill_code_from_text(const char* text, uint8_t out[kFillCodeBytes]);
// "Fill report #12 · MRM-Fuel-<MAC> · 14 Oct 14:05–14:20 · operator 50.0 L · sensor 49.6 L[ · flags: ...]" (UTF-8).
size_t fill_readable(const FillReport& r, char* out, size_t cap);
class JsonOut;
// {"serial","since_reset","line","code","operator_l"(null: not entered),"sensor_l","drained_l","level_before_mm",
//  "level_after_mm","lpmm","phone_start","phone_end","tz_min","duration_s","lookback_s","gen","settings_changes","flags"}
bool fill_write_json(JsonOut& j, const char* key, const uint8_t code[kFillCodeBytes]);
// Litres x 100 from level um and uL/mm (rounded, >= 0).
uint32_t fill_cl(int64_t um, uint32_t lpmm_ul);

// ---------------------------------------------------------------- the open report and the counters

enum class FillStage : uint8_t { Idle = 0, Filling = 1, Measuring = 2, Ready = 3 };
const char* fill_stage_text(FillStage s);

// Durable state (NVS, one blob: the counters and the open report change in ONE atomic write).
struct FillNv {
  uint32_t serial = 0;              // the last lifetime serial issued (0 = none yet); never reset
  uint16_t since_reset = 0;         // reports since the last Reset to defaults
  uint8_t stage = 0;                // FillStage of the open report (Ready is stored as Measuring)
  uint16_t flags = 0;
  uint32_t phone_start = 0, phone_end = 0;
  int16_t tz_min = 0;
  uint16_t lookback_s = 0;
  uint32_t dur_s = 0;               // device seconds of the open report before the current boot's reference (End or restart)
  int64_t base_filled_um = 0, base_drained_um = 0;   // the ledger totals at the window start
  int32_t base_level_um = 0;
  uint16_t base_gen = 0;
  uint32_t base_settings = 0;
};
constexpr size_t kFillNvBytes = 52;
void fill_nv_encode(const FillNv& nv, uint8_t out[kFillNvBytes]);
bool fill_nv_decode(const uint8_t* in, size_t n, FillNv* nv);
// Reset to defaults: the since-reset number to 0 and no open report; the lifetime serial is kept.
FillNv fill_nv_after_reset(const FillNv& nv);

// What the device measures, every tick (from the status snapshot).
struct FillLive {
  int64_t filled_um = 0, drained_um = 0;
  int32_t pending_um = 0;           // an event in progress (0: none)
  int32_t level_um = 0;
  bool has_level = true;            // false: the tracker has no level yet (just after boot); level_um is then 0
  bool confirmed = true;            // level confirmed and the radar delivering
  uint16_t gen = 0;
  uint32_t lpmm_ul = 0;
};

class FillStore {
 public:
  virtual ~FillStore() = default;
  virtual bool save(const FillNv& nv) = 0;                          // durable (committed) when it returns true
  virtual bool append(const uint8_t code[kFillCodeBytes]) = 0;      // the device's report list (best effort)
  virtual void logged(const FillReport& r) = 0;                     // FILL_REPORT event
};

enum class FillResult : uint8_t {
  Ok, Off, Busy, NotOpen, NotReady, Measuring, BadInput, RateLimited, SignError, StoreError,
};
const char* fill_result_text(FillResult r);

struct FillDone {
  FillReport r;
  uint8_t code[kFillCodeBytes] = {};
};

struct FillSample {
  uint32_t t_s = 0;
  int32_t level_um = 0;
  int64_t filled_um = 0, drained_um = 0;
  bool confirmed = true;
  bool has_level = true;
};

class FillBook {
 public:
  static constexpr int kRingN = 45;                 // 11 min of samples
  static constexpr uint32_t kSampleS = 15;
  static constexpr uint32_t kLookbackS = 600;       // 10 min before Start
  static constexpr uint32_t kSettleMinS = 75;       // after End: the ledger's 60-s after-window + margin
  static constexpr uint32_t kSettleMaxS = 300;      // then "unsettled"
  static constexpr uint32_t kAutoCloseS = 7200;     // 2 h without End / without litres
  static constexpr uint32_t kRetryS = 60;           // a failed automatic close (sign / NVS) is retried this often
  static constexpr uint32_t kTokens = 6;            // reports in a burst (anyone on the Wi-Fi can start one) ...
  static constexpr uint32_t kTokenS = 600;          // ... then one per 10 min: <= ~150 a day, 3 NVS writes each

  // key_d: the 32-byte private scalar (nullptr: the image has no fill key, the feature is off). now_s: uptime seconds.
  void begin(FillStore* store, const FillNv& loaded, const uint8_t* key_d, uint8_t key_id, const uint8_t mac[6],
             uint16_t fw_version, uint32_t now_s);
  bool enabled() const { return key_ != nullptr; }
  // Every second (loop task): samples, settling, automatic close. settings: the settings-changed counter.
  void tick(uint32_t now_s, const FillLive& live, uint32_t settings, FillDone* auto_done = nullptr, bool* auto_closed = nullptr);
  void note_tank_change(uint32_t now_s) { tank_t_ = now_s; tank_any_ = true; if (open()) nv_.flags |= kFillFlagTankChanged; }
  void note_unverified(uint32_t now_s) { unv_t_ = now_s; unv_any_ = true; if (open()) nv_.flags |= kFillFlagUnverified; }

  FillResult start(uint32_t now_s, uint32_t phone_t, int16_t tz_min, const FillLive& live, uint32_t settings);
  FillResult end(uint32_t now_s, uint32_t phone_t);
  // operator_cl: litres x 100 (<= 10,000,000). On Ok *out holds the report and its signed code.
  FillResult confirm(uint32_t now_s, uint32_t operator_cl, const FillLive& live, uint32_t settings, FillDone* out);

  FillStage stage() const { return stage_; }
  bool open() const { return stage_ != FillStage::Idle; }
  const FillNv& nv() const { return nv_; }
  uint32_t since_end_s(uint32_t now_s) const { return now_s - end_t_; }
  uint32_t since_start_s(uint32_t now_s) const { return now_s - start_t_; }
  uint32_t tokens(uint32_t now_s);
  // Reset to defaults while running (tests; the device resets at boot): counters and open report as fill_nv_after_reset.
  void apply_reset();

 private:
  int32_t last_level(int32_t fallback) const;
  void push_sample(uint32_t now_s, const FillLive& live);
  void settle(uint32_t now_s, const FillLive& live, bool timed_out);
  FillResult finish(uint32_t now_s, uint32_t operator_cl, const FillLive& live, uint32_t settings, FillDone* out);

  FillStore* store_ = nullptr;
  const uint8_t* key_ = nullptr;
  uint8_t key_id_ = 0;
  uint8_t mac_[6] = {};
  uint16_t fw_ = 0;
  FillNv nv_;
  FillStage stage_ = FillStage::Idle;
  uint32_t boot_t_ = 0, start_t_ = 0, end_t_ = 0, settle_t_ = 0, retry_t_ = 0;
  bool retry_any_ = false;
  // the settled values (Ready)
  int64_t end_filled_ = 0, end_drained_ = 0;
  int32_t end_level_ = 0;
  // the previous report's end (this boot): the next window never reaches back past it
  bool prev_ = false;
  uint32_t prev_t_ = 0;
  int64_t prev_filled_ = 0, prev_drained_ = 0;
  int32_t prev_level_ = 0;
  bool tank_any_ = false, unv_any_ = false;
  uint32_t tank_t_ = 0, unv_t_ = 0;
  uint32_t tok_milli_ = kTokens * 1000, tok_t_ = 0;
  FillSample ring_[kRingN];
  int ring_n_ = 0, ring_head_ = 0;                  // head = the next slot to write
  uint32_t last_sample_t_ = 0;
  bool sampled_ = false;
};

}  // namespace fs
