// Status model for /api/status (DESIGN.md §8) and a small bounded JSON writer used by every API reply.
#pragma once
#include <cstddef>
#include <cstdint>
#include "live.h"

namespace fs {

// Bounded JSON writer: never overflows; ok() turns false if the buffer was too small.
class JsonOut {
 public:
  JsonOut(char* buf, size_t cap);
  JsonOut& begin_obj(const char* key = nullptr);
  JsonOut& end_obj();
  JsonOut& begin_arr(const char* key = nullptr);
  JsonOut& end_arr();
  JsonOut& str(const char* key, const char* v);        // escapes
  JsonOut& num(const char* key, double v, int decimals);  // NaN/inf -> null
  JsonOut& integer(const char* key, long long v);
  JsonOut& uinteger(const char* key, unsigned long long v);
  JsonOut& boolean(const char* key, bool v);
  JsonOut& null(const char* key);
  bool ok() const { return ok_; }
  size_t len() const { return len_; }
  const char* c_str() const { return buf_; }

 private:
  void raw(const char* s);
  void rawn(const char* s, size_t n);
  void key(const char* k);
  void escaped(const char* s);
  char* buf_;
  size_t cap_, len_ = 0;
  bool ok_ = true;
  bool need_comma_ = false;
};

struct TankConfig {
  bool custom = false;
  double l_mm = 1755, w_mm = 790, h_mm = 190;
  double lpmm() const;                                  // custom: L*W/1e6; default: the calibrated factor
};
constexpr double kDefaultLitresPerMm = 1.3863856924996534;   // saved Eicher calibration factor

enum class StatusWord : uint8_t { Steady, Moving, Checking, Holding, Starting, NoRadar, SafeMode };

// Cumulative fuel since installation (fuel ledger, DESIGN.md §13). Level um: litres = um x lpmm / 1000 with the tank setting
// at the time of reading, so a later area calibration corrects the history too.
struct FuelStatus {
  int64_t filled_um = 0, drained_um = 0;
  uint32_t fills = 0, drains = 0;
  int32_t pending_um = 0;                               // an event in progress (signed; 0 = none)
  uint8_t active = 0;                                   // LedgerActive: 0 none, 1 fill, 2 drain, 3 slow drain
  uint16_t gen = 0;                                     // the record's reset generation (a decrease at the same gen = lost)
};
const char* fuel_active_text(uint8_t a);
const char* status_word_text(StatusWord w);
// status.cpp's "raw_litres" field (§8): 0-4095, linear on litres, 0 = empty, 4095 = this tank's capacity.
constexpr int kRawLitresMax = 4095;
int raw_litres_of(double litres, double capacity);
const char* source_text(Source s);
const char* state_text(TrackState s);
const char* cycle_text(CycleSource c);

struct DeviceStatus {
  // identity
  const char* fw = "";
  const char* tracker = "";
  const char* calibration = "";                         // "0x4242": the tracker calibration's id (calib_data.h)
  const char* variant = "";
  const char* chip = "";
  char mac[13] = {};                                    // 12 uppercase hex + NUL
  // level
  Output out;
  double age_s = 0;                                     // since the last frame that produced an estimate
  bool radar_ok = true;
  bool safe_mode = false;
  bool trial = false;
  int temp_c = 0;
  bool temp_valid = false;
  // counters
  uint64_t uptime_s = 0;
  uint32_t boots = 0, power_interruptions = 0;
  uint32_t settings_changes = 0;                        // fs_sys/chg: +1 per tank change and per reset to defaults, never reset
  uint32_t frames = 0, bad_packets = 0, gaps = 0;
  const char* reset_reason = "";
  bool coredump = false;
  uint32_t heap_min = 0;
  uint32_t max_est_us = 0;
  bool session = false;
  TankConfig tank;
  FuelStatus fuel;
  uint8_t raw_stream = 0;                               // debug raw stream (port 3333): 0 off (production), 1 ok, 2 no memory
};

StatusWord status_word(const DeviceStatus& s);

// Which fields a status reply carries (DESIGN.md §8):
//   Customer  what the customer dashboard and Maintenance read: identity (fw, variant, chip, mac), the level (level_mm,
//             litres, capacity_l, fill_pct, source, status, age_s), temp_c, uptime_s, boots, power_interruptions, tank,
//             radar.link and health.{safe_mode, trial, reset_reason, coredump}. Production /api/status.
//   Full      Customer + the technical fields: tracker, calibration, state, moving, settings_changes, radar.{frames, bad_packets, gaps},
//             health.{heap_min, max_est_us}. Debug /api/status (the raw-data recorders poll it) and /api/service/status.
// The fill/drain totals of the ledger are customer data: they go into both views.
enum class StatusView : uint8_t { Customer, Full };

// Writes the status fields into an open JSON object (for replies that add more objects after them).
void write_status_fields(JsonOut& j, const DeviceStatus& s, StatusView view);
// Writes the /api/status object. Returns false if the buffer was too small.
bool write_status_json(const DeviceStatus& s, char* buf, size_t cap, size_t* len, StatusView view = StatusView::Full);

}  // namespace fs
