// Runtime wiring (DESIGN.md §10, §11). Ownership: the tracker task is the ONLY user of the Live object; every other
// task reads the published snapshot under g_snap_lock. No heap allocation after setup.
#include "app.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <cmath>
#include <cstddef>
#include <cstring>
#include "../core/calib_data.h"
#include "../core/coredump_note.h"
#include "../core/ledger.h"
#include "../core/version.h"
#include "board.h"
#include "events.h"
#include "fill.h"
#include "keys_gen.h"
#include "logs.h"
#include "net.h"
#include "ota.h"
#include "radar_source.h"
#include "rawstream.h"
#include "store.h"
#include "testpad.h"
#include "timeouts.h"

namespace fs {

namespace {

AppInfo g_info;
Live g_live;
Auth g_auth;

// Frame queue: radar task -> tracker task.
constexpr UBaseType_t kQueueDepth = 8;
StaticQueue_t g_queue_buf;
uint8_t g_queue_storage[kQueueDepth * sizeof(Frame)];
QueueHandle_t g_queue = nullptr;
Frame g_radar_frame;                    // radar task's working frame (too big for its stack)
Frame g_dropped_frame;                  // radar task: the oldest frame dropped on overflow (692 B; not on its stack)
Frame g_tracker_frame;                  // tracker task's working frame

// Published snapshot.
SemaphoreHandle_t g_snap_lock = nullptr;
StaticSemaphore_t g_snap_lock_buf;
struct Snapshot {
  Output out;
  Estimate last_est;
  bool have_est = false;
  bool stored_valid = false;
  double stored_mm = NAN;
  int temp_c = 0;
  bool temp_valid = false;
  uint64_t last_frame_ms = 0, last_est_ms = 0;
  uint32_t frames = 0, estimates = 0, queue_drops = 0;
  uint32_t max_est_us = 0;
  // fuel ledger (DESIGN.md §13): the tracker task publishes, the loop task logs and saves
  LedgerRecord fuel;
  uint32_t fuel_bookings = 0;
  uint8_t fuel_active = 0;
  LedgerLog fuel_log[FuelLedger::kLogCap];
  int fuel_log_n = 0;
  uint32_t fuel_log_dropped = 0;
} g_snap;

// Fuel ledger (DESIGN.md §13): owned by the tracker task once it runs (set up in app_setup before).
FuelLedger g_ledger;
LedgerAdapter g_ledger_ad;
LedgerSaver g_ledger_saver;             // loop task, under g_saver_lock (with NVS fuel and /fuel.bin)
SemaphoreHandle_t g_saver_lock = nullptr;
StaticSemaphore_t g_saver_lock_buf;
volatile int32_t g_lpmm_req = 0;        // app_set_tank -> tracker task: the tank's uL per mm
bool g_fuel_save_error_logged = false;
bool g_boot_has_stored = false;         // the level the tracker starts from at boot (the ledger's GAP), and the age of the
int32_t g_boot_stored_um = 0;           // carried slow account at this boot's start: from its run's last estimate to the previous
int64_t g_carry_pre_ms = -1;            // boot's last heartbeat (-1: no carried slow account)
uint32_t g_jump_n = 0;                  // loop task: level jumps absorbed since the last LEVEL_JUMP entry (one per 30 min)
int32_t g_jump_last = 0;
uint64_t g_jump_logged_ms = 0;
bool g_jump_ever = false;
constexpr uint64_t kJumpLogMs = 30ULL * 60 * 1000;
// RTC copies for warm resets (no flash wear): the ledger record (the tracker task, on change), the save policy's token bucket
// (a warm reset loop cannot refill it), the slow account (carry_out at every block; a warm reset within 10 min continues it)
// and the loop task's heartbeat (the time from the last export to the reset).
struct RtcLedger {
  uint32_t magic;
  uint8_t blob[kLedgerBlobBytes];
  uint32_t check;
};
RTC_NOINIT_ATTR RtcLedger rtc_ledger;
RTC_NOINIT_ATTR LedgerSaveState rtc_ledger_save;
struct RtcCarry {
  uint32_t magic;
  uint32_t size;
  uint64_t uptime_ms;                   // when it was exported (this boot's uptime) ...
  int64_t pre_ms;                       // ... and the account's age then (0 when exported by its own run; a pending account
                                        // re-exported before the next run keeps ageing: a reset loop cannot refresh it)
  LedgerCarry c;
  uint32_t check;
};
RTC_NOINIT_ATTR RtcCarry rtc_carry;
RTC_NOINIT_ATTR uint64_t rtc_hb_ms;
RTC_NOINIT_ATTR uint32_t rtc_hb_check;
// The RTC magics carry a hash of their layouts (sizes and field offsets): a firmware with another layout never reads them.
constexpr uint32_t mix(uint32_t h, uint32_t v) { return (h ^ v) * 16777619u; }
constexpr uint32_t kRtcLedgerMagic = mix(mix(0x464C4733u, sizeof(RtcLedger)), uint32_t(kLedgerBlobBytes));   // "FLG3"
constexpr uint32_t kRtcCarryMagic =
    mix(mix(mix(mix(mix(mix(mix(0x4C435232u, sizeof(RtcCarry)), sizeof(LedgerCarry)), offsetof(RtcCarry, c)),
                    offsetof(LedgerCarry, slow_last)), offsetof(LedgerCarry, chain_end)), offsetof(LedgerCarry, flags)),
        uint32_t(kLedgerSlowN));                          // "LCR2"
constexpr uint32_t kRtcHbMagic = 0x48425431;              // "HBT1" (a u64: no layout)
uint32_t fnv(const void* p, size_t n, uint32_t h = 0x811C9DC5u) {
  const uint8_t* b = static_cast<const uint8_t*>(p);
  for (size_t i = 0; i < n; ++i) h = (h ^ b[i]) * 16777619u;
  return h;
}
uint32_t rtc_ledger_check(const RtcLedger& r) { return fnv(r.blob, sizeof r.blob, 0x811C9DC5u ^ r.magic); }
void rtc_ledger_put(const LedgerRecord& rec) {
  rtc_ledger.magic = kRtcLedgerMagic;
  ledger_encode(rec, 0, rtc_ledger.blob);
  rtc_ledger.check = rtc_ledger_check(rtc_ledger);
}
bool rtc_ledger_get(LedgerRecord* rec) {
  return rtc_ledger.magic == kRtcLedgerMagic && rtc_ledger.check == rtc_ledger_check(rtc_ledger) &&
         ledger_decode(rtc_ledger.blob, kLedgerBlobBytes, rec);
}
uint32_t rtc_carry_check() { return fnv(&rtc_carry, offsetof(RtcCarry, check)); }
// pre_ms: the account's age at this export (0: its run is going; else since that run's last estimate)
void rtc_carry_put(uint64_t now, int64_t pre_ms) {       // tracker task (or the reset with no tracker)
  rtc_carry.magic = 0;
  g_ledger.carry_out(&rtc_carry.c);
  rtc_carry.size = sizeof(LedgerCarry);
  rtc_carry.uptime_ms = now;
  rtc_carry.pre_ms = pre_ms > 0 ? pre_ms : 0;
  rtc_carry.magic = kRtcCarryMagic;
  rtc_carry.check = rtc_carry_check();
}
// The age of the ledger's slow account now: 0 while its run goes; a pending one (no estimate yet in this boot: carried in at
// boot; or a run ended by a Live restart) ages from that run's last estimate.
int64_t carry_age_ms(uint64_t now, bool estimated, uint64_t last_est_ms) {
  if (g_ledger.running()) return 0;
  if (!estimated) return (g_carry_pre_ms > 0 ? g_carry_pre_ms : 0) + int64_t(now);
  return now > last_est_ms ? int64_t(now - last_est_ms) : 0;
}
bool rtc_carry_ok() {
  return rtc_carry.magic == kRtcCarryMagic && rtc_carry.size == sizeof(LedgerCarry) && rtc_carry.check == rtc_carry_check();
}
uint32_t rtc_hb_check_of(uint64_t v) { return fnv(&v, sizeof v, kRtcHbMagic); }
// Warm boot: the record to restore. The RTC copy (always the newest) unless a reset happened that RTC missed: a newer
// generation wins; at the same generation RTC only if none of its totals or counts is below NVS's (a stale copy never undoes
// a saved booking).
bool rtc_preferred(const LedgerRecord& rtc, const LedgerRecord& nvs) {
  const int16_t dg = int16_t(uint16_t(rtc.gen - nvs.gen));
  if (dg != 0) return dg > 0;
  return rtc.filled_um >= nvs.filled_um && rtc.drained_um >= nvs.drained_um && rtc.n_fill >= nvs.n_fill &&
         rtc.n_drain >= nvs.n_drain;
}
int32_t lpmm_ul(const TankConfig& t) { return int32_t(std::llround(t.lpmm() * 1e6)); }
// Move the ledger's new log entries and its state into the snapshot (caller holds the lock).
void publish_fuel_locked() {
  g_snap.fuel = g_ledger.record();
  g_snap.fuel_bookings = g_ledger.bookings();
  g_snap.fuel_active = uint8_t(g_ledger.active());
  LedgerLog e;
  while (g_ledger.pop_log(&e)) {
    if (g_snap.fuel_log_n < FuelLedger::kLogCap) g_snap.fuel_log[g_snap.fuel_log_n++] = e;
    else ++g_snap.fuel_log_dropped;                       // the loop task drains it every second
  }
}

volatile bool g_reset_link = false;
volatile bool g_link_reopened = false;               // radar task -> tracker task: tell Live the link was re-opened
TaskHandle_t g_radar_task = nullptr, g_tracker_task = nullptr;

// Level persistence (loop task).
double g_saved_mm = NAN;
uint64_t g_saved_at_ms = 0;
bool g_save_error_logged = false;       // LEVEL_SAVE_ERROR once per run (a failing NVS would log it every minute)

// RTC copy of the last confirmed level for warm resets (watchdog, crash, software restart, update).
RTC_NOINIT_ATTR double rtc_level_mm;
RTC_NOINIT_ATTR uint32_t rtc_level_check;
constexpr uint32_t kRtcLevelMagic = 0x4C564C31;          // "LVL1"
uint32_t rtc_check_of(double v) {
  uint32_t w[2];
  std::memcpy(w, &v, 8);
  // The calibration id is part of the check: after an update with another calibration the RTC copy is ignored,
  // exactly as a flash record with another id is (DESIGN.md §9.1).
  return (kRtcLevelMagic ^ (uint32_t(cal::kCalibrationId) << 16)) ^ w[0] ^ (w[1] * 2654435761u);
}

// "0x4242": the calibration this image carries (status full view; tools/build.py --calib picks it at build time)
constexpr char hex_digit(unsigned v) { return "0123456789ABCDEF"[v & 15u]; }
constexpr char kCalibrationText[7] = {'0', 'x', hex_digit(cal::kCalibrationId >> 12), hex_digit(cal::kCalibrationId >> 8),
                                      hex_digit(cal::kCalibrationId >> 4), hex_digit(cal::kCalibrationId), '\0'};

// Supervisor / trial state (loop task).
uint64_t g_boot_ms = 0;
bool g_radar_lost_logged = false;
bool g_radar_ok_saved = false;          // fs_sys/radar_ok written true in this run
bool g_radar_ok_cleared = false;        // ... or written false (radar silent for kRadarSilentRestartMs)
uint64_t g_last_reinit_ms = 0;
uint64_t g_healthy_since_ms = 0;        // estimates flowing since (0 = not now)
bool g_trial = false;
uint64_t g_trial_deadline_ms = 0;
bool g_radar_required = false;
bool g_web_ok = false;
uint32_t g_trial_frames_at_start = 0;
uint32_t g_trial_estimates_at_start = 0;
uint64_t g_last_tick_ms = 0;
bool g_counters_flushed = false;
uint64_t g_last_hb_ms = 0;
uint64_t g_upload_seen_ms = 0;        // last supervisor tick with an upload active

bool lock(uint32_t ms = timeouts::kStatusLockMs) {
  return g_snap_lock && xSemaphoreTake(g_snap_lock, pdMS_TO_TICKS(ms)) == pdTRUE;
}
void unlock() { xSemaphoreGive(g_snap_lock); }

void radar_task(void*) {
  health_wdt_subscribe();
  RadarSource& radar = radar_source();
  bool paused = false;
  for (;;) {
    health_wdt_feed();
    if (g_reset_link) {
      g_reset_link = false;
      radar.reset_link();
      g_link_reopened = true;
    }
    // A firmware upload keeps the flash (and with it the cache) busy most of the time: the radar UART overflows, the
    // frames would be garbage and decoding the debris can keep core 0's idle task from running until the task watchdog
    // resets the board (hardware test 14 Oct 2026). The board restarts after an accepted upload anyway.
    if (ota_upload().active()) {
      if (!paused) FS_LOGI("radar", "paused during the firmware upload");
      paused = true;
      radar.drop_input();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (paused) {
      paused = false;
      g_reset_link = true;                                 // a clean start after an aborted upload
      FS_LOGI("radar", "resumed after the upload");
      continue;
    }
    if (!radar.read(g_radar_frame, timeouts::kRadarReadMs)) continue;
    if (xQueueSend(g_queue, &g_radar_frame, 0) != pdTRUE) {   // tracker behind: drop the oldest, keep the newest
      xQueueReceive(g_queue, &g_dropped_frame, 0);
      g_radar_frame.flags |= kFrameGapBefore;
      xQueueSend(g_queue, &g_radar_frame, 0);
      if (lock()) { ++g_snap.queue_drops; unlock(); }
    }
  }
}

void tracker_task(void*) {
  health_wdt_subscribe();
  Estimate est;
  uint32_t restarts_seen = g_live.restarts();
  bool prev_have_stored = false;
  double prev_stored = NAN;
  bool first_est = true;
  uint64_t last_est_ms = 0;
  uint32_t carry_seq = g_ledger.carry_seq() - 1;          // (exported at the first pass)
  LedgerRecord rtc_rec = g_ledger.record();
  for (;;) {
    health_wdt_feed();
    if (g_lpmm_req) {                                     // a new tank size (rates in L/h of the reference tank)
      g_ledger.set_lpmm(g_lpmm_req);
      g_lpmm_req = 0;
    }
    if (xQueueReceive(g_queue, &g_tracker_frame, pdMS_TO_TICKS(timeouts::kFrameQueueWaitMs)) != pdTRUE) continue;
    if (g_link_reopened) {                                // the next frame starts a tracker gap (DESIGN.md §11)
      g_link_reopened = false;
      g_live.note_link_gap();
    }
    int64_t t0 = esp_timer_get_time();
    bool got = g_live.push(g_tracker_frame, &est);
    uint32_t us = uint32_t(esp_timer_get_time() - t0);
    Output out = g_live.output();
    double stored = NAN;
    bool have_stored = g_live.stored_level(&stored);
    // Fuel ledger (a pure consumer of the estimate). The boot's first estimate and each Live restart (radar gap > 60 s)
    // start a new ledger run from the level the tracker (re)started from; the change across the gap is logged, never
    // counted. The outage tells the ledger whether to carry its slow account (<= 10 min): since the last estimate (a Live
    // restart), or since the previous boot's last carry export (a warm reset with the slow account from RTC memory).
    uint64_t now = uint64_t(esp_timer_get_time() / 1000);
    if (got) {
      if (first_est) {
        first_est = false;
        restarts_seen = g_live.restarts();
        g_ledger.begin_run(g_boot_has_stored, g_boot_stored_um, g_carry_pre_ms >= 0 ? g_carry_pre_ms + int64_t(now) : -1);
      } else if (g_live.restarts() != restarts_seen) {
        restarts_seen = g_live.restarts();
        g_ledger.begin_run(prev_have_stored, prev_have_stored ? int32_t(std::lround(prev_stored * 1000.0)) : 0,
                           int64_t(now - last_est_ms));
      }
      g_ledger.push(g_ledger_ad.convert(est));
      last_est_ms = now;
    }
    prev_have_stored = have_stored;
    prev_stored = stored;
    const LedgerRecord rec = g_ledger.record();
    if (rec != rtc_rec) {                                 // RTC copy for warm resets: on change, no flash wear
      rtc_rec = rec;
      rtc_ledger_put(rec);
    }
    if (g_ledger.carry_seq() != carry_seq) {              // the slow account for a warm reset (each block, ~600 B)
      carry_seq = g_ledger.carry_seq();
      rtc_carry_put(now, carry_age_ms(now, !first_est, last_est_ms));
    }
    if (lock()) {
      g_snap.out = out;
      g_snap.temp_c = g_tracker_frame.temperature_c;
      g_snap.temp_valid = true;
      g_snap.last_frame_ms = now;
      ++g_snap.frames;
      if (got) {
        g_snap.last_est = est;
        g_snap.have_est = true;
        g_snap.last_est_ms = now;
        ++g_snap.estimates;
        if (us > g_snap.max_est_us) g_snap.max_est_us = us;
      }
      g_snap.stored_valid = have_stored;
      if (have_stored) g_snap.stored_mm = stored;
      publish_fuel_locked();
      unlock();
    }
    if (have_stored) {                                    // RTC copy for warm resets: every frame, no flash wear
      rtc_level_mm = stored;
      rtc_level_check = rtc_check_of(stored);
    }
    if (board::kDebug) {
      rawstream_frame(g_tracker_frame);
      if (got) rawstream_estimate(g_tracker_frame.seq, est, out);
    }
  }
}

void persist_level(uint64_t now, bool force) {
  double h = NAN;
  bool have = false;
  if (lock()) {
    have = g_snap.stored_valid;
    h = g_snap.stored_mm;
    unlock();
  }
  if (!have || !std::isfinite(h)) return;
  bool changed = !std::isfinite(g_saved_mm) || std::fabs(h - g_saved_mm) >= timeouts::kLevelSaveMinChangeMm;
  // The cadence starts at boot (app_setup), not at the first confirmed level: a reset loop of short runs (5-60 s) would
  // otherwise write one NVS entry per boot (a 10-s brownout loop: ~125k erases per sector in 15 years; review round 3).
  bool due = force || now - g_saved_at_ms >= uint64_t(timeouts::kLevelSaveMinS) * 1000;
  if (!changed || !due) return;
  if (store().save_level(h)) {
    g_saved_mm = h;
    g_saved_at_ms = now;
  } else {
    if (!g_save_error_logged) {
      FS_LOGE("store", "level save failed (%.2f mm); retried every minute", h);
      log_event(Ev::LevelSaveError);
    }
    g_save_error_logged = true;
    g_saved_at_ms = now;                                  // retry at the next minute, not every second
  }
}

// Fuel ledger record -> NVS by the LedgerSaver policy (an event at once, progress of an open event every minute after
// 2 mm, a slow drain every 30 min, else every 3 h; token bucket), and the ledger's log entries -> the event log (Fuel budget
// class; absorbed level jumps: one LEVEL_JUMP per 30 min, Device class).
// LittleFS mirror of the record (/fuel.bin: u16 length, the record, CRC-32 over both): NVS has no static wear levelling and
// a worn or bit-flipped NVS is wiped by the core; the totals are theft evidence. Written with an NVS save at most every 6 h.
constexpr uint64_t kFuelMirrorMs = 6ULL * 3600 * 1000;
uint64_t g_fuel_mirror_ms = 0;
bool g_fuel_mirror_done = false;
LedgerRecord g_fuel_mirrored;
uint32_t crc32_le(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}
uint32_t le32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
// *blob / *n (optional): the record's bytes (a restore keeps a newer version's appended fields). The fw-1.0 development
// layout (the 32-byte blob + CRC-32, 36 bytes) is read too.
bool load_fuel_mirror(LedgerRecord* r, uint8_t* tokens, uint8_t* blob = nullptr, size_t* blob_n = nullptr) {
  uint8_t m[2 + kLedgerBlobMax + 4];
  size_t got = 0;
  if (!fs_file_read_n("fuel.bin", m, sizeof m, &got)) return false;
  const uint8_t* b = nullptr;
  size_t n = 0;
  if (got == size_t(kLedgerBlobBytes) + 4 && le32(m + kLedgerBlobBytes) == crc32_le(m, kLedgerBlobBytes)) {
    b = m;
    n = kLedgerBlobBytes;
  } else if (got >= 2) {
    n = size_t(m[0] | m[1] << 8);
    if (n < size_t(kLedgerBlobBytes) || n > size_t(kLedgerBlobMax) || got != 2 + n + 4 || le32(m + 2 + n) != crc32_le(m, 2 + n)) return false;
    b = m + 2;
  }
  if (!b || !ledger_decode(b, n, r, tokens)) return false;
  if (blob) std::memcpy(blob, b, n);
  if (blob_n) *blob_n = n;
  return true;
}
bool save_fuel_mirror(const LedgerRecord& r, uint8_t tokens) {
  uint8_t m[2 + kLedgerBlobMax + 4];
  const size_t n = store().ledger_blob(r, tokens, m + 2);
  m[0] = uint8_t(n);
  m[1] = uint8_t(n >> 8);
  const uint32_t c = crc32_le(m, 2 + n);
  for (int i = 0; i < 4; ++i) m[2 + n + i] = uint8_t(c >> (8 * i));
  return fs_file_write("fuel.bin", m, 2 + n + 4);
}

void persist_fuel(uint64_t now, bool force) {
  LedgerRecord rec;
  uint32_t bookings = 0;
  LedgerLog logs[FuelLedger::kLogCap];
  int nlog = 0;
  if (!lock()) return;
  rec = g_snap.fuel;
  bookings = g_snap.fuel_bookings;
  nlog = g_snap.fuel_log_n;
  for (int i = 0; i < nlog; ++i) logs[i] = g_snap.fuel_log[i];
  g_snap.fuel_log_n = 0;
  unlock();
  for (int i = 0; i < nlog; ++i) {
    const LedgerLog& e = logs[i];
    // FUEL_FILL / FUEL_DRAIN arg: bits 0-12 duration s (saturating), 13 unverified, 14 anchored repair, 15 provisional
    const uint16_t arg = uint16_t((e.dur_s > 0x1FFF ? 0x1FFF : e.dur_s) | (e.unverified ? 0x2000 : 0) | (e.repaired ? 0x4000 : 0) |
                                  (e.provisional ? 0x8000 : 0));
    const uint32_t v32 = uint32_t(e.value_um > 0xFFFFFFFFLL ? 0xFFFFFFFFLL : (e.value_um < 0 ? 0 : e.value_um));
    const int64_t sv = e.value_um > INT32_MAX ? INT32_MAX : (e.value_um < INT32_MIN ? INT32_MIN : e.value_um);
    if ((e.kind == LedgerLog::kFill || e.kind == LedgerLog::kDrain) && e.unverified) fill_note_unverified();
    switch (e.kind) {
      case LedgerLog::kFill: log_event(Ev::FuelFill, arg, v32); break;
      case LedgerLog::kDrain: log_event(Ev::FuelDrain, arg, v32); break;
      case LedgerLog::kSlow: log_event(Ev::FuelSlow, 0, v32); break;
      case LedgerLog::kGap: log_event(Ev::FuelGap, 0, uint32_t(int32_t(sv))); break;
      case LedgerLog::kUnverified:                        // arg 1: the fill, 2: the drain logged last; value = its volume
        log_event(Ev::FuelUnverified, sv > 0 ? 1 : 2, uint32_t(sv > 0 ? sv : -sv));
        fill_note_unverified();
        break;
      default:                                            // kJump: counted, logged below
        if (g_jump_n < 0xFFFF) ++g_jump_n;
        g_jump_last = int32_t(sv);
    }
  }
  if (g_jump_n && (force || !g_jump_ever || now - g_jump_logged_ms >= kJumpLogMs)) {   // (a restart flushes the count)
    log_event(Ev::LevelJump, uint16_t(g_jump_n), uint32_t(g_jump_last));
    g_jump_n = 0;
    g_jump_logged_ms = now;
    g_jump_ever = true;
  }
  if (!g_saver_lock || xSemaphoreTake(g_saver_lock, pdMS_TO_TICKS(timeouts::kStatusLockMs)) != pdTRUE) return;
  if (g_ledger_saver.unsaved(rec, bookings) && (force || g_ledger_saver.due(rec, bookings, now))) {
    const uint8_t tok = g_ledger_saver.debit(now);        // the write costs a token even if it fails (no retry storm)
    if (store().save_ledger(rec, tok)) {
      g_ledger_saver.saved(rec, bookings, now);
    } else if (!g_fuel_save_error_logged) {               // (unsaved: due again while tokens remain)
      log_event(Ev::LevelSaveError, 2);                   // arg 2: the fuel totals
      g_fuel_save_error_logged = true;
    }
    if ((force || !g_fuel_mirror_done || now - g_fuel_mirror_ms >= kFuelMirrorMs) && rec != g_fuel_mirrored &&
        save_fuel_mirror(rec, tok)) {                     // (a failed mirror write is retried at the next save)
      g_fuel_mirrored = rec;
      g_fuel_mirror_ms = now;
      g_fuel_mirror_done = true;
    }
  }
  xSemaphoreGive(g_saver_lock);
}

void restart_now(uint16_t cause) {
  persist_level(app_now_ms(), true);
  FS_LOGW("app", "restart, cause %u (1 maintenance, 2 update, 3 radar lost, 4 no estimate, 5 low memory, 6 safe-mode retry, "
          "7 reset to defaults)", unsigned(cause));
  persist_fuel(app_now_ms(), true);
  log_event(Ev::Restart, cause);
  delay(100);
  const uint64_t hb = app_now_ms();                       // the heartbeat now: the carried slow account's age is exact
  rtc_hb_ms = hb;
  rtc_hb_check = rtc_hb_check_of(hb);
  esp_restart();
}

// Internal RAM only: on the WROVER the PSRAM is part of the heap (CONFIG_SPIRAM_USE_MALLOC), so the total would
// never fall under the floor even when Wi-Fi and lwIP run out of internal memory.
uint32_t internal_free() { return uint32_t(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)); }
uint32_t internal_min_free() { return uint32_t(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)); }

// Radar-lost / no-estimate restarts: never during the trial (it has its own verdict) or an upload, and at most
// kSupervisorRestarts in a row; after that the device stays up, shows the stored level and keeps re-opening the link.
bool supervisor_restart_ok() {
  return !g_trial && !ota_upload().active() && health_supervisor_restart_allowed();
}

// Milliseconds from t to now, 0 if t is later. The tracker task stamps g_snap with ITS clock reading; when it runs
// between the caller's app_now_ms() and the snapshot read (preemption on the C3, the other core on the WROVER),
// t > now and a plain unsigned difference wraps to ~1.8e19 ms: "radar silent", link reset, radar_ok cleared, restart.
uint64_t ms_since(uint64_t now, uint64_t t) { return now > t ? now - t : 0; }

uint32_t g_low_heap_s = 0;              // consecutive supervisor ticks under the internal-heap floor

void supervise(uint64_t now) {
  if (g_info.health.safe_mode) return;
  uint64_t last_frame, last_est;
  uint32_t frames, free_heap = internal_free();
  if (!lock()) return;
  last_frame = g_snap.last_frame_ms;
  last_est = g_snap.last_est_ms;
  frames = g_snap.frames;
  unlock();
  if (ota_upload().active()) g_upload_seen_ms = now;      // radar paused during an upload: not "silent"
  uint64_t frame_ref = last_frame ? last_frame : g_boot_ms;
  if (g_upload_seen_ms > frame_ref) frame_ref = g_upload_seen_ms;
  uint64_t since_frame = ms_since(now, frame_ref);
  if (since_frame > timeouts::kRadarSilentReinitMs) {
    if (!g_radar_lost_logged) {
      FS_LOGW("radar", "link lost: no frame for %llu ms (%u frames so far)", (unsigned long long)since_frame, unsigned(frames));
      log_event(Ev::RadarLost, 0, frames);
      g_radar_lost_logged = true;
      g_reset_link = true;
      g_last_reinit_ms = now;
    } else if (now - g_last_reinit_ms >= timeouts::kRadarReinitEveryMs) {
      g_reset_link = true;
      g_last_reinit_ms = now;
    }
    if (since_frame > timeouts::kRadarSilentRestartMs) {
      // The radar no longer works: an update must not be rolled back for missing frames (DESIGN.md §10 trial).
      if (!g_radar_ok_cleared) {
        store().set_radar_ok(false);
        g_radar_ok_cleared = true;
        g_radar_ok_saved = false;
      }
      if (supervisor_restart_ok()) {
        health_supervisor_restart_noted();
        restart_now(3);
      }
    }
  } else if (g_radar_lost_logged) {
    FS_LOGI("radar", "link back (%u frames)", unsigned(frames));
    log_event(Ev::RadarBack, 0, frames);
    g_radar_lost_logged = false;
  }
  bool estimating = last_est && ms_since(now, last_est) < 2000;
  if (!estimating) g_healthy_since_ms = 0;
  else if (!g_healthy_since_ms) g_healthy_since_ms = now;
  else if (now - g_healthy_since_ms >= timeouts::kSupervisorHealthyMs && free_heap >= timeouts::kHeapFloorBytes)
    health_supervisor_healthy();                          // a unit short of memory is not "healthy": no re-arm
  if (last_frame && ms_since(now, last_frame) < 2000 && frames > 100 &&
      ms_since(now, last_est ? last_est : g_boot_ms) > timeouts::kNoEstimateRestartMs && supervisor_restart_ok()) {
    health_supervisor_restart_noted();
    restart_now(4);                                       // frames arrive but the tracker produces nothing
  }
  // Low memory must persist (a burst of Wi-Fi RX buffers is transient) and counts like the other supervisor restarts:
  // a unit whose internal heap stays under the floor would otherwise reboot every few seconds, forever.
  if (free_heap >= timeouts::kHeapFloorBytes || ota_upload().active()) {
    g_low_heap_s = 0;
  } else if (++g_low_heap_s >= timeouts::kHeapFloorHoldS && supervisor_restart_ok()) {
    health_supervisor_restart_noted();
    restart_now(5);
  }
  // After a clear, radar_ok becomes true again only after kSupervisorHealthyMs of estimates: a radar that drops out
  // for a minute at a time would otherwise toggle two NVS entries a minute (3x the NVS wear of the level saves).
  bool healthy_long = g_healthy_since_ms && now - g_healthy_since_ms >= timeouts::kSupervisorHealthyMs;
  if (!g_radar_ok_saved && frames >= 100 && since_frame <= timeouts::kRadarSilentReinitMs &&
      (!g_radar_ok_cleared || healthy_long)) {
    store().set_radar_ok(true);
    g_radar_ok_saved = true;
    g_radar_ok_cleared = false;
  }
}

// Safe mode is left by a restart once the crash-loop counter has cleared (kSafeModeClearS): otherwise a unit that
// had three quick crashes (or a burst of them) stays without radar until someone power-cycles it.
void safe_mode_tick(uint64_t now) {
  if (!g_info.health.safe_mode || now - g_boot_ms < uint64_t(timeouts::kSafeModeClearS) * 1000) return;
  if (g_trial || ota_upload().active() || g_auth.session_active(now)) return;   // not under a maintenance session
  restart_now(6);
}

// COREDUMP_FOUND only for a dump not reported before (core/coredump_note.h): its id is the size plus the dump's first
// and last 32 bytes (header and checksum).
void note_coredump() {
  uint32_t id = 0;
  if (g_info.health.coredump) {
    uint8_t head[32] = {}, tail[32] = {};
    size_t n = g_info.health.coredump_size;
    size_t k = n < sizeof head ? n : sizeof head;
    bool ok = health_coredump_read(0, head, k) && health_coredump_read(n - k, tail, k);
    id = coredump_id(uint32_t(n), head, ok ? k : 0, tail, ok ? k : 0);
  }
  CoredumpNotice c = coredump_notice(g_info.health.coredump, id, store().coredump_noted());
  if (c.log) {
    FS_LOGW("boot", "crash dump stored (%u bytes)", unsigned(g_info.health.coredump_size));
    log_event(Ev::CoredumpFound, 0, uint32_t(g_info.health.coredump_size));
  }
  if (c.write) store().set_coredump_noted(c.id);
}

void finish_migration() {
  if (store().migration_done()) return;
  if (store().finish_v63_migration()) {
    FS_LOGI("mig", "v6.3 data erased (migration finished)");
    log_event(Ev::Migrated, 2);   // arg 2: v6.3 data erased
  }
}

void trial_tick(uint64_t now) {
  if (!g_trial) return;
  uint32_t frames = 0, estimates = 0;
  if (lock()) { frames = g_snap.frames; estimates = g_snap.estimates; unlock(); }
  if (!g_web_ok) g_web_ok = net_self_check();
  bool radar_ok = !g_radar_required || (frames - g_trial_frames_at_start >= timeouts::kTrialMinFrames &&
                                        estimates - g_trial_estimates_at_start >= timeouts::kTrialMinEstimates);
  // A radar that could not start fails the trial only through radar_ok (when it worked before the update).
  bool tasks_ok = g_info.health.safe_mode || !g_info.radar_started || (g_radar_task && g_tracker_task);
  if (g_web_ok && radar_ok && tasks_ok && g_info.nvs_ok) {
    if (!ota_mark_valid()) return;                       // retried every second until the deadline
    g_trial = false;
    store().set_fw_ok(uint32_t(kFwVersionU32));
    store().set_pending(0);
    FS_LOGI("trial", "self-test passed: firmware %s kept (frames %u, estimates %u)", FW_VERSION_STRING,
            unsigned(frames - g_trial_frames_at_start), unsigned(estimates - g_trial_estimates_at_start));
    log_event(Ev::TrialPassed, 0, uint32_t(kFwVersionU32));
    finish_migration();                                  // this firmware is kept: now the v6.3 data may go
    return;
  }
  if (now >= g_trial_deadline_ms) {
    FS_LOGE("trial", "self-test FAILED (web %d, radar %d: %u frames %u estimates, tasks %d, nvs %d): rolling back",
            int(g_web_ok), int(radar_ok), unsigned(frames - g_trial_frames_at_start),
            unsigned(estimates - g_trial_estimates_at_start), int(tasks_ok), int(g_info.nvs_ok));
    persist_level(now, true);
    ota_rollback_now();                                  // does not return
  }
}

void make_identity() {
  esp_efuse_mac_get_default(g_info.mac);
  std::snprintf(g_info.mac_hex, sizeof g_info.mac_hex, "%02X%02X%02X%02X%02X%02X", g_info.mac[0], g_info.mac[1],
                g_info.mac[2], g_info.mac[3], g_info.mac[4], g_info.mac[5]);
  std::snprintf(g_info.ssid, sizeof g_info.ssid, "%s%s", board::kSsidPrefix, g_info.mac_hex);   // 21 chars <= 32
}

// The flash level, and the RTC copy when it is valid after a warm reset (fresher than the once-a-minute flash copy).
// rtc_trusted: the previous run was this firmware family. After v6.3 ran (migration not finished) the RTC copy is
// whatever OUR firmware left before v6.3 took over (a failed first-install trial), possibly days old.
double boot_level(double* flash_mm, bool rtc_trusted) {
  *flash_mm = NAN;
  store().load_level(flash_mm);
  // An ESP32 brownout keeps RTC memory (esp_restart_noos), as the boot counters and the event budget assume. On the C3
  // (core 2.0.14) a brownout is a chip reset that reads as ESP_RST_POWERON, i.e. a cold boot (bench item, DESIGN §10).
  bool warm = g_info.health.reset_reason != int(ESP_RST_POWERON);
  if (rtc_trusted && warm && rtc_level_check == rtc_check_of(rtc_level_mm) && std::isfinite(rtc_level_mm)) return rtc_level_mm;
  return *flash_mm;
}

}  // namespace

const AppInfo& app_info() { return g_info; }
uint64_t app_now_ms() { return uint64_t(esp_timer_get_time() / 1000); }
Auth& app_auth() { return g_auth; }
bool app_trial_active() { return g_trial; }
uint32_t app_trial_remaining_s() {
  uint64_t now = app_now_ms();
  return g_trial && g_trial_deadline_ms > now ? uint32_t((g_trial_deadline_ms - now) / 1000) : 0;
}
void app_save_level_now() { persist_level(app_now_ms(), true); }
void app_coredump_erased() {
  g_info.health.coredump = false;
  g_info.health.coredump_size = 0;
}

bool app_set_tank(const TankConfig& t) {
  if (!store().set_tank(t)) return false;
  g_lpmm_req = lpmm_ul(store().tank());                   // the tracker task applies it to the ledger
  log_event(Ev::TankChanged, t.custom ? 1 : 0, uint32_t(std::lround(t.h_mm)));
  if (!store().bump_settings_changes()) FS_LOGE("cfg", "settings-changed counter not saved");
  fill_note_tank_change();                                // an open fill report is flagged
  return true;
}

namespace {

// The fuel record's copies for the ledger item of Reset to defaults (core/reset.h ledger_zero_at_boot), at boot before the
// ledger is restored.
class AppLedgerReset final : public LedgerResetStore {
 public:
  bool newest(LedgerRecord* r, uint8_t* tokens) override {   // the newest copy: what the reset clears (FUEL_RESET's old totals)
    bool have = store().load_ledger(r, tokens);
    LedgerRecord m, x;
    uint8_t mtok = 0;
    if (g_info.events_ok && load_fuel_mirror(&m, &mtok) && (!have || rtc_preferred(m, *r))) { *r = m; *tokens = mtok; have = true; }
    if (g_info.health.reset_reason != int(ESP_RST_POWERON) && rtc_ledger_get(&x) && (!have || rtc_preferred(x, *r))) {
      *r = x;
      have = true;
    }
    return have;
  }
  void log_old(const LedgerRecord& old) override {        // FUEL_RESET: arg 1 filled, arg 2 drained, value mm
    log_event(Ev::FuelReset, 1, uint32_t(old.filled_um / 1000 > 0xFFFFFFFFLL ? 0xFFFFFFFFLL : old.filled_um / 1000));
    log_event(Ev::FuelReset, 2, uint32_t(old.drained_um / 1000 > 0xFFFFFFFFLL ? 0xFFFFFFFFLL : old.drained_um / 1000));
  }
  void drop_rtc() override {
    rtc_ledger.magic = 0;
    rtc_carry.magic = 0;
  }
  bool write_nvs(const LedgerRecord& z, uint8_t tokens) override { return store().save_ledger(z, tokens); }   // (tail kept)
  bool write_mirror(const LedgerRecord& z, uint8_t tokens) override {
    if (!g_info.events_ok || !save_fuel_mirror(z, tokens)) return false;
    g_fuel_mirrored = z;
    return true;
  }
  bool nvs_usable() override { return g_info.nvs_ok; }
};

// What "Reset to defaults" sets back on this device (core/reset.h kResetItems, DESIGN.md §9.3), at the boot after the
// request, before any of it is used. Each case is idempotent: a boot cut in the middle repeats it.
class AppReset final : public ResetTarget {
 public:
  bool reset_item(uint16_t item, uint16_t gen) override {
    switch (item) {
      case kResetTank: return store().set_tank(TankConfig{});   // NVS + /tank.bin (the ledger's L/mm is set from it later)
      case kResetLevel: {                                       // no remembered level: the start-up runs without memory
        rtc_level_check = ~rtc_check_of(rtc_level_mm);
        rtc_level_mm = NAN;
        return store().erase_level();
      }
      case kResetLedger: {                                      // the fuel totals to 0, generation gen (FUEL_RESET first)
        AppLedgerReset s;
        return ledger_zero_at_boot(s, gen);
      }
      case kResetFillReports: return fill_reset_at_boot();     // the list and the since-reset number (serial kept)
      // case kResetTankCalibration: ...                       // v7.0 site calibration
      default: return true;                                   // a reserved item this firmware does not hold
    }
  }
  bool finished(uint16_t gen) override {
    AppLedgerReset s;
    LedgerRecord r;
    uint8_t tok = 0;
    return s.newest(&r, &tok) && r.gen == gen;
  }
  void committed(uint16_t items, bool at_boot) override {
    FS_LOGW("maint", "reset to defaults (items 0x%x)%s", unsigned(items), at_boot ? ": done at boot" : ": requested, at the next boot");
    log_event(Ev::ResetToDefaults, items, at_boot ? 1 : 0);
  }
};
AppReset g_reset;

}  // namespace

// The request (net.cpp h_reset): journal + RESET_TO_DEFAULTS(items, 0), then a restart in about 2 s; the reset itself runs at
// the next boot (resume_reset in app_setup). The fuel totals' target generation = the live record's + 1.
ResetResult app_request_reset() {
  uint16_t gen = 0;
  if (lock()) { gen = g_snap.fuel.gen; unlock(); }
  ResetResult r = request_reset(store_reset_journal(), g_reset, kResetItems, uint16_t(gen + 1));
  if (r == ResetResult::Ok) {
    if (!store().bump_settings_changes()) FS_LOGE("cfg", "settings-changed counter not saved");
    health_clear_crash_loop();                           // (a planned restart, not a crash)
    health_restart_soon(7, 2000);                        // the reply goes out first; level and totals are saved first
  }
  return r;
}

void app_status(DeviceStatus* s) {
  *s = DeviceStatus{};
  s->fw = FW_VERSION_STRING;
  s->tracker = TRACKER_VERSION;
  s->calibration = kCalibrationText;
  s->variant = board::kVariantName;
  s->chip = board::kChipName;
  std::memcpy(s->mac, g_info.mac_hex, sizeof s->mac);
  uint64_t now = app_now_ms();
  s->uptime_s = now / 1000;
  const SysCounters& c = store().counters();
  s->boots = c.boots;
  s->power_interruptions = c.power;
  s->settings_changes = store().settings_changes();
  s->reset_reason = g_info.health.reset_text;
  s->safe_mode = g_info.health.safe_mode;
  s->trial = g_trial;
  s->coredump = g_info.health.coredump;
  s->heap_min = internal_min_free();
  s->session = g_auth.session_active(now);
  s->raw_stream = rawstream_state();
  s->tank = store().tank();
  RadarStats rs = radar_source().stats();
  s->bad_packets = rs.bad;
  s->gaps = rs.gaps;
  if (lock()) {
    s->out = g_snap.out;
    s->frames = g_snap.frames;
    s->max_est_us = g_snap.max_est_us;
    s->temp_c = g_snap.temp_c;
    s->temp_valid = g_snap.temp_valid;
    s->age_s = g_snap.last_est_ms ? double(ms_since(now, g_snap.last_est_ms)) / 1000.0 : 0.0;
    // Before the first frame: "starting" for the first kRadarSilentReinitMs, not "no radar" at every boot.
    s->radar_ok = g_snap.last_frame_ms ? ms_since(now, g_snap.last_frame_ms) <= timeouts::kRadarSilentReinitMs
                                       : g_info.radar_started && now - g_boot_ms <= timeouts::kRadarSilentReinitMs;
    s->fuel.filled_um = g_snap.fuel.filled_um;
    s->fuel.drained_um = g_snap.fuel.drained_um;
    s->fuel.fills = g_snap.fuel.n_fill;
    s->fuel.drains = g_snap.fuel.n_drain;
    s->fuel.pending_um = g_snap.fuel.pend_um;
    s->fuel.active = g_snap.fuel_active;
    s->fuel.gen = g_snap.fuel.gen;
    unlock();
  }
  if (g_info.health.safe_mode) s->radar_ok = false;
  if (!std::isfinite(s->out.level_mm)) s->out.level_mm = 0;   // always a value (DESIGN.md §1)
}

bool app_last_estimate(Estimate* e, uint32_t* stack_tracker, uint32_t* stack_radar) {
  *stack_tracker = g_tracker_task ? uint32_t(uxTaskGetStackHighWaterMark(g_tracker_task)) : 0;
  *stack_radar = g_radar_task ? uint32_t(uxTaskGetStackHighWaterMark(g_radar_task)) : 0;
  if (!lock()) return false;
  bool have = g_snap.have_est;
  if (have) *e = g_snap.last_est;
  unlock();
  return have;
}

void app_setup() {
  g_boot_ms = app_now_ms();
  health_early(&g_info.health);
  health_wdt_begin();
  health_wdt_subscribe();                                 // the Arduino loop task (setup runs in it)
  g_snap_lock = xSemaphoreCreateMutexStatic(&g_snap_lock_buf);
  g_saver_lock = xSemaphoreCreateMutexStatic(&g_saver_lock_buf);
  g_queue = xQueueCreateStatic(kQueueDepth, sizeof(Frame), g_queue_storage, &g_queue_buf);
  make_identity();
  FS_LOGI("boot", "MRM fuel sensor fw %s (tracker %s, %s, %s), %s", FW_VERSION_STRING, TRACKER_VERSION, board::kVariantName,
          board::kChipName, g_info.ssid);
  if (test_pad_bytes()) FS_LOGW("boot", "SIZE-TEST BUILD: %u bytes of padding (checksum %08x)", unsigned(test_pad_bytes()),
                              unsigned(test_pad_checksum()));

  g_info.nvs_ok = store().begin();
  store().count_boot(g_info.health.reset_reason);
  g_info.events_ok = events_begin(store().counters().boots, g_info.health.reset_reason == int(ESP_RST_POWERON));
  store().raise_boots(events_boot());                     // status/HELLO boots = the log's boot number
  {
    const SysCounters& c = store().counters();
    FS_LOGI("boot", "reset %s, boot %u (power-ons %u, crashes %u, watchdogs %u, brownouts %u), nvs %s, event log %s%s",
            g_info.health.reset_text, unsigned(c.boots), unsigned(c.power), unsigned(c.crashes), unsigned(c.watchdogs),
            unsigned(c.brownouts), g_info.nvs_ok ? "ok" : "FAILED", g_info.events_ok ? "ok" : "FAILED",
            g_info.health.safe_mode ? ", SAFE MODE" : "");
  }
  bool nvs_restored = g_info.events_ok && store().sync_tank_mirror();   // NVS wiped by the core: tank size back
  health_wdt_feed();
  log_event(Ev::Boot, uint16_t(g_info.health.reset_reason), store().counters().boots);
  note_coredump();                                       // COREDUMP_FOUND once per dump, not at every boot
  if (nvs_restored) log_event(Ev::NvsRestored, 1, store().counters().boots);

  ota_boot_check(store().pending(), &g_info.boot);
  FS_LOGI("ota", "running %s, trial %s", FW_VERSION_STRING, g_info.boot.trial ? "yes (self-test within 90 s)" : "no");
  if (g_info.boot.rolled_back) {
    FS_LOGW("ota", "rolled back: firmware 0x%06x failed its trial", unsigned(g_info.boot.rejected_version));
    log_event(Ev::RolledBack, 0, g_info.boot.rejected_version);
    store().set_pending(0);
  }
  bool v63_seen = false;
  const bool v63_pending = !store().migration_done();   // v6.3 may have run since our last boot
  if (v63_pending) {
    double mig_level = NAN;
    g_info.migrated = store().import_from_v63(&mig_level, &v63_seen);
    // Until the migration finishes, v6.3's level is the authoritative one (a retry after a rollback re-imports it).
    if (std::isfinite(mig_level)) store().save_level(mig_level);
    if (g_info.migrated) log_event(Ev::Migrated, std::isfinite(mig_level) ? 1 : 0);
    FS_LOGI("mig", "v6.3 data %s, imported %s, level %.2f mm", v63_seen ? "found" : "not found", g_info.migrated ? "yes" : "no",
            mig_level);
    if (!g_info.boot.trial) finish_migration();          // USB install: nothing to roll back to
  }
  // A requested Reset to defaults runs now: after the v6.3 import (a pending migration would otherwise put the tank and the
  // level back at every boot), before the level, the tracker and the fuel ledger are read; a boot cut in the middle
  // finishes it at the next one (the fuel totals' generation marks it done).
  if (resume_reset(store_reset_journal(), g_reset)) FS_LOGW("boot", "reset to defaults pending at boot: done or retried");
  double flash_mm = NAN;
  // v6.3 ran since our last boot only if its namespaces exist (it writes fuel-update at every boot; only our finished
  // migration erases them). A wiped NVS also reads "pending" but keeps the RTC copy valid.
  double level = boot_level(&flash_mm, !v63_pending || !v63_seen);
  // g_saved_mm must be what FLASH holds: comparing against a fresher RTC copy would keep a stale flash level until
  // the level moves 0.05 mm away from the RTC value, i.e. possibly for days. A fresher RTC level is written now.
  g_saved_mm = flash_mm;
  g_saved_at_ms = app_now_ms();                           // the 60-s save cadence starts now (persist_level)
  // Not after a brownout: a brownout loop of short runs would write one entry per boot; the 60-s cadence persists it.
  if (g_info.health.reset_reason != int(ESP_RST_BROWNOUT) && std::isfinite(level) && (!std::isfinite(flash_mm) ||
                               std::fabs(level - flash_mm) >= timeouts::kLevelSaveMinChangeMm) &&
      store().save_level(level)) {
    g_saved_mm = level;
    g_saved_at_ms = app_now_ms();
  }
  FS_LOGI("boot", "start level %.2f mm (flash %.2f mm)", level, flash_mm);
  g_live.begin(level);
  // Fuel ledger: the newest record (RTC after a warm reset unless NVS holds a newer generation, else NVS); a provisional
  // volume left by a cut is booked now. An NVS without the record (new device, or wiped) falls back to the LittleFS mirror
  // (written at most every 6 h), which is then written back to NVS once. After a warm reset the slow account comes back
  // from RTC memory (the ledger carries it if the first estimate comes within 10 min of the last export).
  g_ledger.set_lpmm(lpmm_ul(store().tank()));
  const bool warm = g_info.health.reset_reason != int(ESP_RST_POWERON);
  LedgerRecord flash_rec, rec, rtc_rec, mirror;
  uint8_t flash_tok = 0, mirror_tok = 0;
  uint8_t mirror_blob[kLedgerBlobMax];
  size_t mirror_n = 0;
  bool have_flash = store().load_ledger(&flash_rec, &flash_tok);
  const bool have_mirror = g_info.events_ok && load_fuel_mirror(&mirror, &mirror_tok, mirror_blob, &mirror_n);
  if (have_mirror) g_fuel_mirrored = mirror;              // (the mirror is rewritten only when the record differs)
  bool from_mirror = false;                               // the newer of NVS and the mirror (the warm-boot rule of RTC):
  if (have_mirror && (!have_flash || (mirror != flash_rec && rtc_preferred(mirror, flash_rec)))) {   // NVS wiped, or a write
    if (!have_flash) log_event(Ev::NvsRestored, 2, store().counters().boots);   // it missed (arg 2: fuel totals from /fuel.bin)
    flash_rec = mirror;
    flash_tok = mirror_tok;
    have_flash = from_mirror = true;
    store().adopt_ledger_blob(mirror_blob, mirror_n);
  }
  rec = have_flash ? flash_rec : LedgerRecord{};
  if (warm && rtc_ledger_get(&rtc_rec) && (!have_flash || rtc_preferred(rtc_rec, flash_rec))) rec = rtc_rec;
  g_ledger.restore(rec);
  // the carried account's age at this boot: its age at the export + export to the last heartbeat (an export up to 2 s after
  // the last heartbeat: 0; restart_now refreshes the heartbeat)
  if (warm && rtc_carry_ok() && rtc_hb_check == rtc_hb_check_of(rtc_hb_ms) && rtc_hb_ms + 2000 >= rtc_carry.uptime_ms) {
    g_ledger.carry_in(rtc_carry.c);
    g_carry_pre_ms = rtc_carry.pre_ms + (rtc_hb_ms > rtc_carry.uptime_ms ? int64_t(rtc_hb_ms - rtc_carry.uptime_ms) : 0);
  }
  rtc_carry.magic = 0;                                    // (this boot exports its own once the tracker runs)
  g_boot_has_stored = std::isfinite(level);
  g_boot_stored_um = g_boot_has_stored ? int32_t(std::lround(level * 1000.0)) : 0;
  if (from_mirror && store().save_ledger(g_ledger.record(), flash_tok)) flash_rec = g_ledger.record();   // once
  // a missing, corrupt or stale mirror is repaired now (one LittleFS write; then again at the first save and every 6 h)
  if (g_info.events_ok && have_flash && (!have_mirror || mirror != flash_rec) && save_fuel_mirror(flash_rec, flash_tok))
    g_fuel_mirrored = flash_rec;
  // a cold boot starts the token bucket from the count in the record (no record: a full bucket)
  g_ledger_saver.begin(&rtc_ledger_save, flash_rec, 0, app_now_ms(), have_flash ? int(flash_tok) : -1);
  rtc_ledger_put(g_ledger.record());
  if (lock()) {                                           // until the first frame: output = stored level
    g_snap.out = g_live.output();
    publish_fuel_locked();
    unlock();
  }

  AuthConfig ac;
  ac.keys = kAuthorityKeys;
  ac.n_keys = kAuthorityKeyCount;
  ac.service_keys = kServiceKeys;                         // the /debug page: Procom service key (+ lab on debug builds)
  ac.n_service_keys = kServiceKeyCount;
  std::memcpy(ac.mac, g_info.mac, 6);
  ac.fw_version = uint16_t(kFwVersionU16);
  ac.calibrated = false;                                  // site calibration arrives with fw 2.x (v7.0 tracker)
  // Every boot: on the C3 a brownout is a chip reset (core 2.0.14 sets RTC_CNTL_BROWN_OUT_RST_SEL) that reads as
  // ESP_RST_POWERON, so a brownout loop with an attacker would otherwise rewrite the stored code at every boot.
  ac.warm_boot = true;
  g_auth.begin(&store(), ac, app_now_ms());

  fill_begin(g_info.mac);                                 // fill reports: after the reset at boot, before the HTTP routes
  net_begin(g_info.ssid, g_info.boot.trial);             // radio on before any challenge/token is generated
  health_wdt_feed();

  if (g_info.health.safe_mode) {
    FS_LOGW("boot", "safe mode after %u quick abnormal resets: radar off, output = stored level",
            unsigned(g_info.health.quick_abnormal_resets));
    log_event(Ev::SafeMode, uint16_t(g_info.health.quick_abnormal_resets));
  } else if (radar_source().begin()) {
    g_info.radar_started = true;
    xTaskCreate(radar_task, "radar", 4096, nullptr, 5, &g_radar_task);
    xTaskCreate(tracker_task, "tracker", 12288, nullptr, 4, &g_tracker_task);
  } else {
    FS_LOGE("radar", "radar source did not start");
    log_event(Ev::RadarLost, 1);
  }
  if (board::kDebug) rawstream_begin(g_info);

  g_trial = g_info.boot.trial;
  if (g_trial) {
    // The radar must work on trial if it worked under the previous firmware: ours (fs_sys/radar_ok) or v6.3 (its
    // namespaces exist: the first install from v6.3 must not keep a firmware whose radar path is broken).
    g_radar_required = (store().radar_ok_before() || v63_seen) && !g_info.health.safe_mode;
    g_trial_deadline_ms = app_now_ms() + timeouts::kTrialSelfTestMs;
    g_trial_frames_at_start = 0;
    g_trial_estimates_at_start = 0;
  }
}

void app_loop() {
  health_wdt_feed();
  uint64_t now = app_now_ms();
  uint16_t cause = 0;
  if (health_restart_due(now, &cause)) restart_now(cause);
  if (now - g_last_tick_ms >= 1000) {
    g_last_tick_ms = now;
    rtc_hb_ms = now;                                      // heartbeat: a warm reset knows how old the carried slow account is
    rtc_hb_check = rtc_hb_check_of(now);
    supervise(now);
    persist_level(now, false);
    persist_fuel(now, false);
    fill_tick(now);                                       // fill reports: samples, settling, automatic close
    if (!g_counters_flushed && now - g_boot_ms >= uint64_t(timeouts::kCountersFlushS) * 1000) {
      store().flush_counters();                           // a fast reset loop never gets here: no NVS write per boot
      g_counters_flushed = true;
    }
    trial_tick(now);
    health_tick(now - g_boot_ms);
    events_tick();                                        // budget refill in RTC + hourly EVENTS_DROPPED summary
    if (now - g_last_hb_ms >= timeouts::kHeartbeatMs) {
      g_last_hb_ms = now;
      DeviceStatus s;
      app_status(&s);
      FS_LOGI("hb", "up %lus, level %.2f mm (%s, %s), frames %u gaps %u bad %u, est max %u us, heap %u min %u, temp %d C",
              (unsigned long)s.uptime_s, s.out.level_mm, state_text(s.out.state), status_word_text(status_word(s)),
              unsigned(s.frames), unsigned(s.gaps), unsigned(s.bad_packets), unsigned(s.max_est_us), unsigned(internal_free()),
              unsigned(s.heap_min), s.temp_c);
    }
    safe_mode_tick(now);
  }
  net_tick(now);
  delay(20);
}

}  // namespace fs
