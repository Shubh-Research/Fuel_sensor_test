// Flash persistence (DESIGN.md §9.1). NVS keeps each value with a CRC and writes the new copy before erasing the old,
// so a power cut during a write keeps the previous value. The level is one 8-byte entry at most once a minute. NVS does
// no static wear levelling: the rarely written pages never move and the hot entries ping-pong between TWO sectors,
// ~6 erases per sector a day, ~33k in 15 years against a 100k rating (simulated with the IDF NVS code, rev6 review).
#include "store.h"

#include <Arduino.h>
#include <Preferences.h>
#include <cmath>
#include <cstring>
#include <esp_attr.h>
#include <esp_rom_crc.h>
#include <esp_system.h>
#include <cstddef>
#include "../core/calib_data.h"
#include "events.h"

namespace fs {

namespace {

constexpr const char* kNsLevel = "fs_lvl";
constexpr const char* kNsCfg = "fs_cfg";
constexpr const char* kNsAuth = "fs_auth";
constexpr const char* kNsSys = "fs_sys";
constexpr const char* kNsFill = "fs_fill";
constexpr uint16_t kLevelCheck = 0x5A5A;

uint32_t le32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
void put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i)); }

bool valid_dim(double v) { return std::isfinite(v) && v >= 10.0 && v <= 10000.0; }

constexpr const char* kTankMirror = "tank.bin";          // LittleFS: the 16-byte NVS blob + CRC-32

void encode_tank(const TankConfig& t, uint8_t b[16]) {
  std::memset(b, 0, 16);
  b[0] = 1;
  b[1] = uint8_t(t.custom ? 1 : 0);
  put32(b + 4, uint32_t(std::lround(t.l_mm * 10)));
  put32(b + 8, uint32_t(std::lround(t.w_mm * 10)));
  put32(b + 12, uint32_t(std::lround(t.h_mm * 10)));
}

bool decode_tank(const uint8_t b[16], TankConfig* t) {
  if (b[0] != 1) return false;
  t->custom = b[1] != 0;
  t->l_mm = le32(b + 4) / 10.0;
  t->w_mm = le32(b + 8) / 10.0;
  t->h_mm = le32(b + 12) / 10.0;
  return valid_dim(t->l_mm) && valid_dim(t->w_mm) && valid_dim(t->h_mm);
}

bool write_tank_mirror(const uint8_t b[16]) {
  uint8_t m[20];
  std::memcpy(m, b, 16);
  put32(m + 16, esp_rom_crc32_le(0, m, 16));
  return fs_file_write(kTankMirror, m, sizeof m);
}

Store g_store;

// Reset to defaults (core/reset.h): the journal is one NVS entry, so writing it is the commit point of a reset. (The
// u16 "rst" of the unreleased pre-ledger 1.1.0 builds is only removed.)
class NvsResetJournal final : public ResetJournal {
 public:
  bool read(uint32_t* rec) override {
    Preferences c;
    if (!c.begin(kNsCfg, true)) return false;          // no fs_cfg namespace yet: nothing pending
    bool have = c.isKey("rsj");
    if (have) *rec = c.getUInt("rsj", 0);
    c.end();
    return have;
  }
  bool write(uint32_t rec) override {
    Preferences c;
    if (!c.begin(kNsCfg, false)) return false;
    bool ok = c.putUInt("rsj", rec) == 4;
    c.end();
    return ok;
  }
  bool clear() override {
    Preferences c;
    if (!c.begin(kNsCfg, false)) return false;
    bool ok = !c.isKey("rsj") || c.remove("rsj");
    if (c.isKey("rst")) c.remove("rst");
    c.end();
    return ok;
  }
};
NvsResetJournal g_reset_journal;

// Boot counters not yet in NVS (warm resets since the last flush). RTC memory is random after a power-on: check word.
struct RtcCounters {
  uint32_t magic, boots, power, crashes, watchdogs, brownouts, check;
};
RTC_NOINIT_ATTR RtcCounters rtc_cnt;
constexpr uint32_t kRtcCntMagic = 0x434E5431;              // "CNT1"
uint32_t cnt_check(const RtcCounters& c) {
  uint32_t h = 0x811C9DC5u;
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&c);
  for (size_t i = 0; i < offsetof(RtcCounters, check); ++i) h = (h ^ p[i]) * 16777619u;
  return h;
}
void cnt_clear() {
  rtc_cnt = RtcCounters{kRtcCntMagic, 0, 0, 0, 0, 0, 0};
  rtc_cnt.check = cnt_check(rtc_cnt);
}

}  // namespace

Store& store() { return g_store; }
ResetJournal& store_reset_journal() { return g_reset_journal; }

bool Store::begin() {
  Preferences p;
  ok_ = p.begin(kNsSys, false);
  if (!ok_) return false;
  fresh_ = !p.isKey("boots");
  sys_.boots = p.getUInt("boots", 0);
  sys_.power = p.getUInt("pwr", 0);
  sys_.crashes = p.getUInt("crash", 0);
  sys_.watchdogs = p.getUInt("wdt", 0);
  sys_.brownouts = p.getUInt("brown", 0);
  chg_ = p.getUInt("chg", 0);
  p.end();
  Preferences c;
  if (c.begin(kNsCfg, true)) {
    uint8_t b[16];
    TankConfig t;
    if (c.getBytesLength("tank") == sizeof b && c.getBytes("tank", b, sizeof b) == sizeof b && decode_tank(b, &t)) {
      tank_ = t;
      has_tank_ = true;
    }
    c.end();
  }
  return true;
}

bool Store::load_level(double* mm) {
  *mm = NAN;
  if (!ok_) return false;
  Preferences p;
  if (!p.begin(kNsLevel, true)) return false;
  uint64_t rec = p.getULong64("rec", 0);
  p.end();
  if (uint16_t(rec >> 48) != kLevelCheck || uint16_t(rec >> 32) != cal::kCalibrationId) return false;   // other calibration
  int32_t um = int32_t(uint32_t(rec));
  double v = um / 1000.0;
  if (!(v > -5.0 && v < 2000.0)) return false;
  *mm = v;
  return true;
}

bool Store::save_level(double mm) {
  if (!ok_ || !std::isfinite(mm)) return false;
  int32_t um = int32_t(std::lround(mm * 1000.0));
  uint64_t rec = uint64_t(kLevelCheck) << 48 | uint64_t(cal::kCalibrationId) << 32 | uint32_t(um);
  Preferences p;
  if (!p.begin(kNsLevel, false)) return false;
  bool ok = p.putULong64("rec", rec) == sizeof rec;
  p.end();
  return ok;
}

bool Store::erase_level() {
  if (!ok_) return false;
  Preferences p;
  if (!p.begin(kNsLevel, false)) return false;
  bool ok = !p.isKey("rec") || p.remove("rec");
  p.end();
  return ok;
}

bool Store::load_ledger(LedgerRecord* r, uint8_t* tokens) {
  *r = LedgerRecord{};
  if (!ok_) return false;
  Preferences p;
  if (!p.begin(kNsLevel, true)) return false;
  const size_t n = p.getBytesLength("fuel");
  uint8_t b[kLedgerBlobMax];
  led_foreign_ = n > size_t(kLedgerBlobMax);
  bool ok = n >= size_t(kLedgerBlobBytes) && n <= size_t(kLedgerBlobMax) && p.getBytes("fuel", b, n) == n &&
            ledger_decode(b, n, r, tokens);
  p.end();
  if (ok) adopt_ledger_blob(b, n);
  return ok;
}

void Store::adopt_ledger_blob(const uint8_t* blob, size_t len) {
  led_ver_ = blob[0] > kLedgerVersion ? blob[0] : kLedgerVersion;
  led_tail_n_ = len > size_t(kLedgerBlobBytes) ? len - kLedgerBlobBytes : 0;
  if (led_tail_n_ > sizeof led_tail_) led_tail_n_ = sizeof led_tail_;
  if (led_ver_ == kLedgerVersion) led_tail_n_ = 0;
  std::memcpy(led_tail_, blob + kLedgerBlobBytes, led_tail_n_);
}

size_t Store::ledger_blob(const LedgerRecord& r, uint8_t tokens, uint8_t out[kLedgerBlobMax]) const {
  ledger_encode(r, tokens, out);
  if (led_ver_ == kLedgerVersion) return kLedgerBlobBytes;
  out[0] = led_ver_;                              // a newer record: the v1 prefix updated, its appended fields kept
  std::memcpy(out + kLedgerBlobBytes, led_tail_, led_tail_n_);
  return kLedgerBlobBytes + led_tail_n_;
}

bool Store::save_ledger(const LedgerRecord& r, uint8_t tokens) {
  if (!ok_ || led_foreign_) return false;
  uint8_t b[kLedgerBlobMax];
  const size_t n = ledger_blob(r, tokens, b);
  Preferences p;
  if (!p.begin(kNsLevel, false)) return false;
  bool ok = p.putBytes("fuel", b, n) == n;        // one blob: a power cut keeps the old record or the new one
  p.end();
  return ok;
}

bool Store::set_tank(const TankConfig& t) {
  if (!ok_) return false;
  if (t.custom && !(valid_dim(t.l_mm) && valid_dim(t.w_mm) && valid_dim(t.h_mm))) return false;
  if (!t.custom && !valid_dim(t.h_mm)) return false;
  uint8_t b[16];
  encode_tank(t, b);
  Preferences c;
  if (!c.begin(kNsCfg, false)) return false;
  bool ok = c.putBytes("tank", b, sizeof b) == sizeof b;
  c.end();
  if (ok) {
    tank_ = t;
    has_tank_ = true;
    write_tank_mirror(b);                                 // best effort; sync_tank_mirror() repairs it at boot
  }
  return ok;
}

bool Store::sync_tank_mirror() {
  uint8_t m[20], cur[16];
  TankConfig t;
  bool have = fs_file_read(kTankMirror, m, sizeof m) && le32(m + 16) == esp_rom_crc32_le(0, m, 16) && decode_tank(m, &t);
  if (!has_tank_) {
    encode_tank(TankConfig{}, cur);
    if (!have || std::memcmp(m, cur, 16) == 0) return false;   // a new device (default size): nothing to restore
    tank_ = t;                                            // used from now on, also if NVS cannot take it
    if (!ok_ || !set_tank(t)) return false;               // the mirror stays; the next boot tries again
    AuthPersist a;                                        // a wipe also reset the failed-attempt count
    if (fresh_ && load(&a) && a.fails < Auth::kMaxFailsBeforeLock) {
      a.fails = uint8_t(Auth::kMaxFailsBeforeLock);
      save(a);
    }
    return true;
  }
  encode_tank(tank_, cur);
  if (!have || std::memcmp(m, cur, 16) != 0) write_tank_mirror(cur);
  return false;
}

void Store::count_boot(int reason) {
  if (!ok_) return;
  bool cold = esp_reset_reason_t(reason) == ESP_RST_POWERON;
  if (cold || rtc_cnt.magic != kRtcCntMagic || rtc_cnt.check != cnt_check(rtc_cnt)) cnt_clear();
  RtcCounters& r = rtc_cnt;                                // earlier warm boots not yet in NVS
  sys_.boots += r.boots;
  sys_.power += r.power;
  sys_.crashes += r.crashes;
  sys_.watchdogs += r.watchdogs;
  sys_.brownouts += r.brownouts;
  ++sys_.boots, ++r.boots;
  switch (esp_reset_reason_t(reason)) {
    case ESP_RST_POWERON: ++sys_.power, ++r.power; break;
    case ESP_RST_BROWNOUT: ++sys_.power, ++r.power; ++sys_.brownouts, ++r.brownouts; break;
    case ESP_RST_PANIC: ++sys_.crashes, ++r.crashes; break;
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: ++sys_.watchdogs, ++r.watchdogs; break;
    default: break;
  }
  r.check = cnt_check(r);
  // A power-on keeps nothing in RTC memory; a fresh NVS needs fs_sys/boots now (it is the wipe marker).
  if (cold || fresh_) flush_counters();
}

void Store::flush_counters() {
  if (!ok_ || rtc_cnt.boots == 0) return;
  Preferences p;
  if (!p.begin(kNsSys, false)) return;
  bool ok = p.putUInt("boots", sys_.boots) == 4;
  ok = p.putUInt("pwr", sys_.power) == 4 && ok;
  ok = p.putUInt("crash", sys_.crashes) == 4 && ok;
  ok = p.putUInt("wdt", sys_.watchdogs) == 4 && ok;
  ok = p.putUInt("brown", sys_.brownouts) == 4 && ok;
  p.end();
  if (ok) cnt_clear();                                     // else kept in RTC and written by a later flush
}

void Store::raise_boots(uint32_t at_least) {
  if (!ok_ || at_least <= sys_.boots) return;
  rtc_cnt.boots += at_least - sys_.boots;                  // valid here: count_boot() ran (and sealed or cleared it)
  rtc_cnt.check = cnt_check(rtc_cnt);
  sys_.boots = at_least;
}

bool Store::load_fill(FillNv* nv) {
  Preferences p;
  if (!p.begin(kNsFill, true)) return false;              // no namespace yet: no report issued
  uint8_t b[kFillNvBytes];
  const bool ok = p.getBytesLength("rec") == sizeof b && p.getBytes("rec", b, sizeof b) == sizeof b && fill_nv_decode(b, sizeof b, nv);
  p.end();
  return ok;
}

bool Store::save_fill(const FillNv& nv) {
  Preferences p;
  if (!ok_ || !p.begin(kNsFill, false)) return false;
  uint8_t b[kFillNvBytes];
  fill_nv_encode(nv, b);
  const bool ok = p.putBytes("rec", b, sizeof b) == sizeof b;   // (committed when it returns)
  p.end();
  return ok;
}

bool Store::bump_settings_changes() {
  Preferences p;
  if (!ok_ || !p.begin(kNsSys, false)) return false;
  const uint32_t v = p.getUInt("chg", chg_) + 1;
  const bool ok = p.putUInt("chg", v) == sizeof v;
  p.end();
  if (ok) chg_ = v;
  return ok;
}

bool Store::radar_ok_before() const {
  Preferences p;
  if (!p.begin(kNsSys, true)) return false;
  bool v = p.getUChar("radar_ok", 0) != 0;
  p.end();
  return v;
}

void Store::set_radar_ok(bool ok) {
  Preferences p;
  if (!p.begin(kNsSys, false)) return;
  if (p.getUChar("radar_ok", 0) != (ok ? 1 : 0)) p.putUChar("radar_ok", ok ? 1 : 0);   // write only on change
  p.end();
}

uint32_t Store::fw_ok() const {
  Preferences p;
  if (!p.begin(kNsSys, true)) return 0;
  uint32_t v = p.getUInt("fw_ok", 0);
  p.end();
  return v;
}

void Store::set_fw_ok(uint32_t v) {
  Preferences p;
  if (!p.begin(kNsSys, false)) return;
  if (p.getUInt("fw_ok", 0) != v) p.putUInt("fw_ok", v);
  p.end();
}

uint32_t Store::pending() const {
  Preferences p;
  if (!p.begin(kNsSys, true)) return 0;
  uint32_t v = p.getUInt("pending", 0);
  p.end();
  return v;
}

void Store::set_pending(uint32_t v) {
  Preferences p;
  if (!p.begin(kNsSys, false)) return;
  if (p.getUInt("pending", 0) != v) p.putUInt("pending", v);
  p.end();
}

uint32_t Store::coredump_noted() const {
  Preferences p;
  if (!p.begin(kNsSys, true)) return 0;
  uint32_t v = p.getUInt("cd_id", 0);
  p.end();
  return v;
}

void Store::set_coredump_noted(uint32_t id) {
  Preferences p;
  if (!p.begin(kNsSys, false)) return;
  if (p.getUInt("cd_id", 0) != id) p.putUInt("cd_id", id);
  p.end();
}

bool Store::migration_done() const {
  Preferences sys;
  if (!sys.begin(kNsSys, true)) return false;
  bool done = sys.getUChar("mig", 0) != 0;
  sys.end();
  return done;
}

// Read-only opens: a namespace that does not exist is not created, and nothing of v6.3 is erased here.
bool Store::import_from_v63(double* level_mm, bool* v63_seen) {
  *level_mm = NAN;
  *v63_seen = false;
  if (!ok_) return false;
  bool imported = false;
  // 1) Tank size: struct Config {u32 schema=1; u32 mode (0 original, 1 dimensions); double length, width, height;}
  Preferences old;
  if (old.begin("a121-tank", true)) {
    *v63_seen = true;
    uint8_t b[32];
    if (old.getBytesLength("config") == sizeof b && old.getBytes("config", b, sizeof b) == sizeof b && le32(b) == 1) {
      double l, w, h;
      std::memcpy(&l, b + 8, 8);
      std::memcpy(&w, b + 16, 8);
      std::memcpy(&h, b + 24, 8);
      TankConfig t;
      t.custom = le32(b + 4) == 1;
      t.l_mm = valid_dim(l) ? l : 1755;
      t.w_mm = valid_dim(w) ? w : 790;
      t.h_mm = valid_dim(h) ? h : 190;
      if (set_tank(t)) imported = true;
    }
    old.end();
  }
  // 2) Remembered level: struct SavedLevel {u32 schema=1; char calibration[65] (SHA-256 hex); double mm;} = 80 bytes.
  if (old.begin("a121-v63", true)) {
    *v63_seen = true;
    size_t n = old.getBytesLength("level");
    uint8_t b[80];
    if (n == sizeof b && old.getBytes("level", b, n) == n && le32(b) == 1) {
      double lv;
      std::memcpy(&lv, b + 72, 8);
      if (std::memcmp(b + 4, cal::kCalibrationSha256Hex, 64) == 0 && std::isfinite(lv) && lv >= -3 && lv <= 250) {
        *level_mm = lv;
        imported = true;
      }
    }
    old.end();
  }
  // 3) Every v6.3 build stores its update credentials at boot.
  if (old.begin("fuel-update", true)) { *v63_seen = true; old.end(); }
  return imported;
}

bool Store::finish_v63_migration() {
  if (!ok_) return false;
  bool erased = false;
  // The old fleet-wide update key ("fuel-update") is no longer used once this firmware is kept and must not stay
  // in flash. Read-only probe first, so a namespace that never existed is not created.
  for (const char* ns : {"a121-tank", "a121-v63", "fuel-update"}) {
    Preferences old;
    if (!old.begin(ns, true)) continue;
    old.end();
    if (old.begin(ns, false)) { old.clear(); old.end(); erased = true; }
  }
  Preferences sys;
  if (sys.begin(kNsSys, false)) { sys.putUChar("mig", 1); sys.end(); }
  return erased;
}

bool Store::load(AuthPersist* p) {
  *p = AuthPersist{};
  if (!ok_) return false;
  Preferences a;
  if (!a.begin(kNsAuth, true)) return true;              // nothing stored yet
  p->ok_count = a.getUShort("ok", 0);
  p->fails = a.getUChar("fails", 0);
  p->last_min = a.getUInt("last_min", 0);
  p->last_ok = a.getUChar("last_ok", 0) != 0;
  p->have_chal = a.getBytesLength("chal") == kChallengeBytes && a.getBytes("chal", p->chal, kChallengeBytes) == kChallengeBytes;
  a.end();
  return true;
}

bool Store::save(const AuthPersist& p) {
  if (!ok_) return false;
  Preferences a;
  if (!a.begin(kNsAuth, false)) return false;
  bool ok = a.putUShort("ok", p.ok_count) == 2 && a.putUChar("fails", p.fails) == 1 &&
            a.putUInt("last_min", p.last_min) == 4 && a.putUChar("last_ok", p.last_ok ? 1 : 0) == 1;
  if (p.have_chal) ok = ok && a.putBytes("chal", p.chal, kChallengeBytes) == kChallengeBytes;
  else if (a.isKey("chal")) ok = a.remove("chal") && ok;   // a used challenge that stays in flash replays after reboot
  a.end();
  return ok;
}

}  // namespace fs
