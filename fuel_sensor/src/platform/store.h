// Flash persistence in NVS (DESIGN.md §9.1) + one-time migration from the v6.3 firmware's namespaces.
#pragma once
#include <cstdint>
#include "../core/auth.h"
#include "../core/fillreport.h"
#include "../core/reset.h"
#include "../core/ledger.h"
#include "../core/status.h"

namespace fs {

struct SysCounters {
  uint32_t boots = 0;               // every boot
  uint32_t power = 0;               // power-on + brownout resets ("power interruptions")
  uint32_t crashes = 0;             // panic / exception resets
  uint32_t watchdogs = 0;           // task / interrupt / other watchdog resets
  uint32_t brownouts = 0;
};

class Store final : public AuthStore {
 public:
  bool begin();                                   // opens NVS; false if NVS is unusable (the device still runs)
  // Level (8-byte record, written by the caller's 60 s / 0.05 mm rule).
  bool load_level(double* mm);
  bool save_level(double mm);
  bool erase_level();                              // Reset to defaults: no remembered level (true if none is left)
  // Fuel ledger record (fs_lvl/fuel, DESIGN.md §9.1, §13): written by the loop task's LedgerSaver policy (and the reset).
  // Any version >= 1 is read through its 32-byte v1 prefix (*tokens = byte 1); a newer firmware's record keeps its version
  // and appended fields when this one writes it (an older firmware never shortens or downgrades a newer record), and one
  // longer than kLedgerBlobMax is never written over.
  bool load_ledger(LedgerRecord* r, uint8_t* tokens);
  bool save_ledger(const LedgerRecord& r, uint8_t tokens);
  size_t ledger_blob(const LedgerRecord& r, uint8_t tokens, uint8_t out[kLedgerBlobMax]) const;   // the bytes save_ledger writes
  void adopt_ledger_blob(const uint8_t* blob, size_t len);         // a record restored from /fuel.bin: its version and tail
  // Tank settings.
  TankConfig tank() const { return tank_; }
  bool set_tank(const TankConfig& t);              // NVS, then the LittleFS mirror (tank.bin)
  // Once LittleFS is mounted: if NVS holds no valid tank size but the mirror does, NVS was erased by the core
  // (nvs_flash_init NO_FREE_PAGES / NEW_VERSION_FOUND) or lost the page: use the mirror's size, write it back to NVS
  // and return true if that worked (a fresh NVS also starts locked out: only valid responses pass for one window).
  // The mirror is refreshed from NVS only when NVS holds a size, so a failed or interrupted restore never replaces
  // the only copy with the default.
  bool sync_tank_mirror();
  // Counters. count_boot() adds this boot in RAM and in RTC memory; NVS gets them from flush_counters() once a run has
  // lasted timeouts::kCountersFlushS, and at once on a power-on or on a fresh (new or wiped) NVS. Warm resets (crash,
  // watchdog, software restart, ESP32 brownout) carry the unwritten increments in RTC memory, so a fast reset loop
  // costs no NVS write per boot (a brownout loop every 2 s would otherwise use the 15-year NVS budget in ~4 months).
  void count_boot(int reset_reason);
  void flush_counters();
  // Boot numbers must stay unique and increasing (the event log and the dashboard key on them), but boots not yet in
  // NVS are lost with RTC memory at a power cut: raise the count to the event log's floor (newest record's boot + 1).
  // RAM and RTC only; the next flush_counters() writes it.
  void raise_boots(uint32_t at_least);
  const SysCounters& counters() const { return sys_; }
  // Flags.
  bool radar_ok_before() const;                   // the radar delivered frames during the previous firmware's run
  void set_radar_ok(bool ok);
  uint32_t fw_ok() const;                         // last firmware version that passed its trial boot
  void set_fw_ok(uint32_t v);
  uint32_t pending() const;                       // version just installed by an update (0 = none)
  void set_pending(uint32_t v);
  uint32_t coredump_noted() const;                // id of the crash dump COREDUMP_FOUND last reported (0 = none)
  void set_coredump_noted(uint32_t id);
  // v6.3 migration in two steps, so a rollback to v6.3 still finds its tank size, level and update key:
  // import (every boot until finished; erases nothing) copies the tank size and returns the old level (*level_mm,
  // else NaN) and whether the v6.3 firmware ran on this device (*v63_seen); finish erases the old namespaces and sets
  // fs_sys/mig, and runs only once this firmware is kept (trial passed, or no trial).
  bool migration_done() const;
  bool import_from_v63(double* level_mm, bool* v63_seen);
  bool finish_v63_migration();                    // true if any v6.3 namespace was erased
  // Fill reports (DESIGN.md §14): the counters and the open report in ONE blob (fs_fill/rec), so a report's serial and
  // its close are one atomic write. Preferences commits every put.
  bool load_fill(FillNv* nv);
  bool save_fill(const FillNv& nv);
  // Settings-changed counter (fs_sys/chg, the future RS-485 register): +1 per tank change and per reset to defaults;
  // never reset.
  uint32_t settings_changes() const { return chg_; }
  bool bump_settings_changes();
  // AuthStore
  bool load(AuthPersist* p) override;
  bool save(const AuthPersist& p) override;

 private:
  bool ok_ = false;
  bool fresh_ = false;                            // fs_sys/boots did not exist at begin(): NVS new or wiped
  bool has_tank_ = false;                         // NVS holds a valid fs_cfg/tank
  TankConfig tank_;
  SysCounters sys_;
  volatile uint32_t chg_ = 0;
  uint8_t led_ver_ = kLedgerVersion;              // the stored ledger record's version and appended bytes (kept on writes)
  uint8_t led_tail_[kLedgerBlobMax - kLedgerBlobBytes] = {};
  size_t led_tail_n_ = 0;
  bool led_foreign_ = false;                      // a record longer than kLedgerBlobMax: never written over
};

Store& store();
// The reset-to-defaults journal (core/reset.h, DESIGN.md §9.3): NVS fs_cfg/rsj, one u32 entry (atomic in NVS): the items
// and the fuel totals' target generation.
ResetJournal& store_reset_journal();

}  // namespace fs
