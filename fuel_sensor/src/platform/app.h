// Runtime wiring: boot sequence, radar + tracker tasks, status snapshot, level persistence, supervisor, trial boot.
#pragma once
#include <cstdint>
#include "../core/auth.h"
#include "../core/ledger.h"
#include "../core/live.h"
#include "../core/reset.h"
#include "../core/status.h"
#include "health.h"
#include "ota.h"

namespace fs {

struct AppInfo {
  uint8_t mac[6] = {};
  char mac_hex[13] = {};
  char ssid[33] = {};
  HealthInfo health;
  OtaBootInfo boot;
  bool migrated = false;
  bool nvs_ok = false;
  bool events_ok = false;
  bool radar_started = false;
};

void app_setup();
void app_loop();

const AppInfo& app_info();
uint64_t app_now_ms();
void app_status(DeviceStatus* out);                 // consistent snapshot for /api/status
Auth& app_auth();                                   // used only from the HTTP task
bool app_set_tank(const TankConfig& t);             // saves and applies
// Maintenance "Reset to defaults" (core/reset.h kResetItems: tank size, fuel totals, remembered level): writes the journal and
// RESET_TO_DEFAULTS, restarts in about 2 s; the items are reset at that boot (DESIGN.md §9.3).
ResetResult app_request_reset();
bool app_trial_active();
uint32_t app_trial_remaining_s();
void app_save_level_now();                          // before a controlled restart
void app_coredump_erased();                         // after /api/maint/coredump/erase succeeded
// Debug builds: the last estimate's internals for /api/debug.
bool app_last_estimate(Estimate* e, uint32_t* stack_free_tracker, uint32_t* stack_free_radar);

}  // namespace fs
