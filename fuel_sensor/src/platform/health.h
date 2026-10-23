// Reliability: task watchdog, crash-loop guard (safe mode), reset reasons, trial self-test, coredump access
// (DESIGN.md §10).
#pragma once
#include <cstddef>
#include <cstdint>

namespace fs {

struct HealthInfo {
  int reset_reason = 0;                 // esp_reset_reason_t
  const char* reset_text = "";
  bool safe_mode = false;
  uint32_t quick_abnormal_resets = 0;   // consecutive abnormal resets within kSafeModeWindowS of boot
  bool coredump = false;
  size_t coredump_size = 0;
};

// Very early in setup(): reads the reset reason and the RTC crash-loop counter; decides safe mode.
void health_early(HealthInfo* info);
// Re-initialises the task watchdog (kTaskWdtS, panic) and subscribes the calling task.
void health_wdt_begin();
void health_wdt_subscribe();            // the calling task
void health_wdt_feed();
// Once a minute from loop(): clears the crash-loop counter after kSafeModeClearS of healthy uptime.
void health_tick(uint64_t uptime_ms);
// Leave safe mode on the next boot (maintenance "restart").
void health_clear_crash_loop();
// Supervisor restarts (radar lost, no estimate) are bounded by an RTC counter: at most kSupervisorRestarts in a row,
// re-armed after kSupervisorHealthyMs of estimates. A dead radar or a frozen sensor then no longer reboots the device
// every minute forever (which also cut every maintenance session and update short).
bool health_supervisor_restart_allowed();
void health_supervisor_restart_noted();
void health_supervisor_healthy();
// Coredump in the "coredump" partition (written by the core on a crash).
bool health_coredump_read(size_t offset, uint8_t* out, size_t len);
bool health_coredump_erase();
// Restart after a short delay (lets the HTTP reply go out first). Saves the level via the callback first.
void health_restart_soon(uint16_t cause, uint32_t delay_ms);
bool health_restart_due(uint64_t now_ms, uint16_t* cause);

}  // namespace fs
