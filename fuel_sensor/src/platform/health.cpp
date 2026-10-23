// Reliability (DESIGN.md §10).
#include "health.h"

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_core_dump.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <hal/brownout_hal.h>
#include "timeouts.h"

namespace fs {

namespace {

// Survives software/watchdog/panic resets, not power loss (RTC_NOINIT keeps its value across resets).
constexpr uint32_t kRtcMagic = 0x4D524D32;             // "MRM2" (bump whenever an RTC field is added)
RTC_NOINIT_ATTR uint32_t rtc_magic;
RTC_NOINIT_ATTR uint32_t rtc_quick_resets;
RTC_NOINIT_ATTR uint32_t rtc_boot_ok;                   // set after kSafeModeWindowS of uptime in the previous run
RTC_NOINIT_ATTR uint32_t rtc_sup_restarts;              // supervisor restarts in a row (radar lost / no estimate)

// Written by the HTTP task, read by the loop task: a 64-bit value is two stores on these 32-bit CPUs, so both sides
// take the spinlock (a torn read after 49.7 days of uptime would restart before the reply went out).
portMUX_TYPE g_restart_mux = portMUX_INITIALIZER_UNLOCKED;
uint64_t g_restart_at_ms = 0;
uint16_t g_restart_cause = 0;

const char* reason_text(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "ext";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

// A firmware fault. A brownout is a supply problem: safe mode (radar off) cannot help it, so it neither counts nor
// breaks a row (it is still counted as a power interruption).
bool crashed(esp_reset_reason_t r) {
  return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT;
}

}  // namespace

// The core starts the brown-out detector at its sdkconfig level: 0 = 2.43 V on the ESP32, 7 = 2.51 V on the C3. Both
// are below the 2.7 V minimum of the module's SPI flash, so a slowly sagging supply could program NVS/LittleFS/OTA
// pages out of spec. Same settings as esp_brownout_init() of core 2.0.14 (disassembled), only a higher threshold:
//   ESP32: level 7 = 2.80 V +-0.05 (interrupt -> esp_restart, as the core configured it);
//   C3:    level 5 = 2.76 V (hardware reset, as the core configured it; the C3 numbering is reversed).
// Needs a supply that never dips below ~3.0 V during Wi-Fi TX bursts (check on the board, see DESIGN.md §10).
static void raise_brownout_threshold() {
  brownout_hal_config_t cfg = {};
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  cfg.threshold = 5;
  cfg.reset_enabled = true;
#else
  cfg.threshold = 7;
  cfg.reset_enabled = false;
#endif
  cfg.enabled = true;
  cfg.flash_power_down = true;
  cfg.rf_power_down = true;
  brownout_hal_config(&cfg);
}

// setup() is too late for it: app_main -> initArduino() runs nvs_flash_init() first, which repairs pages after a cut
// and erases the WHOLE NVS partition on NO_FREE_PAGES / NEW_VERSION_FOUND (e.g. after misreads below the flash
// minimum). Static constructors run after the core's esp_brownout_init() (do_core_init) and before app_main on both
// chips (start_cpu0_default of core 2.0.14, disassembled in review round 2), so the threshold is raised from here.
struct EarlyBrownout {
  EarlyBrownout() { raise_brownout_threshold(); }
};
static EarlyBrownout g_early_brownout;

void health_early(HealthInfo* info) {
  esp_reset_reason_t r = esp_reset_reason();
  info->reset_reason = int(r);
  info->reset_text = reason_text(r);
  if (rtc_magic != kRtcMagic || r == ESP_RST_POWERON) {   // cold start: RTC memory content is random
    rtc_magic = kRtcMagic;
    rtc_quick_resets = 0;
    rtc_boot_ok = 0;
    rtc_sup_restarts = 0;
  }
  // A crash before the previous run reached kSafeModeWindowS counts towards safe mode; a later crash starts a new row.
  if (crashed(r)) rtc_quick_resets = rtc_boot_ok ? 0 : rtc_quick_resets + 1;
  else if (r != ESP_RST_BROWNOUT) rtc_quick_resets = 0;
  rtc_boot_ok = 0;
  info->quick_abnormal_resets = rtc_quick_resets;
  info->safe_mode = rtc_quick_resets >= timeouts::kSafeModeResets;
  size_t addr = 0, size = 0;
  info->coredump = esp_core_dump_image_get(&addr, &size) == ESP_OK && size > 0;
  info->coredump_size = info->coredump ? size : 0;
}

void health_wdt_begin() {
  // IDF 4.4: calling init again re-configures timeout and panic (the core initialised it with 5 s).
  esp_task_wdt_init(timeouts::kTaskWdtS, true);
}

void health_wdt_subscribe() { esp_task_wdt_add(nullptr); }
void health_wdt_feed() { esp_task_wdt_reset(); }

void health_tick(uint64_t uptime_ms) {
  if (uptime_ms >= uint64_t(timeouts::kSafeModeWindowS) * 1000) rtc_boot_ok = 1;
  if (uptime_ms >= uint64_t(timeouts::kSafeModeClearS) * 1000) rtc_quick_resets = 0;
}

void health_clear_crash_loop() {
  rtc_quick_resets = 0;
  rtc_boot_ok = 1;
  rtc_sup_restarts = 0;
}

bool health_supervisor_restart_allowed() { return rtc_sup_restarts < timeouts::kSupervisorRestarts; }
void health_supervisor_restart_noted() { ++rtc_sup_restarts; }
void health_supervisor_healthy() { rtc_sup_restarts = 0; }

bool health_coredump_read(size_t offset, uint8_t* out, size_t len) {
  size_t addr = 0, size = 0;
  if (esp_core_dump_image_get(&addr, &size) != ESP_OK || offset + len > size) return false;
  const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
  if (!p) return false;
  return esp_partition_read(p, addr - p->address + offset, out, len) == ESP_OK;
}

bool health_coredump_erase() { return esp_core_dump_image_erase() == ESP_OK; }

void health_restart_soon(uint16_t cause, uint32_t delay_ms) {
  uint64_t at = uint64_t(esp_timer_get_time() / 1000) + delay_ms;
  portENTER_CRITICAL(&g_restart_mux);
  g_restart_cause = cause;
  g_restart_at_ms = at;
  portEXIT_CRITICAL(&g_restart_mux);
}

bool health_restart_due(uint64_t now_ms, uint16_t* cause) {
  portENTER_CRITICAL(&g_restart_mux);
  uint64_t at = g_restart_at_ms;
  uint16_t c = g_restart_cause;
  portEXIT_CRITICAL(&g_restart_mux);
  if (!at || now_ms < at) return false;
  *cause = c;
  return true;
}

}  // namespace fs
