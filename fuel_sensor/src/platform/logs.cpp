// Device log: UART0 always, an in-RAM ring for Wi-Fi readers (DESIGN.md §8 /api/logs). See logs.h.
#include "logs.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <new>
#include "../core/logring.h"
#include "../core/status.h"
#include "timeouts.h"

namespace fs {

namespace {

LogRing g_ring;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t g_reader_until_ms = 0;      // a Wi-Fi reader polled recently: info lines go to the ring too
volatile uint64_t g_debug_until_ms = 0;       // ... and asked for debug lines
vprintf_like_t g_idf_prev = nullptr;

uint64_t now_ms() { return uint64_t(esp_timer_get_time() / 1000); }

// Errors and warnings always; everything during the first kLogBootKeepMs (the boot sequence stays readable); info and
// debug otherwise only while a reader polls.
bool keep(char level, uint64_t now) {
  return level == 'E' || level == 'W' || now < timeouts::kLogBootKeepMs || now < g_reader_until_ms;
}

void to_ring(char level, const char* text, size_t len, uint64_t now) {
  portENTER_CRITICAL(&g_mux);
  g_ring.push(uint32_t(now), level, text, len);
  portEXIT_CRITICAL(&g_mux);
}

// ESP-IDF's log lines ("\033[0;31mE (1234) wifi: ...\033[0m\n"): UART as before, the ring with the colour codes removed.
int idf_vprintf(const char* fmt, va_list ap) {
  va_list cp;
  va_copy(cp, ap);
  int r = g_idf_prev ? g_idf_prev(fmt, ap) : vprintf(fmt, ap);
  char raw[LogRing::kLineMax + 24];
  int n = vsnprintf(raw, sizeof raw, fmt, cp);
  va_end(cp);
  if (n <= 0) return r;
  char text[LogRing::kLineMax + 1];
  size_t k = 0;
  for (int i = 0; raw[i] && k < LogRing::kLineMax; ++i) {
    if (raw[i] == '\033') {                          // skip "ESC [ ... m"
      while (raw[i] && raw[i] != 'm') ++i;
      if (!raw[i]) break;
      continue;
    }
    text[k++] = raw[i];
  }
  text[k] = '\0';
  char level = k ? text[0] : 'I';
  if (level != 'E' && level != 'W' && level != 'I' && level != 'D' && level != 'V') level = 'I';
  if (level == 'V') level = 'D';
  uint64_t now = now_ms();
  if (k && keep(level, now)) to_ring(level, text, k, now);
  return r;
}

}  // namespace

void logs_begin() {
  const size_t bytes = LogRing::kSlots * sizeof(LogRing::Line);
  void* m = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);   // the internal heap on both chips (no PSRAM)
  if (m) {
    portENTER_CRITICAL(&g_mux);
    g_ring.begin(new (m) LogRing::Line[LogRing::kSlots], LogRing::kSlots);
    portEXIT_CRITICAL(&g_mux);
  }
  g_idf_prev = esp_log_set_vprintf(idf_vprintf);
  if (!m) FS_LOGE("log", "no memory for the Wi-Fi log ring (%u B): UART only", unsigned(bytes));
}

bool logs_debug_on() { return now_ms() < g_debug_until_ms; }

void logf(char level, const char* tag, const char* fmt, ...) {
  char msg[LogRing::kLineMax + 1];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(msg, sizeof msg, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  char line[LogRing::kLineMax + 32];
  uint64_t now = now_ms();
  int len = snprintf(line, sizeof line, "%c %s: %s", level, tag, msg);
  if (len < 0) return;
  if (size_t(len) >= sizeof line) len = int(sizeof line) - 1;
  // UART0, always (the USB-cable view), through the ROM printer like the core's own log_x(): not HardwareSerial, whose
  // driver path mixed with ESP-IDF's console writes is suspected of a task-watchdog hang during OTA uploads (1.0.3).
  esp_rom_printf("[%lu.%03lu] %s\n", (unsigned long)(now / 1000), (unsigned long)(now % 1000), line);
  if (keep(level, now)) to_ring(level, line, size_t(len), now);
}

void logs_read_json(JsonOut& j, uint32_t since, bool want_debug, uint32_t max_bytes) {
  uint64_t now = now_ms();
  g_reader_until_ms = now + timeouts::kLogReaderHoldMs;
  if (want_debug) g_debug_until_ms = now + timeouts::kLogReaderHoldMs;
  portENTER_CRITICAL(&g_mux);
  uint32_t first = g_ring.first_seq(), next = g_ring.next_seq();
  portEXIT_CRITICAL(&g_mux);
  uint32_t from = since < first ? first : since;
  if (from > next) from = next;                      // a reader from before a reboot: start at what is held
  j.begin_obj().boolean("ok", true).uinteger("first", first).uinteger("lost", since < first ? first - since : 0)
      .uinteger("uptime_ms", now).boolean("debug", logs_debug_on());
  j.begin_arr("lines");
  uint32_t s = from;
  for (; s < next && j.len() < max_bytes; ++s) {
    LogRing::Line l;
    portENTER_CRITICAL(&g_mux);
    bool ok = g_ring.get(s, &l);
    portEXIT_CRITICAL(&g_mux);
    if (!ok) continue;                               // overwritten meanwhile: counted by the reader via "first"
    char lv[2] = {l.level, '\0'};
    j.begin_obj().uinteger("s", l.seq).uinteger("t", l.t_ms).str("l", lv).str("m", l.text).end_obj();
  }
  j.end_arr().uinteger("next", s).end_obj();
}

}  // namespace fs
