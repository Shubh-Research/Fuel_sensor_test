// Device log (DESIGN.md §8 /api/logs): every line goes to UART0 as before (a board on its USB cable shows everything),
// and into an in-RAM ring readable over Wi-Fi. Errors and warnings are kept in the ring from boot; info and debug lines
// only while a Wi-Fi reader has polled within kLogReaderHoldMs, so nobody reading costs one flag check per line.
// ESP-IDF's own log output (Wi-Fi, OTA, NVS ...) is captured the same way (esp_log_set_vprintf).
#pragma once
#include <cstdint>

namespace fs {

class JsonOut;

void logs_begin();                                     // setup(), after Serial.begin()
void logf(char level, const char* tag, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
bool logs_debug_on();                                  // a reader asked for debug lines (debug=1) recently
// /api/logs: lines with seq >= since (oldest first) as JSON; marks the reader active (debug lines if want_debug).
void logs_read_json(JsonOut& j, uint32_t since, bool want_debug, uint32_t max_bytes);

}  // namespace fs

#define FS_LOGE(tag, ...) ::fs::logf('E', tag, __VA_ARGS__)
#define FS_LOGW(tag, ...) ::fs::logf('W', tag, __VA_ARGS__)
#define FS_LOGI(tag, ...) ::fs::logf('I', tag, __VA_ARGS__)
#define FS_LOGD(tag, ...) \
  do {                                  \
    if (::fs::logs_debug_on()) ::fs::logf('D', tag, __VA_ARGS__); \
  } while (0)
