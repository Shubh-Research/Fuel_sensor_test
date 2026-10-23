// SoftAP and HTTP API (DESIGN.md §8) on ESP-IDF's esp_http_server (inside the unchanged Arduino core): handlers run
// in their own task, bodies stream (firmware uploads never sit in RAM), sockets have receive/send timeouts.
#pragma once
#include <cstdint>

namespace fs {

void net_begin(const char* ssid, bool trial);   // SoftAP + HTTP + keep-alive; trial: loopback-only until self-test
void net_tick(uint64_t now_ms);         // from loop()
bool net_self_check();                  // trial boot: GET /api/ping over loopback

}  // namespace fs
