// Raw radar stream (DESIGN.md §7.2), DEBUG builds only. Producer: the tracker task writes complete messages into a
// stream buffer without waiting; consumer: one task sends them to the single connected laptop (port 3333).
#include "rawstream.h"
#if defined(FS_VARIANT_DEBUG)

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/stream_buffer.h>
#include <lwip/sockets.h>
#include <cstdio>
#include <cstring>
#include "../core/calib_data.h"
#include "../core/iq_packet.h"
#include "../core/status.h"
#include "../core/version.h"
#include "app.h"
#include "board.h"
#include "health.h"
#include "store.h"
#include "timeouts.h"
#include "logs.h"

namespace fs {

namespace {

constexpr size_t kBufBytes = 12288;                     // ~16 frames
StaticStreamBuffer_t g_sb_struct;
uint8_t* g_sb_storage = nullptr;                         // internal heap at rawstream_begin(), not static DRAM (review r2 M5)
StreamBufferHandle_t g_sb = nullptr;
volatile bool g_client = false;
volatile uint32_t g_drops = 0;
uint8_t g_state = 0;                                     // rawstream_state()
uint8_t g_msg[16 + 4 * kPoints + 32];                  // producer scratch (tracker task only)
char g_hello_fmt[512];                                   // HELLO with %u placeholders for boots and uptime
char g_hello[560];
const AppInfo* g_info = nullptr;

void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
void put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i)); }

// "MRMS" | type | len(u16 LE) | payload | crc32(type|len|payload)
size_t frame_msg(uint8_t type, const uint8_t* payload, size_t len, uint8_t* out) {
  std::memcpy(out, "MRMS", 4);
  out[4] = type;
  put16(out + 5, uint16_t(len));
  std::memcpy(out + 7, payload, len);
  put32(out + 7 + len, crc32_ieee(out + 4, 3 + len));
  return 7 + len + 4;
}

void enqueue(uint8_t type, const uint8_t* payload, size_t len) {
  if (!g_client || !g_sb) return;
  static uint8_t out[sizeof g_msg + 16];
  size_t n = frame_msg(type, payload, len, out);
  if (xStreamBufferSpacesAvailable(g_sb) < n) { ++g_drops; return; }   // whole messages only
  xStreamBufferSend(g_sb, out, n, 0);
}

// Bounded in total, not only per call: a peer that opens its window a few bytes at a time makes every lwip_send
// return a little after up to kStreamSendMs, so a 1460-byte chunk could take minutes without a watchdog feed.
bool send_all(int s, const uint8_t* p, size_t n) {
  const uint64_t deadline = app_now_ms() + 2000;
  while (n) {
    if (app_now_ms() > deadline) return false;
    int r = lwip_send(s, p, n, 0);
    if (r <= 0) return false;
    p += r;
    n -= size_t(r);
  }
  return true;
}

void stream_task(void*) {
  health_wdt_subscribe();
  int listener = lwip_socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(board::kRawStreamPort);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  int one = 1;
  lwip_setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  if (lwip_bind(listener, reinterpret_cast<struct sockaddr*>(&a), sizeof a) != 0 || lwip_listen(listener, 1) != 0) {
    FS_LOGE("raw", "raw stream: cannot listen on %u", board::kRawStreamPort);
    for (;;) { health_wdt_feed(); delay(1000); }
  }
  struct timeval accept_to = {1, 0};
  lwip_setsockopt(listener, SOL_SOCKET, SO_RCVTIMEO, &accept_to, sizeof accept_to);
  static uint8_t chunk[1460];
  int next = -1;                                        // a client that arrived while another was connected
  for (;;) {
    health_wdt_feed();
    int c = next >= 0 ? next : lwip_accept(listener, nullptr, nullptr);
    next = -1;
    if (c < 0) continue;                                // 1 s accept timeout keeps the watchdog fed
    struct timeval to = {0, int(timeouts::kStreamSendMs) * 1000};
    lwip_setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof to);
    int nodelay = 1;
    lwip_setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof nodelay);
    // Drain (reader side) instead of xStreamBufferReset: a reset from the reader can race a send still in progress in
    // the tracker task on the other core of the WROVER. The producer only writes whole messages, so after the drain
    // the next byte starts a message.
    while (xStreamBufferReceive(g_sb, chunk, sizeof chunk, 0) > 0) {}
    std::snprintf(g_hello, sizeof g_hello, g_hello_fmt, unsigned(store().counters().boots),
                  unsigned(app_now_ms() / 1000));
    uint8_t hello[sizeof g_hello + 16];
    size_t hl = frame_msg(1, reinterpret_cast<const uint8_t*>(g_hello), std::strlen(g_hello), hello);
    bool ok = send_all(c, hello, hl);
    g_client = ok;
    if (ok) FS_LOGI("raw", "raw stream client connected");
    while (ok) {
      health_wdt_feed();
      fd_set rd;                                        // a new laptop connection replaces this one
      FD_ZERO(&rd);
      FD_SET(listener, &rd);
      struct timeval zero = {0, 0};
      if (lwip_select(listener + 1, &rd, nullptr, nullptr, &zero) > 0) {
        next = lwip_accept(listener, nullptr, nullptr);
        if (next >= 0) break;
      }
      size_t n = xStreamBufferReceive(g_sb, chunk, sizeof chunk, pdMS_TO_TICKS(500));
      if (n) ok = send_all(c, chunk, n);
      else {                                            // idle: detect a closed peer
        char probe;
        int r = lwip_recv(c, &probe, 1, MSG_DONTWAIT);
        if (r == 0) ok = false;
      }
    }
    g_client = false;
    FS_LOGI("raw", "raw stream client %s", next >= 0 ? "replaced by a new connection" : "disconnected");
    lwip_close(c);
  }
}

}  // namespace

void rawstream_begin(const AppInfo& info) {
  g_info = &info;
  g_sb_storage = static_cast<uint8_t*>(heap_caps_malloc(kBufBytes + 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!g_sb_storage) {                                     // (no raw stream; the device itself is unaffected: shown in
    log_w("raw stream: no %u bytes of internal RAM", unsigned(kBufBytes + 1));   // /api/status health.raw_stream)
    g_state = 2;
    return;
  }
  g_state = 1;
  g_sb = xStreamBufferCreateStatic(kBufBytes, 1, g_sb_storage, &g_sb_struct);
  std::snprintf(g_hello_fmt, sizeof g_hello_fmt,
                "{\"fw\":\"%s\",\"tracker\":\"%s\",\"calibration\":\"0x%04X\",\"variant\":\"%s\",\"chip\":\"%s\",\"mac\":\"%s\","
                "\"boots\":%%u,\"uptime_s\":%%u,"
                "\"radar\":{\"start_point\":-8,\"num_points\":169,\"step_length\":1,\"hwaas\":183,\"sweeps_per_frame\":1,"
                "\"profile\":1,\"prf\":\"PRF_19_5_MHz\",\"receiver_gain\":10,\"frame_rate_hz\":10,"
                "\"base_step_length_m\":0.002502274001017213},\"tick_hz\":1000}",
                FW_VERSION_STRING, TRACKER_VERSION, unsigned(cal::kCalibrationId), board::kVariantName, board::kChipName, info.mac_hex);
  xTaskCreate(stream_task, "stream", 4096, nullptr, 2, nullptr);
}

void rawstream_frame(const Frame& f) {
  if (!g_client) return;
  uint8_t* p = g_msg;
  put32(p, f.seq);
  put32(p + 4, f.tick_ms);
  put32(p + 8, f.rx_ms);
  put16(p + 12, uint16_t(f.temperature_c));
  put16(p + 14, f.flags);
  put16(p + 16, uint16_t(kPoints));
  for (int i = 0; i < kPoints; ++i) {
    put16(p + 18 + 4 * i, uint16_t(f.iq[i].i));
    put16(p + 20 + 4 * i, uint16_t(f.iq[i].q));
  }
  enqueue(2, g_msg, 18 + 4 * kPoints);
}

void rawstream_estimate(uint32_t seq, const Estimate& e, const Output& o) {
  if (!g_client) return;
  char js[320];
  JsonOut j(js, sizeof js);
  j.begin_obj().uinteger("seq", seq).num("t", e.t_s, 3).num("h", e.h_mm, 4).num("out", o.level_mm, 4)
      .str("state", state_text(e.state)).str("cycle", cycle_text(e.cycle)).str("src", source_text(o.source));
  j.begin_arr("flags");                  // DESIGN.md 7.2 / esp32_client.py KNOWN keys: a list of flag names
  if (e.uncertain) j.str(nullptr, "cycle_uncertain");
  if (e.verifying) j.str(nullptr, "verifying");
  if (e.recheck) j.str(nullptr, "recheck");
  if (e.doubt) j.str(nullptr, "doubt");
  if (e.disturbed) j.str(nullptr, "disturbed");
  j.end_arr().end_obj();
  if (j.ok()) enqueue(3, reinterpret_cast<const uint8_t*>(js), j.len());
}

uint32_t rawstream_drops() { return g_drops; }
uint8_t rawstream_state() { return g_state; }

}  // namespace fs

#endif  // FS_VARIANT_DEBUG
