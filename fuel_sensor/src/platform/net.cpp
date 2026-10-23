// SoftAP + HTTP API (DESIGN.md §8). All handlers run in the single esp_http_server task, so the static reply buffers
// below are never used concurrently.
#include "net.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <lwip/etharp.h>
#include <lwip/netif.h>
#include <lwip/sockets.h>
#include <lwip/tcpip.h>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "../core/access.h"
#include "../core/challenge.h"
#include "../core/eventlog.h"
#include "../core/reset.h"
#include "../core/status.h"
#include "../core/version.h"
#include "app.h"
#include "board.h"
#include "events.h"
#include "fill.h"
#include "health.h"
#include "keys_gen.h"
#include "logs.h"
#include "ota.h"
#include "store.h"
#include "timeouts.h"
#include "web_assets.h"
#include "web_service.h"

namespace fs {

namespace {

httpd_handle_t g_server = nullptr;
char g_reply[6144];                     // JSON replies (largest: /api/service/events with 50 events)
char g_body[1024];                      // JSON request bodies (everything except the firmware upload)
uint8_t g_chunk[4096];                  // firmware upload / coredump chunks
esp_timer_handle_t g_arp_timer = nullptr;
volatile bool g_ping_ok = false;
bool g_ap_ok = false;                   // WiFi.softAP() succeeded (trial self-test)
volatile bool g_loopback_only = false;  // trial boot: refuse Wi-Fi clients until the self-test has passed
uint64_t g_last_lockout_log_ms = 0;
uint64_t g_verify_free_at_ms = 0;       // h_response: earliest start of the next signature check (pacing)

// ---------- small helpers ----------

uint32_t client_ip(httpd_req_t* req) {
  int fd = httpd_req_to_sockfd(req);
  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  if (getpeername(fd, reinterpret_cast<struct sockaddr*>(&addr), &len) != 0) return 0;
  if (addr.ss_family == AF_INET) return reinterpret_cast<struct sockaddr_in*>(&addr)->sin_addr.s_addr;
#if LWIP_IPV6
  if (addr.ss_family == AF_INET6) {     // IPv4-mapped IPv6 (::ffff:a.b.c.d) only; any other IPv6 peer gets no IP
    const struct sockaddr_in6* a6 = reinterpret_cast<struct sockaddr_in6*>(&addr);
    if (a6->sin6_addr.un.u32_addr[0] != 0 || a6->sin6_addr.un.u32_addr[1] != 0 ||
        a6->sin6_addr.un.u32_addr[2] != PP_HTONL(0x0000FFFFUL))
      return 0;
    uint32_t v4;
    std::memcpy(&v4, &a6->sin6_addr.un.u32_addr[3], 4);
    return v4;
  }
#endif
  return 0;
}

esp_err_t send_json(httpd_req_t* req, const char* status, const char* json, size_t len) {
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, ssize_t(len));
}

esp_err_t send_error(httpd_req_t* req, const char* status, const char* message, const char* code = "error") {
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj().boolean("ok", false).str("error", code).str("message", message).end_obj();
  return send_json(req, status, j.c_str(), j.len());
}

// Machine codes the dashboard maps to its own (translatable) text.
const char* login_code(AuthResult r) {
  switch (r) {
    case AuthResult::BadFormat: return "format";
    case AuthResult::LockedOut: return "locked";
    case AuthResult::NoChallenge: return "no_challenge";
    case AuthResult::StoreError: return "store";
    default: return "wrong";
  }
}

// Upload error code and the OTA_REJECTED event arg (1..9), as the dashboard reads them.
const char* pkg_code(PkgError e, uint16_t* arg) {
  switch (e) {
    case PkgError::Chip: *arg = 2; return "chip";
    case PkgError::Variant: *arg = 3; return "variant";
    case PkgError::Downgrade: *arg = 4; return "older";
    case PkgError::TooBig: *arg = 5; return "big";
    case PkgError::Signature: *arg = 6; return "signature";
    case PkgError::ImageHeader: *arg = 7; return "image";
    case PkgError::Hash: *arg = 8; return "hash";
    case PkgError::SinkBegin: case PkgError::SinkWrite: case PkgError::SinkEnd: *arg = 9; return "write";
    default: *arg = 1; return "format";
  }
}

// Reads a small JSON body into g_body (NUL-terminated). Refuses bodies over 1 KB.
bool read_body(httpd_req_t* req) {
  size_t n = req->content_len;
  if (n >= sizeof g_body) return false;
  size_t got = 0;
  int retries = 0;
  const uint64_t deadline = app_now_ms() + 3000;   // a client trickling bytes must not hold the only HTTP task
  while (got < n) {
    if (app_now_ms() > deadline) return false;
    int r = httpd_req_recv(req, g_body + got, n - got);
    if (r == HTTPD_SOCK_ERR_TIMEOUT && ++retries < 3) continue;
    if (r <= 0) return false;
    got += size_t(r);
  }
  g_body[got] = '\0';
  return true;
}

// Minimal flat-JSON readers for our own request bodies ({"key":value,...}).
const char* json_find(const char* body, const char* key) {
  char pat[40];
  std::snprintf(pat, sizeof pat, "\"%s\"", key);
  const char* p = std::strstr(body, pat);
  if (!p) return nullptr;
  p += std::strlen(pat);
  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
  if (*p != ':') return nullptr;
  ++p;
  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
  return p;
}

bool json_str(const char* body, const char* key, char* out, size_t cap) {
  const char* p = json_find(body, key);
  if (!p || *p != '"') return false;
  ++p;
  size_t k = 0;
  while (*p && *p != '"') {
    if (*p == '\\') return false;      // our values never need escapes
    if (k + 1 >= cap) return false;
    out[k++] = *p++;
  }
  if (*p != '"') return false;
  out[k] = '\0';
  return true;
}

bool json_num(const char* body, const char* key, double* out) {
  const char* p = json_find(body, key);
  if (!p) return false;
  char* end = nullptr;
  double v = std::strtod(p, &end);
  if (end == p || !std::isfinite(v)) return false;
  *out = v;
  return true;
}

bool json_bool(const char* body, const char* key, bool* out) {
  const char* p = json_find(body, key);
  if (!p) return false;
  if (std::strncmp(p, "true", 4) == 0) { *out = true; return true; }
  if (std::strncmp(p, "false", 5) == 0) { *out = false; return true; }
  return false;
}

uint32_t phone_time(const char* body) {
  double t = 0;
  return json_num(body, "t", &t) && t > 0 && t < 4.2e9 ? uint32_t(t) : 0;
}

// Session check. On failure the reply has been sent and false is returned.
bool require_session(httpd_req_t* req, Grant* grant) {
  char token[48] = {};
  if (httpd_req_get_hdr_value_str(req, "X-Session", token, sizeof token) != ESP_OK ||
      !app_auth().check(token, client_ip(req), app_now_ms(), grant)) {
    send_error(req, "401 Unauthorized", "Session ended. Unlock again.");
    return false;
  }
  return true;
}

const char* refusal_text(Need n) {
  switch (n) {
    case Need::Settings: return "This approval does not allow changing settings.";
    case Need::Firmware: return "This approval does not allow firmware updates.";
    case Need::Maintenance: return "This needs a maintenance approval from IT.";
    case Need::Service: return "This needs a service approval (Procom service key).";
    default: return "Session ended. Unlock again.";
  }
}

// The access rule of core/access.h for route r: a session whose grant meets the route's need (none for public routes).
// On failure the reply (401 no session, 403 grant) has been sent and false is returned.
bool require(httpd_req_t* req, Route r, Grant* grant = nullptr) {
  Need n = route_need(r, board::kDebug);
  Grant g;
  if (n != Need::Public) {
    if (!require_session(req, &g)) return false;
    if (!grant_allows(g, n)) {
      send_error(req, "403 Forbidden", refusal_text(n), "grant");
      return false;
    }
  }
  if (grant) *grant = g;
  return true;
}

void version_text(uint32_t v, char* out, size_t cap) {
  std::snprintf(out, cap, "%u.%u.%u", unsigned(v >> 16 & 255), unsigned(v >> 8 & 255), unsigned(v & 255));
}

// ---------- public routes ----------

esp_err_t h_index(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, reinterpret_cast<const char*>(kIndexHtmlGz), ssize_t(kIndexHtmlGzLen));
}

// The service page (board::kServicePath, web/service.html): static, with its own login; every byte of service data
// behind it (/api/service/*, production /api/logs) needs a service session.
esp_err_t h_service_page(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_set_hdr(req, "X-Robots-Tag", "noindex");
  return httpd_resp_send(req, reinterpret_cast<const char*>(kServiceHtmlGz), ssize_t(kServiceHtmlGzLen));
}

DeviceStatus g_status;                  // httpd task only

// GET /api/status: production builds the customer fields only, debug builds all of them (core/status.h StatusView).
esp_err_t h_status(httpd_req_t* req) {
  app_status(&g_status);
  size_t len = 0;
  if (!write_status_json(g_status, g_reply, sizeof g_reply, &len, status_view(Route::Status, board::kDebug)))
    return send_error(req, "500 Internal Server Error", "status");
  return send_json(req, "200 OK", g_reply, len);
}

esp_err_t h_ping(httpd_req_t* req) {
  g_ping_ok = true;
  return send_json(req, "200 OK", "{\"ok\":true}", 11);
}

// ---------- authorisation ----------

esp_err_t h_challenge(httpd_req_t* req) {
  if (!read_body(req)) return send_error(req, "400 Bad Request", "Bad request.");
  bool force_new = false;
  json_bool(g_body, "new", &force_new);
  // Anyone on the Wi-Fi may ask for a new code; each one is an NVS write. Forced replacements are rate-limited inside Auth (kNewChallengeMinMs, 10 min): bounded NVS writes, no nonce reuse.
  char text[kChallengeTextMax];
  uint32_t lockout = 0;
  if (!app_auth().challenge(phone_time(g_body), force_new, app_now_ms(), text, sizeof text, &lockout))
    return send_error(req, "500 Internal Server Error", "Could not create a code (device memory error).");
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj().boolean("ok", true).str("challenge", text).uinteger("lockout_s", lockout)
      .uinteger("new_wait_s", app_auth().new_challenge_wait_s(app_now_ms())).str("mac", app_info().mac_hex)
      .str("fw", FW_VERSION_STRING).end_obj();
  return send_json(req, "200 OK", j.c_str(), j.len());
}

esp_err_t h_response(httpd_req_t* req) {
  if (!read_body(req)) return send_error(req, "400 Bad Request", "Bad request.");
  char resp[kResponseTextMax + 32];
  if (!json_str(g_body, "response", resp, sizeof resp)) return send_error(req, "400 Bad Request", "Paste the response code.");
  char token[33];
  Grant g;
  uint32_t lockout = 0;
  // Every well-formed response costs one Ed25519 check per authority key (10-50 ms each), also during a lockout (a
  // valid one must pass). This task runs above loopTask (priority 3 vs 1; the C3 has one core) and loopTask feeds the
  // task watchdog, so back-to-back checks are paced: sleep (yield) so they use at most ~25 % of a core. Nothing is
  // refused; a valid response still passes, only later.
  uint64_t now0 = app_now_ms();
  if (now0 < g_verify_free_at_ms) {
    uint64_t wait = g_verify_free_at_ms - now0;
    vTaskDelay(pdMS_TO_TICKS(wait < 500 ? wait : 500));
  }
  uint64_t t0 = app_now_ms();
  AuthResult r = app_auth().respond(resp, phone_time(g_body), client_ip(req), t0, token, &g, &lockout);
  uint64_t spent = app_now_ms() - t0;
  g_verify_free_at_ms = app_now_ms() + 3 * spent;
  JsonOut j(g_reply, sizeof g_reply);
  if (r == AuthResult::Ok) {
    FS_LOGI("auth", "login ok: %s grant 0x%02x, %u logins so far", g.service ? "service" : "maintenance",
            unsigned(g.service ? g.service_byte() : g.byte()), unsigned(app_auth().persist().ok_count));
    log_event(Ev::LoginOk, g.event_arg(), app_auth().persist().ok_count);
    j.begin_obj().boolean("ok", true).str("token", token).str("kind", g.service ? "service" : "maintenance")
        .begin_obj("grant").boolean("settings", g.settings).boolean("firmware", g.firmware)
        .boolean("calibration", g.calibration).boolean("older", g.older).boolean("service", g.service)
        .uinteger("hours", g.hours).end_obj()
        .uinteger("expires_s", app_auth().session_remaining_s(app_now_ms())).end_obj();
    return send_json(req, "200 OK", j.c_str(), j.len());
  }
  if (r == AuthResult::LockedOut) {
    // One event per lockout window: every locked attempt would otherwise rewrite LittleFS blocks (wear, lost radar
    // bytes while the cache is off) and push the real history out of the 512-record ring.
    uint64_t now = app_now_ms();
    if (!g_last_lockout_log_ms || now - g_last_lockout_log_ms >= Auth::kLockoutMs) {
      FS_LOGW("auth", "locked out for %u s after repeated wrong codes", unsigned(lockout));
      log_event(Ev::Lockout, 0, lockout);
      g_last_lockout_log_ms = now;
    }
  }
  else if (r != AuthResult::NoChallenge) {              // no open code: not an attempt (Auth does not count it) and
    FS_LOGW("auth", "login refused (%s), %u wrong in a row", login_code(r), unsigned(app_auth().persist().fails));
    log_event(Ev::LoginFail, uint16_t(r), app_auth().persist().fails);   // not rate-limited, so not logged
  }
  j.begin_obj().boolean("ok", false).str("error", login_code(r)).str("message", auth_result_text(r))
      .uinteger("fails", app_auth().persist().fails).uinteger("lockout_s", lockout).end_obj();
  return send_json(req, r == AuthResult::LockedOut ? "429 Too Many Requests" : "403 Forbidden", j.c_str(), j.len());
}

esp_err_t h_logout(httpd_req_t* req) {
  Grant g;
  if (require_session(req, &g)) app_auth().logout();
  else return ESP_OK;                  // reply already sent
  return send_json(req, "200 OK", "{\"ok\":true}", 11);
}

// ---------- maintenance ----------

// GET /api/maint/state: what the dashboard's Maintenance shows (session, grant, tank, crash report, trial). The device
// counters, memory and the event log are service data (/api/service/status, /api/service/events).
esp_err_t h_state(httpd_req_t* req) {
  Grant g;
  if (!require(req, Route::MaintState, &g)) return ESP_OK;
  const AppInfo& info = app_info();
  TankConfig t = store().tank();
  uint64_t now = app_now_ms();
  char rejected[16] = "";
  if (info.boot.rolled_back) version_text(info.boot.rejected_version, rejected, sizeof rejected);
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj().boolean("ok", true)
      .begin_obj("build").str("fw", FW_VERSION_STRING).str("variant", board::kVariantName).str("chip", board::kChipName)
      .str("mac", info.mac_hex).end_obj()
      .begin_obj("session").uinteger("expires_s", app_auth().session_remaining_s(now))
      .uinteger("idle_s", uint32_t(Auth::kIdleMs / 1000)).end_obj()
      .begin_obj("grant").boolean("settings", g.settings).boolean("firmware", g.firmware).boolean("older", g.older)
      .boolean("calibration", g.calibration).boolean("service", g.service).uinteger("hours", g.hours).end_obj()
      .begin_obj("coredump").boolean("present", info.health.coredump).uinteger("size", info.health.coredump_size).end_obj()
      .begin_obj("tank").boolean("custom", t.custom).num("l_mm", t.l_mm, 1).num("w_mm", t.w_mm, 1).num("h_mm", t.h_mm, 1)
      .num("lpmm", t.lpmm(), 5).end_obj()
      .begin_obj("ota").boolean("trial", app_trial_active()).uinteger("trial_remaining_s", app_trial_remaining_s())
      .boolean("rolled_back", info.boot.rolled_back).str("rejected_version", rejected).end_obj()
      .end_obj();
  if (!j.ok()) return send_error(req, "500 Internal Server Error", "state too large");
  return send_json(req, "200 OK", j.c_str(), j.len());
}

// POST /api/maint/reset {"confirm":"RESET"}: reset to defaults (core/reset.h). Needs the settings grant.
esp_err_t h_reset(httpd_req_t* req) {
  if (!require(req, Route::Reset)) return ESP_OK;
  if (!read_body(req)) return send_error(req, "400 Bad Request", "Bad request.");
  char word[16] = "";
  if (!json_str(g_body, "confirm", word, sizeof word) || !reset_confirm_ok(word))
    return send_error(req, "400 Bad Request", "Confirm the reset.", "confirm");
  // Not while a new firmware is on trial or arriving, nor before the v6.3 migration has finished (its import would put the
  // old tank size and level back at every boot until then).
  if (app_trial_active() || ota_upload().active() || !store().migration_done())
    return send_error(req, "409 Conflict", "Wait until the firmware update has finished, then reset.", "busy");
  ResetResult r = app_request_reset();
  if (r != ResetResult::Ok) {
    FS_LOGE("maint", "reset to defaults: journal not written");
    return send_error(req, "500 Internal Server Error", reset_result_text(r), "store");
  }
  app_auth().logout();                                   // the sensor restarts: the session ends with it
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj().boolean("ok", true).uinteger("items", kResetItems).uinteger("restart_in_s", 2).end_obj();
  return send_json(req, "200 OK", j.c_str(), j.len());
}

// ---------- fill reports (DESIGN.md §14): the operator needs no login; the device's list needs Maintenance ----------

const char* status_line(int code) {
  switch (code) {
    case 200: return "200 OK";
    case 400: return "400 Bad Request";
    case 409: return "409 Conflict";
    case 429: return "429 Too Many Requests";
    case 503: return "503 Service Unavailable";
    default: return "500 Internal Server Error";
  }
}

esp_err_t send_fill(httpd_req_t* req, int code, JsonOut& j) {
  if (!j.ok()) return send_error(req, "500 Internal Server Error", "fill reply too large");
  return send_json(req, status_line(code), j.c_str(), j.len());
}

// GET /api/fill: the open report's stage (never the sensor's litres before Confirm) and the last report.
esp_err_t h_fill_state(httpd_req_t* req) {
  if (!require(req, Route::FillState)) return ESP_OK;
  JsonOut j(g_reply, sizeof g_reply);
  return send_fill(req, fill_http_state(j), j);
}

// POST /api/fill/start {"t": phone unix s, "tz_min": minutes east of UTC (the page's -getTimezoneOffset())}
esp_err_t h_fill_start(httpd_req_t* req) {
  if (!require(req, Route::FillStart)) return ESP_OK;
  if (!read_body(req)) return send_error(req, "400 Bad Request", "Bad request.");
  double tz = 0;
  if (!json_num(g_body, "tz_min", &tz)) tz = 0;
  JsonOut j(g_reply, sizeof g_reply);
  return send_fill(req, fill_http_start(phone_time(g_body), int(std::lround(tz)), j), j);
}

// POST /api/fill/end {"t": phone unix s}
esp_err_t h_fill_end(httpd_req_t* req) {
  if (!require(req, Route::FillEnd)) return ESP_OK;
  if (!read_body(req)) return send_error(req, "400 Bad Request", "Bad request.");
  JsonOut j(g_reply, sizeof g_reply);
  return send_fill(req, fill_http_end(phone_time(g_body), j), j);
}

// POST /api/fill/confirm {"litres": the operator's litres}: signs the report (the serial is committed to NVS first).
esp_err_t h_fill_confirm(httpd_req_t* req) {
  if (!require(req, Route::FillConfirm)) return ESP_OK;
  if (!read_body(req)) return send_error(req, "400 Bad Request", "Bad request.");
  double litres = NAN;
  if (!json_num(g_body, "litres", &litres)) return send_error(req, "400 Bad Request", "Enter the litres.", "bad_input");
  JsonOut j(g_reply, sizeof g_reply);
  return send_fill(req, fill_http_confirm(litres, j), j);
}

// GET /api/fill/log: the device's fill reports, newest first (at most 50), one chunk per report.
esp_err_t h_fill_log(httpd_req_t* req) {
  if (!require(req, Route::FillLog)) return ESP_OK;
  httpd_resp_set_status(req, "200 OK");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  const int n = fill_log_count();
  int len = std::snprintf(g_reply, sizeof g_reply, "{\"ok\":true,\"count\":%d,\"reports\":[", n);
  if (httpd_resp_send_chunk(req, g_reply, len) != ESP_OK) return ESP_FAIL;
  bool first = true;
  for (int i = 0; i < n && i < 50; ++i) {
    uint8_t code[kFillCodeBytes];
    if (!fill_log_get(i, code)) break;
    JsonOut j(g_reply + 1, sizeof g_reply - 1);
    if (!fill_write_json(j, nullptr, code)) continue;
    g_reply[0] = ',';
    if (httpd_resp_send_chunk(req, first ? g_reply + 1 : g_reply, ssize_t(j.len() + (first ? 0 : 1))) != ESP_OK) return ESP_FAIL;
    first = false;
  }
  if (httpd_resp_send_chunk(req, "]}", 2) != ESP_OK) return ESP_FAIL;
  return httpd_resp_send_chunk(req, nullptr, 0);
}

// ---------- service (the /debug page; a service session: Procom service key, lab key on debug builds) ----------

// GET /api/service/status: every status field (debug /api/status) plus the device counters, OTA and memory.
esp_err_t h_service_status(httpd_req_t* req) {
  if (!require(req, Route::ServiceStatus)) return ESP_OK;
  app_status(&g_status);
  const AppInfo& info = app_info();
  const SysCounters& c = store().counters();
  char rejected[16] = "";
  if (info.boot.rolled_back) version_text(info.boot.rejected_version, rejected, sizeof rejected);
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj();
  write_status_fields(j, g_status, status_view(Route::ServiceStatus, board::kDebug));
  j.begin_obj("service")
      .begin_obj("device").str("ssid", info.ssid).boolean("test_authority_key", kAuthorityHasTestKey)
      .boolean("nvs_ok", info.nvs_ok).boolean("events_ok", info.events_ok).boolean("radar_started", info.radar_started)
      .boolean("migrated", info.migrated).end_obj()
      .begin_obj("counters").uinteger("boots", c.boots).uinteger("power_interruptions", c.power)
      .uinteger("crashes", c.crashes).uinteger("watchdogs", c.watchdogs).uinteger("brownouts", c.brownouts)
      .uinteger("logins_ok", app_auth().persist().ok_count).uinteger("login_fails", app_auth().persist().fails).end_obj()
      .begin_obj("ota").boolean("trial", app_trial_active()).uinteger("trial_remaining_s", app_trial_remaining_s())
      .boolean("rolled_back", info.boot.rolled_back).str("rejected_version", rejected)
      .uinteger("slot_bytes", ota_slot_size()).end_obj()
      .begin_obj("memory").uinteger("heap_free", heap_caps_get_free_size(MALLOC_CAP_INTERNAL))
      .uinteger("heap_min", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)).end_obj()
      .begin_obj("coredump").boolean("present", info.health.coredump).uinteger("size", info.health.coredump_size).end_obj()
      .begin_obj("session").uinteger("expires_s", app_auth().session_remaining_s(app_now_ms())).end_obj()
      .end_obj();
  j.end_obj();
  if (!j.ok()) return send_error(req, "500 Internal Server Error", "status too large");
  return send_json(req, "200 OK", j.c_str(), j.len());
}

// GET /api/service/events: the event log, newest first (at most 50).
esp_err_t h_service_events(httpd_req_t* req) {
  if (!require(req, Route::ServiceEvents)) return ESP_OK;
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj().boolean("ok", true).uinteger("count", events_count()).uinteger("boot", events_boot());
  j.begin_arr("events");
  uint16_t n = events_count();
  for (uint16_t i = 0; i < n && i < 50; ++i) {
    EventRecord r;
    if (!events_get(i, &r)) break;
    j.begin_obj().uinteger("boot", r.boot).uinteger("uptime_s", r.uptime_s).uinteger("type", r.code())
        .str("name", event_name(r.code())).uinteger("dropped", r.dropped()).uinteger("arg", r.arg)
        .uinteger("value", r.value).end_obj();
  }
  j.end_arr().end_obj();
  if (!j.ok()) return send_error(req, "500 Internal Server Error", "events too large");
  return send_json(req, "200 OK", j.c_str(), j.len());
}

esp_err_t h_tank(httpd_req_t* req) {
  if (!require(req, Route::Tank)) return ESP_OK;
  if (!read_body(req)) return send_error(req, "400 Bad Request", "Bad request.");
  TankConfig t = store().tank();
  bool use_default = false;
  if (json_bool(g_body, "default", &use_default) && use_default) {
    t = TankConfig{};
  } else {
    double l, w, h;
    if (!json_num(g_body, "l_mm", &l) || !json_num(g_body, "w_mm", &w) || !json_num(g_body, "h_mm", &h) ||
        l < 10 || l > 10000 || w < 10 || w > 10000 || h < 10 || h > 10000)
      return send_error(req, "400 Bad Request", "Each size must be between 10 and 10000 mm.");
    t.custom = true;
    t.l_mm = l;
    t.w_mm = w;
    t.h_mm = h;
  }
  if (!app_set_tank(t)) {
    FS_LOGE("maint", "tank size not saved (NVS)");
    return send_error(req, "500 Internal Server Error", "Could not save to the device memory.");
  }
  FS_LOGI("maint", "tank set to %.0f x %.0f x %.0f mm (%s)", t.l_mm, t.w_mm, t.h_mm, t.custom ? "custom" : "default");
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj().boolean("ok", true).begin_obj("tank").boolean("custom", t.custom).num("l_mm", t.l_mm, 1)
      .num("w_mm", t.w_mm, 1).num("h_mm", t.h_mm, 1).num("lpmm", t.lpmm(), 5).end_obj().end_obj();
  return send_json(req, "200 OK", j.c_str(), j.len());
}

esp_err_t h_update(httpd_req_t* req) {
  Grant g;
  if (!require(req, Route::Update, &g)) return ESP_OK;
  size_t total = req->content_len;
  if (total < kPkgPrefixBytes) return send_error(req, "400 Bad Request", pkg_error_text(PkgError::Short), "format");
  if (app_trial_active())               // esp_ota_begin refuses while this image is PENDING_VERIFY
    return send_error(req, "409 Conflict", "The device is still checking the installed firmware. Try again in 2 minutes.",
                      "busy");
  OtaUpload& up = ota_upload();
  if (!up.start(uint32_t(total), g)) return send_error(req, "409 Conflict", "Another update is in progress.", "busy");
  size_t got = 0;
  int timeouts_in_row = 0;
  PkgError e = PkgError::Ok;
  while (got < total) {
    size_t want = total - got < sizeof g_chunk ? total - got : sizeof g_chunk;
    int r = httpd_req_recv(req, reinterpret_cast<char*>(g_chunk), want);
    if (r == HTTPD_SOCK_ERR_TIMEOUT) {
      if (++timeouts_in_row < 3) continue;
      e = PkgError::Short;
      break;
    }
    if (r <= 0) { e = PkgError::Short; break; }
    timeouts_in_row = 0;
    got += size_t(r);
    e = up.feed(g_chunk, size_t(r));
    if (e != PkgError::Ok) break;
  }
  if (e == PkgError::Ok) e = up.finish();
  else up.abort();
  if (e != PkgError::Ok) {
    uint16_t arg = 1;
    const char* code = pkg_code(e, &arg);
    FS_LOGW("ota", "update refused after %u bytes: %s", unsigned(got), code);
    log_event(Ev::OtaRejected, arg, uint32_t(got));
    return send_error(req, "400 Bad Request", pkg_error_text(e), code);
  }
  const PkgHeader& h = up.header();
  store().set_pending(h.version);
  log_event(Ev::OtaAccepted, h.variant, h.version);
  char v[16];
  version_text(h.version, v, sizeof v);
  FS_LOGI("ota", "update to %s accepted (%u bytes); restarting into its trial", v, unsigned(got));
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj().boolean("ok", true).str("version", v).uinteger("restart_in_s", 2).end_obj();
  esp_err_t res = send_json(req, "200 OK", j.c_str(), j.len());
  app_auth().logout();
  health_restart_soon(2, 2000);        // the reply goes out first; the level is saved before the restart
  return res;
}

esp_err_t h_coredump(httpd_req_t* req) {
  if (!require(req, Route::Coredump)) return ESP_OK;
  const AppInfo& info = app_info();
  if (!info.health.coredump) return send_error(req, "404 Not Found", "No crash dump stored.");
  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"coredump.elf\"");
  for (size_t off = 0; off < info.health.coredump_size; off += sizeof g_chunk) {
    size_t n = info.health.coredump_size - off < sizeof g_chunk ? info.health.coredump_size - off : sizeof g_chunk;
    if (!health_coredump_read(off, g_chunk, n)) return ESP_FAIL;   // close the connection: no silently short file
    if (httpd_resp_send_chunk(req, reinterpret_cast<const char*>(g_chunk), ssize_t(n)) != ESP_OK) return ESP_FAIL;
  }
  return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t h_coredump_erase(httpd_req_t* req) {
  if (!require(req, Route::CoredumpErase)) return ESP_OK;
  bool ok = health_coredump_erase();
  if (ok) app_coredump_erased();
  return ok ? send_json(req, "200 OK", "{\"ok\":true}", 11) : send_error(req, "500 Internal Server Error", "Erase failed.");
}

esp_err_t h_restart(httpd_req_t* req) {
  if (!require(req, Route::Restart)) return ESP_OK;
  FS_LOGI("maint", "restart requested");
  health_clear_crash_loop();           // a maintenance restart also leaves safe mode
  app_auth().logout();
  esp_err_t r = send_json(req, "200 OK", "{\"ok\":true,\"restart_in_s\":2}", 28);
  health_restart_soon(1, 1500);
  return r;
}

// GET /api/logs?since=N[&debug=1]: the device log (logs.h). Debug builds: anyone on the Wi-Fi; production builds: a
// service session. Polling it keeps info (and with debug=1 debug) lines flowing into the ring.
esp_err_t h_logs(httpd_req_t* req) {
  if (!require(req, Route::Logs)) return ESP_OK;
  char q[64] = {}, v[16] = {};
  uint32_t since = 0;
  bool dbg = false;
  if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK) {
    if (httpd_query_key_value(q, "since", v, sizeof v) == ESP_OK) since = uint32_t(std::strtoul(v, nullptr, 10));
    if (httpd_query_key_value(q, "debug", v, sizeof v) == ESP_OK) dbg = v[0] == '1';
  }
  JsonOut j(g_reply, sizeof g_reply);
  logs_read_json(j, since, dbg, sizeof g_reply - 512);   // a line is at most ~300 B of JSON
  if (!j.ok()) return send_error(req, "500 Internal Server Error", "log reply too large");
  return send_json(req, "200 OK", j.c_str(), j.len());
}

#if defined(FS_VARIANT_DEBUG)
esp_err_t h_debug(httpd_req_t* req) {
  Estimate e;
  uint32_t st = 0, sr = 0;
  bool have = app_last_estimate(&e, &st, &sr);
  JsonOut j(g_reply, sizeof g_reply);
  j.begin_obj().boolean("have_estimate", have);
  if (have) {
    j.num("t_s", e.t_s, 2).num("h_mm", e.h_mm, 3).str("state", state_text(e.state)).str("cycle", cycle_text(e.cycle))
        .uinteger("channel", unsigned(e.channel)).boolean("uncertain", e.uncertain).boolean("stable", e.stable)
        .boolean("verifying", e.verifying).boolean("recheck", e.recheck).boolean("doubt", e.doubt)
        .boolean("disturbed", e.disturbed).num("hb", e.hb, 3).num("hs", e.hs, 3).num("b", e.b, 3).num("s", e.s, 3)
        .num("b_env", e.b_env, 3).num("s_env", e.s_env, 3).num("pstd", e.pstd, 2).num("n", e.n, 5).num("c0_s", e.c0_s, 3)
        .num("leak_mm", e.leak_mm, 4).num("gap_mm", e.gap_mm, 3);
  }
  j.uinteger("stack_free_tracker", st).uinteger("stack_free_radar", sr)
      .uinteger("heap_free", heap_caps_get_free_size(MALLOC_CAP_INTERNAL))
      .end_obj();
  return send_json(req, "200 OK", j.c_str(), j.len());
}
#endif

esp_err_t h_not_found(httpd_req_t* req, httpd_err_code_t) {
  return send_error(req, "404 Not Found", "Not found.");
}

void reg(const char* uri, httpd_method_t m, esp_err_t (*fn)(httpd_req_t*)) {
  httpd_uri_t u = {};
  u.uri = uri;
  u.method = m;
  u.handler = fn;
  httpd_register_uri_handler(g_server, &u);
}

// ---------- Android keep-alive: gratuitous ARP from the AP every 500 ms (ESP32-Smart-Devices practice) ----------

void arp_on_tcpip(void*) {
  for (struct netif* n = netif_list; n; n = n->next) {
    if ((n->flags & NETIF_FLAG_UP) && (n->flags & NETIF_FLAG_ETHARP) && ip4_addr_get_u32(netif_ip4_addr(n)) != 0 &&
        ip4_addr1(netif_ip4_addr(n)) == board::kApIp[0] && ip4_addr2(netif_ip4_addr(n)) == board::kApIp[1] &&
        ip4_addr3(netif_ip4_addr(n)) == board::kApIp[2] && ip4_addr4(netif_ip4_addr(n)) == board::kApIp[3])
      etharp_gratuitous(n);
  }
}

void arp_timer(void*) { tcpip_callback_with_block(arp_on_tcpip, nullptr, 0); }

// Session-open hook: while on trial, only the loopback self-test may connect, so no Wi-Fi client can occupy the
// single HTTP task during the 90-s window and force a rollback of a freshly installed firmware.
esp_err_t on_open(httpd_handle_t, int fd) {
  if (!g_loopback_only) return ESP_OK;
  struct sockaddr_storage a;
  socklen_t len = sizeof a;
  if (getpeername(fd, reinterpret_cast<struct sockaddr*>(&a), &len) != 0) return ESP_FAIL;
  // The device itself: 127.0.0.1 or its own AP address (a spoofed source 192.168.4.1 cannot complete a handshake).
  const uint32_t self_ip = PP_HTONL(uint32_t(board::kApIp[0]) << 24 | uint32_t(board::kApIp[1]) << 16 |
                                    uint32_t(board::kApIp[2]) << 8 | board::kApIp[3]);
  if (a.ss_family == AF_INET) {
    uint32_t ip = reinterpret_cast<struct sockaddr_in*>(&a)->sin_addr.s_addr;
    return ip == PP_HTONL(0x7F000001UL) || ip == self_ip ? ESP_OK : ESP_FAIL;
  }
#if LWIP_IPV6
  if (a.ss_family == AF_INET6) {
    const struct sockaddr_in6* a6 = reinterpret_cast<struct sockaddr_in6*>(&a);
    uint32_t ip = a6->sin6_addr.un.u32_addr[3];
    return a6->sin6_addr.un.u32_addr[0] == 0 && a6->sin6_addr.un.u32_addr[1] == 0 &&
                   a6->sin6_addr.un.u32_addr[2] == PP_HTONL(0x0000FFFFUL) &&
                   (ip == PP_HTONL(0x7F000001UL) || ip == self_ip) ? ESP_OK : ESP_FAIL;
  }
#endif
  return ESP_FAIL;
}

}  // namespace

void net_begin(const char* ssid, bool trial) {
  g_loopback_only = trial;
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t info) {
    const uint8_t* m = info.wifi_ap_staconnected.mac;
    FS_LOGI("wifi", "client %02X:%02X:%02X:%02X:%02X:%02X joined", m[0], m[1], m[2], m[3], m[4], m[5]);
  }, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t info) {
    const uint8_t* m = info.wifi_ap_stadisconnected.mac;
    FS_LOGI("wifi", "client %02X:%02X:%02X:%02X:%02X:%02X left", m[0], m[1], m[2], m[3], m[4], m[5]);
  }, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  IPAddress ip(board::kApIp[0], board::kApIp[1], board::kApIp[2], board::kApIp[3]);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  g_ap_ok = WiFi.softAP(ssid, board::kWifiPassword, board::kApChannel, 0, board::kApMaxClients);
  if (!g_ap_ok) FS_LOGE("wifi", "SoftAP failed to start");
  else FS_LOGI("wifi", "SoftAP %s up at %u.%u.%u.%u", ssid, board::kApIp[0], board::kApIp[1], board::kApIp[2], board::kApIp[3]);

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = board::kHttpPort;
  cfg.max_uri_handlers = 26;             // 23 routes on debug builds
  cfg.max_open_sockets = 5;
  cfg.lru_purge_enable = true;
  cfg.recv_wait_timeout = timeouts::kHttpRecvS;
  cfg.send_wait_timeout = timeouts::kHttpSendS;
  cfg.stack_size = 8192;
  cfg.core_id = tskNO_AFFINITY;
  cfg.open_fn = on_open;                 // trial boot: loopback/AP-address connections only until the self-test passes
  cfg.task_priority = 3;                // below radar (5) and tracker (4): an upload is flash-bound on the C3
  if (httpd_start(&g_server, &cfg) != ESP_OK) {
    FS_LOGE("http", "HTTP server failed to start");
    return;
  }
  reg("/", HTTP_GET, h_index);
  reg(board::kServicePath, HTTP_GET, h_service_page);
  reg("/api/status", HTTP_GET, h_status);
  reg("/api/ping", HTTP_GET, h_ping);
  reg("/api/auth/challenge", HTTP_POST, h_challenge);
  reg("/api/auth/response", HTTP_POST, h_response);
  reg("/api/auth/logout", HTTP_POST, h_logout);
  reg("/api/maint/state", HTTP_GET, h_state);
  reg("/api/maint/tank", HTTP_POST, h_tank);
  reg("/api/maint/reset", HTTP_POST, h_reset);
  reg("/api/maint/update", HTTP_POST, h_update);
  reg("/api/maint/coredump", HTTP_GET, h_coredump);
  reg("/api/maint/coredump/erase", HTTP_POST, h_coredump_erase);
  reg("/api/maint/restart", HTTP_POST, h_restart);
  reg("/api/logs", HTTP_GET, h_logs);
  reg("/api/service/status", HTTP_GET, h_service_status);
  reg("/api/service/events", HTTP_GET, h_service_events);
  reg("/api/fill", HTTP_GET, h_fill_state);
  reg("/api/fill/start", HTTP_POST, h_fill_start);
  reg("/api/fill/end", HTTP_POST, h_fill_end);
  reg("/api/fill/confirm", HTTP_POST, h_fill_confirm);
  reg("/api/fill/log", HTTP_GET, h_fill_log);
#if defined(FS_VARIANT_DEBUG)
  reg("/api/debug", HTTP_GET, h_debug);
#endif
  httpd_register_err_handler(g_server, HTTPD_404_NOT_FOUND, h_not_found);

  esp_timer_create_args_t ta = {};
  ta.callback = arp_timer;
  ta.name = "arp";
  if (esp_timer_create(&ta, &g_arp_timer) == ESP_OK) esp_timer_start_periodic(g_arp_timer, 500000);
}

void net_tick(uint64_t) {}

namespace {

// Non-blocking connect with a timeout: a blocking lwIP connect has no timeout (SYN retries for a minute when the
// listen backlog is full) and would trip the loop task's watchdog during the trial.
bool connect_within(int s, const struct sockaddr_in& a, uint32_t timeout_ms) {
  int fl = lwip_fcntl(s, F_GETFL, 0);
  if (fl < 0 || lwip_fcntl(s, F_SETFL, fl | O_NONBLOCK) < 0) return false;
  int r = lwip_connect(s, reinterpret_cast<const struct sockaddr*>(&a), sizeof a);
  if (r != 0) {
    if (errno != EINPROGRESS) return false;
    fd_set wr;
    FD_ZERO(&wr);
    FD_SET(s, &wr);
    struct timeval tv = {long(timeout_ms / 1000), long(timeout_ms % 1000) * 1000};
    if (lwip_select(s + 1, nullptr, &wr, nullptr, &tv) <= 0) return false;
    int err = 0;
    socklen_t len = sizeof err;
    if (lwip_getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) return false;
  }
  return lwip_fcntl(s, F_SETFL, fl) >= 0;
}

}  // namespace

bool net_self_check() {
  // GET /api/ping to the AP's OWN address (looped back inside lwIP): proves the HTTP task is serving AND that the
  // SoftAP interface is up with 192.168.4.1 -- 127.0.0.1 would pass with the radio off, keeping an unreachable image.
  g_ping_ok = false;
  if (!g_ap_ok) return false;
  int s = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) return false;
  struct timeval tv = {2, 0};
  lwip_setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  lwip_setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
  struct sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(board::kHttpPort);
  a.sin_addr.s_addr = htonl(uint32_t(board::kApIp[0]) << 24 | uint32_t(board::kApIp[1]) << 16 |
                            uint32_t(board::kApIp[2]) << 8 | board::kApIp[3]);
  bool ok = false;
  if (connect_within(s, a, timeouts::kSelfCheckConnectMs)) {
    const char req[] = "GET /api/ping HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n";
    if (lwip_send(s, req, sizeof req - 1, 0) == int(sizeof req - 1)) {
      char buf[128];
      int n = lwip_recv(s, buf, sizeof buf - 1, 0);
      ok = n > 12 && std::strncmp(buf, "HTTP/1.1 200", 12) == 0;
    }
  }
  lwip_close(s);
  bool pass = ok && g_ping_ok;
  if (pass) g_loopback_only = false;   // trial self-test done: the dashboard opens to Wi-Fi clients
  return pass;
}

}  // namespace fs
