// Who may call which HTTP route (DESIGN.md §8, "Access"). net.cpp asks route_need() for every route; the table is
// pure C++ so the host tests check it for both build variants (test_access.cpp).
#pragma once
#include <cstdint>
#include "challenge.h"
#include "status.h"

namespace fs {

enum class Route : uint8_t {
  Index,           // GET /             customer dashboard
  ServicePage,     // GET /debug        service page: static HTML with its own login; all its data needs Service
  Status,          // GET /api/status
  Ping,            // GET /api/ping     trial self-test
  Challenge,       // POST /api/auth/challenge
  Response,        // POST /api/auth/response
  Logout,          // POST /api/auth/logout
  MaintState,      // GET /api/maint/state
  Tank,            // POST /api/maint/tank
  Reset,           // POST /api/maint/reset           reset to defaults
  Update,          // POST /api/maint/update
  Coredump,        // GET /api/maint/coredump         crash report download
  CoredumpErase,   // POST /api/maint/coredump/erase
  Restart,         // POST /api/maint/restart
  ServiceStatus,   // GET /api/service/status         full status + device counters
  ServiceEvents,   // GET /api/service/events         the event log
  Logs,            // GET /api/logs                   the device log
  Debug,           // GET /api/debug                  tracker internals (compiled into debug builds only)
  FillState,       // GET /api/fill                   the open fill report's stage + the last report (no sensor litres
                   //                                 before Confirm: the operator enters the litres blind)
  FillStart,       // POST /api/fill/start            the operator, no login (DESIGN.md §14)
  FillEnd,         // POST /api/fill/end
  FillConfirm,     // POST /api/fill/confirm          signs and returns the report
  FillLog,         // GET /api/fill/log               the device's report list (Maintenance)
};

enum class Need : uint8_t {
  Public,          // anyone on the Wi-Fi
  Session,         // any session: maintenance or service
  Maintenance,     // a maintenance grant (settings, firmware or calibration): customer key, lab key on debug builds
  Settings,        // maintenance grant bit0
  Firmware,        // maintenance grant bit1
  Service,         // a service grant: Procom service key, lab key on debug builds
};

Need route_need(Route r, bool debug_build);
// For a valid session's grant: does it meet the need? (Public is always met.)
bool grant_allows(const Grant& g, Need n);
// The fields of a status reply: /api/status is Full on debug builds (the raw-data recorders poll it without a session)
// and Customer on production builds; /api/service/status is Full on both.
StatusView status_view(Route r, bool debug_build);
const char* need_text(Need n);

}  // namespace fs
