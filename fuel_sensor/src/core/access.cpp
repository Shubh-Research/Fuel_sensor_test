// Route access table (DESIGN.md §8).
#include "access.h"

namespace fs {

Need route_need(Route r, bool debug_build) {
  switch (r) {
    case Route::Index: case Route::ServicePage: case Route::Status: case Route::Ping:
    case Route::Challenge: case Route::Response: case Route::Debug:
    case Route::FillState: case Route::FillStart: case Route::FillEnd: case Route::FillConfirm:
      return Need::Public;                     // fill reports: the operator needs no login (rate-limited in FillBook)
    case Route::Logout: case Route::Coredump:
      return Need::Session;                    // the crash report: Maintenance ("send it to support") and the service page
    case Route::MaintState: case Route::CoredumpErase: case Route::Restart: case Route::FillLog:
      return Need::Maintenance;
    case Route::Tank: case Route::Reset:
      return Need::Settings;
    case Route::Update:
      return Need::Firmware;
    case Route::ServiceStatus: case Route::ServiceEvents:
      return Need::Service;
    case Route::Logs:
      return debug_build ? Need::Public : Need::Service;   // debug builds: anyone (bench tools read it)
  }
  return Need::Service;                        // unknown: the strictest
}

bool grant_allows(const Grant& g, Need n) {
  switch (n) {
    case Need::Public: return true;
    case Need::Session: return g.any_scope();
    case Need::Maintenance: return g.maintenance();
    case Need::Settings: return g.settings;
    case Need::Firmware: return g.firmware;
    case Need::Service: return g.service;
  }
  return false;
}

StatusView status_view(Route r, bool debug_build) {
  if (r == Route::ServiceStatus) return StatusView::Full;
  return debug_build ? StatusView::Full : StatusView::Customer;
}

const char* need_text(Need n) {
  switch (n) {
    case Need::Public: return "public";
    case Need::Session: return "session";
    case Need::Maintenance: return "maintenance";
    case Need::Settings: return "settings";
    case Need::Firmware: return "firmware";
    case Need::Service: return "service";
  }
  return "service";
}

}  // namespace fs
