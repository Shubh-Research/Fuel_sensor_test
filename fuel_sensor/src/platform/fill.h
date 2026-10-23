// Fill reports on the device (DESIGN.md §14): the FillBook (core/fillreport.h) on the heap, its NVS record, the report
// list in LittleFS (/fills.bin: 126-byte codes, newest last, 50-100 kept), the FILL_REPORT event and the signing key
// compiled in from the gitignored fill_key_gen.h (tools/build.py). Thread-safe: the loop task ticks, the HTTP task asks.
#pragma once
#include <cstddef>
#include <cstdint>
#include "../core/fillreport.h"
#include "../core/status.h"

namespace fs {

void fill_begin(const uint8_t mac[6]);             // app_setup: after the store, the event log (LittleFS) and the reset
void fill_tick(uint64_t now_ms);                   // loop task, once a second
void fill_note_tank_change();                      // after a tank size was saved
void fill_note_unverified();                       // the ledger marked an event FUEL_UNVERIFIED (or booked one unverified)
bool fill_enabled();                               // the image carries the fill-report key

// HTTP replies (the reply is written into j; the HTTP status is returned).
int fill_http_state(JsonOut& j);
int fill_http_start(uint32_t phone_t, int tz_min, JsonOut& j);
int fill_http_end(uint32_t phone_t, JsonOut& j);
int fill_http_confirm(double litres, JsonOut& j);

// The report list, newest first (i = 0 the newest).
int fill_log_count();
bool fill_log_get(int i, uint8_t code[kFillCodeBytes]);

// Reset to defaults at boot (core/reset.h kResetFillReports): the since-reset number to 0, no open report, the list removed;
// the lifetime serial is kept. Idempotent.
bool fill_reset_at_boot();

}  // namespace fs
