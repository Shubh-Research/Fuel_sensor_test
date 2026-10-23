// Device event log on LittleFS (DESIGN.md §9.2). Thread-safe; callable from every task.
#pragma once
#include <cstddef>
#include <cstdint>
#include "../core/eventlog.h"

namespace fs {

// Mounts LittleFS on the "spiffs" partition. cold_boot: power-on (RTC memory random), so the budget starts full.
bool events_begin(uint32_t boot_count, bool cold_boot);
// The boot number the log stamps: boot_count, raised above the newest stored record's boot (boot counts still in
// RTC memory are lost at a power cut; a reused number would merge two runs in the log). Store::raise_boots() it.
uint32_t events_boot();
// Writes the event unless its class budget is used up (EventBudget); dropped ones are counted in the next record.
void log_event(Ev type, uint16_t arg = 0, uint32_t value = 0);
// loopTask, about once a second: keeps the budget refilling between events (RTC) and, at most once per hour of
// uptime, writes an EVENTS_DROPPED summary (arg = bit (code-1) per dropped code, value = total) so dropped counts reach
// flash even when their own code never recurs or a power-on would clear RTC memory.
void events_tick();
uint16_t events_count();
bool events_get(uint16_t newest_index, EventRecord* r);
// Small whole-file storage next to the log (e.g. the tank-size mirror). name without a leading slash.
bool fs_file_write(const char* name, const uint8_t* data, size_t len);
bool fs_file_read(const char* name, uint8_t* out, size_t len);
bool fs_file_read_n(const char* name, uint8_t* out, size_t cap, size_t* got);   // up to cap bytes; *got = read
// Record files (the fill-report list): append at the end, size (-1: none), read at an offset, remove (true when gone),
// atomic rename over an existing file.
bool fs_file_append(const char* name, const uint8_t* data, size_t len);
long fs_file_size(const char* name);
bool fs_file_read_at(const char* name, uint32_t offset, uint8_t* out, size_t len);
bool fs_file_remove(const char* name);
bool fs_file_rename(const char* from, const char* to);

}  // namespace fs
