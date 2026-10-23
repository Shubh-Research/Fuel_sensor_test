// COREDUMP_FOUND once per stored crash dump (DESIGN.md §9.2). The dump stays in its partition until it is erased, so
// logging it at every boot filled the event log with copies of one old crash (hardware check 14 Oct 2026, fw 1.0.1:
// a dump from a 1.0.5 test build gave an event at each of 3 boots). The device keeps the id of the dump it reported
// last (NVS fs_sys/cd_id) and logs only a dump with another id. Pure C++.
#pragma once
#include <cstddef>
#include <cstdint>

namespace fs {

// Identity of a dump: FNV-1a over its size, its first bytes (header) and its last bytes (the core's checksum). Never 0.
uint32_t coredump_id(uint32_t size, const uint8_t* head, size_t n_head, const uint8_t* tail, size_t n_tail);

struct CoredumpNotice {
  bool log = false;      // log COREDUMP_FOUND for this dump
  bool write = false;    // store `id` as the noted dump (an NVS write: only when it changes)
  uint32_t id = 0;       // 0 = no dump noted
};
// present: a dump is stored now, with id `id`; noted: the id kept in NVS (0 = none).
CoredumpNotice coredump_notice(bool present, uint32_t id, uint32_t noted);

}  // namespace fs
