// COREDUMP_FOUND once per dump (DESIGN.md §9.2).
#include "coredump_note.h"

namespace fs {

namespace {
uint32_t fnv(uint32_t h, const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 16777619u;
  return h;
}
}  // namespace

uint32_t coredump_id(uint32_t size, const uint8_t* head, size_t n_head, const uint8_t* tail, size_t n_tail) {
  const uint8_t sz[4] = {uint8_t(size), uint8_t(size >> 8), uint8_t(size >> 16), uint8_t(size >> 24)};
  uint32_t h = fnv(0x811C9DC5u, sz, 4);
  h = fnv(h, head, head ? n_head : 0);
  h = fnv(h, tail, tail ? n_tail : 0);
  return h ? h : 1;
}

CoredumpNotice coredump_notice(bool present, uint32_t id, uint32_t noted) {
  CoredumpNotice n;
  if (present && id != 0 && id != noted) {          // a dump this device has not reported yet
    n.log = true;
    n.write = true;
    n.id = id;
  } else if (!present && noted != 0) {              // erased (Maintenance or USB): forget its id
    n.write = true;
    n.id = 0;
  } else {
    n.id = noted;
  }
  return n;
}

}  // namespace fs
