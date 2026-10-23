#include "logring.h"

#include <cstring>

namespace fs {

void LogRing::push(uint32_t t_ms, char level, const char* text, size_t len) {
  if (!n_) return;
  Line& l = slot_[next_ % n_];
  while (len && (text[len - 1] == '\n' || text[len - 1] == '\r')) --len;   // one line, no trailing newline
  if (len > kLineMax) len = kLineMax;
  l.seq = next_;
  l.t_ms = t_ms;
  l.level = level;
  l.len = uint8_t(len);
  std::memcpy(l.text, text, len);
  l.text[len] = '\0';
  ++next_;
}

bool LogRing::get(uint32_t seq, Line* out) const {
  if (seq >= next_ || seq < first_seq()) return false;
  *out = slot_[seq % n_];
  return true;
}

}  // namespace fs
