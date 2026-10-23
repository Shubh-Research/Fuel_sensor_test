// In-RAM ring of recent log lines for reading over Wi-Fi (DESIGN.md §8, /api/logs). Fixed slots: a line is at most
// kLineMax characters (longer ones are cut), the newest slots() lines are kept, each with a sequence number so a reader
// can ask for "everything after N" and notice what it missed. The slot memory is given at begin() (the platform takes
// it from the internal heap: the WROVER build has under 2 KB of static DRAM to spare). Not thread-safe: the
// platform wraps it in a lock. Without begin() every push is dropped.
#pragma once
#include <cstddef>
#include <cstdint>

namespace fs {

class LogRing {
 public:
  static constexpr size_t kSlots = 64;
  static constexpr size_t kLineMax = 120;

  struct Line {
    uint32_t seq = 0;
    uint32_t t_ms = 0;          // uptime when logged
    char level = 'I';           // E W I D
    uint8_t len = 0;
    char text[kLineMax + 1] = {};
  };

  void begin(Line* slots, size_t n) { slot_ = slots; n_ = slots ? n : 0; next_ = 0; }
  size_t slots() const { return n_; }
  void push(uint32_t t_ms, char level, const char* text, size_t len);
  uint32_t next_seq() const { return next_; }
  // Oldest sequence number still held (== next_seq() when empty).
  uint32_t first_seq() const { return next_ > n_ ? next_ - uint32_t(n_) : 0; }
  // Copies line `seq` into *out; false if it is not held (too old or not written yet).
  bool get(uint32_t seq, Line* out) const;

 private:
  Line* slot_ = nullptr;
  size_t n_ = 0;
  uint32_t next_ = 0;
};

}  // namespace fs
