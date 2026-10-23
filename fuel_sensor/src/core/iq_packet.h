// UART radar packets from the WROVER board's sensor MCU (DESIGN.md §7.1; format unchanged from the v6.3 firmware).
// Byte-wise decoder with resync on the magic, CRC-32, config check against the calibration, and frame continuity.
#pragma once
#include <cstddef>
#include <cstdint>
#include "frame.h"

namespace fs {

constexpr size_t kPacketBytes = 760;

struct PacketStats {
  uint32_t packets = 0;          // good packets
  uint32_t bad = 0;              // CRC / header errors (each costs at most a resync)
  uint32_t discarded = 0;        // bytes skipped while resyncing
  uint32_t config_rejects = 0;   // well-formed packets whose radar config differs from the calibration
  uint32_t seq_gaps = 0;         // missing or out-of-order sequence numbers
  uint32_t epoch_changes = 0;    // sensor re-calibrations
  uint32_t overruns = 0;         // sensor-side overrun counter (last value)
};

class PacketDecoder {
 public:
  // Feeds one byte; returns true when `out` holds a new frame. kFrameGapBefore is set when frames are missing
  // before it (sequence break, epoch change, or after a bad packet).
  bool feed(uint8_t byte, uint32_t rx_ms, Frame* out);
  void reset();                  // after a link re-open: the next frame is marked as following a gap
  const PacketStats& stats() const { return stats_; }

 private:
  void discard(size_t n);
  size_t next_start() const;
  bool parse(uint32_t rx_ms, Frame* out);
  uint8_t buf_[kPacketBytes] = {};
  size_t used_ = 0;
  bool have_prev_ = false, gap_pending_ = true;
  uint32_t prev_seq_ = 0, prev_epoch_ = 0;
  PacketStats stats_;
};

uint32_t crc32_ieee(const uint8_t* data, size_t len);

}  // namespace fs
