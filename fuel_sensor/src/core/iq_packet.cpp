// UART radar packets (DESIGN.md §7.1). Layout from the v6.3 firmware's iq_protocol.h; behaviour changed so that a
// corrupt packet or a sequence break is a GAP for the live layer, never a tracker reset.
#include "iq_packet.h"

#include <cmath>
#include <cstring>

namespace fs {

namespace {
constexpr uint8_t kMagic[8] = {'A', '1', '2', '1', 'I', 'Q', '\r', '\n'};
uint16_t le16(const uint8_t* p) { return uint16_t(p[0] | p[1] << 8); }
uint32_t le32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
float lef32(const uint8_t* p) { uint32_t u = le32(p); float f; std::memcpy(&f, &u, 4); return f; }

uint32_t crc_table[256];
bool crc_ready = false;
void crc_init() {
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int b = 0; b < 8; ++b) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    crc_table[i] = c;
  }
  crc_ready = true;
}
}  // namespace

uint32_t crc32_ieee(const uint8_t* data, size_t len) {
  if (!crc_ready) crc_init();
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) crc = crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return crc ^ 0xFFFFFFFFu;
}

void PacketDecoder::reset() {
  used_ = 0;
  gap_pending_ = true;
}

void PacketDecoder::discard(size_t n) {
  std::memmove(buf_, buf_ + n, used_ - n);
  used_ -= n;
  stats_.discarded += uint32_t(n);
}

// First position after 0 where a packet could start (its first magic byte), else everything: a resync then costs one
// memmove instead of one per byte.
size_t PacketDecoder::next_start() const {
  const void* p = used_ > 1 ? std::memchr(buf_ + 1, kMagic[0], used_ - 1) : nullptr;
  return p ? size_t(static_cast<const uint8_t*>(p) - buf_) : used_;
}

bool PacketDecoder::feed(uint8_t byte, uint32_t rx_ms, Frame* out) {
  buf_[used_++] = byte;
  while (used_) {
    size_t cmp = used_ < 8 ? used_ : 8;
    if (std::memcmp(buf_, kMagic, cmp) != 0) { discard(next_start()); continue; }
    if (used_ < 16) return false;
    if (le16(buf_ + 8) != 1 || le16(buf_ + 10) != 80 || le32(buf_ + 12) != 676) {
      ++stats_.bad;
      gap_pending_ = true;
      discard(next_start());
      continue;
    }
    if (used_ < kPacketBytes) return false;
    if (crc32_ieee(buf_, 756) != le32(buf_ + 756) || le16(buf_ + 66) != 0 ||
        uint32_t(le16(buf_ + 40)) * le16(buf_ + 46) * 4u != 676u) {
      ++stats_.bad;
      gap_pending_ = true;
      discard(next_start());
      continue;
    }
    bool got = parse(rx_ms, out);
    used_ = 0;
    return got;
  }
  return false;
}

bool PacketDecoder::parse(uint32_t rx_ms, Frame* out) {
  const uint8_t* b = buf_;
  float step = lef32(b + 56);
  // The radar settings echoed by the sensor MCU must equal the calibration (start -8, 169 points, step 1, HWAAS 183,
  // 1 sweep/frame, profile 1, 10 Hz, PRF code 7, ...). Same test as the v6.3 firmware.
  bool config_ok = le32(b + 36) == uint32_t(-8) && le16(b + 40) == 169 && le16(b + 42) == 1 && le16(b + 44) == 183 &&
                   le16(b + 46) == 1 && b[48] == 1 && b[49] == 10 && b[50] == 0 && b[51] == 7 && le32(b + 52) == 100 &&
                   std::isfinite(step) && std::fabs(double(step) - 0.002502274001017213) <= 1e-9 &&
                   le32(b + 60) == 0x00010d00u && b[64] == 0 && b[65] == 2 && lef32(b + 68) == 0.0f &&
                   lef32(b + 72) == 0.0f;
  if (!config_ok) {
    ++stats_.config_rejects;
    gap_pending_ = true;
    return false;
  }
  uint32_t seq = le32(b + 16), epoch = le32(b + 24);
  if (have_prev_) {
    if (seq != prev_seq_ + 1u) { ++stats_.seq_gaps; gap_pending_ = true; }
    if (epoch != prev_epoch_) { ++stats_.epoch_changes; gap_pending_ = true; }
  }
  have_prev_ = true;
  prev_seq_ = seq;
  prev_epoch_ = epoch;
  out->seq = seq;
  out->tick_ms = le32(b + 20);
  out->rx_ms = rx_ms;
  out->temperature_c = int16_t(le16(b + 32));
  // The sensor MCU's quality flags: bit0 saturated, bit1 frame delayed, bit2 calibration needed (A121 result order).
  uint32_t qf = le32(b + 28);
  out->flags = uint16_t((qf & 1 ? kFrameSaturated : 0) | (qf & 2 ? kFrameDelayed : 0) | (qf & 4 ? kFrameCalNeeded : 0) |
                        (gap_pending_ ? kFrameGapBefore : 0));
  stats_.overruns = le32(b + 76);
  for (int i = 0; i < kPoints; ++i) {
    out->iq[i].i = int16_t(le16(b + 80 + 4 * i));
    out->iq[i].q = int16_t(le16(b + 82 + 4 * i));
  }
  gap_pending_ = false;
  ++stats_.packets;
  return true;
}

}  // namespace fs
