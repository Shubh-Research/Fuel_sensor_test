// One radar frame as every radar source delivers it (DESIGN.md §6). Pure C++, no platform headers.
#pragma once
#include <cstdint>

namespace fs {

constexpr int kPoints = 169;            // sparse I/Q points per frame (must equal the calibration)

struct IQ16 { int16_t i, q; };

enum FrameFlag : uint16_t {
  kFrameSaturated   = 1u << 0,          // A121 data_saturated
  kFrameDelayed     = 1u << 1,          // A121 frame_delayed
  kFrameCalNeeded   = 1u << 2,          // A121 calibration_needed
  kFrameGapBefore   = 1u << 3,          // the source lost frames (or the link) just before this one
};

struct Frame {
  uint32_t seq = 0;                     // radar sequence number (diagnostics)
  uint32_t tick_ms = 0;                 // MEASUREMENT time in ms (sensor tick or MCU timer at trigger); wraps
  uint32_t rx_ms = 0;                   // MCU time the frame reached the firmware (diagnostics only)
  int16_t temperature_c = 0;            // A121 die temperature, whole degrees C (relative only)
  uint16_t flags = 0;                   // FrameFlag bits
  IQ16 iq[kPoints] = {};
};

}  // namespace fs
