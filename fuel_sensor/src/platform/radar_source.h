// Radar input, the one module that differs per board (DESIGN.md §6). Both sources deliver identical fs::Frame objects
// with the MEASUREMENT time in tick_ms, so everything above this line is shared code.
#pragma once
#include <cstdint>
#include "../core/frame.h"

namespace fs {

struct RadarStats {
  uint32_t frames = 0;           // frames delivered
  uint32_t bad = 0;              // corrupt packets (UART) / failed measurements (SPI)
  uint32_t gaps = 0;             // frames marked kFrameGapBefore
  uint32_t config_rejects = 0;   // radar settings differ from the calibration
  uint32_t link_resets = 0;      // reset_link() calls
  uint32_t recalibrations = 0;   // sensor calibrations after the first
};

class RadarSource {
 public:
  virtual ~RadarSource() = default;
  virtual bool begin() = 0;                                  // once, from setup()
  virtual bool read(Frame& out, uint32_t timeout_ms) = 0;    // blocks at most timeout_ms
  virtual void reset_link() = 0;                             // supervisor: re-open after silence
  virtual void drop_input() {}                               // firmware upload: discard what arrives (UART: flush)
  virtual RadarStats stats() const = 0;
  virtual const char* name() const = 0;
};

RadarSource& radar_source();                                 // the board's source (radar_uart.cpp or radar_a121.cpp)

}  // namespace fs
