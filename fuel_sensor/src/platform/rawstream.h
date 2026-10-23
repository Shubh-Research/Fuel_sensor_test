// Raw radar stream for experiments (DESIGN.md §7.2). DEBUG builds only; production builds compile this to nothing.
#pragma once
#include <cstdint>
#include "../core/frame.h"
#include "../core/live.h"

namespace fs {

struct AppInfo;

#if defined(FS_VARIANT_DEBUG)
void rawstream_begin(const AppInfo& info);
// Called by the tracker task; never blocks (drops and counts when the laptop is slow).
void rawstream_frame(const Frame& f);
void rawstream_estimate(uint32_t seq, const Estimate& e, const Output& o);
uint32_t rawstream_drops();
uint8_t rawstream_state();                          // 1 running, 2 no memory for its buffer (DeviceStatus::raw_stream)
#else
inline void rawstream_begin(const AppInfo&) {}
inline void rawstream_frame(const Frame&) {}
inline void rawstream_estimate(uint32_t, const Estimate&, const Output&) {}
inline uint32_t rawstream_drops() { return 0; }
inline uint8_t rawstream_state() { return 0; }       // production: off
#endif

}  // namespace fs
