// Live level: the v6.10 LiveLevel logic around TrackerV4 (DESIGN.md §11). Pure C++, no platform headers.
// The device calls begin() once with the stored level, push() for every radar frame, and output() whenever it
// needs the value. Everything is statically sized; no allocation after construction.
#pragma once
#include <cstdint>
#include "frame.h"

namespace fs {

enum class Source : uint8_t { None = 0, Stored = 1, LastConfirmed = 2, Estimate = 3 };     // which output_level() term
enum class TrackState : uint8_t { NoLevel = 0, Confirmed = 1, Uncertain = 2, Holding = 3, Fault = 4 };
enum class CycleSource : uint8_t { Assumed = 0, Shape = 1, Anchored = 2, Memory = 3 };
enum class Channel : uint8_t { None = 0, NearEmpty = 1, Bottom = 2, Surface = 3 };

// One tracker estimate (every 4th frame). Diagnostics are NaN when not available.
struct Estimate {
  double t_s = 0;                       // tracker clock (seconds, from the frame ticks)
  double h_mm = 0;                      // estimate (NaN while there is none)
  TrackState state = TrackState::NoLevel;
  CycleSource cycle = CycleSource::Assumed;
  Channel channel = Channel::None;
  bool uncertain = true, stable = false, verifying = false, recheck = false, doubt = false, disturbed = false;
  double hb = 0, hs = 0, b = 0, s = 0, b_env = 0, s_env = 0, pstd = 0, n = 0, c0_s = 0, leak_mm = 0, gap_mm = 0;
  int cycle_step = 0;                   // fuel ledger: whole floor cycles the reported level moved off phase continuity (TrackResult)
  bool cont = false;                    // the floor phase was carried from an earlier estimate (false at a start / restart)
};

// What the device shows and sends: always a value (v6.10 output_level()).
struct Output {
  double level_mm = 0;                  // output_level(): last estimate -> last confirmed -> stored -> 0
  Source source = Source::None;
  TrackState state = TrackState::NoLevel;
  CycleSource cycle = CycleSource::Assumed;
  bool moving = false;                  // |rate| > 0.05 mm/s over the last 10 s of outputs
  double rate_mm_s = 0;
  uint32_t estimates = 0;               // estimates since begin()
};

class LiveImpl;                         // defined in live.cpp (holds the tracker, ~40 KB)

class Live {
 public:
  Live();
  ~Live();
  Live(const Live&) = delete;
  Live& operator=(const Live&) = delete;

  // Boot: the level stored before power-off (NaN if none). Same role as LiveLevel's memory file.
  void begin(double stored_level_mm);
  // One radar frame. Returns true and fills *est (if non-null) when this frame produced an estimate.
  // Frame time comes from f.tick_ms (wrap-safe). Gaps > 1 s are tracker gaps; > 60 s restart from the stored level.
  bool push(const Frame& f, Estimate* est);
  // The radar link was lost or re-opened (the next frame starts a gap even if its tick looks continuous).
  void note_link_gap();
  Output output() const;
  // The last CONFIRMED level (what the device saves to flash); false while there is none in this run.
  bool stored_level(double* mm) const;
  uint32_t gaps() const;
  uint32_t restarts() const;

 private:
  LiveImpl* impl_;                      // points into static storage owned by live.cpp (no heap)
};

}  // namespace fs
