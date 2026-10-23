// Live level: LiveLevel v6.10 (dev/live_level.py) around the tracker, minus the console, CSV, capacitive and memory-file parts
// (DESIGN.md 11). Pure C++17, no platform headers, no heap: the one Live of the firmware lives in static storage here.
//
// Mirrors LiveLevel.push: a repeated frame is dropped (frozen radar), frames more than kGapS apart are a gap (TrackerV4.note_gap;
// longer than restart_gap_s = a restart from the last confirmed level), one estimate on every 4th accepted frame, LiveLevel.last
// keeps the last estimate that had a level. The frame clock is the frame's measurement tick (Frame.tick_ms, u32 ms, wrap-safe),
// accumulated in 64-bit milliseconds and handed to the tracker as seconds (double ms / 1000.0) -- never the processing time.
#include "live.h"

#include <cstring>
#include <new>

#include "tracker.h"

namespace fs {

class LiveImpl {
 public:
  // push_at() results
  enum PushResult : int { kNoEstimate = 0, kEstimateLevel = 1, kEstimateNoLevel = 2, kFrozen = 3 };
  static constexpr double kMovingRateMmS = 0.05;          // Output.moving: |rate| above this (DESIGN.md 11)
  static constexpr double kRateWindowS = 10.0;            // rate from the outputs of the last 10 s ...
  static constexpr double kRateMinSpanS = 5.0;            // ... once they span at least 5 s (3 samples or more)
  static constexpr int64_t kUnknownGapMs = static_cast<int64_t>(Tracker::kRestartGapS * 1000.0) + 1000;   // > restart_gap_s
  static constexpr int64_t kLinkGapMinMs = static_cast<int64_t>(cal::kGapS * 1000.0) + 1;                  // > the gap threshold

  LiveImpl() { begin(kNaN); }

  // Boot (a fresh LiveLevel): the level stored before power-off, used if it is a level the tank can hold
  // (LiveLevel._load_memory: -3 mm .. 1.5 x tank height); NaN = none.
  void begin(double stored_mm) {
    tracker.init();
    if (stored_mm >= -3.0 && stored_mm <= cal::kMemMaxMm) tracker.set_memory(stored_mm);
    n_frames_ = 0; has_t_prev_ = false; t_prev_ = 0.0; has_prev_ = false; frozen_ = false;
    gaps_ = 0; restarts_ = 0; has_last_ = false; estimates_ = 0;
    has_tick_ = false; last_tick_ = 0; clock_ms_ = 0; link_gap_ = false;
    rate_win_.clear(); rate_ = 0.0; moving_ = false;
  }

  // One radar frame on the device clock.
  bool push(const Frame& f, Estimate* est) {
    const int r = push_at(f.iq, clock_seconds(f.tick_ms), est);
    return r == kEstimateLevel || r == kEstimateNoLevel;
  }

  // LiveLevel.push(iq, elapsed_s) with the frame time t in seconds (the tests drive this directly with the recorder's clock).
  int push_at(const IQ16* iq, double t, Estimate* est) {
    frozen_ = has_prev_ && std::memcmp(iq, prev_, sizeof(prev_)) == 0;   // r3: a repeated (stale) frame is not new data
    if (frozen_) return kFrozen;
    std::memcpy(prev_, iq, sizeof(prev_));
    has_prev_ = true;
    ++n_frames_;
    if (has_t_prev_ && t - t_prev_ > cal::kGapS) {              // v6.2: a gap drops the old frames
      const double g = t - t_prev_;
      tracker.note_gap(g);
      ++gaps_;
      if (g > Tracker::kRestartGapS) restarted();               // LiveLevel._restarted: the pre-gap reading is not current
    }
    t_prev_ = t;
    has_t_prev_ = true;
    tracker.push(iq);
    if (n_frames_ % cal::kEstimateEvery) return kNoEstimate;
    const TrackResult& r = tracker.estimate(t);
    sample_output(t);
    if (est) fill(r, t, est);
    if (!r.has_h) return kEstimateNoLevel;                      // (no level: LiveLevel.last unchanged)
    last_ = r;
    has_last_ = true;
    ++estimates_;
    return kEstimateLevel;
  }

  // The link was lost or re-opened: the next frame's tick is not continuous with the last one.
  void note_link_gap() { link_gap_ = true; }

  Output output() const {
    Output o;
    o.level_mm = tracker.output_level();                        // output_level(): last estimate -> last confirmed -> stored -> 0
    if (tracker.last()) o.source = Source::Estimate;
    else if (is_set(tracker.stored_level())) o.source = Source::LastConfirmed;
    else if (is_set(tracker.mem())) o.source = Source::Stored;
    else o.source = Source::None;
    o.state = has_last_ ? last_.state : TrackState::NoLevel;
    o.cycle = has_last_ ? last_.cycle_source : CycleSource::Assumed;
    o.moving = moving_;
    o.rate_mm_s = rate_;
    o.estimates = estimates_;
    return o;
  }

  bool stored_level(double* mm) const {
    const double h = tracker.stored_level();
    if (!is_set(h)) return false;
    if (mm) *mm = h;
    return true;
  }

  uint32_t gaps() const { return gaps_; }
  uint32_t restarts() const { return restarts_; }
  uint32_t frames() const { return n_frames_; }
  bool frozen() const { return frozen_; }
  const TrackResult* last() const { return has_last_ ? &last_ : nullptr; }   // LiveLevel.last

  Tracker tracker;

 private:
  struct RateSample { double t, h; };

  // u32 tick -> seconds since begin(). A backwards tick (the sensor clock restarted) means an unknown interval: it is taken as
  // longer than restart_gap_s, so the tracker restarts from the last confirmed level. After note_link_gap() the interval is at
  // least just over the gap threshold, so the frames before the link loss are dropped (a tracker gap).
  double clock_seconds(uint32_t tick) {
    if (has_tick_) {
      const int32_t dt = static_cast<int32_t>(tick - last_tick_);   // wrap-safe difference
      int64_t step = dt;
      if (dt < 0) step = kUnknownGapMs;
      else if (link_gap_ && step < kLinkGapMinMs) step = kLinkGapMinMs;
      clock_ms_ += step;
    }
    has_tick_ = true;
    last_tick_ = tick;
    link_gap_ = false;
    return static_cast<double>(clock_ms_) / 1000.0;
  }

  void restarted() {
    has_last_ = false;
    ++restarts_;
    rate_win_.clear();
    rate_ = 0.0;
    moving_ = false;
  }

  // Output rate: least-squares slope of the outputs over the last kRateWindowS (one sample per estimate).
  void sample_output(double t) {
    rate_win_.push(RateSample{t, tracker.output_level()});
    double st = 0, sh = 0, t0 = 0, t1 = 0;
    int m = 0;
    for (int i = 0; i < rate_win_.size(); ++i) {
      const RateSample& s = rate_win_[i];
      if (s.t < t - kRateWindowS) continue;
      if (m == 0) t0 = s.t;
      t1 = s.t;
      st += s.t - t;
      sh += s.h;
      ++m;
    }
    rate_ = 0.0;
    if (m >= 3 && t1 - t0 >= kRateMinSpanS) {
      const double mt = st / m, mh = sh / m;
      double sxx = 0, sxy = 0;
      for (int i = 0; i < rate_win_.size(); ++i) {
        const RateSample& s = rate_win_[i];
        if (s.t < t - kRateWindowS) continue;
        const double dt = (s.t - t) - mt;
        sxx += dt * dt;
        sxy += dt * (s.h - mh);
      }
      if (sxx > 0) rate_ = sxy / sxx;
    }
    moving_ = std::fabs(rate_) > kMovingRateMmS;
  }

  static void fill(const TrackResult& r, double t, Estimate* e) {
    e->t_s = t;
    e->h_mm = r.has_h ? r.h : kNaN;
    e->state = r.state;
    e->cycle = r.cycle_source;
    e->channel = r.channel;
    e->uncertain = r.cycle_uncertain;
    e->stable = r.has_h && r.pstd < 8.0;                        // diagnostics only (not in Python): a clean echo phase
    e->verifying = r.verifying;
    e->recheck = r.recheck;
    e->doubt = r.doubt;
    e->disturbed = r.disturbed;
    e->hb = r.h_b; e->hs = r.h_s; e->b = r.b; e->s = r.s; e->b_env = r.b_env; e->s_env = r.s_env;
    e->pstd = r.pstd; e->n = r.n; e->c0_s = r.c0_s; e->leak_mm = r.leak_mm; e->gap_mm = r.gap_ema;
    e->cycle_step = r.cycle_step; e->cont = r.cont;
  }

  IQ16 prev_[kPoints];                                          // LiveLevel._z_prev
  bool has_prev_, frozen_;
  uint32_t n_frames_;                                           // accepted frames (the every-4th-frame count)
  bool has_t_prev_;
  double t_prev_;
  uint32_t gaps_, restarts_, estimates_;
  bool has_last_;
  TrackResult last_;
  bool has_tick_, link_gap_;
  uint32_t last_tick_;
  int64_t clock_ms_;
  Ring<RateSample, 64> rate_win_;
  double rate_;
  bool moving_;
};

static_assert(kPoints == cal::kPoints, "frame size must equal the calibration's");

namespace {
alignas(LiveImpl) unsigned char g_live_storage[sizeof(LiveImpl)];   // the firmware's one LiveImpl (no heap)
bool g_live_in_use = false;
}  // namespace

Live::Live() {
  if (g_live_in_use) __builtin_trap();                          // one Live per firmware: the storage is static
  g_live_in_use = true;
  impl_ = new (g_live_storage) LiveImpl();
}

Live::~Live() {
  impl_->~LiveImpl();
  g_live_in_use = false;
}

void Live::begin(double stored_level_mm) { impl_->begin(stored_level_mm); }
bool Live::push(const Frame& f, Estimate* est) { return impl_->push(f, est); }
void Live::note_link_gap() { impl_->note_link_gap(); }
Output Live::output() const { return impl_->output(); }
bool Live::stored_level(double* mm) const { return impl_->stored_level(mm); }
uint32_t Live::gaps() const { return impl_->gaps(); }
uint32_t Live::restarts() const { return impl_->restarts(); }

}  // namespace fs
