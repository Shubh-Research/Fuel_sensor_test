// TrackerV4 v6.10 (dev/level_v4.py) in C++, with the fixed-point template search and start-up vote of
// dev/esp32_port/reference/level_v4_fixedpoint.py (kern.c). Pure C++17, no platform headers, no heap, no exceptions.
//
// Equivalence contract (DESIGN.md 1, 11): the same decisions as Python v6.10 fixed point given the same frames and frame times.
// Every float expression follows the numpy semantics the Python code gets (numpy 2.x, NEP 50): float32 where numpy computes in
// float32 (the phase and leakage means, the surface thresholds), pairwise sums, Smith complex division, the SIMD complex abs and
// complex multiply with a fused multiply-add, Python's % and // on floats, np.percentile's linear interpolation, np.round = rint.
// What cannot be bit-identical: numpy's float32 log (its own SIMD polynomial) and LAPACK least squares in np.polyfit / lstsq (the
// closed form here): a few ulp in pstd, the pre-hold rate and n_fit; decisions only differ if a value grazes a threshold.
// Host build vs ESP32 (glibc vs newlib libm): the integer kernels, the IEEE arithmetic, atan2f on the tracker's inputs and the
// float32 pstd chain (|u| by sqrt, log in double) give the same bits. The double-precision log of the start-up votes and the
// ramp model's atan2 / sin / cos / hypot / log / fma can differ in the last bit (newlib's fma is not fused): vote log-ratios and
// the ramp-model phase then differ by ~1e-16 relative, far below any decision margin seen (README of test/equiv).
//
// Mapping of names: members keep the Python names without the leading underscore and with a trailing underscore
// (_su_votes -> su_votes_); Python None is NaN for doubles (is_set()) or an explicit flag. Comments cite level_v4.py functions.
#pragma once
#include <cstdint>
#include <cmath>
#include "calib_data.h"
#include "frame.h"
#include "kern.h"
#include "live.h"           // TrackState, CycleSource, Channel

namespace fs {

constexpr double kNaN = __builtin_nan("");
inline bool is_set(double x) { return x == x; }   // Python "is not None" for doubles kept as NaN

// collections.deque(maxlen=N): push drops the oldest; [0] is the oldest.
template <typename T, int N>
class Ring {
 public:
  void clear() { n_ = 0; head_ = 0; }
  void push(const T& v) {
    if (n_ < N) { buf_[(head_ + n_) % N] = v; ++n_; }
    else { buf_[head_] = v; head_ = (head_ + 1) % N; }
  }
  int size() const { return n_; }
  bool empty() const { return n_ == 0; }
  const T& operator[](int i) const { return buf_[(head_ + i) % N]; }
  const T& back() const { return (*this)[n_ - 1]; }
  static constexpr int capacity() { return N; }
 private:
  T buf_[N] = {};
  int n_ = 0, head_ = 0;
};

enum class SuState : uint8_t { Collecting = 0, Decided = 1, Undecided = 2, Off = 3 };   // su_state ('off': no shapes)

// One estimate, as the Python estimate() dict. has_h false = h None (state NoLevel for 'none', Fault for 'fault').
struct TrackResult {
  bool has_h = false;
  double h = kNaN;                       // reported level (the floor channel, or the floor envelope's cycle while 'assumed')
  Channel channel = Channel::None;       // 'bottom' (> 15 mm) / 'near-empty' / 'none'
  TrackState state = TrackState::NoLevel;
  bool check = false;                    // calibration check wanted
  double s = kNaN, b = kNaN, s_env = kNaN, b_env = kNaN;
  bool cycle_uncertain = true;
  CycleSource cycle_source = CycleSource::Assumed;
  SuState su_state = SuState::Collecting;
  double h_b = kNaN, h_s = kNaN, pstd = kNaN;
  bool disturbed = false;
  double n = kNaN, c0_s = kNaN, leak_mm = kNaN, gap_ema = kNaN;
  bool verifying = false, recheck = false, doubt = false;
  // fuel ledger (diagnostics; no effect on the level): whole floor cycles by which this estimate's reported level moved away
  // from phase continuity (re-anchor, start-up or stored-level decision, bound reset, output cycle while assumed; 0 for a hold),
  // and whether the floor phase was carried from an earlier estimate (false at a start or restart: no continuity)
  int cycle_step = 0;
  bool cont = false;
};

// mem_result: the check of the stored level at the first estimate (None until then).
struct MemResult {
  bool valid = false;
  double r = kNaN, env = kNaN;
  bool env_ok = false, accepted = false;
};

struct TrackerCounters {                 // the Python counters (tests and diagnostics)
  uint32_t resnaps = 0, bridged = 0, shape_resnaps = 0, env_conflicts = 0, bounds_hit = 0, sc_updates = 0, n_updates = 0;
  uint32_t leak_corrections = 0, rechecks = 0, recheck_disagree = 0, gaps = 0, mem_overruled = 0;
  uint32_t su_votes_run = 0, su_levels_run = 0;   // start-up votes computed and levels fitted (cost diagnostics, not in Python)
};

class Tracker {
 public:
  // ---- configuration (TrackerV4 class attributes and __init__ defaults, v6.10)
  static constexpr int kFrames = 20;                       // frames=20: ring of the last 20 frames
  static constexpr double kScAlpha = 0.02, kScBeta = 0.05;
  static constexpr double kScGateH = 45.0, kScLimZero = 1.0, kScLimN = 0.01, kScNSpan = 80.0, kScBinMm = 10.0;
  static constexpr double kLeakDeadbandMm = 0.10;
  static constexpr double kSuLeakMm = 0.15;                  // su_leak_mm: a drift beyond this enters the start-up fit
  static constexpr double kEnvOutLo = 25.0, kEnvTrust = 60.0, kBoundLo = -3.0;
  static constexpr double kMemTau = 0.45, kMemEtol = 3.0, kMemEnvGuard = 16.0;
  static constexpr int kMemVerifyVotes = 60;
  static constexpr double kRestartGapS = 60.0, kPostS = 5.0, kHoldRecheckMm = 2.2, kGapAlarmMm = 0.8, kAmbMax = 0.3;
  static constexpr int kTplDecim = 5, kTplCoarse = 10;
  static constexpr double kTplWin = 10.0;
  static constexpr double kSuWinLo = -1.5, kSuWinHi = 0.5, kSuMaxH = 70.0;
  static constexpr double kSurfaceMin = 45.0, kMinSep = 30.0, kAnchorLo = 30.0, kPstdMax = 45.0, kAmpMin = 0.4;
  static constexpr double kEnvTol = 2.7, kSteadyMm = 1.5;
  static constexpr int kSuVotesN = 60, kSuMinVotes = 10;
  static constexpr double kSuSteadyMm = 0.5, kSuRelMm = 8.0, kSuEarlyLead = 10.0, kSuMinLead = 5.0;
  static constexpr int kMaxNBins = 64;                     // 10-mm level bins of the n loop (h_b < 640 mm; the sweep ends ~370 mm)
  static constexpr int kTplScratch = 1536;                 // local fine-grid envelope of one template search (<= 401 + 879 points)

  Tracker();
  void init();                                             // a fresh tracker: __init__ + set_calibration + set_shapes + lpmm

  void push(const IQ16* iq);                               // push(): one frame (169 int16 I/Q) into the ring
  void note_gap(double seconds);                           // note_gap()
  const TrackResult& estimate(double now);                 // t_now = now; estimate(). Valid until the next call.

  void set_memory(double h) { mem_ = h; }                  // set_memory(h); NaN = None
  double mem() const { return mem_; }
  double stored_level() const { return stored_; }          // _stored: the last confirmed level (NaN = None)
  double output_level() const;                             // output_level(): last.h -> _stored -> mem -> 0
  const TrackResult* last() const { return has_last_ ? &last_ : nullptr; }
  const MemResult& mem_result() const { return mem_result_; }
  const TrackerCounters& counters() const { return cnt_; }

  // diagnostics (tests)
  double n() const { return n_; }
  double c0_s() const { return c0_s_; }
  bool anchored() const { return anchored_; }
  int su_votes() const { return su_votes_.size(); }
  bool has_out_k() const { return has_out_k_; }
  int out_k() const { return out_k_; }
  double leak_ref() const { return leak_ref_; }

 private:
  struct SuVote { uint32_t mask; double lr[cal::kSuSlots]; };   // {j: log(residual / best)}, slot j + 4
  struct HistItem { double t, h; };
  struct BlkItem { double hb, gap, braw, sraw; };
  static_assert(cal::kSuSlots <= 32, "candidate mask is 32 bits");
  static_assert(cal::kPoints == kPoints, "the calibration grid must match the frame size");

  // helpers (level_v4.py names)
  void reset_lock();
  double h_bottom(double b) const { return (b - cal::kD0) / (n_ - 1); }
  static double snap(double x, double ref);
  bool select_floor(double* b_c, double* b_amp) const;
  double tpl_pos(double guess);
  double select_surface(double b_env, double b_amp);
  double leak_update();
  double phase(double x, double* pstd) const;
  bool su_vote(double b_fine, double corr, SuVote* out);         // corr: this estimate's leakage-phase correction (mm)
  bool su_best(double b, int* k, double* lead) const;
  double apply_memory(double xb, double b_assumed, double h_env_b);
  double rate_pre() const;
  void hold_spread(double spread);
  void end_recheck(bool flag);
  void sc_block();
  const TrackResult& fault(TrackState st);

  // frame ring (_fb, _nfb, _ifb) and per-estimate scratch
  int16_t fb_[kFrames][cal::kPoints][2];
  int nfb_ = 0, ifb_ = 0;
  float env_[cal::kPoints];
  int32_t tpl_u_[kTplScratch];
  int32_t su_w_[3 * 2 * cal::kSuBins];
  fs_su_ctx su_ctx_;

  // calibration state (n and c0_s self-calibrate; d0, c0 fixed)
  double n_, c0_s_;
  double leak_ref_;                                        // NaN: the first estimate sets it
  // _nbins: one (hb, braw, sraw) per 10-mm level bin, in dict insertion order
  struct NBin { double hb, braw, sraw; };
  NBin nbins_[kMaxNBins];
  bool nbin_used_[kMaxNBins];
  int8_t nbin_order_[kMaxNBins];
  int n_nbins_ = 0;

  // memory and output
  double mem_ = kNaN, stored_ = kNaN;
  MemResult mem_result_;
  double t_now_ = 0.0;
  TrackerCounters cnt_;

  // reset_lock() state
  double prev_b_;
  bool anchored_, cycle_uncertain_, bridging_;
  Ring<double, cal::kVoteN> vote_;
  Ring<double, cal::kAmpN> amp_hist_;
  bool has_last_;
  TrackResult last_;
  Ring<SuVote, kSuVotesN> su_votes_;
  Ring<double, kSuVotesN> su_levels_;
  int su_tick_;
  CycleSource cycle_source_;
  bool su_decided_;
  SuState su_state_;
  BlkItem blk_[cal::kScBlkN];
  int n_blk_, blk_k_;
  double gap_ema_;
  int mem_verify_;
  double leak_ema_, leak_mm_;
  Ring<HistItem, cal::kHistN> hist_;
  bool recheck_;
  int recheck_left_;
  bool recheck_keep_;
  bool has_post_;
  double post_t0_, post_T_;                                // _post = (start, summed hold time)
  bool doubt_;
  bool has_out_k_;
  int out_k_;
  bool out_env_;
  // A checked, separated surface can resolve one floor-cycle discrepancy.
  bool surface_anchor_, surface_conflict_, vote_surface_;

  TrackResult res_;                                        // the value estimate() returns
};

}  // namespace fs
