// TrackerV4 v6.10 port -- see tracker.h. Each function cites the level_v4.py function it mirrors; comments marked "numpy:" explain
// why an expression is written the way it is (the dtype or evaluation order numpy uses, so the bits match).
#include "tracker.h"

#include <cstring>

namespace fs {

namespace {

using namespace cal;

constexpr double kPhaseDegPerMm = 144.0;                       // PHASE_DEG_PER_MM: 360 deg per 2.5 mm of range
constexpr double kLeakWrapMm = 360.0 / kPhaseDegPerMm;         // one turn of the leakage phase (2.5 mm)
constexpr float kDegF = 180.0f / 3.14159265358979323846f;      // numpy float32 degrees: x * (180.0f / NPY_PIf)
constexpr double kDegD = 180.0 / 3.14159265358979323846;       // numpy float64 degrees: x * (180.0 / NPY_PI)

struct C32 { float re, im; };
struct C64 { double re, im; };

// Python's min(a, b) / max(a, b): the first argument unless the second compares strictly smaller / greater (NaN stays first).
inline double py_min(double a, double b) { return b < a ? b : a; }
inline double py_max(double a, double b) { return b > a ? b : a; }

// Exact float32 fused multiply-add on any platform (numpy's SIMD loops use FMA on this host; newlib's fmaf is not exact):
// the double product is exact, the double sum is rounded to odd (TwoSum error), and the final rounding to float32 is then
// correct (53 >= 24 + 2 bits).
float fma32(float x, float y, float z) {
  double p = static_cast<double>(x) * static_cast<double>(y);
  double zd = z, s = p + zd;
  double bv = s - p, e = (p - (s - bv)) + (zd - bv);
  if (e != 0.0) {
    uint64_t u;
    std::memcpy(&u, &s, 8);
    if ((u & 1u) == 0) {
      u = ((e > 0.0) == (s > 0.0)) ? u + 1u : u - 1u;
      std::memcpy(&s, &u, 8);
    }
  }
  return static_cast<float>(s);
}

// numpy complex64 abs on arrays (SIMD loop): sqrt(fma(r, r, 1)) * larger, r = smaller / larger (0 for 0 + 0j).
float cabs_np(float re, float im) {
  re = std::fabs(re);
  im = std::fabs(im);
  float larger = re > im ? re : im, smaller = im < re ? im : re;
  if (larger == 0.0f) return 0.0f;
  float ratio = smaller / larger;
  return std::sqrt(fma32(ratio, ratio, 1.0f)) * larger;
}

// numpy complex64 / float32 (the float32 operand becomes c + 0j; Smith's division as in CFLOAT_divide).
C32 cdiv_real(C32 a, float c) {
  if (std::fabs(c) == 0.0f) return C32{a.re / std::fabs(c), a.im / std::fabs(c)};
  const float in2i = 0.0f;
  const float rat = in2i / c;
  const float scl = 1.0f / (c + in2i * rat);
  return C32{(a.re + a.im * rat) * scl, (a.im - a.re * rat) * scl};
}

// numpy pairwise sum of n complex values (n <= 64: one block of 4 complex accumulators, then the rest in order).
template <typename F, typename C>
void pw_csum(const C* a, int n, F* rr, F* ri) {
  if (n < 4) {
    F r = -0.0, i = -0.0;
    for (int k = 0; k < n; ++k) { r += a[k].re; i += a[k].im; }
    *rr = r; *ri = i;
    return;
  }
  F r0 = a[0].re, i0 = a[0].im, r1 = a[1].re, i1 = a[1].im, r2 = a[2].re, i2 = a[2].im, r3 = a[3].re, i3 = a[3].im;
  int k = 4;
  for (; k < n - (n % 4); k += 4) {
    r0 += a[k].re; i0 += a[k].im; r1 += a[k + 1].re; i1 += a[k + 1].im;
    r2 += a[k + 2].re; i2 += a[k + 2].im; r3 += a[k + 3].re; i3 += a[k + 3].im;
  }
  F sr = (r0 + r1) + (r2 + r3), si = (i0 + i1) + (i2 + i3);
  for (; k < n; ++k) { sr += a[k].re; si += a[k].im; }
  *rr = sr; *ri = si;
}

// np.mean of a complex64 array: float32 pairwise sum, then (complex128) sum * (1 / n) -> complex64 (np.mean divides by an intp).
C32 cmean32(const C32* a, int n) {
  float sr, si;
  pw_csum<float>(a, n, &sr, &si);
  const double scl = 1.0 / static_cast<double>(n);
  return C32{static_cast<float>(static_cast<double>(sr) * scl), static_cast<float>(static_cast<double>(si) * scl)};
}

// np.mean of a complex128 array.
C64 cmean64(const C64* a, int n) {
  double sr, si;
  pw_csum<double>(a, n, &sr, &si);
  const double scl = 1.0 / static_cast<double>(n);
  return C64{sr * scl, si * scl};
}

// numpy pairwise sum of n doubles (8 accumulators per block of 128, recursive above).
double pw_sum(const double* a, int n) {
  if (n < 8) {
    double r = -0.0;
    for (int i = 0; i < n; ++i) r += a[i];
    return r;
  }
  if (n <= 128) {
    double r[8];
    for (int j = 0; j < 8; ++j) r[j] = a[j];
    int i = 8;
    for (; i < n - (n % 8); i += 8)
      for (int j = 0; j < 8; ++j) r[j] += a[i + j];
    double res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
    for (; i < n; ++i) res += a[i];
    return res;
  }
  int n2 = n / 2;
  n2 -= n2 % 8;
  return pw_sum(a, n2) + pw_sum(a + n2, n - n2);
}

// np.std(x) (ddof 0): mean by pairwise sum / n, then the pairwise sum of squared deviations / n. x is overwritten.
double np_std(double* x, int n) {
  const double m = pw_sum(x, n) / n;
  for (int i = 0; i < n; ++i) { const double d = x[i] - m; x[i] = d * d; }
  return std::sqrt(pw_sum(x, n) / n);
}

// Python float % (the result takes the divisor's sign).
double py_mod(double x, double y) {
  double m = std::fmod(x, y);
  if (m != 0.0) {
    if ((y < 0) != (m < 0)) m += y;
  } else {
    m = std::copysign(0.0, y);
  }
  return m;
}

// Python float // (CPython float_floor_div / _float_div_mod).
double py_floordiv(double vx, double wx) {
  double mod = std::fmod(vx, wx);
  double div = (vx - mod) / wx;
  if (mod != 0.0) {
    if ((wx < 0) != (mod < 0)) div -= 1.0;
  }
  double fd;
  if (div != 0.0) {
    fd = std::floor(div);
    if (div - fd > 0.5) fd += 1.0;
  } else {
    fd = std::copysign(0.0, vx / wx);
  }
  return fd;
}

template <typename T>
void insertion_sort(T* a, int n) {
  for (int i = 1; i < n; ++i) {
    T v = a[i];
    int j = i;
    while (j > 0 && a[j - 1] > v) { a[j] = a[j - 1]; --j; }
    a[j] = v;
  }
}

// np.percentile(x, q) of n sorted values, method 'linear': virtual index (n - 1) q, numpy's _lerp (from the upper value at t >= 0.5).
double percentile_sorted(const double* s, int n, double q_pct) {
  const double vi = (n - 1) * (q_pct / 100.0);
  const double fl = std::floor(vi);
  int lo = static_cast<int>(fl);
  if (lo >= n - 1) return s[n - 1];
  if (lo < 0) lo = 0;
  const double g = vi - fl;
  const double a = s[lo], b = s[lo + 1], diff = b - a;
  return g >= 0.5 ? b - diff * (1.0 - g) : a + diff * g;
}

// np.median of n sorted doubles: the middle value, or the mean of the middle two ((a + b) / 2).
double median_sorted(const double* s, int n) {
  return (n % 2) ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2.0;
}

// Least-squares slope of y on x (np.polyfit(x, y, 1) / np.linalg.lstsq with an intercept), closed form on centred data.
// Returns false when the x values do not vary. *intercept is the value at x = 0.
bool line_fit(const double* x, const double* y, int n, double* slope, double* intercept) {
  double sx = 0, sy = 0;
  for (int i = 0; i < n; ++i) { sx += x[i]; sy += y[i]; }
  const double mx = sx / n, my = sy / n;
  double sxx = 0, sxy = 0;
  for (int i = 0; i < n; ++i) { const double dx = x[i] - mx; sxx += dx * dx; sxy += dx * (y[i] - my); }
  if (!(sxx > 0)) return false;
  *slope = sxy / sxx;
  *intercept = my - *slope * mx;
  return true;
}

// Python 3.12+ sum() of floats: the first item, then Neumaier-compensated additions; the compensation is added at the end
// (CPython builtin_sum_impl). R is a Ring of votes and s the slot.
template <typename R>
double py_fsum(const R& votes, int s) {
  const int n = votes.size();
  if (n == 0) return 0.0;
  double f = 0.0 + votes[0].lr[s];                              // 0 (int start) + the first float
  double c = 0.0;
  for (int i = 1; i < n; ++i) {
    const double x = votes[i].lr[s];
    const double t = f + x;
    if (std::fabs(f) >= std::fabs(x)) c += (f - t) + x;
    else c += (x - t) + f;
    f = t;
  }
  if (c != 0.0 && std::isfinite(c)) f += c;
  return f;
}

int argmin_abs_grid(double x) {                               // int(np.argmin(np.abs(self.d - x))): the first minimum
  int i = 0;
  double best = std::fabs(kGrid[0] - x);
  for (int k = 1; k < cal::kPoints; ++k) {
    const double v = std::fabs(kGrid[k] - x);
    if (v < best) { best = v; i = k; }
  }
  return i;
}

}  // namespace

// ------------------------------------------------------------------------------------------------------------------- setup
Tracker::Tracker() { init(); }

void Tracker::init() {
  // __init__, set_calibration, set_shapes (fixed point) and LiveLevel's lpmm: everything from the generated tables
  std::memset(fb_, 0, sizeof(fb_));
  nfb_ = ifb_ = 0;
  n_ = kN;
  c0_s_ = kC0s;
  leak_ref_ = kHasLeakRef ? kLeakRefDeg : kNaN;
  for (int k = 0; k < kMaxNBins; ++k) nbin_used_[k] = false;
  n_nbins_ = 0;
  mem_ = kNaN;
  stored_ = kNaN;
  mem_result_ = MemResult{};
  t_now_ = 0.0;
  cnt_ = TrackerCounters{};
  su_ctx_.wr[0] = kWbRe; su_ctx_.wi[0] = kWbIm;                 // floor
  su_ctx_.wr[1] = kWsRe; su_ctx_.wi[1] = kWsIm;                 // surface
  su_ctx_.wr[2] = kWbRe; su_ctx_.wi[2] = kWbIm;                 // reverberation: the floor shape (fixed-point set_shapes)
  su_ctx_.nw = kShapeN;
  su_ctx_.doff = kShapeDoff;
  su_ctx_.dq = kSuDq;
  su_ctx_.nb = kSuBins;
  su_ctx_.sintab = kSinTab;
  su_ctx_.w = su_w_;
  reset_lock();
}

void Tracker::reset_lock() {                                     // reset_lock()
  nfb_ = 0; ifb_ = 0; prev_b_ = kNaN; anchored_ = false; cycle_uncertain_ = true; bridging_ = false;
  vote_.clear(); amp_hist_.clear(); has_last_ = false;
  su_votes_.clear(); su_levels_.clear(); su_tick_ = 0;
  cycle_source_ = CycleSource::Assumed; su_decided_ = false; su_state_ = SuState::Collecting;
  n_blk_ = 0; blk_k_ = 0; gap_ema_ = kNaN; mem_verify_ = 0;
  leak_ema_ = kNaN; leak_mm_ = 0.0;
  hist_.clear();
  recheck_ = false; recheck_left_ = 0; recheck_keep_ = false; has_post_ = false; post_t0_ = post_T_ = 0.0;
  doubt_ = false;
  has_out_k_ = false; out_k_ = 0; out_env_ = true;
  surface_anchor_ = surface_conflict_ = vote_surface_ = false;
}

double Tracker::output_level() const {                          // output_level(): last estimate -> last confirmed -> stored -> 0
  if (has_last_) return last_.h;
  if (is_set(stored_)) return stored_;
  if (is_set(mem_)) return mem_;
  return 0.0;
}

void Tracker::note_gap(double seconds) {                        // note_gap()
  nfb_ = 0; ifb_ = 0; ++cnt_.gaps; vote_.clear();
  if (seconds > kRestartGapS) {
    const double h = is_set(stored_) ? stored_ : mem_;          // nothing confirmed yet: the stored level again
    reset_lock();
    set_memory(h);
    mem_result_ = MemResult{};
    return;
  }
  if (is_set(prev_b_)) bridging_ = true;
}

// ------------------------------------------------------------------------------------------------------------------- frames
void Tracker::push(const IQ16* iq) {                            // push(): the int16 frame into the ring
  for (int k = 0; k < cal::kPoints; ++k) { fb_[ifb_][k][0] = iq[k].i; fb_[ifb_][k][1] = iq[k].q; }
  ifb_ = (ifb_ + 1) % kFrames;
  nfb_ = nfb_ + 1 < kFrames ? nfb_ + 1 : kFrames;
}

// ------------------------------------------------------------------------------------------------------------------- helpers
double Tracker::snap(double x, double ref) {                    // _snap: x + np.round((ref - x) / step) * step (rint)
  return x + std::nearbyint((ref - x) / kStep) * kStep;
}

double Tracker::tpl_pos(double guess) {                         // _tpl_pos (fixed point, kern.c)
  const int c = fs_tpl_pos(env_, cal::kPoints, kGrid[0], kFineN, guess, kTplWin, kTplQ, kTplLen, kTplDecim, kTplCoarse, kTplIncQ32,
                           tpl_u_, kTplScratch);
  return c < 0 ? guess : kFine0 + c * kFineStep;
}

double Tracker::phase(double x, double* pstd_out) const {       // _phase(Z, x)
  const int i = argmin_abs_grid(x);
  // v = Z[:, i] in ring-slot order, the non-zero samples; u = np.mean(v / np.abs(v))
  C32 v[kFrames];
  int nz = 0;
  for (int f = 0; f < nfb_; ++f) {
    const float re = fb_[f][i][0], im = fb_[f][i][1];
    if (re != 0.0f || im != 0.0f) v[nz++] = cdiv_real(C32{re, im}, cabs_np(re, im));
  }
  if (nz == 0) { *pstd_out = 999.0; return kNaN; }
  const C32 u = cmean32(v, nz);
  // R = abs(u) (numpy scalar abs = hypot rounded to float32; the double square root of the double sum of squares gives the same
  // float32 and uses only IEEE operations); pstd = degrees(sqrt(-2 log(max(R, 1e-9)))) in float32 (numpy: R is float32, the weak
  // Python 1e-9 compares as float32 and is returned as a Python float, i.e. float64 maths)
  const double ur = u.re, ui = u.im;
  const float R = static_cast<float>(std::sqrt(ur * ur + ui * ui));
  double pstd;
  if (R < static_cast<float>(1e-9)) {
    pstd = std::sqrt(-2.0 * std::log(1e-9)) * kDegD;
  } else {
    // numpy's float32 log is its own SIMD polynomial (a few ulp from any libm); log in double rounded to float32 gives the same
    // bits with glibc and newlib, so the host build and the ESP32 agree exactly (libm logf does not: 6 % of values differ)
    const float lg = static_cast<float>(std::log(static_cast<double>(R)));
    pstd = static_cast<double>(std::sqrt(-2.0f * lg) * kDegF);
  }
  const float ang32 = std::atan2(u.im, u.re);                   // np.angle(u): float32
  bool ramp = false;
  double ang64 = 0.0;
  if (nz == kFrames) {                                          // v6.2 ramp model (full buffer, no zero sample)
    C32 w[kFrames];                                             // w = np.roll(Z[:, i], -ifb) / |.|: frames in time order
    for (int k = 0; k < kFrames; ++k) {
      const int f = (k + ifb_) % kFrames;
      const float re = fb_[f][i][0], im = fb_[f][i][1];
      w[k] = cdiv_real(C32{re, im}, cabs_np(re, im));
    }
    C32 pr[kFrames - 1];                                        // w[1:] * conj(w[:-1]): numpy SIMD complex multiply (FMA form)
    for (int k = 0; k < kFrames - 1; ++k) {
      const float ar = w[k + 1].re, ai = w[k + 1].im, br = w[k].re, bi = -w[k].im;
      pr[k] = C32{fma32(ar, br, -(ai * bi)), fma32(ar, bi, ai * br)};
    }
    const C32 ms = cmean32(pr, kFrames - 1);
    const float stp = std::atan2(ms.im, ms.re);                 // mean frame-to-frame phase step (float32)
    const double y = -static_cast<double>(stp);                 // np.exp(-1j * float(stp)) = cos(y) + i sin(y)
    const double er = std::cos(y), ei = std::sin(y);
    C64 prod[kFrames];                                          // w * np.cumprod([1, e, e, ...]): complex128
    double cr = 1.0, ci = 0.0;
    for (int k = 0; k < kFrames; ++k) {
      if (k > 0) {                                              // cumprod: plain complex products
        const double t = cr * er - ci * ei;
        ci = cr * ei + ci * er;
        cr = t;
      }
      const double ar = w[k].re, ai = w[k].im;                  // numpy SIMD complex128 multiply (FMA form)
      prod[k] = C64{std::fma(ar, cr, -(ai * ci)), std::fma(ar, ci, ai * cr)};
    }
    const C64 u2 = cmean64(prod, kFrames);
    const double a2 = std::hypot(u2.re, u2.im);
    const double pd = std::sqrt(-2.0 * std::log(py_max(a2, 1e-9))) * kDegD;
    if (pstd > kPstdMax && kPstdMax >= pd) {                    // only the ramp model makes this estimate clean: its phase
      ramp = true;
      ang64 = std::atan2(u2.im, u2.re) + static_cast<double>((stp * 19.0f) / 2.0f);
    }
    pstd = py_min(pstd, pd);
  }
  *pstd_out = pstd;
  if (ramp) return kGrid[i] + (ang64 * kDegD) / kPhaseDegPerMm;
  // numpy: float32 degrees / 144.0 stays float32 (weak Python float), then added to the float64 bin distance
  return kGrid[i] + static_cast<double>((ang32 * kDegF) / 144.0f);
}

bool Tracker::select_floor(double* b_c, double* b_amp) const {  // _select_floor(env)
  const int i0 = kFloorBin0, i1 = cal::kPoints - 1;                  // idx = where(d >= d0 - 15): to the sweep end
  if (i1 - i0 + 1 < 3) return false;
  int cand[cal::kPoints];
  int nc = 0, ks = -1;
  for (int k = i0 + 1; k < i1; ++k) {
    if (!(env_[k] > env_[k - 1] && env_[k] >= env_[k + 1])) continue;
    bool ok = true;
    for (int j = 0; j < kExN; ++j) if (!(std::fabs(kGrid[k] - kEx[j]) > 6)) ok = false;
    if (!ok) continue;
    cand[nc++] = k;
    if (ks < 0 || env_[k] > env_[ks]) ks = k;                   // max(cands, key=env): the first maximum
  }
  if (nc == 0) {
    int k = i0;
    for (int j = i0 + 1; j <= i1; ++j) if (env_[j] > env_[k]) k = j;
    *b_c = kGrid[k]; *b_amp = env_[k];
    return true;
  }
  if (is_set(prev_b_)) {
    const float thr = 0.35f * env_[ks];                         // numpy: 0.35 * float32 -> float32
    int kn = -1;
    for (int j = 0; j < nc; ++j) {
      const int k = cand[j];
      if (std::fabs(kGrid[k] - prev_b_) <= 15 && env_[k] >= thr && (kn < 0 || env_[k] > env_[kn])) kn = k;
    }
    if (kn >= 0) ks = kn;
  }
  *b_c = kGrid[ks]; *b_amp = env_[ks];
  return true;
}

double Tracker::select_surface(double b_env, double b_amp) {    // _select_surface(env, b_env, b_amp)
  const double lo = kSurfaceMin, hi = b_env - kMinSep;
  if (hi <= lo) return kNaN;
  int m0 = -1, m1 = -1;                                         // m = (d >= lo) & (d <= hi): contiguous bins m0..m1
  for (int k = 0; k < cal::kPoints; ++k) if (kGrid[k] >= lo && kGrid[k] <= hi) { if (m0 < 0) m0 = k; m1 = k; }
  if (m0 < 0) return kNaN;                                      // (no bins: np.median is nan and no candidate exists)
  const int cnt = m1 - m0 + 1;
  float srt[cal::kPoints];
  for (int k = 0; k < cnt; ++k) srt[k] = env_[m0 + k];
  insertion_sort(srt, cnt);
  const float floor32 = (cnt % 2) ? srt[cnt / 2] : (srt[cnt / 2 - 1] + srt[cnt / 2]) / 2.0f;   // np.median of float32
  const int bi = argmin_abs_grid(b_env);
  // max(4 * floor, 0.15 * b_amp): numpy compares the Python float as float32, and env[k] < it compares in float32 too
  const float thr_a = 4.0f * floor32, thr_b = static_cast<float>(0.15 * b_amp);
  const float thr = thr_b > thr_a ? thr_b : thr_a;
  int best = -1;
  for (int k = m0 + 1; k < m1; ++k) {                           // idx[1:-1]
    if (!(env_[k] > env_[k - 1] && env_[k] >= env_[k + 1])) continue;
    bool ex = false;
    for (int j = 0; j < kExN; ++j) if (std::fabs(kGrid[k] - kEx[j]) < 6) ex = true;
    if (ex) continue;
    if (env_[k] < thr) continue;
    if (k + 1 >= bi) continue;                                  // (env[k+1:bi] empty: impossible, k is >= 30 mm before b_env)
    float mn = env_[k + 1];
    for (int j = k + 2; j < bi; ++j) if (env_[j] < mn) mn = env_[j];
    if (mn > 0.6f * env_[k]) continue;                          // no dip between it and the floor echo: its skirt
    if (best < 0 || env_[k] > env_[best]) best = k;
  }
  if (best < 0) return kNaN;
  return tpl_pos(kGrid[best]);
}

double Tracker::leak_update() {                                 // _leak_update(Z)
  if (kLeakBin < 0) return 0.0;
  C32 v[kFrames];
  for (int f = 0; f < nfb_; ++f) {                              // v / np.maximum(np.abs(v), 1e-6) (float32)
    const float re = fb_[f][kLeakBin][0], im = fb_[f][kLeakBin][1];
    float a = cabs_np(re, im);
    const float lim = static_cast<float>(1e-6);
    a = (a >= lim) ? a : lim;
    v[f] = cdiv_real(C32{re, im}, a);
  }
  const C32 u = cmean32(v, nfb_);
  const double ph = static_cast<double>(std::atan2(u.im, u.re) * kDegF);   // float(np.degrees(np.angle(u)))
  // rev4: only a coherent leakage phase over the frame ring (|mean phasor| >= 0.9; 0.984-1.0 on every recording, 1e-7 for random phases) sets the reference
  // or moves the mean (random phases would random-walk the unwrapped mean below); none yet: no correction
  const bool coherent = static_cast<double>(u.re) * u.re + static_cast<double>(u.im) * u.im >= 0.81;
  if (!is_set(leak_ref_) && coherent) leak_ref_ = ph;           // no install reference: this power-up is the reference
  if (!is_set(leak_ref_)) return 0.0;
  double dmm = (py_mod(ph - leak_ref_ + 180.0, 360.0) - 180.0) / kPhaseDegPerMm;
  // rev4: over months the electronics phase can drift past +-180 deg (1.25 mm). The mean of the wrapped value would then run through
  // 0 over minutes and move the carried floor phase by a whole 2.5 mm (one floor cycle, confirmed). Take the alias nearest the running
  // mean instead (unchanged while |dmm - ema| < 1.25 mm, i.e. always on the recordings).
  if (is_set(leak_ema_)) dmm += kLeakWrapMm * std::nearbyint((leak_ema_ - dmm) / kLeakWrapMm);
  const double a = 1.0 / kLeakN;
  if (coherent) leak_ema_ = !is_set(leak_ema_) ? dmm : (1 - a) * leak_ema_ + a * dmm;
  if (!is_set(leak_ema_)) return 0.0;
  leak_mm_ = leak_ema_;
  const double w = py_min(py_max((std::fabs(leak_ema_) - 0.5 * kLeakDeadbandMm) / kLeakDeadbandMm, 0.0), 1.0);
  if (w > 0) { ++cnt_.leak_corrections; return w * leak_ema_; }
  return 0.0;
}

// ------------------------------------------------------------------------------------------------------------------- start-up check
bool Tracker::su_vote(double b_fine, double corr, SuVote* out) {   // _su_vote (fixed point): {j: log(residual / best)}
  int32_t zr[kSuBins], zi[kSuBins];                             // int32 frame sum over the fit window (|sum| <= 20 x 32768)
  for (int b = 0; b < kSuBins; ++b) {
    int32_t sr = 0, si = 0;
    for (int f = 0; f < nfb_; ++f) { sr += fb_[f][kSuBin0 + b][0]; si += fb_[f][kSuBin0 + b][1]; }
    zr[b] = sr; zi[b] = si;
  }
  double res[kSuSlots];
  const int nj = fs_su_vote(&su_ctx_, zr, zi, kD0, n_, kC0 - corr, kStep, b_fine, kSuMaxH, kSuWinLo, kSuWinHi, kHs, kHsN, res, kSuSlots);
  ++cnt_.su_votes_run;
  cnt_.su_levels_run += static_cast<uint32_t>(fs_kern_stats.su_nres);
  double m = 0.0;
  bool any = false;
  for (int s = 0; s < nj; ++s) if (res[s] >= 0 && (!any || res[s] < m)) { m = res[s]; any = true; }
  if (!any) return false;                                       // no candidate level in range: None
  if (!(m > 0)) return false;                                   // all-zero frames (Python: ZeroDivisionError) -> no vote
  out->mask = 0;
  for (int s = 0; s < kSuSlots; ++s) {
    if (s < nj && res[s] >= 0) { out->mask |= 1u << s; out->lr[s] = std::log(res[s] / m); }
    else out->lr[s] = 0.0;
  }
  return true;
}

bool Tracker::su_best(double b, int* k_out, double* lead) const {   // _su_best(b)
  uint32_t keys = ~0u;
  for (int i = 0; i < su_votes_.size(); ++i) keys &= su_votes_[i].mask;
  if (su_votes_.empty() || !keys) return false;
  double score[kSuSlots];
  int kbest = -1, nk = 0;
  for (int s = 0; s < kSuSlots; ++s) {
    if (!(keys & (1u << s))) continue;
    const double sum = py_fsum(su_votes_, s);                   // sum(q[j] for q in votes): oldest vote first
    score[s] = sum;
    ++nk;
    if (kbest < 0 || sum < score[kbest]) kbest = s;             // (a tie at the minimum has lead 0: it decides nothing)
  }
  const int k = kbest - 4;
  if (k != 0 && h_bottom(b + k * kStep) + kSuWinLo < 0.0) return false;   // the fit window reaches below the floor: no evidence
  double first = 0, second = 0;
  bool h1 = false, h2 = false;                                  // the two smallest scores
  for (int s = 0; s < kSuSlots; ++s) {
    if (!(keys & (1u << s))) continue;
    const double v = score[s];
    if (!h1 || v < first) { second = first; h2 = h1; first = v; h1 = true; }
    else if (!h2 || v < second) { second = v; h2 = true; }
  }
  *k_out = k;
  *lead = nk > 1 ? second - first : __builtin_inf();
  return true;
}

// ------------------------------------------------------------------------------------------------------------------- memory
double Tracker::apply_memory(double xb, double b_assumed, double h_env_b) {   // _apply_memory
  const double hm = mem_, nm1 = n_ - 1;
  const double b_pred = kD0 + nm1 * hm, b_m = snap(xb, b_pred), r = (b_m - b_pred) / nm1;
  const double tol = h_env_b >= kEnvTrust ? kMemEtol : kMemEnvGuard;
  const bool env_ok = std::fabs(h_env_b - (hm + r)) <= tol;
  const bool ok = std::fabs(r) <= kMemTau && env_ok;
  mem_result_ = MemResult{true, r, h_env_b, env_ok, ok};
  if (!ok) {                                                    // r8a: rejected by the phase residual alone -> the output keeps
    if (env_ok && std::fabs(r) * nm1 <= kAmbMax * kStep) {      // that cycle (the floor envelope may still move it)
      has_out_k_ = true;
      out_k_ = static_cast<int>(std::nearbyint((b_m - b_assumed) / kStep));
    }
    return b_assumed;
  }
  cycle_source_ = CycleSource::Memory;
  cycle_uncertain_ = h_env_b < kEnvTrust;
  mem_verify_ = cycle_uncertain_ ? kMemVerifyVotes : 0;
  if (mem_verify_ == 0) su_decided_ = true;
  return b_m;
}

// ------------------------------------------------------------------------------------------------------------------- holds and gaps
double Tracker::rate_pre() const {                              // _rate_pre(): the pre-hold rate of a sustained ramp, else 0
  const double tl = hist_.back().t;
  double tt[cal::kHistN], hh[cal::kHistN];
  int m = 0;
  for (int i = 0; i < hist_.size(); ++i) {
    if (hist_[i].t >= tl - 30.0) { tt[m] = hist_[i].t; hh[m] = hist_[i].h; ++m; }
  }
  if (m < 12 || tt[m - 1] - tt[0] < 20.0) return 0.0;
  const double t_end = tt[m - 1];
  for (int i = 0; i < m; ++i) tt[i] = tt[i] - t_end;
  double b, a;
  if (!line_fit(tt, hh, m, &b, &a)) return 0.0;                 // np.polyfit(tt, h, 1)
  double r[cal::kHistN];
  for (int i = 0; i < m; ++i) r[i] = hh[i] - (a + b * tt[i]);
  const double s = np_std(r, m);
  return std::fabs(b) * (tt[m - 1] - tt[0]) > 3.0 * s + 0.3 ? b : 0.0;
}

void Tracker::hold_spread(double spread) {                      // _hold_spread(spread)
  if (!(spread > kHoldRecheckMm)) return;
  su_votes_.clear(); su_levels_.clear(); has_post_ = false;
  if (cycle_uncertain_ && !su_decided_) {
    if (mem_verify_) mem_verify_ = kMemVerifyVotes;             // a stored-level check gets its 60 votes again
  } else {
    recheck_keep_ = (recheck_ && recheck_keep_) || spread > 0.5 * kStep / (n_ - 1);   // past half a cycle
    recheck_ = true;
    recheck_left_ = kMemVerifyVotes * kSuEvery;
    ++cnt_.rechecks;
  }
}

void Tracker::end_recheck(bool flag) {                          // _end_recheck(flag)
  recheck_ = false;
  if (!flag) return;
  doubt_ = true; cycle_uncertain_ = true; anchored_ = false; su_decided_ = true;
  su_votes_.clear(); su_levels_.clear();
}

// ------------------------------------------------------------------------------------------------------------------- self-calibration
void Tracker::sc_block() {                                      // _sc_block(): one 10-s block
  const int m = n_blk_;
  n_blk_ = 0;
  if (m < 0.8 * kScBlkN) return;
  double col[4][cal::kScBlkN];
  for (int i = 0; i < m; ++i) { col[0][i] = blk_[i].hb; col[1][i] = blk_[i].gap; col[2][i] = blk_[i].braw; col[3][i] = blk_[i].sraw; }
  for (int c = 0; c < 4; ++c) insertion_sort(col[c], m);
  if (percentile_sorted(col[0], m, 90) - percentile_sorted(col[0], m, 10) > 0.3) return;
  const double hb = median_sorted(col[0], m), gap = median_sorted(col[1], m);
  const double braw = median_sorted(col[2], m), sraw = median_sorted(col[3], m);
  gap_ema_ = !is_set(gap_ema_) ? gap : 0.9 * gap_ema_ + 0.1 * gap;
  const double c = c0_s_ + kScAlpha * gap, lo = kC0s - kScLimZero, hi = kC0s + kScLimZero;   // np.clip
  c0_s_ = c < lo ? lo : (c > hi ? hi : c);
  ++cnt_.sc_updates;
  const int k = static_cast<int>(py_floordiv(hb, kScBinMm));    // one point per 10-mm level bin (the latest)
  if (k < 0 || k >= kMaxNBins) return;                          // (h_b >= 45 and below the sweep end: never)
  if (!nbin_used_[k]) { nbin_used_[k] = true; nbin_order_[n_nbins_++] = static_cast<int8_t>(k); }
  nbins_[k] = NBin{hb, braw, sraw};
  if (n_nbins_ < 3) return;
  double xs[kMaxNBins], ys[kMaxNBins], hmin = 0, hmax = 0;
  for (int i = 0; i < n_nbins_; ++i) {                          // dict insertion order
    const NBin& q = nbins_[nbin_order_[i]];
    xs[i] = q.sraw; ys[i] = q.braw;
    if (i == 0 || q.hb < hmin) hmin = q.hb;
    if (i == 0 || q.hb > hmax) hmax = q.hb;
  }
  if (!(hmax - hmin >= kScNSpan)) return;
  double slope, icpt;                                           // the two-echo line b = n d0 - (n - 1) s: n = 1 - slope
  if (!line_fit(xs, ys, n_nbins_, &slope, &icpt)) return;
  const double n_fit = 1.0 - slope;
  if (std::fabs(n_fit - kN) <= kScLimN) { n_ += kScBeta * (n_fit - n_); ++cnt_.n_updates; }
}

// ------------------------------------------------------------------------------------------------------------------- main
const TrackResult& Tracker::fault(TrackState st) {              // dict(h=None, channel='none', state='none' | 'fault')
  res_ = TrackResult{};
  res_.state = st;
  return res_;
}

const TrackResult& Tracker::estimate(double now) {              // estimate()
  t_now_ = now;
  const int out_prev = has_out_k_ ? out_k_ : 0;                // fuel ledger: the output's cycle offset before this estimate
  if (nfb_ < 5) return fault(TrackState::NoLevel);
  const double nm1 = n_ - 1;                                    // (n as at the start of this estimate, as in Python)
  fs_env_median(&fb_[0][0][0], nfb_, cal::kPoints, env_);
  double b_c, b_amp;
  if (!select_floor(&b_c, &b_amp)) return fault(TrackState::Fault);
  const double b_env = tpl_pos(b_c);
  const double s_env = select_surface(b_env, b_amp);
  const double corr = leak_update();
  const double xref = (is_set(prev_b_) && std::fabs(prev_b_ - b_env) < 12) ? prev_b_ : b_env;
  double pstd;
  double xb = phase(xref, &pstd);
  amp_hist_.push(b_amp);
  double amps[cal::kAmpN];
  for (int i = 0; i < amp_hist_.size(); ++i) amps[i] = amp_hist_[i];
  insertion_sort(amps, amp_hist_.size());
  const bool amp_ok = b_amp >= kAmpMin * median_sorted(amps, amp_hist_.size());
  const bool disturbed = !is_set(xb) || pstd > kPstdMax || !amp_ok;
  if (is_set(xb)) xb = xb + kC0 - corr;
  const double h_env_s = is_set(s_env) ? kD0 - s_env : kNaN;
  const double h_env_b = h_bottom(b_env);
  const bool b_fits = b_env < kBFitsLimit;                      // the template fits inside the sweep
  // A separated surface may resolve ONE floor-cycle discrepancy, but only when
  // its own phase/envelope and the floor phase independently agree. Do not relax
  // the coarse 4-mm gate to accept an arbitrary echo or a multi-cycle mismatch.
  bool surface_ref = false;
  double surface_h = kNaN;
  if (is_set(s_env) && h_env_s >= kEnvTrust && b_fits && is_set(xb) && pstd < 8.0) {
    double ps;
    const double xs = phase(s_env, &ps);
    if (is_set(xs) && ps < 8.0) {
      const double hs = kD0 - snap(xs - corr + c0_s_, s_env);
      const double bs = snap(xb, kD0 + nm1 * hs);
      const double hs_floor = h_bottom(bs);
      const double dk = (bs - snap(xb, b_env)) / kStep;
      if (std::fabs(hs - h_env_s) <= kEnvTol && std::fabs(hs - hs_floor) <= 1.0 && std::fabs(dk) < 1.5 &&
          (surface_anchor_ || surface_conflict_ || std::fabs(h_env_s - h_env_b) >= 4.0)) {
        surface_ref = true;
        surface_h = hs_floor;
      }
    }
  }
  double ref_h = kNaN;
  bool sep_ok = is_set(h_env_s) && h_env_s >= 35.0;
  if (sep_ok && h_env_s < kEnvTrust && is_set(xb))
    sep_ok = std::fabs(snap(xb, kD0 + nm1 * h_env_s) - snap(xb, b_env)) < 0.5 * kStep;
  if (surface_ref) {
    ref_h = surface_h;
    if (is_set(prev_b_)) {
      const double displayed = prev_b_ + (has_out_k_ ? out_k_ * kStep : 0.0);
      if (std::fabs(snap(xb, kD0 + nm1 * ref_h) - snap(xb, displayed)) > 0.5 * kStep)
        surface_conflict_ = true;  // a pending disagreement must not be labelled confirmed
    }
  } else if (sep_ok && (h_env_s < kEnvTrust || (b_fits && std::fabs(h_env_s - h_env_b) < 4.0))) {
    ref_h = h_env_s < kEnvTrust ? h_env_s : h_env_b;
  } else if (!is_set(s_env) && h_env_b >= kEnvTrust && b_fits && !surface_anchor_ && !surface_conflict_) {
    ref_h = h_env_b;
  }
  // After a checked surface anchor, a coarse floor observation cannot undo its
  // cycle when the surface momentarily disappears or its candidate changes.
  if (!surface_ref && (surface_anchor_ || surface_conflict_) && is_set(ref_h) && is_set(prev_b_) &&
      std::fabs(snap(xb, kD0 + nm1 * ref_h) - snap(xb, prev_b_)) > 0.5 * kStep) ref_h = kNaN;
  if (is_set(ref_h)) {
    if (surface_ref != vote_surface_) vote_.clear();
    vote_surface_ = surface_ref;
    vote_.push(ref_h);
  } else {
    vote_.clear();
  }
  if (disturbed) {                                              // splash / slosh: hold the output, keep voting
    if (!has_last_) return fault(TrackState::Fault);
    ++cnt_.bridged;
    bridging_ = true;
    res_ = last_;
    res_.disturbed = true;
    res_.state = TrackState::Holding;
    res_.cycle_step = 0;
    return res_;
  }
  const int need = (anchored_ || surface_ref) ? kVoteN : kFirstN;
  double med = kNaN;
  if (vote_.size() >= need) {                                   // a steady vote: 10-90 % spread below steady_mm
    double v[cal::kVoteN];
    const int nv = vote_.size();
    for (int i = 0; i < nv; ++i) v[i] = vote_[i];
    insertion_sort(v, nv);
    if (percentile_sorted(v, nv, 90) - percentile_sorted(v, nv, 10) < kSteadyMm) med = median_sorted(v, nv);
  }
  if (is_set(med)) {                                            // a steady vote between two floor cycles decides nothing
    const double r = (kD0 + nm1 * med - xb) / kStep;
    if (std::fabs(r - std::nearbyint(r)) > kAmbMax) med = kNaN;
  }
  const bool in_window = h_env_b < kEnvTrust;                   // the start-up check's window
  if (bridging_) {                                              // first clean estimate after a hold or a gap
    bridging_ = false;
    const double T = hist_.empty() ? 0.0 : now - hist_.back().t;   // (hist is never empty here: set by an earlier estimate)
    if (T > 0) {                                                // carry the level across at its earlier rate ...
      const double rate = rate_pre();
      prev_b_ = prev_b_ + nm1 * rate * T;
      if (!has_post_) { has_post_ = true; post_t0_ = now; post_T_ = T; }   // ... tested again post_s later
      else post_T_ = post_T_ + T;
      hold_spread(py_max(2.0 * std::fabs(rate), kRatePlausEff) * T);
    }
  } else if (has_post_ && now - post_t0_ >= kPostS) {           // r5c: a faster fill shows after the hold
    int i0 = -1, i1 = -1, np_ = 0;                              // P = the _hist entries since the hold
    for (int i = 0; i < hist_.size(); ++i) if (hist_[i].t >= post_t0_) { if (i0 < 0) i0 = i; i1 = i; ++np_; }
    if (np_ >= 3)
      hold_spread(2.0 * std::fabs(hist_[i1].h - hist_[i0].h) / (hist_[i1].t - hist_[i0].t) * post_T_);
    has_post_ = false;
  }
  // --- floor cycle: carried by phase continuity, (re)anchored by the vote, seeded by the checked memory
  const double b_cont = is_set(prev_b_) ? snap(xb, prev_b_) : kNaN;
  double b_fine;
  if (!anchored_) {
    if (is_set(ref_h) && (is_set(med) || (cycle_source_ != CycleSource::Shape && cycle_source_ != CycleSource::Memory))) {
      const double cand = snap(xb, kD0 + nm1 * (is_set(med) ? med : ref_h));
      // v6.12: a single coarse reference cannot re-seed a carried track. Wait for
      // the median vote; keep collecting it while waiting. The inherited 1.2.5
      // guard also prevents a first anchor from overriding a Shape/Memory track.
      if (is_set(b_cont) && std::fabs(cand - b_cont) > 0.5 * kStep
          && (!is_set(med) || (!surface_ref && (cycle_source_ == CycleSource::Shape || cycle_source_ == CycleSource::Memory)))) {
        b_fine = b_cont;
        if (is_set(med)) vote_.clear();
      } else {
        b_fine = cand;
        if (ref_h < kEnvTrust) { has_out_k_ = false; out_env_ = false; }   // r8a: the SURFACE echo set the cycle
        if (is_set(med)) {
          anchored_ = true; cycle_uncertain_ = false; cycle_source_ = CycleSource::Anchored; doubt_ = false; mem_verify_ = 0;
          if (surface_ref) { surface_anchor_ = true; surface_conflict_ = false; }
        }
      }
    } else if (is_set(prev_b_)) {
      b_fine = b_cont;
    } else {                                                    // first estimate: near-empty assumption or the checked memory
      b_fine = snap(xb, kD0 + nm1 * 2.7);
      if (is_set(mem_)) b_fine = apply_memory(xb, b_fine, h_env_b);
    }
  } else {
    b_fine = b_cont;
    if (is_set(med) && std::fabs(snap(xb, kD0 + nm1 * med) - b_fine) > 0.5 * kStep) {   // the steady vote picks another cycle
      b_fine = snap(xb, kD0 + nm1 * med);
      ++cnt_.resnaps;
      vote_.clear();
    }
    if (is_set(med) && surface_ref) {
      surface_anchor_ = true; surface_conflict_ = false;
      cycle_uncertain_ = false; doubt_ = false; mem_verify_ = 0;
    }
  }
  if (h_bottom(b_fine) < kBoundLo) {                            // below the floor is impossible: back to the near-empty assumption
    const double b0 = b_fine;
    b_fine = snap(xb, kD0 + nm1 * 2.7);
    cycle_uncertain_ = true; anchored_ = false; mem_verify_ = 0;
    if (has_out_k_) out_k_ -= static_cast<int>(std::nearbyint((b_fine - b0) / kStep));   // r8d: the output keeps its cycle
    out_env_ = true;
    cycle_source_ = CycleSource::Assumed; su_decided_ = false; ++cnt_.bounds_hit;
    recheck_ = false; doubt_ = false;
  }
  if (is_set(b_cont) && std::fabs(b_fine - b_cont) > 0.5 * kStep) {   // a re-seed voids the votes (they count from the carried cycle)
    su_votes_.clear(); su_levels_.clear();
  }
  // --- start-up check below env_trust while not anchored; also checks a stored level (verifying) and a carried level (rechecking)
  const bool verifying = mem_verify_ > 0, rechecking = recheck_;
  if (in_window && (!anchored_ || rechecking)) {
    if (++su_tick_ == kSuEvery) su_tick_ = 0;                  // (kept modulo kSuEvery: a plain int count overflows after 2^31 estimates)
    if (su_tick_ == 0) {
      SuVote ev;
      // rev4: the start-up fit sees the raw frames, so an electronics drift beyond kSuLeakMm (0.15 mm, where leak_update()
      // reaches full correction) must enter its floor phase too; below that it is left out (always applying it changed Exp12's
      // marginal cold start: 416 -> 1306 s value-wrong). Rounds 2-3: 0.5 and then 0.2 left a drift band out, which moved
      // cold starts at 8-30 mm one or more cycles low, confirmed for hours
      if (su_vote(b_fine, std::fabs(leak_mm_) > kSuLeakMm ? corr : 0.0, &ev)) { su_votes_.push(ev); su_levels_.push(h_bottom(b_fine)); }
      const int nv = su_votes_.size();
      const double need_lead = (rechecking || nv < kSuVotesN) ? kSuEarlyLead : kSuMinLead;
      int k = 0;
      double lead = 0;
      if (verifying || rechecking) {
        if (verifying) --mem_verify_;
        const bool best = nv >= kSuMinVotes && su_best(b_fine, &k, &lead);
        if (best && lead >= need_lead) {
          if (k != 0 && !verifying) {                           // a re-check confirms or flags, it never moves the level
            ++cnt_.recheck_disagree;
            end_recheck(true);
          } else {
            if (k != 0) {                                       // (verifying) a stored level is overruled
              b_fine = b_fine + k * kStep; ++cnt_.mem_overruled;
              cycle_source_ = CycleSource::Shape; su_decided_ = true; su_votes_.clear(); su_levels_.clear();
            }
            mem_verify_ = 0; cycle_uncertain_ = false; doubt_ = false; recheck_ = false;   // clear lead: confirmed
          }
        }
        if (verifying && mem_verify_ <= 0) su_decided_ = true;   // stored level kept -> later re-decisions
      } else if (nv >= (su_decided_ ? kSuVotesN : kSuMinVotes)) {
        const bool first = !su_decided_;                        // first decision, or a re-decision on still fuel
        const bool best = su_best(b_fine, &k, &lead);
        double lmin = su_levels_[0], lmax = su_levels_[0];
        for (int i = 1; i < su_levels_.size(); ++i) { lmin = py_min(lmin, su_levels_[i]); lmax = py_max(lmax, su_levels_[i]); }
        const bool stdy = (lmax - lmin) <= kSuSteadyMm;
        if (best && (stdy || first) && lead >= need_lead) {
          const double h_new = h_bottom(b_fine + k * kStep), h_cur = h_bottom(b_fine);
          // v6.12: the first fit must also respect the level already displayed. The internal
          // assumed track may be near zero while out_k_ puts the display on an envelope cycle.
          // Even k == 0 can jump the display when choosing Shape clears that output offset.
          const double h_display = h_bottom(b_fine + (has_out_k_ ? out_k_ * kStep : 0.0));
          const bool first_ok = h_env_b < kEnvOutLo ||
              (std::fabs(h_new - h_env_b) <= kEnvTol &&
               std::fabs(h_new - h_env_b) <= std::fabs(h_display - h_env_b));
          if ((first && first_ok) || (!first && (k == 0 ||
              (h_cur >= kSuRelMm && h_new >= kSuRelMm &&
               (h_env_b < kEnvOutLo || std::fabs(h_new - h_env_b) < std::fabs(h_cur - h_env_b)))))) {
            if (k != 0) {
              b_fine = b_fine + k * kStep; ++cnt_.shape_resnaps; su_votes_.clear(); su_levels_.clear();
            }
            cycle_source_ = CycleSource::Shape; su_decided_ = true; doubt_ = false;   // (a lead it cannot follow confirms nothing)
          }
        }
      }
      su_state_ = su_decided_ ? SuState::Decided : (su_votes_.size() >= kSuVotesN ? SuState::Undecided : SuState::Collecting);
    }
  }
  if (recheck_) {                                               // a steady anchor vote confirms; no clear lead in 120 s: inconclusive
    --recheck_left_;
    if (is_set(med)) recheck_ = false;
    else if (recheck_left_ <= 0) end_recheck(recheck_keep_);
  }
  prev_b_ = b_fine;
  const double h_b = h_bottom(b_fine);
  if (is_set(b_cont) && std::fabs(b_fine - b_cont) > 0.5 * kStep) hist_.clear();   // a cycle change is not a level rate
  hist_.push(HistItem{now, h_b});
  // --- surface channel: cycle from the floor level; checks it, feeds the self-calibration
  double h_s = kNaN, s_fine = kNaN;
  if (is_set(s_env) && h_env_s >= kAnchorLo) {
    double pstd_s;
    const double xs = phase(s_env, &pstd_s);
    if (is_set(xs)) {
      s_fine = snap(xs - corr + c0_s_, kD0 - h_b);
      h_s = kD0 - s_fine;
      if (std::fabs(h_env_s - h_s) > kEnvTol) { ++cnt_.env_conflicts; h_s = kNaN; }
    }
  }
  // --- A2/A3 self-calibration sample
  if (++blk_k_ == kScBlkN) blk_k_ = 0;                          // (modulo kScBlkN: no signed overflow after 2^31 estimates, 27 years)
  if (anchored_ && !recheck_ && is_set(h_s) && h_b >= kScGateH && pstd < 20 && n_blk_ < cal::kScBlkN)
    blk_[n_blk_++] = BlkItem{h_b, h_s - h_b, b_fine - kC0, s_fine - c0_s_};
  if (blk_k_ == 0) sc_block();
  const TrackState state = (!surface_conflict_ && !recheck_ && !doubt_ && (!cycle_uncertain_ || cycle_source_ == CycleSource::Shape))
                               ? TrackState::Confirmed : TrackState::Uncertain;   // a confidence-gated decision counts as confirmed
  if (state == TrackState::Confirmed) stored_ = h_b;
  const bool check = surface_conflict_ || (is_set(gap_ema_) && std::fabs(gap_ema_) > kGapAlarmMm) || std::fabs(leak_mm_) > 0.5;
  double h_out = h_b;                                           // while assumed, seed the display from the envelope once
  if (cycle_source_ != CycleSource::Assumed) {
    has_out_k_ = false;
  } else if (!has_out_k_ && out_env_ && h_env_b >= kEnvOutLo) {
    const double r = (kD0 + nm1 * h_env_b - b_fine) / kStep;
    // v6.12: seed once, then carry by phase. A noisy envelope is not new cycle evidence.
    // Shape/anchor/memory decisions still establish an absolute cycle through their gates.
    has_out_k_ = true; out_k_ = static_cast<int>(std::nearbyint(r));
  }
  if (has_out_k_) h_out = h_bottom(b_fine + out_k_ * kStep);
  TrackResult& L = last_;
  L = TrackResult{};
  L.has_h = true;
  L.h = h_out;
  L.channel = h_out > 15 ? Channel::Bottom : Channel::NearEmpty;
  L.state = state;
  L.check = check;
  L.s = s_fine; L.b = b_fine; L.s_env = s_env; L.b_env = b_env;
  L.cycle_uncertain = cycle_uncertain_ || recheck_ || surface_conflict_;
  L.cycle_source = cycle_source_;
  L.su_state = su_state_;
  L.h_b = h_b; L.h_s = h_s; L.pstd = pstd; L.disturbed = false;
  L.n = n_; L.c0_s = c0_s_; L.leak_mm = leak_mm_; L.gap_ema = gap_ema_;
  L.verifying = mem_verify_ > 0 && in_window;
  L.recheck = recheck_;
  L.doubt = doubt_ || surface_conflict_;
  L.cont = is_set(b_cont);                                      // fuel ledger: the phase-continuity move of the reported level
  L.cycle_step = L.cont ? static_cast<int>(std::nearbyint((b_fine - b_cont) / kStep)) + ((has_out_k_ ? out_k_ : 0) - out_prev) : 0;
  has_last_ = true;
  res_ = last_;
  return res_;
}

}  // namespace fs
