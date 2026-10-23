// Fuel ledger -- see ledger.h. Line-for-line port of dev/fuel_ledger.py (comments there); integer arithmetic only.
#include "ledger.h"

#include <cmath>
#include <cstddef>
#include <cstring>

#include "calib_data.h"

namespace fs {

namespace {

// round(a / b) half away from zero, b > 0 (fuel_ledger.rdiv)
int64_t rdiv(int64_t a, int64_t b) { return a >= 0 ? (a + b / 2) / b : -((-a + b / 2) / b); }

// lower median of 0 < n <= 32 values (fuel_ledger.lmed); v is not modified
constexpr int kLmedMax = 32;
int32_t lmed(const int32_t* v, int n) {
  int32_t s[kLmedMax];
  for (int i = 0; i < n; ++i) {
    int32_t x = v[i];
    int j = i;
    while (j > 0 && s[j - 1] > x) { s[j] = s[j - 1]; --j; }
    s[j] = x;
  }
  return s[(n - 1) / 2];
}

int64_t iabs(int64_t x) { return x < 0 ? -x : x; }

void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
void put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i)); }
void put64(uint8_t* p, uint64_t v) { for (int i = 0; i < 8; ++i) p[i] = uint8_t(v >> (8 * i)); }
uint16_t get16(const uint8_t* p) { return uint16_t(p[0] | p[1] << 8); }
uint32_t get32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
uint64_t get64(const uint8_t* p) { return uint64_t(get32(p)) | uint64_t(get32(p + 4)) << 32; }

constexpr int64_t kD0Nm = static_cast<int64_t>(cal::kD0 * 1e6 + 0.5);   // 232769935 (the run's cycle count from h, n, pb)
constexpr int32_t kNm1Min = 100000, kNm1Max = 1000000;                  // n within [1.1, 2]
bool nm1_ok(int32_t nm1) { return nm1 >= kNm1Min && nm1 <= kNm1Max; }

}  // namespace

// ------------------------------------------------------------------------------------------------------------------- adapter
LedgerInput LedgerAdapter::convert(const Estimate& e) {
  LedgerInput in;
  in.t_ms = static_cast<int64_t>(std::floor(e.t_s * 1000.0 + 0.5));
  if (!(e.h_mm == e.h_mm) || !(e.n >= 1.0 && e.n <= 2.0) ||
      !nm1_ok(static_cast<int32_t>(std::floor((e.n - 1.0) * 1e6 + 0.5)))) {   // (n = 1 or NaN would divide by zero)
    in.kind = LedgerInput::kNoLevel;
    return in;
  }
  if (e.state == TrackState::Holding) { in.kind = LedgerInput::kHold; in.cont = true; return in; }
  if (!e.cont) k_ = 0;
  k_ += e.cycle_step;
  const double b = cal::kD0 + (e.n - 1.0) * e.h_mm;        // the reported floor position (range mm)
  in.kind = LedgerInput::kLevel;
  in.cont = e.cont;
  in.pb_um = static_cast<int32_t>(std::floor((b - static_cast<double>(k_) * cal::kStep) * 1000.0 + 0.5));
  in.h_um = static_cast<int32_t>(std::floor(e.h_mm * 1000.0 + 0.5));
  in.nm1_ppm = static_cast<int32_t>(std::floor((e.n - 1.0) * 1e6 + 0.5));
  in.conf = e.state == TrackState::Confirmed;
  in.ac = in.conf && e.cycle == CycleSource::Anchored;
  return in;
}

// ------------------------------------------------------------------------------------------------------------------- record
void ledger_encode(const LedgerRecord& r, uint8_t tokens, uint8_t out[kLedgerBlobBytes]) {
  std::memset(out, 0, kLedgerBlobBytes);
  out[0] = kLedgerVersion;
  out[1] = tokens;
  put16(out + 2, r.gen);
  put32(out + 4, r.n_fill);
  put32(out + 8, r.n_drain);
  put32(out + 12, static_cast<uint32_t>(r.pend_um));
  put64(out + 16, static_cast<uint64_t>(r.filled_um));
  put64(out + 24, static_cast<uint64_t>(r.drained_um));
}

bool ledger_decode(const uint8_t* in, size_t len, LedgerRecord* r, uint8_t* tokens, uint8_t* version) {
  if (len < size_t(kLedgerBlobBytes) || in[0] < 1) return false;   // any later version: its v1 prefix
  LedgerRecord x;
  x.gen = get16(in + 2);
  x.n_fill = get32(in + 4);
  x.n_drain = get32(in + 8);
  x.pend_um = static_cast<int32_t>(get32(in + 12));
  x.filled_um = static_cast<int64_t>(get64(in + 16));
  x.drained_um = static_cast<int64_t>(get64(in + 24));
  if (x.filled_um < 0 || x.drained_um < 0) return false;
  *r = x;
  if (tokens) *tokens = in[1];
  if (version) *version = in[0];
  return true;
}

// ------------------------------------------------------------------------------------------------------------------- ledger
void FuelLedger::init() {
  filled_um_ = drained_um_ = dgoff_um_ = slow_um_ = 0;
  n_fill_ = n_drain_ = 0;
  gen_ = 0;
  dg_ = DgState::Unknown;
  run_ = false;
  has_stored_ = has_next_stored_ = false;
  stored_um_ = next_stored_um_ = 0;
  next_outage_ = kOutClock;
  anchor_pending_ = false;
  t_anchor_ = t_last_ = t_lev_ = 0;
  set_lpmm(kLpmmRefUl);
  bookings_ = 0;
  log_n_ = log_head_ = 0;
  log_dropped_ = 0;
  has_watch_ = false;
  watch_until_ = 0;
  watch_v_ = 0;
  carry_seq_ = 0;
  carry_pending_ = false;
  carry_t_ = carry_tlev_ = 0;
  chain_end_ = carry_end_ = 0;
  clear_slow(0);
  new_run_state();
}

void FuelLedger::set_lpmm(int64_t lpmm_ul) {
  lpmm_ = lpmm_ul < 10000 ? 10000 : (lpmm_ul > 100000000 ? 100000000 : lpmm_ul);
  const int64_t lk = rdiv(kLeakUmH * kLpmmRefUl, lpmm_), sm = rdiv(int64_t(kSlowMinUm) * kLpmmRefUl, lpmm_);
  const int64_t em = rdiv(int64_t(kEvMinUm) * kLpmmRefUl, lpmm_);
  leak_um_h_ = lk > kLeakMinUmH ? lk : kLeakMinUmH;
  slow_min_um_ = sm > kSlowMinUm ? sm : kSlowMinUm;
  ev_min_um_ = em > kEvFloorUm ? em : kEvFloorUm;
}

void FuelLedger::new_run_state() {
  t0_ = 0; blk_idx_ = -1; blk_n_ = 0; blk_ac_ = true;
  rep_ = 0; goff_ = 0; ring_n_ = 0; after_n_ = 0;
  has_prev_move_ = false; t_prev_move_ = 0;
  has_z_ = false; z_ = 0; t_blk_ = 0; has_move_ = false; t_move_ = 0; recentred_ = true;
  has_ep_ = false; ep_ = Ep{};
  has_rest_ = false; rest_bp_ = rest_bh_ = 0; rest_bac_ = false; rest_bt1_ = rest_t_ = 0;
  ev_b_ = 0;
  co_ = 0; co_set_ = true; carried_ = false;
  has_first_ = false; p_first_ = p_gap_ = rh_first_ = 0; gap_done_ = false;
  min_idx_ = -1; min_n_ = 0; min_open_ = false;
  raw_n_ = 0;
  has_k_ = has_corr_ = false; k_last_ = 0; corr_t_ = 0;
  nm1_ = 462290; h_last_ = 0;
  pend_um_ = 0;
}

void FuelLedger::clear_slow(int64_t t0m) {
  t0m_ = t0m;
  slow_n_ = 0; slow_last_ = 0; drain_mode_ = false; slow_low_ = 0; slow_ep_um_ = 0;
  has_floor_ = false; floor_ref_ = 0; floor_left_ = 0;
  rise_mode_ = false; has_ceil_ = false; rise_high_ = 0; ceil_ref_ = 0; ceil_left_ = 0; credit_um_ = 0; credit_t_ = 0;
  has_arm_ = false; arm_t_ = 0; a_arm_ = 0;
  carry_pending_ = false;
}

LedgerRecord FuelLedger::record() const {
  LedgerRecord r;
  r.filled_um = filled_um_;
  r.drained_um = drained_um_;
  r.n_fill = n_fill_;
  r.n_drain = n_drain_;
  r.pend_um = pend_um_;
  r.gen = gen_;
  return r;
}

void FuelLedger::restore(const LedgerRecord& r) {
  filled_um_ = r.filled_um;
  drained_um_ = r.drained_um;
  n_fill_ = r.n_fill;
  n_drain_ = r.n_drain;
  gen_ = r.gen;
  pend_um_ = 0;
  if (r.pend_um) book(r.pend_um, 0, true, false, false, false, 0);   // a power cut mid-event: its progress (the tail is lost)
}

LedgerRecord FuelLedger::clear() {
  const LedgerRecord old = record();
  has_watch_ = false;
  filled_um_ = drained_um_ = dgoff_um_ = slow_um_ = 0;
  n_fill_ = n_drain_ = 0;
  pend_um_ = 0;
  if (has_ep_) {
    has_ep_ = false;
    has_prev_move_ = has_move_;
    t_prev_move_ = t_move_;
    after_n_ = 0;
  }
  clear_slow(t0m_);
  ++bookings_;
  gen_ = uint16_t(gen_ + 1);
  ++carry_seq_;
  return old;
}

void FuelLedger::begin_run(bool has_stored, int32_t stored_um, int64_t outage_ms) {
  if (run_) stash();
  has_next_stored_ = has_stored;
  next_stored_um_ = stored_um;
  next_outage_ = outage_ms < 0 ? kOutUnknown : outage_ms;
  anchor_pending_ = true;
}

void FuelLedger::end_run() {
  if (run_) stash();
  if (carry_pending_) {
    slow_end();
    clear_slow(t0m_);
  }
}

// ------------------------------------------------------------------------------------------------------------------- carry
void FuelLedger::carry_out(LedgerCarry* c) const {
  std::memset(c, 0, sizeof *c);
  int32_t end = chain_end_;
  if (run_ && has_ep_ && pend_um_ && co_set_)              // pend is booked at restore (raw: the glitch legs stay in)
    end = static_cast<int32_t>(ep_.bp + (ep_.unc ? 0 : ep_.g) - goff_ - ev_b_ + co_);
  const int64_t ref = run_ ? t_last_ : carry_t_;
  for (int i = 0; i < slow_n_; ++i) { c->q[i] = slow_q_[i]; c->m[i] = slow_m_[i]; }
  c->n = uint8_t(slow_n_);
  c->slow_last = slow_last_;
  c->t0m_rel = t0m_ - ref;
  c->tlev_rel = (run_ ? t_lev_ : carry_tlev_) - ref;
  c->credit_t_rel = credit_t_ - ref;
  c->arm_t_rel = has_arm_ ? arm_t_ - ref : 0;
  c->credit_um = credit_um_;
  c->slow_ep_um = slow_ep_um_;
  c->chain_end = end;
  c->slow_low = slow_low_;
  c->rise_high = rise_high_;
  c->floor_ref = floor_ref_;
  c->ceil_ref = ceil_ref_;
  c->a_arm = a_arm_;
  c->floor_left = int16_t(floor_left_);
  c->ceil_left = int16_t(ceil_left_);
  c->flags = uint8_t((drain_mode_ ? 1 : 0) | (rise_mode_ ? 2 : 0) | (has_floor_ ? 4 : 0) | (has_ceil_ ? 8 : 0) |
                     (has_arm_ ? 16 : 0));
}

void FuelLedger::carry_in(const LedgerCarry& c) {
  slow_n_ = c.n <= kSlowN ? c.n : kSlowN;
  for (int i = 0; i < slow_n_; ++i) { slow_q_[i] = c.q[i]; slow_m_[i] = c.m[i]; }
  slow_last_ = c.slow_last;
  t0m_ = c.t0m_rel;
  chain_end_ = c.chain_end;
  drain_mode_ = c.flags & 1;
  rise_mode_ = c.flags & 2;
  has_floor_ = c.flags & 4;
  has_ceil_ = c.flags & 8;
  has_arm_ = c.flags & 16;
  slow_low_ = c.slow_low;
  slow_ep_um_ = c.slow_ep_um;
  floor_ref_ = c.floor_ref;
  floor_left_ = c.floor_left;
  rise_high_ = c.rise_high;
  ceil_ref_ = c.ceil_ref;
  ceil_left_ = c.ceil_left;
  credit_um_ = c.credit_um;
  credit_t_ = c.credit_t_rel;
  arm_t_ = c.arm_t_rel;
  a_arm_ = c.a_arm;
  carry_t_ = 0;
  carry_tlev_ = c.tlev_rel;
  carry_pending_ = true;
  next_outage_ = kOutUnknown;                              // (only a begin_run with the outage carries it)
  ++carry_seq_;
}

void FuelLedger::stash() {
  if (blk_idx_ >= 0 && blk_n_) finish_block(t0_ + (blk_idx_ + 1) * kBlockMs);
  if (has_ep_) close_ep(true, 0);
  has_watch_ = false;
  if (ring_n_ && co_set_) chain_end_ = static_cast<int32_t>(ring_[ring_n_ - 1].p - goff_ - ev_b_ + co_);
  carry_t_ = t_last_;
  carry_tlev_ = t_lev_;
  carry_pending_ = true;
  run_ = false;
  has_stored_ = false;
  new_run_state();
  ++carry_seq_;
}

void FuelLedger::start_run(int64_t t_ms) {
  bool adopt = false;
  if (carry_pending_) {
    bool has_base = false;
    int64_t base = 0;                                      // the old run's last input on this run's clock
    if (next_outage_ >= 0) { base = t_anchor_ - next_outage_; has_base = true; }
    else if (next_outage_ == kOutClock) { base = carry_t_; has_base = true; }
    if (has_base) {
      const int64_t g = t_ms - base + carry_t_ - carry_tlev_;
      adopt = g >= 0 && g <= kCarryMs;
    }
    if (adopt) {
      carry_end_ = chain_end_;
      const int64_t sh = base - carry_t_;
      t0m_ += sh;
      credit_t_ += sh;
      if (has_arm_) arm_t_ += sh;
      carry_pending_ = false;
    } else {
      slow_end();                                          // (a running slow drain closes: FUEL_SLOW)
    }
  }
  if (!adopt) clear_slow(t_ms);
  has_stored_ = has_next_stored_;
  stored_um_ = next_stored_um_;
  has_next_stored_ = false;
  next_outage_ = kOutClock;
  run_ = true;
  t0_ = t_ms;
  carried_ = adopt;
  co_set_ = !adopt;
  ++carry_seq_;
}

// ------------------------------------------------------------------------------------------------------------------- log
void FuelLedger::log(const LedgerLog& e) {
  if (log_n_ == kLogCap) {                                 // the app drains it every estimate: only a stall overflows it
    log_head_ = (log_head_ + 1) % kLogCap;
    --log_n_;
    ++log_dropped_;
  }
  log_[(log_head_ + log_n_) % kLogCap] = e;
  ++log_n_;
}

bool FuelLedger::pop_log(LedgerLog* out) {
  if (!log_n_) return false;
  *out = log_[log_head_];
  log_head_ = (log_head_ + 1) % kLogCap;
  --log_n_;
  return true;
}

LedgerActive FuelLedger::active() const {
  if (pend_um_ > 0) return LedgerActive::Fill;
  if (pend_um_ < 0) return LedgerActive::Drain;
  return drain_mode_ ? LedgerActive::Slow : LedgerActive::None;
}

int32_t FuelLedger::lvl(int64_t b_um) const { return static_cast<int32_t>(rdiv(b_um * 1000000, nm1_)); }
int32_t FuelLedger::rng(int64_t l_um) const { return static_cast<int32_t>(rdiv(l_um * nm1_, 1000000)); }

// ------------------------------------------------------------------------------------------------------------------- input
void FuelLedger::push(const LedgerInput& in) {
  if (anchor_pending_) { t_anchor_ = in.t_ms; anchor_pending_ = false; }
  uint8_t kind = in.kind;
  if (kind == LedgerInput::kLevel && !nm1_ok(in.nm1_ppm)) kind = LedgerInput::kNoLevel;
  if (kind == LedgerInput::kLevel && (!in.cont || !run_)) {
    if (run_) stash();                                     // cont false inside a run: an in-process restart
    start_run(in.t_ms);
  }
  t_last_ = in.t_ms;
  if (kind != LedgerInput::kLevel) return;
  t_lev_ = in.t_ms;
  nm1_ = in.nm1_ppm;
  h_last_ = in.h_um;
  const int64_t kk = rdiv(rdiv(int64_t(in.h_um) * in.nm1_ppm, 1000) + kD0Nm - int64_t(in.pb_um) * 1000, kStepNm);
  if (has_k_ && kk != k_last_) {                           // a tracker cycle correction: a watched event is now unverified
    has_corr_ = true;
    corr_t_ = in.t_ms;
    if (has_watch_) {
      LedgerLog e;
      e.kind = LedgerLog::kUnverified;
      e.value_um = watch_v_;
      e.unverified = true;
      log(e);
      has_watch_ = false;
    }
  }
  has_k_ = true;
  k_last_ = static_cast<int32_t>(kk);
  if (has_watch_ && in.t_ms >= watch_until_) has_watch_ = false;
  int32_t p = in.pb_um + rep_ + goff_;
  const int32_t rh = static_cast<int32_t>(rdiv(int64_t(in.h_um) * in.nm1_ppm, 1000000));   // the level as range
  if (!has_first_) { has_first_ = true; p_first_ = p; rh_first_ = rh; p_gap_ = p; }
  if (!gap_done_ && in.conf) gap(p - goff_, in.h_um);    // (the reported level includes the absorbed jumps: so does this)
  const int64_t idx = (in.t_ms - t0_) / kBlockMs;
  if (idx != blk_idx_) {
    if (blk_idx_ >= 0) {
      finish_block(t0_ + (blk_idx_ + 1) * kBlockMs);
      p = in.pb_um + rep_ + goff_;                         // (a close there may have moved the frame: this estimate is in the new)
    }
    blk_idx_ = idx;
    blk_n_ = 0;
    blk_ac_ = true;
  }
  if (blk_n_ < kBlockCap) { blk_p_[blk_n_] = p; blk_h_[blk_n_] = rh; ++blk_n_; }
  blk_ac_ = blk_ac_ && in.ac;
  if (raw_n_ == kRawN) { std::memmove(raw_, raw_ + 1, sizeof(Raw) * (kRawN - 1)); --raw_n_; }
  raw_[raw_n_++] = Raw{p, static_cast<uint32_t>(in.t_ms)};
  if (has_ep_) {                                           // the open episode's step tests follow the level
    const int64_t tol = rng(kStepUm);
    follow(&ep_.cur, in.t_ms, p, tol);
    follow(&ep_.first, in.t_ms, p, tol);
  }
}

void FuelLedger::gap(int32_t p, int32_t h_um) {
  gap_done_ = true;
  if (!has_stored_) return;
  const int64_t g = int64_t(h_um) - lvl(int64_t(p) - p_gap_) - stored_um_;
  if (g >= kGapMinUm || g <= -kGapMinUm) {
    LedgerLog e;
    e.kind = LedgerLog::kGap;
    e.value_um = g;
    log(e);
  }
}

// ------------------------------------------------------------------------------------------------------------------- blocks
void FuelLedger::finish_block(int64_t t_end) {
  ++carry_seq_;
  if (blk_n_ >= kBlockMin) {
    Blk b{lmed(blk_p_, blk_n_), lmed(blk_h_, blk_n_), static_cast<int32_t>(blk_idx_), blk_ac_};
    if (!co_set_) { co_ = carry_end_ - (b.p - goff_); co_set_ = true; }   // a carried run: its chain continues the old end
    block(b);
    minute(t_end, &ring_[ring_n_ - 1]);                    // (the newest ring block: a repair in block() moved it too)
  } else {
    minute(t_end, nullptr);
  }
}

void FuelLedger::block(const Blk& b) {
  const int64_t t = bt(b);
  if (ring_n_ == kRingN) { std::memmove(ring_, ring_ + 1, sizeof(Blk) * (kRingN - 1)); --ring_n_; }
  ring_[ring_n_++] = b;
  if (has_ep_ && after_n_ < kRingN) ++after_n_;           // (an episode closes by its 12th valid block after the last move)
  int32_t ys[kYN];
  const int ny = ring_n_ < kYN ? ring_n_ : kYN;
  for (int i = 0; i < ny; ++i) ys[i] = ring_[ring_n_ - ny + i].p;
  const int32_t y = lmed(ys, ny);
  const int32_t d = rng(kDeltaUm);
  if (!has_z_) { has_z_ = true; z_ = y; has_move_ = false; recentred_ = true; t_blk_ = t; }
  const int64_t dt = t - t_blk_ < kLeakDtMaxMs ? t - t_blk_ : kLeakDtMaxMs;
  t_blk_ = t;
  int mv = 0;
  if (y > z_ + d) { z_ = y - d; mv = 1; }
  else if (y < z_ - d) { z_ = y + d; mv = -1; }
  if (mv) {
    const bool refresh = has_ep_ && ep_.s == mv && !(ep_.cur.has && ep_.cur.n);   // its leg had not moved yet
    if (has_ep_ && ep_.s != mv) {
      const int drop = after_n_ > kChDrop ? kChDrop : 1;  // the blocks after the last move without the reversing move's start
      int na = after_n_ - drop;
      if (na > kWinN) na = kWinN;
      int32_t ps[kWinN];
      int32_t a1;
      if (na > 0) {
        for (int i = 0; i < na; ++i) ps[i] = ring_[ring_n_ - after_n_ + i].p;
        a1 = lmed(ps, na);
      } else {
        a1 = ring_[ring_n_ > kChDrop ? ring_n_ - 1 - kChDrop : 0].p;
      }
      if (2000 * iabs(int64_t(a1) - ep_.bp) < kChainNm * 2 && t - t_move_ <= kChainMs) {
        const Leg k = leg(ep_, a1);                        // under half a cycle: CHAINED (the net books at the close); the
        if (k == Leg::Step) ep_.g += int64_t(a1) - ep_.leg0;   // leg that ends here is told on its own: a step is a glitch
        else if (k == Leg::Unclear) ep_.unc = true;
        ep_.s = static_cast<int8_t>(mv);
        ep_.chain = true;
        ep_.leg0 = a1;
        steprec(a1, mv, &ep_.cur);
      } else {
        const Ep old = ep_;
        if (!close_ep(false, drop)) open_ep(mv, t, &old);  // nothing booked (a splash dip): keep its before-level
      }
    }
    if (!has_ep_) {
      open_ep(mv, t, nullptr);
    } else if (refresh) {                                  // the leg's step test from its real transition: a leg that starts
      steprec(ep_.leg0, mv, &ep_.cur);                     // at a reversal seen late (the level already back at its start)
      if (!ep_.chain) ep_.first = ep_.cur;                 // moves later
    }
    has_move_ = true;
    t_move_ = t;
    recentred_ = false;
    after_n_ = 0;
  } else {
    if (has_ep_ && t - t_move_ >= kQuietMs && after_n_ >= kCloseMin) close_ep(false, 0);
    if (!recentred_ && t - t_move_ >= kQuietMs) {
      int32_t w[kWinN] = {};
      int nw = 0;
      const int i0 = ring_n_ > kWinN ? ring_n_ - kWinN : 0;
      for (int i = i0; i < ring_n_; ++i) if (t - bt(ring_[i]) < kQuietMs) w[nw++] = ring_[i].p;
      if (nw) z_ = lmed(w, nw);
      recentred_ = true;
    } else {
      const int64_t lk = rdiv(leak_um_h_ * dt * nm1_, 3600000LL * 1000000LL);
      int64_t dz = int64_t(y) - z_;
      if (dz > lk) dz = lk;
      if (dz < -lk) dz = -lk;
      z_ = static_cast<int32_t>(z_ + dz);
    }
    if (!has_ep_ && iabs(int64_t(y) - z_) < rng(kRestUm)) rest(t);
  }
  if (has_ep_) {
    min_open_ = true;                                      // progress without the glitch (no provisional volume of one)
    const int32_t v = lvl(int64_t(y) - glitch(ep_, y) - ep_.bp);
    const bool ok = ep_.chain ? iabs(v) >= ev_min_um_ : int64_t(v) * ep_.s >= ev_min_um_;
    pend_um_ = ok ? v : 0;
  }
  chain_end_ = static_cast<int32_t>(ring_[ring_n_ - 1].p - goff_ - ev_b_ + co_);
}

void FuelLedger::rest(int64_t t) {
  const int hi = ring_n_ - kPreSkip;                       // ring_[0 .. hi - 1]: the blocks ending 2 before this one ...
  if (hi <= 0) return;
  int c0 = 0;                                              // ... after the previous episode
  if (has_prev_move_) while (c0 < hi && !(bt(ring_[c0]) > t_prev_move_)) ++c0;
  const int lo = hi - kWinN > c0 ? hi - kWinN : c0;
  if (hi <= lo) return;
  int32_t ps[kWinN], hs[kWinN];
  bool ac = true;
  for (int i = lo; i < hi; ++i) { ps[i - lo] = ring_[i].p; hs[i - lo] = ring_[i].h; ac = ac && ring_[i].ac; }
  has_rest_ = true;
  rest_bp_ = lmed(ps, hi - lo);
  rest_bh_ = lmed(hs, hi - lo);
  rest_bac_ = ac;
  rest_bt1_ = bt(ring_[hi - 1]);
  rest_t_ = t;
}

void FuelLedger::steprec(int32_t level, int dir, StepRec* r) const {
  r->has = false;
  r->moved = false;
  r->moved_t = 0;
  r->n = r->post = 0;
  r->dir = int8_t(dir > 0 ? 1 : -1);
  r->ext = level;
  r->tb = 0;
  const int64_t tol = rng(kStepUm);
  for (int i = raw_n_ - 1; i >= 0; --i) {
    if (iabs(int64_t(raw_[i].p) - level) > tol) continue;
    r->has = true;
    r->tb = raw_[i].t;
    for (int j = i + 1; j < raw_n_ && r->n < kStepC; ++j) {
      if (int64_t(uint32_t(raw_[j].t - raw_[i].t)) > kStepMs) continue;
      const int32_t c = raw_[j].p;
      if (!r->n || int64_t(r->dir) * c > int64_t(r->dir) * r->ext) r->ext = c;   // (the first furthest, as max())
      r->c[r->n++] = c;
    }
    for (int j = i + 1; j < raw_n_; ++j)                   // (the full time of a kept estimate: t_last_ minus its age)
      follow(r, t_last_ - int64_t(uint32_t(uint32_t(t_last_) - raw_[j].t)), raw_[j].p, tol);
    return;
  }
}

void FuelLedger::follow(StepRec* r, int64_t t, int32_t p, int64_t tol) const {
  if (!r->has || int64_t(int32_t(uint32_t(t) - r->tb)) <= kStepMs) return;
  if (r->post < 255) ++r->post;
  if (!r->moved && int64_t(r->dir) * (int64_t(p) - r->ext) > tol) {   // it went on moving (from then)
    r->moved = true;
    r->moved_t = t;
  }
}

bool FuelLedger::steplike(const StepRec& r, int64_t a0, int64_t a) const {
  if (!r.has || !r.n) return false;
  const int64_t tol = rng(kStepUm);
  if (iabs(a - a0) <= 2 * tol) return false;
  bool near = false;
  for (int i = 0; i < r.n; ++i) if (iabs(r.c[i] - a) <= tol) { near = true; break; }
  return near && settled(r);
}

FuelLedger::Leg FuelLedger::leg(const Ep& ep, int64_t a) const {
  const int64_t tol = rng(kStepUm);
  const int64_t d = a - ep.leg0;
  if (iabs(d) <= 2 * tol) return Leg::Ok;                  // too small to matter
  if (steplike(ep.cur, ep.leg0, a)) return Leg::Step;
  if (!ep.cur.has || !ep.cur.n) return Leg::Unclear;
  const int64_t s = d > 0 ? 1 : -1;                        // ramped: the kStepMs after it left leg0 ended clearly short of a
  for (int i = 0; i < ep.cur.n; ++i) if (s * (a - ep.cur.c[i]) <= tol) return Leg::Unclear;
  return Leg::Ok;
}

int64_t FuelLedger::glitch(const Ep& ep, int64_t a) const {
  if (ep.chain && !ep.unc) {                               // a chained episode whose legs can all be told: its step-like legs
    const Leg k = leg(ep, a);
    if (k != Leg::Unclear) return ep.g + (k == Leg::Step ? a - ep.leg0 : 0);
  }
  return steplike(ep.first, ep.bp, a) ? a - ep.bp : 0;    // else as one leg: all of it when it is one step from the start
}

void FuelLedger::open_ep(int s, int64_t t, const Ep* inherit) {
  if (inherit) {
    ep_ = *inherit;
    ep_.s = static_cast<int8_t>(s);
    has_ep_ = true;
    min_open_ = true;
    return;
  }
  Ep e{};
  e.s = static_cast<int8_t>(s);
  e.t0 = t;
  e.chain = false;
  bool early = false;
  int64_t bt1 = 0;
  if (has_rest_ && (!has_prev_move_ || rest_t_ > t_prev_move_) && t - rest_t_ <= kRestMaxMs) {
    e.bp = rest_bp_;                                       // the last block at rest since the previous episode, recent
    e.bh = rest_bh_;
    e.bac = rest_bac_;
    bt1 = rest_bt1_;
    early = ring_n_ >= 2 && rest_t_ < bt(ring_[ring_n_ - 2]);   // ... earlier than the block before the move
  } else {
    const int n = ring_n_;                                 // the first-move block is ring_[n - 1]
    int c0 = 0;                                            // candidates: ring_[c0 .. n - 2], after the previous episode
    if (has_prev_move_) while (c0 < n - 1 && !(bt(ring_[c0]) > t_prev_move_)) ++c0;
    const int nc = n - 1 - c0 > 0 ? n - 1 - c0 : 0;
    int lo = nc - kPreSkip - kWinN, hi = nc - kPreSkip;
    if (lo < 0) lo = 0;
    if (hi < 0) hi = 0;
    if (hi <= lo && has_prev_move_) { lo = nc > kWinN ? nc - kWinN : 0; hi = nc; }   // a short pause: what there is
    if (hi > lo) {
      int32_t ps[kWinN] = {}, hs[kWinN] = {};
      bool ac = true;
      for (int i = lo; i < hi; ++i) { ps[i - lo] = ring_[c0 + i].p; hs[i - lo] = ring_[c0 + i].h; ac = ac && ring_[c0 + i].ac; }
      e.bp = lmed(ps, hi - lo);
      e.bh = lmed(hs, hi - lo);
      e.bac = ac;
    } else if (!has_prev_move_) {                          // moving from the start of the run (a fill running at power-up):
      e.bp = p_first_;                                     // from its first measurement
      e.bh = rh_first_;
      e.bac = false;
      if (carried_) co_ = carry_end_ - (p_first_ - goff_); // (no quiet minute of this run yet: the carried chain continues)
    } else {                                               // straight after an episode dropped by clear(): the block before
      const Blk& x = n >= 2 ? ring_[n - 2] : ring_[n - 1];
      e.bp = x.p;
      e.bh = x.h;
      e.bac = false;
    }
  }
  e.leg0 = e.bp;
  e.g = 0;
  e.unc = false;
  steprec(e.bp, s, &e.cur);                                // the last clean estimate at the before-level, the 2.2 s after it
  e.first = e.cur;
  ep_ = e;
  has_ep_ = true;
  min_open_ = true;
  if (early) {                                             // the minutes since the level began to move belong to the event
    while (slow_n_ > 0 && st(slow_n_ - 1) > bt1) {
      --slow_n_;
      if (slow_n_) slow_last_ = sidx(slow_n_ - 1);
    }
  }
}

bool FuelLedger::close_ep(bool provisional, int drop) {
  const Ep ep = ep_;
  has_ep_ = false;
  has_prev_move_ = has_move_;
  t_prev_move_ = t_move_;
  pend_um_ = 0;
  const Blk* aft = ring_ + (ring_n_ - after_n_);           // the blocks after the last move (newest of the ring)
  int na = after_n_ - drop;
  if (na > kWinN) na = kWinN;                              // (defensive: an episode closes by its 12th block after the move)
  if (na < 0) na = 0;
  if (!na && provisional && ring_n_) {
    na = 1;                                                // the newest valid block
    aft = ring_ + (ring_n_ - 1);
  }
  if (!na) return false;
  int32_t ps[kWinN] = {}, hs[kWinN] = {};
  const int64_t tr = rng(kAftTrimUm);                      // the window ends at the last block AT REST: trailing blocks already
  while (na > 1) {                                         // back against the episode by > 0.2 mm are the next change
    for (int i = 0; i < na - 1; ++i) ps[i] = aft[i].p;
    if (ep.s * (int64_t(aft[na - 1].p) - lmed(ps, na - 1)) >= -tr) break;
    --na;
  }
  bool aac = true;
  for (int i = 0; i < na; ++i) { ps[i] = aft[i].p; hs[i] = aft[i].h; aac = aac && aft[i].ac; }
  int64_t ap = lmed(ps, na);
  const int32_t ah = lmed(hs, na);
  const int64_t t_after = bt(aft[0]);
  const int64_t g = glitch(ep, ap);                        // the radar, not fuel, at any size: into the glitch offset
  if (g) {
    const int32_t d = static_cast<int32_t>(g);
    goff_ -= d;
    for (int i = 0; i < ring_n_; ++i) ring_[i].p -= d;
    for (int i = 0; i < raw_n_; ++i) raw_[i].p -= d;
    z_ -= d;
    ap -= g;
    LedgerLog e;
    e.kind = LedgerLog::kJump;
    e.value_um = lvl(g);
    e.provisional = provisional;
    log(e);
  }
  int64_t dp = ap - ep.bp;
  if (g && iabs(dp) <= 2 * int64_t(rng(kStepUm))) return true;   // (nothing but the glitch)
  int64_t k = 0;
  if (aac && ep.bac && !provisional)                       // both windows anchored: the absolute level (range) arbitrates
    k = rdiv((int64_t(ah) - ep.bh - dp - g) * 1000, kStepNm);   // (the absolute level includes the glitch)
  if (k) {
    const int32_t rep = static_cast<int32_t>(rdiv(k * kStepNm, 1000));
    dp += rep;
    ap += rep;
    rep_ += rep;                                           // the later blocks are on the repaired chain
    for (int i = 0; i < ring_n_; ++i) ring_[i].p += rep;
    for (int i = 0; i < raw_n_; ++i) raw_[i].p += rep;
    z_ += rep;
  }
  const int32_t v = lvl(dp);
  if ((!k && !ep.chain && int64_t(v) * ep.s <= 0) || iabs(v) < ev_min_um_) return g != 0;   // (a glitch: used up)
  const int64_t t = bt(ring_[ring_n_ - 1]);
  ev_b_ += dp;
  const int64_t te = provisional ? t : t_after;
  const uint32_t dur = te > ep.t0 ? static_cast<uint32_t>((te - ep.t0) / 1000) : 0;
  bool unv = false, watch = false;
  if (provisional || !(aac && ep.bac)) {                   // not anchored: a following cycle correction flags it unverified
    unv = has_corr_ && corr_t_ >= ep.t0 - kBlockMs;
    watch = !unv && !provisional;
  }
  book(v, dur, provisional, k != 0, unv, watch, t + kVerifyMs);
  return true;
}

void FuelLedger::book(int32_t v, uint32_t dur_s, bool provisional, bool repaired, bool unverified, bool watch, int64_t until) {
  LedgerLog e;
  e.dur_s = dur_s;
  e.provisional = provisional;
  e.repaired = repaired;
  e.unverified = unverified;
  if (v > 0) {
    filled_um_ += v;
    ++n_fill_;
    e.kind = LedgerLog::kFill;
    e.value_um = v;
  } else {
    drained_um_ += -int64_t(v);
    ++n_drain_;
    if (dg_ == DgState::Off) dgoff_um_ += -int64_t(v);
    e.kind = LedgerLog::kDrain;
    e.value_um = -int64_t(v);
  }
  ++bookings_;
  log(e);                                                  // at once (a power cut never loses it); an unanchored event is
  if (watch) {                                             // watched kVerifyMs for a cycle correction (then kUnverified)
    has_watch_ = true;
    watch_until_ = until;
    watch_v_ = v;
  }
}

// ------------------------------------------------------------------------------------------------------------------- slow account
void FuelLedger::minute(int64_t t_end, const Blk* b) {
  const int64_t idx = (t_end - 1 - t0m_) / 60000;
  if (idx != min_idx_) {
    if (min_idx_ >= 0) finish_minute(t0m_ + (min_idx_ + 1) * 60000);
    min_idx_ = idx;
    min_n_ = 0;
    min_open_ = false;
  }
  if (b && min_n_ < kBlockCap) min_p_[min_n_++] = b->p - goff_;   // the RAW position: a glitch cannot ratchet the chain
  if (has_ep_) min_open_ = true;
}

void FuelLedger::finish_minute(int64_t t) {
  if (min_open_ || min_n_ < kMinBlocksMin) return;
  if (slow_n_ && (min_idx_ - slow_last_) * 60000 > kSlowSpanMs) slow_n_ = 0;   // all older than the span (keeps st() exact)
  if (slow_n_ == kSlowN) {
    std::memmove(slow_q_, slow_q_ + 1, sizeof(slow_q_[0]) * (kSlowN - 1));
    std::memmove(slow_m_, slow_m_ + 1, sizeof(slow_m_[0]) * (kSlowN - 1));
    --slow_n_;
  }
  slow_q_[slow_n_] = static_cast<int32_t>(lmed(min_p_, min_n_) - ev_b_ + co_);
  slow_m_[slow_n_++] = static_cast<uint16_t>(min_idx_);
  slow_last_ = min_idx_;
  int drop0 = 0;
  while (drop0 < slow_n_ && t - st(drop0) > kSlowSpanMs) ++drop0;
  if (drop0) {
    std::memmove(slow_q_, slow_q_ + drop0, sizeof(slow_q_[0]) * (slow_n_ - drop0));
    std::memmove(slow_m_, slow_m_ + drop0, sizeof(slow_m_[0]) * (slow_n_ - drop0));
    slow_n_ -= drop0;
  }
  if (floor_left_ > 0 && --floor_left_ == 0) has_floor_ = false;
  if (ceil_left_ > 0 && --ceil_left_ == 0) has_ceil_ = false;
  const int n = slow_n_;
  if (n < kSlowNMin) return;
  const int w = n / 4;                                     // medians of the oldest and newest quarter ...
  int32_t a[kSlowN / 4], c[kSlowN / 4];
  for (int i = 0; i < w; ++i) { a[i] = slow_q_[i]; c[i] = slow_q_[n - w + i]; }
  const int32_t A = lmed(a, w), B = lmed(c, w);
  const int64_t dt = st(n - w / 2) - st(w / 2);
  if (dt < kSlowDtMs) return;                              // ... at least 60 min apart (sparse quiet minutes between events)
  int64_t thr_l = int64_t(h_last_) * kSlowKPpmH * dt / (1000000LL * 3600000LL);
  if (thr_l < slow_min_um_) thr_l = slow_min_um_;
  const int32_t thr = rng(thr_l);
  const int32_t drop = A - B;
  int64_t fall = 0, rise = 0;                              // STEP-SHAPED: 2/3 of the drop (rise) within 3 quiet minutes is a
  for (int i = 0; i + 2 < n; ++i) {                        // step of the level (a glitch, a sub-event change), not a slow
    const int64_t f = int64_t(slow_q_[i]) - slow_q_[i + 2];   // drain (nor credit)
    if (f > fall) fall = f;
    if (-f > rise) rise = -f;
  }
  const bool step_dn = drop > 0 && 3 * fall >= 2 * int64_t(drop);
  const bool step_up = drop < 0 && 3 * rise >= -2 * int64_t(drop);
  if (credit_um_ && t - credit_t_ > kCreditMs) credit_um_ = 0;   // a rise not followed by a fall within 36 h: not thermal
  if (drain_mode_) {
    slow_book(B);
    if (int64_t(drop) * 3 < int64_t(thr) * 2) slow_end();
  } else if (rise_mode_) {
    bank(B, t);
    if (-int64_t(drop) * 3 < int64_t(thr) * 2) {
      rise_mode_ = false;
      has_ceil_ = true;
      ceil_ref_ = rise_high_;
      ceil_left_ = kSlowN;
    }
  } else if (drop >= thr && !step_dn) {
    if (!has_arm_) { has_arm_ = true; arm_t_ = t; a_arm_ = A; }   // the base: the oldest quarter when the drop first passed
    if (t - arm_t_ >= kSlowArmMs) {                        // ... and it stayed so for 20 min: a slow drain
      has_arm_ = false;
      drain_mode_ = true;
      slow_low_ = has_floor_ && floor_ref_ < a_arm_ ? floor_ref_ : a_arm_;
      slow_ep_um_ = 0;
      slow_book(B);
    }
  } else if (-int64_t(drop) >= thr && !step_up) {          // a rise as fast: never a fill; credit for later slow falls
    has_arm_ = false;
    rise_mode_ = true;
    rise_high_ = has_ceil_ && ceil_ref_ > A ? ceil_ref_ : A;
    bank(B, t);
  } else {
    has_arm_ = false;
  }
}

void FuelLedger::bank(int32_t b, int64_t t) {
  if (b > rise_high_) {
    const int32_t v = lvl(int64_t(b) - rise_high_);
    rise_high_ = b;
    int64_t cap = int64_t(h_last_ > 0 ? h_last_ : 0) * kCreditPpm / 1000000;
    if (cap < kCreditMinUm) cap = kCreditMinUm;
    credit_um_ = credit_um_ + v < cap ? credit_um_ + v : cap;
    credit_t_ = t;
  }
}

void FuelLedger::slow_book(int32_t b) {
  if (b < slow_low_) {
    int64_t v = lvl(int64_t(slow_low_) - b);
    slow_low_ = b;
    const int64_t c = credit_um_ < (v > 0 ? v : 0) ? credit_um_ : (v > 0 ? v : 0);   // an earlier slow rise's credit first
    credit_um_ -= c;
    v -= c;
    if (v > 0) {
      drained_um_ += v;
      slow_um_ += v;
      slow_ep_um_ += v;
      if (dg_ == DgState::Off) dgoff_um_ += v;
    }
  }
}

void FuelLedger::slow_end() {
  if (!drain_mode_) return;
  drain_mode_ = false;
  has_floor_ = true;
  floor_ref_ = slow_low_;
  floor_left_ = kSlowN;
  if (slow_ep_um_) {                                       // (a slow drain is not a drain event: n_drain counts events)
    LedgerLog e;
    e.kind = LedgerLog::kSlow;
    e.value_um = slow_ep_um_;
    ++bookings_;
    log(e);
  }
  slow_ep_um_ = 0;
}

// ------------------------------------------------------------------------------------------------------------------- saver
void LedgerSaver::seal() {
  uint32_t h = 0x811C9DC5u;
  const uint8_t* p = reinterpret_cast<const uint8_t*>(st_);
  for (size_t i = 0; i < offsetof(LedgerSaveState, check); ++i) h = (h ^ p[i]) * 16777619u;
  st_->check = h;
}

void LedgerSaver::begin(LedgerSaveState* st, const LedgerRecord& saved, uint32_t bookings, uint64_t now_ms, int cold_tokens) {
  st_ = st;
  constexpr uint32_t kMagic = 0x4C534156;                  // "LSAV"
  const uint32_t c = st_->check;
  seal();
  if (st_->magic != kMagic || st_->check != c || st_->milli_tokens > kBurst * 1000) {   // a cold boot: the stored count
    st_->magic = kMagic;
    st_->milli_tokens = cold_tokens < 0 ? kBurst * 1000 : (uint32_t(cold_tokens) > kBurst ? kBurst : uint32_t(cold_tokens)) * 1000;
    seal();
  } else {
    st_->check = c;
  }
  last_refill_ms_ = now_ms;
  saved_ = saved;
  saved_bookings_ = bookings;
  saved_ms_ = now_ms;
}

void LedgerSaver::refill(uint64_t now_ms) {
  const uint64_t dt = now_ms > last_refill_ms_ ? now_ms - last_refill_ms_ : 0;
  const uint64_t add = dt * 1000 / kRefillMs;
  last_refill_ms_ += add * kRefillMs / 1000;
  const uint64_t t = uint64_t(st_->milli_tokens) + add;
  st_->milli_tokens = uint32_t(t > kBurst * 1000 ? kBurst * 1000 : t);
  seal();
}

bool LedgerSaver::due(const LedgerRecord& cur, uint32_t bookings, uint64_t now_ms) {
  if (!st_) return false;
  refill(now_ms);
  if (cur == saved_ && bookings == saved_bookings_) return false;
  if (st_->milli_tokens < 1000) return false;
  if (bookings != saved_bookings_ || cur.n_fill != saved_.n_fill || cur.n_drain != saved_.n_drain || cur.gen != saved_.gen)
    return true;                                           // (a booking, or a reset that the stored record missed)
  if (st_->milli_tokens < kReserve * 1000 + 1000) return now_ms - saved_ms_ >= kPeriodicMs;   // keep tokens for bookings
  const int64_t dpend = int64_t(cur.pend_um) - saved_.pend_um;
  if ((dpend >= kCheckpointUm || dpend <= -kCheckpointUm) && now_ms - saved_ms_ >= kCheckpointMs) return true;
  const int64_t dtot = (cur.filled_um - saved_.filled_um) + (cur.drained_um - saved_.drained_um);   // slow bookings
  if (dtot >= kSlowSaveUm && now_ms - saved_ms_ >= kSlowSaveMs) return true;
  return now_ms - saved_ms_ >= kPeriodicMs;
}

uint8_t LedgerSaver::debit(uint64_t now_ms) {
  if (!st_) return 0;
  refill(now_ms);
  st_->milli_tokens = st_->milli_tokens >= 1000 ? st_->milli_tokens - 1000 : 0;
  seal();
  return uint8_t(st_->milli_tokens / 1000);
}

void LedgerSaver::saved(const LedgerRecord& r, uint32_t bookings, uint64_t now_ms) {
  saved_ = r;
  saved_bookings_ = bookings;
  saved_ms_ = now_ms;
}

void LedgerSaver::restart(const LedgerRecord& saved, uint32_t bookings, uint64_t now_ms) {
  saved_ = saved;
  saved_bookings_ = bookings;
  saved_ms_ = now_ms;
}

}  // namespace fs
