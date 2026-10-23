// Fuel ledger: cumulative fuel FILLED and DRAINED since installation (DESIGN.md §13), from the tracker's per-estimate output.
// C++ port of dev/fuel_ledger.py (the reference): integer arithmetic only (level and range in um, time in ms), so it books the
// same totals and events bit for bit, needs no FPU on the ESP32-C3, and keeps the tracker untouched (a pure consumer).
// Pure C++17, no platform headers, no heap; FuelLedger is about 2.4 KB of state.
#pragma once
#include <cstddef>
#include <cstdint>
#include "live.h"

namespace fs {

// One tracker estimate as the ledger sees it (LedgerAdapter::convert builds it from fs::Estimate).
struct LedgerInput {
  enum Kind : uint8_t { kLevel = 0, kHold = 1, kNoLevel = 2 };
  int64_t t_ms = 0;          // tracker clock
  uint8_t kind = kNoLevel;
  bool cont = false;         // floor phase carried from an earlier estimate (false: a new run)
  int32_t pb_um = 0;         // phase-continuous floor position (range um): reported position minus the run's cycle moves
  int32_t h_um = 0;          // reported (absolute) level
  int32_t nm1_ppm = 0;       // (n - 1) x 1e6 (a level only with n within [1.1, 2])
  bool ac = false;           // confirmed and anchored (memoryless absolute cycle)
  bool conf = false;         // confirmed
};

// Estimate -> LedgerInput. Keeps the run's cycle count (sum of cycle_step since cont was false).
class LedgerAdapter {
 public:
  LedgerInput convert(const Estimate& e);
  void reset() { k_ = 0; }
 private:
  int32_t k_ = 0;
};

// The persistent part (NVS blob, RTC copy, /fuel.bin): totals in level um, event counts, the open episode's provisional
// volume and the reset generation.
struct LedgerRecord {
  int64_t filled_um = 0;
  int64_t drained_um = 0;
  uint32_t n_fill = 0;       // fill events
  uint32_t n_drain = 0;      // drain events (slow drains are not events)
  int32_t pend_um = 0;       // provisional volume of an open episode (signed; booked at the next boot if the run was cut)
  uint16_t gen = 0;          // reset generation (FuelLedger::clear() + 1, wraps)
  bool operator==(const LedgerRecord& o) const {
    return filled_um == o.filled_um && drained_um == o.drained_um && n_fill == o.n_fill && n_drain == o.n_drain &&
           pend_um == o.pend_um && gen == o.gen;
  }
  bool operator!=(const LedgerRecord& o) const { return !(*this == o); }
};
// Blob v1 (DESIGN.md §9.1), little-endian: u8 version = 1, u8 save tokens left after this write (LedgerSaver), u16 generation,
// u32 fills, u32 drains, i32 provisional um, i64 filled um, i64 drained um. Later versions only APPEND fields: any record of
// version >= 1 and length >= 32 is read through this prefix, and an older firmware never shortens a newer record.
constexpr int kLedgerBlobBytes = 32;                     // 3 NVS entries per write (blob index + chunk header + 32 B)
constexpr int kLedgerBlobMax = 64;                       // longest record kept (a newer firmware's appended fields)
constexpr uint8_t kLedgerVersion = 1;
void ledger_encode(const LedgerRecord& r, uint8_t tokens, uint8_t out[kLedgerBlobBytes]);
// false: too short, version 0, impossible values. *tokens / *version (optional) = bytes 0 and 1.
bool ledger_decode(const uint8_t* in, size_t len, LedgerRecord* r, uint8_t* tokens = nullptr, uint8_t* version = nullptr);

// What the ledger reports for the event log (the app writes them, budget-limited, from the loop task).
struct LedgerLog {
  enum Kind : uint8_t { kFill = 0, kDrain = 1, kSlow = 2, kGap = 3, kJump = 4, kUnverified = 5 };
  int64_t value_um = 0;      // fill / drain / slow: the volume (> 0); gap: the level change across the outage (signed);
                             // jump: the absorbed step (signed: not fuel, diagnostics); unverified: the signed volume of the
                             // fill (> 0) or drain (< 0) logged last, which a tracker cycle correction followed
  uint32_t dur_s = 0;        // fill / drain: first move to the after-window
  uint8_t kind = kFill;
  bool provisional = false;  // booked from the progress at a run end or a power cut (the event's tail is missing)
  bool repaired = false;     // the anchored level arbitrated whole cycles across the event
  bool unverified = false;   // windows not both anchored and a tracker cycle correction followed: may be a cycle off
};

enum class LedgerActive : uint8_t { None = 0, Fill = 1, Drain = 2, Slow = 3 };
enum class DgState : uint8_t { Unknown = 0, Off = 1, On = 2 };   // hook for the drained-while-DG-off bucket (no DG input yet)

constexpr int kLedgerSlowN = 80;                         // quiet minutes in the slow window

// The slow account across a warm reset (RTC memory): FuelLedger::carry_out() / carry_in(). Times relative to the last input.
struct LedgerCarry {
  int32_t q[kLedgerSlowN];   // quiet minutes: chain value ...
  uint16_t m[kLedgerSlowN];  // ... and minute index mod 2^16
  int64_t slow_last;         // full minute index of the newest
  int64_t t0m_rel, tlev_rel, credit_t_rel, arm_t_rel;
  int64_t credit_um, slow_ep_um;
  int32_t chain_end, slow_low, rise_high, floor_ref, ceil_ref, a_arm;
  int16_t floor_left, ceil_left;
  uint8_t n;
  uint8_t flags;             // 1 drain, 2 rise, 4 floor, 8 ceil, 16 armed
};

class FuelLedger {
 public:
  // ---- constants (dev/fuel_ledger.py)
  static constexpr int64_t kBlockMs = 5000;
  static constexpr int kBlockMin = 3;
  static constexpr int kBlockCap = 16;                     // estimates per block (12.5 at 2.5 estimates/s)
  static constexpr int kYN = 3;
  static constexpr int32_t kDeltaUm = 400;
  static constexpr int64_t kLeakUmH = 3000;                // on the reference tank (x ref / lpmm), never under kLeakMinUmH
  static constexpr int64_t kLeakMinUmH = 1000;
  static constexpr int64_t kLeakDtMaxMs = 3600LL * 1000;
  static constexpr int64_t kQuietMs = 60000;
  static constexpr int kCloseMin = 6;
  static constexpr int kWinN = 12;
  static constexpr int kPreSkip = 2;
  static constexpr int32_t kEvMinUm = 700;                 // on the reference tank (x ref / lpmm on a smaller one: ~1 L) ...
  static constexpr int32_t kEvFloorUm = 700;               // ... never under 0.7 mm (the noise floor, review r2 NIT 1)
  static constexpr int kRingN = 15;                        // before-window + skip + the block at rest / the first move
  static constexpr int64_t kStepNm = 2502274;              // one phase cycle of range (calib kStep), nm
  static constexpr int64_t kChainNm = kStepNm / 2;         // an episode reversed under half a cycle ...
  static constexpr int64_t kChainMs = 30000;               // ... within 30 s of its last move is chained
  static constexpr int32_t kRestUm = 100;                  // at rest: |y - z| < 0.1 mm ...
  static constexpr int64_t kRestMaxMs = 600LL * 1000;      // ... at most 10 min before the first move
  static constexpr int64_t kVerifyMs = 300LL * 1000;       // a correction this long after an unanchored close: unverified
  static constexpr int32_t kStepUm = 200;                  // step-like: from the last estimate within 0.2 mm of the before-
  static constexpr int64_t kStepMs = 2200;                 // level to the first within 0.2 mm of the after-level in <= 2.2 s
  static constexpr int kStepC = 8;                         // (the clean estimates of those 2.2 s kept with the episode) ...
  static constexpr int kPostMin = 3;                       // ... and the level settled after them (>= 3, all within 0.2 mm)
  static constexpr int kChDrop = 2;                        // at a reversal the after-window ends 2 blocks before the move
  static constexpr int32_t kAftTrimUm = 200;               // trailing after-blocks back against the episode by > 0.2 mm: cut
  static constexpr int kRawN = 96;                         // recent clean estimates kept (38 s)
  static constexpr int kMinBlocksMin = 6;
  static constexpr int kSlowN = kLedgerSlowN;              // quiet minutes kept (at least kSlowNMin); medians of the oldest /
  static constexpr int kSlowNMin = 24;                     // newest quarter, centres at least kSlowDtMs apart
  static constexpr int64_t kSlowDtMs = 3600LL * 1000;
  static constexpr int32_t kSlowMinUm = 450;               // x ref / lpmm on a smaller tank, never less
  static constexpr int64_t kSlowKPpmH = 5000;              // 3 x the fastest 15-C thermal fall; ends below 2/3 of the threshold
  static constexpr int64_t kSlowArmMs = 20LL * 60 * 1000;  // the drop passes the threshold this long before a slow drain books
  static constexpr int64_t kSlowSpanMs = 6LL * 3600 * 1000;
  static constexpr int64_t kCreditPpm = 25000;             // slow-rise credit: <= max(0.25 mm, 2.5 % of the level) ...
  static constexpr int64_t kCreditMinUm = 250;
  static constexpr int64_t kCreditMs = 36LL * 3600 * 1000; // ... kept 36 h
  static constexpr int64_t kCarryMs = 600LL * 1000;        // the slow account is carried across an outage of <= 10 min
  static constexpr int32_t kGapMinUm = 700;                // GAP logged from 0.7 mm
  static constexpr int64_t kLpmmRefUl = 1386386;           // the reference tank, uL per mm
  static constexpr int kLogCap = 6;                        // the app drains it after every estimate (<= 5 per estimate)
  static constexpr int64_t kOutClock = -2, kOutUnknown = -1;

  FuelLedger() { init(); }
  void init();                                             // zero totals, no run
  void restore(const LedgerRecord& r);                     // boot: totals; a provisional volume (power cut) is booked
  LedgerRecord record() const;
  // the tracker (re)starts from this stored level, just before the next estimate; outage_ms = time since the last estimate
  // (kOutUnknown: e.g. a cold boot, no carry)
  void begin_run(bool has_stored, int32_t stored_um, int64_t outage_ms = kOutUnknown);
  void push(const LedgerInput& in);
  void end_run();                                          // controlled end: an open episode books its progress
  LedgerRecord clear();                                    // zero totals, gen + 1 (tests; the firmware resets at boot)
  void set_dg(DgState s) { dg_ = s; }
  void set_lpmm(int64_t lpmm_ul);                          // the tank's L/mm x 1e6 (rates are L/h of the reference tank)
  void carry_out(LedgerCarry* c) const;                    // the slow account for a warm reset (RTC)
  void carry_in(const LedgerCarry& c);                     // after a warm reset, before begin_run(..., outage)
  uint32_t carry_seq() const { return carry_seq_; }        // changes when carry_out() would (each block, run change, reset)

  // log entries for the event log, oldest first; false when empty
  bool pop_log(LedgerLog* out);
  uint32_t log_dropped() const { return log_dropped_; }
  uint32_t bookings() const { return bookings_; }          // +1 per booked event, finished slow drain, clear()

  int64_t filled_um() const { return filled_um_; }
  int64_t drained_um() const { return drained_um_; }
  int64_t dgoff_um() const { return dgoff_um_; }
  int64_t slow_um() const { return slow_um_; }             // drained by the slow account since boot (diagnostics)
  uint32_t n_fill() const { return n_fill_; }
  uint32_t n_drain() const { return n_drain_; }
  int32_t pend_um() const { return pend_um_; }
  uint16_t gen() const { return gen_; }
  bool running() const { return run_; }                    // a run is going (false: no estimate yet, or its slow account waits)
  LedgerActive active() const;

 private:
  // blocks and quiet minutes store their index since the run start / the minute grid's origin
  struct Blk { int32_t p, h, idx; bool ac; };
  // the step test of a transition away from a level (fuel_ledger._steprec): the clean estimates of the kStepMs after the last
  // one at that level (has: one was found; tb its time), the furthest of them in the direction dir, and the later estimates
  // followed as they come while the episode is open (post: counted up to 255; moved_t: when the first went further on by
  // > 0.2 mm, if moved)
  struct StepRec { int64_t moved_t; int32_t c[kStepC]; int32_t ext; uint32_t tb; int8_t dir; uint8_t n, post; bool has, moved; };
  // an open episode; its current LEG starts at leg0 (the before-level, or the after-level at its last chained reversal) with
  // the step test cur (first: the first leg's, from the before-level); g = the step-like legs so far; unc: a leg that could not
  // be told (neither a step nor ramped): the episode is then judged as one leg (first)
  struct Ep { int64_t t0, g; int32_t bp, bh, leg0; StepRec cur, first; int8_t s; bool bac, chain, unc; };
  enum class Leg : uint8_t { Ok, Step, Unclear };
  struct Raw { int32_t p; uint32_t t; };                   // a clean estimate (time mod 2^32 ms)

  int64_t bt(const Blk& b) const { return t0_ + (int64_t(b.idx) + 1) * kBlockMs; }
  // full minute index of quiet minute i (every entry is within kSlowSpanMs of the newest) and its end time
  int64_t sidx(int i) const { return slow_last_ - int64_t(uint16_t(uint16_t(slow_last_) - slow_m_[i])); }
  int64_t st(int i) const { return t0m_ + (sidx(i) + 1) * 60000; }

  int32_t lvl(int64_t b_um) const;                         // range -> level um
  int32_t rng(int64_t l_um) const;                         // level -> range um
  void new_run_state();
  void clear_slow(int64_t t0m);
  void stash();
  void start_run(int64_t t_ms);
  void finish_block(int64_t t_end);
  void block(const Blk& b);
  void rest(int64_t t);
  void open_ep(int s, int64_t t, const Ep* inherit);
  bool close_ep(bool provisional, int drop);               // after-window = the first kWinN after-blocks without `drop` newest
  void steprec(int32_t level, int dir, StepRec* r) const; // the step test of a transition leaving `level` in direction dir
  void follow(StepRec* r, int64_t t, int32_t p, int64_t tol) const;   // one more clean estimate after the transition
  // >= kPostMin later estimates, none further on by > 0.2 mm up to the episode's last move
  bool settled(const StepRec& r) const { return r.has && r.post >= kPostMin && (!r.moved || r.moved_t > t_move_); }
  bool steplike(const StepRec& r, int64_t a0, int64_t a) const;   // a transition from a0 (test r) ends at a in one step
  Leg leg(const Ep& ep, int64_t a) const;                  // the current leg (leg0 -> a): a step, ramped / small, or unclear
  int64_t glitch(const Ep& ep, int64_t a) const;           // the part of the episode (ending at a) that is not fuel
  void book(int32_t v, uint32_t dur_s, bool provisional, bool repaired, bool unverified, bool watch, int64_t until);
  void minute(int64_t t_end, const Blk* b);
  void finish_minute(int64_t t);
  void slow_book(int32_t b);
  void bank(int32_t b, int64_t t);
  void slow_end();
  void log(const LedgerLog& e);
  void gap(int32_t p, int32_t h_um);

  // persistent totals
  int64_t filled_um_, drained_um_, dgoff_um_, slow_um_;
  uint32_t n_fill_, n_drain_;
  uint16_t gen_;
  DgState dg_;
  bool run_;
  bool has_stored_, has_next_stored_;
  int32_t stored_um_, next_stored_um_;
  int64_t next_outage_;
  bool anchor_pending_;
  int64_t t_anchor_, t_last_, t_lev_;
  int64_t lpmm_, leak_um_h_, slow_min_um_, ev_min_um_;
  uint32_t bookings_;
  LedgerLog log_[kLogCap];
  int log_n_, log_head_;
  uint32_t log_dropped_;
  bool has_watch_;                                         // an unanchored event watched for a cycle correction until
  int64_t watch_until_;                                    // watch_until_: then an unverified entry follows its own
  int32_t watch_v_;
  uint32_t carry_seq_;

  // run state
  int64_t t0_, blk_idx_;
  int32_t blk_p_[kBlockCap], blk_h_[kBlockCap];
  int blk_n_;
  bool blk_ac_;
  int32_t rep_, goff_;                                     // continuity repairs and the glitch offset (absorbed jumps)
  Blk ring_[kRingN];
  int ring_n_;
  int after_n_;                                            // valid blocks since the last move: the newest after_n_ of ring_
  bool has_prev_move_;
  int64_t t_prev_move_;
  bool has_z_;
  int32_t z_;
  int64_t t_blk_;
  bool has_move_;
  int64_t t_move_;
  bool recentred_;
  bool has_ep_;
  Ep ep_;
  bool has_rest_;                                          // before-window at the last block at rest
  int32_t rest_bp_, rest_bh_;
  bool rest_bac_;
  int64_t rest_bt1_, rest_t_;
  int64_t ev_b_;
  int32_t co_;                                             // chain offset (a carried run continues the old chain)
  bool co_set_, carried_;
  bool has_first_;
  int32_t p_first_, rh_first_, p_gap_;
  bool gap_done_;
  int64_t min_idx_;
  int32_t min_p_[kBlockCap];
  int min_n_;
  bool min_open_;
  Raw raw_[kRawN];                                         // the last clean estimates (step test at an episode's open)
  int raw_n_;
  bool has_k_, has_corr_;                                  // the run's cycle count at the last clean estimate, last change
  int32_t k_last_;
  int64_t corr_t_;
  int32_t nm1_, h_last_;
  int32_t pend_um_;

  // slow account (carried across a short outage)
  int64_t t0m_;                                            // origin of the minute grid
  int32_t slow_q_[kSlowN];                                 // quiet minutes: chain value (position - events + offset) ...
  uint16_t slow_m_[kSlowN];                                // ... and minute index mod 2^16 (6 B each)
  int slow_n_;
  int64_t slow_last_;                                      // full minute index of the newest quiet minute
  bool drain_mode_;
  int32_t slow_low_;
  int64_t slow_ep_um_;
  bool has_floor_;
  int32_t floor_ref_;
  int floor_left_;
  bool rise_mode_, has_ceil_;                              // slow rises: credit for later slow falls (thermal swings)
  int32_t rise_high_, ceil_ref_;
  int ceil_left_;
  int64_t credit_um_, credit_t_;
  bool has_arm_;                                           // the drop passed the threshold since arm_t_ (base a_arm_)
  int64_t arm_t_;
  int32_t a_arm_;
  bool carry_pending_;                                     // an ended run's slow account waits for the next run
  int64_t carry_t_, carry_tlev_;
  int32_t chain_end_, carry_end_;
};

// When the loop task writes the ledger record to NVS (DESIGN.md §13): an event at once, an open
// event's progress every minute once it moved 2 mm since the last write, a slow drain every 30 min once 2 mm were booked,
// anything else every 3 h. A token bucket bounds a fault loop: kBurst back to back, then one per kRefillMs; event bookings
// may use every token, progress checkpoints and slow-drain saves only while more than kReserve remain (the 3-h periodic
// save may use the reserve). The bucket lives in RTC memory (kept across warm resets) and its whole-token count is written
// into every record (byte 1), so a cold boot (power-on, C3 brownout) starts from it, not from a full bucket: with the bucket
// empty, a booking waits up to kRefillMs (10 min) for its write. A failed write still costs its token (no retry storm) but
// keeps the record unsaved. A controlled restart writes regardless (unless nothing changed).
struct LedgerSaveState {
  uint32_t magic;
  uint32_t milli_tokens;
  uint32_t check;
};
class LedgerSaver {
 public:
  static constexpr uint32_t kBurst = 32;                       // an operator session of 30+ cans
  static constexpr uint64_t kRefillMs = 10ULL * 60 * 1000;     // <= 144 + 32 writes a day (a flood); a few a day normally
  static constexpr uint32_t kReserve = 24;                     // tokens kept for event bookings
  static constexpr uint64_t kCheckpointMs = 60000;
  static constexpr int32_t kCheckpointUm = 2000;
  static constexpr uint64_t kPeriodicMs = 3ULL * 3600 * 1000;
  static constexpr uint64_t kSlowSaveMs = 30ULL * 60 * 1000;
  static constexpr int64_t kSlowSaveUm = 2000;
  // cold_tokens: the token count of the stored record (a cold boot starts from it, floor 0); < 0: no record (full bucket).
  // A valid RTC state (warm reset) wins.
  void begin(LedgerSaveState* st, const LedgerRecord& saved, uint32_t bookings, uint64_t now_ms, int cold_tokens = -1);
  bool due(const LedgerRecord& cur, uint32_t bookings, uint64_t now_ms);
  bool unsaved(const LedgerRecord& cur, uint32_t bookings) const { return cur != saved_ || bookings != saved_bookings_; }
  uint8_t debit(uint64_t now_ms);                              // a write is attempted: one token; returns the whole tokens left
  void saved(const LedgerRecord& r, uint32_t bookings, uint64_t now_ms);   // ... and it succeeded
  void restart(const LedgerRecord& saved, uint32_t bookings, uint64_t now_ms);   // a forced write (reset) of this record
  uint32_t tokens() const { return st_ ? st_->milli_tokens / 1000 : 0; }

 private:
  void refill(uint64_t now_ms);
  void seal();
  LedgerSaveState* st_ = nullptr;
  uint64_t last_refill_ms_ = 0;
  LedgerRecord saved_;
  uint32_t saved_bookings_ = 0;
  uint64_t saved_ms_ = 0;
};

}  // namespace fs
