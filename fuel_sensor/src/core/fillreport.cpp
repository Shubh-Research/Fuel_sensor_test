#include "fillreport.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include "challenge.h"
#include "crypto.h"
#include "status.h"

namespace fs {

namespace {

void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v >> 8); p[1] = uint8_t(v); }
void put32(uint8_t* p, uint32_t v) { put16(p, uint16_t(v >> 16)); put16(p + 2, uint16_t(v)); }
void put64(uint8_t* p, uint64_t v) { put32(p, uint32_t(v >> 32)); put32(p + 4, uint32_t(v)); }
uint16_t get16(const uint8_t* p) { return uint16_t(p[0] << 8 | p[1]); }
uint32_t get32(const uint8_t* p) { return uint32_t(get16(p)) << 16 | get16(p + 2); }
uint64_t get64(const uint8_t* p) { return uint64_t(get32(p)) << 32 | get32(p + 4); }

// Days since 1970-01-01 -> civil date (H. Hinnant's algorithm).
void civil(int64_t z, int* y, unsigned* m, unsigned* d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = unsigned(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  *d = doy - (153 * mp + 2) / 5 + 1;
  *m = mp < 10 ? mp + 3 : mp - 9;
  *y = int(yoe + era * 400 + (*m <= 2));
}

const char* const kMonths[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

struct Local { int64_t day; unsigned hh, mm; };
Local local_of(uint32_t unix_s, int16_t tz_min) {
  const int64_t t = int64_t(unix_s) + int64_t(tz_min) * 60;
  int64_t day = t >= 0 ? t / 86400 : (t - 86399) / 86400;
  const int64_t sod = t - day * 86400;
  return Local{day, unsigned(sod / 3600), unsigned(sod % 3600 / 60)};
}

// Appends with snprintf semantics: *k never passes cap - 1.
void app(char* out, size_t cap, size_t* k, const char* fmt, ...) __attribute__((format(printf, 4, 5)));
void app(char* out, size_t cap, size_t* k, const char* fmt, ...) {
  if (*k >= cap) return;
  va_list ap;
  va_start(ap, fmt);
  const int n = std::vsnprintf(out + *k, cap - *k, fmt, ap);
  va_end(ap);
  if (n > 0) *k = *k + size_t(n) < cap ? *k + size_t(n) : cap - 1;
}

void app_litres(char* out, size_t cap, size_t* k, uint32_t cl) {
  const uint32_t dl = (cl + 5) / 10;                     // one decimal
  app(out, cap, k, "%lu.%lu L", (unsigned long)(dl / 10), (unsigned long)(dl % 10));
}

}  // namespace

void fill_pack(const FillReport& r, uint8_t o[kFillPayloadBytes]) {
  o[0] = r.format;
  o[1] = r.key_id;
  std::memcpy(o + 2, r.mac, 6);
  put16(o + 8, r.fw_version);
  put32(o + 10, r.serial);
  put16(o + 14, r.since_reset);
  put16(o + 16, r.gen);
  put16(o + 18, r.settings_changes);
  put32(o + 20, r.phone_start);
  put32(o + 24, r.phone_end);
  put16(o + 28, uint16_t(r.tz_min));
  put32(o + 30, r.duration_s);
  put16(o + 34, r.lookback_s);
  put32(o + 36, r.operator_cl);
  put32(o + 40, r.sensor_cl);
  put32(o + 44, r.drained_cl);
  put32(o + 48, uint32_t(r.level_before));
  put32(o + 52, uint32_t(r.level_after));
  put32(o + 56, r.lpmm_ul);
  put16(o + 60, r.flags);
}

bool fill_unpack(const uint8_t* in, size_t n, FillReport* r) {
  if (n < kFillPayloadBytes || in[0] != kFillFormat) return false;
  r->format = in[0];
  r->key_id = in[1];
  std::memcpy(r->mac, in + 2, 6);
  r->fw_version = get16(in + 8);
  r->serial = get32(in + 10);
  r->since_reset = get16(in + 14);
  r->gen = get16(in + 16);
  r->settings_changes = get16(in + 18);
  r->phone_start = get32(in + 20);
  r->phone_end = get32(in + 24);
  r->tz_min = int16_t(get16(in + 28));
  r->duration_s = get32(in + 30);
  r->lookback_s = get16(in + 34);
  r->operator_cl = get32(in + 36);
  r->sensor_cl = get32(in + 40);
  r->drained_cl = get32(in + 44);
  r->level_before = int32_t(get32(in + 48));
  r->level_after = int32_t(get32(in + 52));
  r->lpmm_ul = get32(in + 56);
  r->flags = get16(in + 60);
  return true;
}

void fill_digest(const uint8_t payload[kFillPayloadBytes], uint8_t digest[32]) {
  Sha256Ctx c;
  sha256_begin(&c);
  sha256_update(&c, reinterpret_cast<const uint8_t*>(kFillDomain), kFillDomainLen);
  sha256_update(&c, payload, kFillPayloadBytes);
  sha256_finish(&c, digest);
}

size_t fill_code_text(const uint8_t code[kFillCodeBytes], char* out, size_t cap) {
  char sym[kFillTextMax];
  const size_t n = b32_encode(code, kFillCodeBytes, sym, sizeof sym);
  if (!n || cap < 3 + n + (n - 1) / 5 + 1) return 0;
  size_t k = 0;
  out[k++] = 'F';
  out[k++] = 'R';
  for (size_t i = 0; i < n; ++i) {
    if (i % 5 == 0) out[k++] = '-';
    out[k++] = sym[i];
  }
  out[k] = '\0';
  return k;
}

bool fill_code_from_text(const char* text, uint8_t out[kFillCodeBytes]) {
  if (!text) return false;
  while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') ++text;
  // A code starts with format 1 (its first symbol is '0'), so "FR" can only be the prefix.
  if ((text[0] == 'F' || text[0] == 'f') && (text[1] == 'R' || text[1] == 'r')) text += 2;
  return b32_decode(text, out, kFillCodeBytes);
}

uint32_t fill_cl(int64_t um, uint32_t lpmm_ul) {
  if (um <= 0) return 0;
  const int64_t cl = (um * int64_t(lpmm_ul) + 5000000) / 10000000;   // um x uL/mm / 1e7 = litres x 100
  return cl > 0xFFFFFFFELL ? 0xFFFFFFFEu : uint32_t(cl);
}

size_t fill_readable(const FillReport& r, char* out, size_t cap) {
  if (!cap) return 0;
  size_t k = 0;
  out[0] = '\0';
  app(out, cap, &k, "Fill report #%lu \xC2\xB7 MRM-Fuel-%02X%02X%02X%02X%02X%02X \xC2\xB7 ", (unsigned long)r.serial,
      r.mac[0], r.mac[1], r.mac[2], r.mac[3], r.mac[4], r.mac[5]);
  if (r.phone_start) {
    const Local a = local_of(r.phone_start, r.tz_min);
    int y;
    unsigned m, d;
    civil(a.day, &y, &m, &d);
    app(out, cap, &k, "%u %s %02u:%02u", d, kMonths[(m - 1) % 12], a.hh, a.mm);
    if (r.phone_end) {
      const Local b = local_of(r.phone_end, r.tz_min);
      if (b.day != a.day) {
        civil(b.day, &y, &m, &d);
        app(out, cap, &k, "\xE2\x80\x93%u %s %02u:%02u", d, kMonths[(m - 1) % 12], b.hh, b.mm);
      } else {
        app(out, cap, &k, "\xE2\x80\x93%02u:%02u", b.hh, b.mm);
      }
    } else {
      app(out, cap, &k, " (not ended)");
    }
  } else {
    app(out, cap, &k, "time unknown");
  }
  app(out, cap, &k, " \xC2\xB7 operator ");
  if (r.operator_cl == kFillOperatorNone) app(out, cap, &k, "not entered");
  else app_litres(out, cap, &k, r.operator_cl);
  app(out, cap, &k, " \xC2\xB7 sensor ");
  app_litres(out, cap, &k, r.sensor_cl);
  if (r.flags & kFillFlagsKnown) {
    static const char* const kNames[7] = {"restart", "unsettled", "short look-back", "tank changed", "level unconfirmed",
                                          "unverified", "auto-closed"};
    app(out, cap, &k, " \xC2\xB7 flags:");
    bool first = true;
    for (int i = 0; i < 7; ++i) {
      if (!(r.flags & (1u << i))) continue;
      app(out, cap, &k, "%s %s", first ? "" : ",", kNames[i]);
      first = false;
    }
  }
  return k;
}

const char* fill_stage_text(FillStage s) {
  switch (s) {
    case FillStage::Idle: return "idle";
    case FillStage::Filling: return "filling";
    case FillStage::Measuring: return "measuring";
    case FillStage::Ready: return "ready";
  }
  return "idle";
}

const char* fill_result_text(FillResult r) {
  switch (r) {
    case FillResult::Ok: return "ok";
    case FillResult::Off: return "off";
    case FillResult::Busy: return "busy";
    case FillResult::NotOpen: return "not_open";
    case FillResult::NotReady: return "not_ready";
    case FillResult::Measuring: return "measuring";
    case FillResult::BadInput: return "bad_input";
    case FillResult::RateLimited: return "rate_limited";
    case FillResult::SignError: return "sign_error";
    case FillResult::StoreError: return "store_error";
  }
  return "error";
}

void fill_nv_encode(const FillNv& nv, uint8_t o[kFillNvBytes]) {
  o[0] = 1;
  o[1] = nv.stage;
  put32(o + 2, nv.serial);
  put16(o + 6, nv.since_reset);
  put16(o + 8, nv.flags);
  put32(o + 10, nv.phone_start);
  put32(o + 14, nv.phone_end);
  put16(o + 18, uint16_t(nv.tz_min));
  put16(o + 20, nv.lookback_s);
  put32(o + 22, nv.dur_s);
  put64(o + 26, uint64_t(nv.base_filled_um));
  put64(o + 34, uint64_t(nv.base_drained_um));
  put32(o + 42, uint32_t(nv.base_level_um));
  put16(o + 46, nv.base_gen);
  put32(o + 48, nv.base_settings);
}

bool fill_nv_decode(const uint8_t* in, size_t n, FillNv* nv) {
  if (n < kFillNvBytes || in[0] != 1 || in[1] > uint8_t(FillStage::Ready)) return false;
  nv->stage = in[1];
  nv->serial = get32(in + 2);
  nv->since_reset = get16(in + 6);
  nv->flags = get16(in + 8);
  nv->phone_start = get32(in + 10);
  nv->phone_end = get32(in + 14);
  nv->tz_min = int16_t(get16(in + 18));
  nv->lookback_s = get16(in + 20);
  nv->dur_s = get32(in + 22);
  nv->base_filled_um = int64_t(get64(in + 26));
  nv->base_drained_um = int64_t(get64(in + 34));
  nv->base_level_um = int32_t(get32(in + 42));
  nv->base_gen = get16(in + 46);
  nv->base_settings = get32(in + 48);
  return true;
}

FillNv fill_nv_after_reset(const FillNv& nv) {
  FillNv z;
  z.serial = nv.serial;                                  // the lifetime serial is never reset
  return z;
}

bool fill_write_json(JsonOut& j, const char* key, const uint8_t code[kFillCodeBytes]) {
  FillReport r;
  if (!fill_unpack(code, kFillCodeBytes, &r)) return false;
  char text[kFillTextMax], line[kFillLineMax];
  fill_code_text(code, text, sizeof text);
  fill_readable(r, line, sizeof line);
  j.begin_obj(key).uinteger("serial", r.serial).uinteger("since_reset", r.since_reset).str("line", line).str("code", text);
  if (r.operator_cl == kFillOperatorNone) j.null("operator_l");
  else j.num("operator_l", r.operator_cl / 100.0, 2);
  j.num("sensor_l", r.sensor_cl / 100.0, 2).num("drained_l", r.drained_cl / 100.0, 2)
      .num("level_before_mm", r.level_before / 100.0, 2).num("level_after_mm", r.level_after / 100.0, 2)
      .num("lpmm", r.lpmm_ul / 1e6, 6).uinteger("phone_start", r.phone_start).uinteger("phone_end", r.phone_end)
      .integer("tz_min", r.tz_min).uinteger("duration_s", r.duration_s).uinteger("lookback_s", r.lookback_s)
      .uinteger("gen", r.gen).uinteger("settings_changes", r.settings_changes).uinteger("flags", r.flags).end_obj();
  return j.ok();
}

// ---------------------------------------------------------------- FillBook

void FillBook::begin(FillStore* store, const FillNv& loaded, const uint8_t* key_d, uint8_t key_id, const uint8_t mac[6],
                     uint16_t fw_version, uint32_t now_s) {
  store_ = store;
  key_ = key_d;
  key_id_ = key_id;
  std::memcpy(mac_, mac, 6);
  fw_ = fw_version;
  nv_ = loaded;
  boot_t_ = now_s;
  tok_t_ = now_s;
  stage_ = FillStage(nv_.stage > uint8_t(FillStage::Ready) ? 0 : nv_.stage);
  if (stage_ == FillStage::Ready) stage_ = FillStage::Measuring;
  if (stage_ != FillStage::Idle) {
    // A report open across a restart continues from this boot (its base totals are in NVS); the time the device was
    // off is not measured and the ledger does not count fuel across the outage: flagged.
    nv_.flags |= kFillFlagRestart;
    start_t_ = now_s;
    end_t_ = now_s;
  }
}

uint32_t FillBook::tokens(uint32_t now_s) {
  const uint32_t dt = now_s - tok_t_;
  tok_t_ = now_s;
  const uint64_t m = uint64_t(tok_milli_) + uint64_t(dt) * 1000 / kTokenS;
  tok_milli_ = m > kTokens * 1000 ? kTokens * 1000 : uint32_t(m);
  return tok_milli_ / 1000;
}

void FillBook::push_sample(uint32_t now_s, const FillLive& live) {
  FillSample& s = ring_[ring_head_];
  s.t_s = now_s;
  s.level_um = live.level_um;
  s.filled_um = live.filled_um;
  s.drained_um = live.drained_um;
  s.confirmed = live.confirmed;
  s.has_level = live.has_level;
  ring_head_ = (ring_head_ + 1) % kRingN;
  if (ring_n_ < kRingN) ++ring_n_;
  last_sample_t_ = now_s;
  sampled_ = true;
}

void FillBook::tick(uint32_t now_s, const FillLive& live, uint32_t settings, FillDone* auto_done, bool* auto_closed) {
  if (auto_closed) *auto_closed = false;
  if (!sampled_ || now_s - last_sample_t_ >= kSampleS) push_sample(now_s, live);
  if (stage_ == FillStage::Idle) return;
  if (!live.confirmed) nv_.flags |= kFillFlagUncertain;
  if (stage_ == FillStage::Filling && now_s - start_t_ >= kAutoCloseS) {
    // Never ended: the device ends it (phone end time unknown) and closes it once the level has settled.
    nv_.flags |= kFillFlagAutoClosed;
    nv_.dur_s += now_s - start_t_;
    nv_.phone_end = 0;
    nv_.stage = uint8_t(FillStage::Measuring);
    end_t_ = now_s;
    stage_ = FillStage::Measuring;
    store_->save(nv_);                                   // (a failure only loses the End across a restart)
  }
  if (stage_ == FillStage::Measuring) {
    const uint32_t since = now_s - end_t_;
    if (since >= kSettleMaxS) settle(now_s, live, true);
    else if (since >= kSettleMinS && live.pending_um == 0) settle(now_s, live, false);
  }
  if (stage_ == FillStage::Ready && ((nv_.flags & kFillFlagAutoClosed) || now_s - end_t_ >= kAutoCloseS)) {
    if (retry_any_ && now_s - retry_t_ < kRetryS) return;
    nv_.flags |= kFillFlagAutoClosed;
    FillDone tmp;
    FillDone* d = auto_done ? auto_done : &tmp;
    if (finish(now_s, kFillOperatorNone, live, settings, d) == FillResult::Ok) {
      if (auto_closed) *auto_closed = true;
      retry_any_ = false;
    } else {
      retry_any_ = true;
      retry_t_ = now_s;
    }
  }
}

int32_t FillBook::last_level(int32_t fallback) const {
  for (int i = 0; i < ring_n_; ++i) {                    // the newest sample with a level
    const FillSample& s = ring_[(ring_head_ - 1 - i + 2 * kRingN) % kRingN];
    if (s.has_level) return s.level_um;
  }
  return fallback;
}

void FillBook::settle(uint32_t now_s, const FillLive& live, bool timed_out) {
  if (timed_out) nv_.flags |= kFillFlagUnsettled;
  end_filled_ = live.filled_um;
  end_drained_ = live.drained_um;
  end_level_ = live.has_level ? live.level_um : last_level(end_level_);
  settle_t_ = now_s;
  stage_ = FillStage::Ready;
}

FillResult FillBook::start(uint32_t now_s, uint32_t phone_t, int16_t tz_min, const FillLive& live, uint32_t settings) {
  if (!enabled()) return FillResult::Off;
  if (stage_ != FillStage::Idle) return FillResult::Busy;
  if (tz_min < -840 || tz_min > 840) return FillResult::BadInput;
  if (tokens(now_s) < 1) return FillResult::RateLimited;
  if (!sampled_ || now_s != last_sample_t_) push_sample(now_s, live);   // the newest sample is "now"
  // The window start: 10 min before Start, never before the previous report's end (this boot) nor before the boot.
  const uint32_t want = now_s - boot_t_ >= kLookbackS ? now_s - kLookbackS : boot_t_;
  const FillSample* base = nullptr;
  const FillSample* oldest = nullptr;
  for (int i = 0; i < ring_n_; ++i) {
    const FillSample& s = ring_[(ring_head_ - 1 - i + 2 * kRingN) % kRingN];   // newest first
    oldest = &s;
    if (!base && s.t_s - boot_t_ <= want - boot_t_) base = &s;   // the newest sample at or before `want`
  }
  bool short_lb = false;
  if (!base) {                                           // the ring does not reach back that far: its oldest sample
    base = oldest;
    short_lb = true;
  } else if (want == boot_t_ && now_s - boot_t_ < kLookbackS) {
    short_lb = true;
  }
  FillNv nv = nv_;
  nv.flags = 0;
  uint32_t t0 = base->t_s;
  nv.base_filled_um = base->filled_um;
  nv.base_drained_um = base->drained_um;
  nv.base_level_um = base->level_um;
  if (!base->has_level) {                                // no level yet at the window start (just after boot): the first
    nv.base_level_um = live.has_level ? live.level_um : 0;   // level the tracker gave after it (or now; 0 if none, flagged)
    for (int i = 0; i < ring_n_; ++i) {                  // newest first, down to the base
      const FillSample& s = ring_[(ring_head_ - 1 - i + 2 * kRingN) % kRingN];
      if (s.t_s - boot_t_ < t0 - boot_t_) break;
      if (s.has_level) nv.base_level_um = s.level_um;
    }
  }
  if (prev_ && prev_t_ - boot_t_ >= t0 - boot_t_) {       // the previous report ended inside the look-back: start exactly
    t0 = prev_t_;                                        // at its end (the totals it was settled with)
    nv.base_filled_um = prev_filled_;
    nv.base_drained_um = prev_drained_;
    nv.base_level_um = prev_level_;
    short_lb = false;
  }
  if (short_lb) nv.flags |= kFillFlagShortLookback;
  for (int i = 0; i < ring_n_; ++i) {
    const FillSample& s = ring_[i];
    if (s.t_s - boot_t_ >= t0 - boot_t_ && !s.confirmed) nv.flags |= kFillFlagUncertain;
  }
  if (tank_any_ && tank_t_ - boot_t_ >= t0 - boot_t_) nv.flags |= kFillFlagTankChanged;
  if (unv_any_ && unv_t_ - boot_t_ >= t0 - boot_t_) nv.flags |= kFillFlagUnverified;
  if (!live.confirmed) nv.flags |= kFillFlagUncertain;
  nv.stage = uint8_t(FillStage::Filling);
  nv.phone_start = phone_t;
  nv.phone_end = 0;
  nv.tz_min = tz_min;
  nv.lookback_s = uint16_t(now_s - t0 > 0xFFFF ? 0xFFFF : now_s - t0);
  nv.dur_s = 0;
  nv.base_gen = live.gen;
  nv.base_settings = settings;
  if (!store_->save(nv)) return FillResult::StoreError;
  tok_milli_ -= 1000;
  nv_ = nv;
  stage_ = FillStage::Filling;
  start_t_ = now_s;
  return FillResult::Ok;
}

FillResult FillBook::end(uint32_t now_s, uint32_t phone_t) {
  if (!enabled()) return FillResult::Off;
  if (stage_ != FillStage::Filling) return stage_ == FillStage::Idle ? FillResult::NotOpen : FillResult::Busy;
  FillNv nv = nv_;
  nv.stage = uint8_t(FillStage::Measuring);
  nv.phone_end = phone_t;
  nv.dur_s += now_s - start_t_;
  if (!store_->save(nv)) return FillResult::StoreError;
  nv_ = nv;
  stage_ = FillStage::Measuring;
  end_t_ = now_s;
  return FillResult::Ok;
}

FillResult FillBook::confirm(uint32_t now_s, uint32_t operator_cl, const FillLive& live, uint32_t settings, FillDone* out) {
  if (!enabled()) return FillResult::Off;
  if (stage_ == FillStage::Idle) return FillResult::NotOpen;
  if (stage_ == FillStage::Filling) return FillResult::NotReady;
  if (stage_ == FillStage::Measuring) return FillResult::Measuring;
  if (operator_cl > 10000000u) return FillResult::BadInput;       // 100,000 L
  return finish(now_s, operator_cl, live, settings, out);
}

FillResult FillBook::finish(uint32_t /*now_s*/, uint32_t operator_cl, const FillLive& live, uint32_t settings, FillDone* out) {
  FillReport& r = out->r;
  r = FillReport{};
  r.key_id = key_id_;
  std::memcpy(r.mac, mac_, 6);
  r.fw_version = fw_;
  r.serial = nv_.serial + 1;
  r.since_reset = uint16_t(nv_.since_reset == 0xFFFF ? 0xFFFF : nv_.since_reset + 1);
  r.gen = live.gen;
  r.settings_changes = uint16_t(settings > 0xFFFF ? 0xFFFF : settings);
  r.phone_start = nv_.phone_start;
  r.phone_end = nv_.phone_end;
  r.tz_min = nv_.tz_min;
  r.duration_s = nv_.dur_s;
  r.lookback_s = nv_.lookback_s;
  r.operator_cl = operator_cl;
  const bool same_gen = live.gen == nv_.base_gen;
  r.sensor_cl = same_gen ? fill_cl(end_filled_ - nv_.base_filled_um, live.lpmm_ul) : 0;
  r.drained_cl = same_gen ? fill_cl(end_drained_ - nv_.base_drained_um, live.lpmm_ul) : 0;
  r.level_before = nv_.base_level_um / 10;
  r.level_after = end_level_ / 10;
  r.lpmm_ul = live.lpmm_ul;
  uint16_t flags = nv_.flags;
  if (settings != nv_.base_settings) flags |= kFillFlagTankChanged;
  if (!same_gen) flags |= kFillFlagRestart;
  r.flags = uint16_t(flags & kFillFlagsKnown);
  fill_pack(r, out->code);
  uint8_t digest[32];
  fill_digest(out->code, digest);
  if (!ecdsa_p256_sign(key_, digest, out->code + kFillPayloadBytes)) return FillResult::SignError;
  // The counters and the closed report in ONE durable write BEFORE the code leaves the device: a serial is never
  // issued twice, and a failed write issues none.
  FillNv nv;
  nv.serial = r.serial;
  nv.since_reset = r.since_reset;
  if (!store_->save(nv)) return FillResult::StoreError;
  nv_ = nv;
  stage_ = FillStage::Idle;
  store_->append(out->code);
  store_->logged(r);
  prev_ = true;
  prev_t_ = settle_t_;
  prev_filled_ = end_filled_;
  prev_drained_ = end_drained_;
  prev_level_ = end_level_;
  return FillResult::Ok;
}

void FillBook::apply_reset() {
  nv_ = fill_nv_after_reset(nv_);
  stage_ = FillStage::Idle;
  store_->save(nv_);
}

}  // namespace fs
