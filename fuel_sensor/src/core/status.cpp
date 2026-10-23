// Status model and JSON writer (DESIGN.md §8).
#include "status.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace fs {

JsonOut::JsonOut(char* buf, size_t cap) : buf_(buf), cap_(cap) {
  if (cap_) buf_[0] = '\0';
  else ok_ = false;
}

void JsonOut::rawn(const char* s, size_t n) {
  if (!ok_) return;
  if (len_ + n + 1 > cap_) { ok_ = false; return; }
  std::memcpy(buf_ + len_, s, n);
  len_ += n;
  buf_[len_] = '\0';
}

void JsonOut::raw(const char* s) { rawn(s, std::strlen(s)); }

void JsonOut::escaped(const char* s) {
  raw("\"");
  for (const char* p = s ? s : ""; *p; ++p) {
    unsigned char c = static_cast<unsigned char>(*p);
    if (c == '"' || c == '\\') { char e[3] = {'\\', char(c), 0}; raw(e); }
    else if (c == '\n') raw("\\n");
    else if (c < 0x20) { char e[8]; std::snprintf(e, sizeof e, "\\u%04x", c); raw(e); }
    else rawn(reinterpret_cast<const char*>(p), 1);
  }
  raw("\"");
}

void JsonOut::key(const char* k) {
  if (need_comma_) raw(",");
  if (k) { escaped(k); raw(":"); }
  need_comma_ = true;
}

JsonOut& JsonOut::begin_obj(const char* k) { key(k); raw("{"); need_comma_ = false; return *this; }
JsonOut& JsonOut::end_obj() { raw("}"); need_comma_ = true; return *this; }
JsonOut& JsonOut::begin_arr(const char* k) { key(k); raw("["); need_comma_ = false; return *this; }
JsonOut& JsonOut::end_arr() { raw("]"); need_comma_ = true; return *this; }
JsonOut& JsonOut::str(const char* k, const char* v) { key(k); escaped(v); return *this; }
JsonOut& JsonOut::null(const char* k) { key(k); raw("null"); return *this; }
JsonOut& JsonOut::boolean(const char* k, bool v) { key(k); raw(v ? "true" : "false"); return *this; }

JsonOut& JsonOut::num(const char* k, double v, int decimals) {
  key(k);
  if (!std::isfinite(v)) { raw("null"); return *this; }
  char t[40];
  std::snprintf(t, sizeof t, "%.*f", decimals < 0 ? 0 : (decimals > 9 ? 9 : decimals), v);
  raw(t);
  return *this;
}

JsonOut& JsonOut::integer(const char* k, long long v) {
  key(k);
  char t[24];
  std::snprintf(t, sizeof t, "%lld", v);
  raw(t);
  return *this;
}

JsonOut& JsonOut::uinteger(const char* k, unsigned long long v) {
  key(k);
  char t[24];
  std::snprintf(t, sizeof t, "%llu", v);
  raw(t);
  return *this;
}

double TankConfig::lpmm() const { return custom ? l_mm * w_mm / 1e6 : kDefaultLitresPerMm; }

const char* status_word_text(StatusWord w) {
  switch (w) {
    case StatusWord::Steady: return "steady";
    case StatusWord::Moving: return "moving";
    case StatusWord::Checking: return "checking";
    case StatusWord::Holding: return "holding";
    case StatusWord::Starting: return "starting";
    case StatusWord::NoRadar: return "no_radar";
    case StatusWord::SafeMode: return "safe_mode";
  }
  return "checking";
}

const char* source_text(Source s) {
  switch (s) {
    case Source::None: return "none";
    case Source::Stored: return "stored";
    case Source::LastConfirmed: return "last_confirmed";
    case Source::Estimate: return "estimate";
  }
  return "none";
}

const char* state_text(TrackState s) {
  switch (s) {
    case TrackState::NoLevel: return "starting";
    case TrackState::Confirmed: return "confirmed";
    case TrackState::Uncertain: return "uncertain";
    case TrackState::Holding: return "holding";
    case TrackState::Fault: return "fault";
  }
  return "starting";
}

const char* cycle_text(CycleSource c) {
  switch (c) {
    case CycleSource::Assumed: return "assumed";
    case CycleSource::Shape: return "shape";
    case CycleSource::Anchored: return "anchored";
    case CycleSource::Memory: return "memory";
  }
  return "assumed";
}

const char* fuel_active_text(uint8_t a) {
  switch (a) {
    case 1: return "fill";
    case 2: return "drain";
    case 3: return "slow_drain";
    default: return "none";
  }
}

StatusWord status_word(const DeviceStatus& s) {
  if (s.safe_mode) return StatusWord::SafeMode;
  if (!s.radar_ok) return StatusWord::NoRadar;
  if (s.out.source != Source::Estimate) return StatusWord::Starting;
  switch (s.out.state) {
    case TrackState::Holding: return StatusWord::Holding;
    case TrackState::Confirmed: return s.out.moving ? StatusWord::Moving : StatusWord::Steady;
    case TrackState::Fault: return StatusWord::NoRadar;
    default: return StatusWord::Checking;
  }
}

// A customer's downstream register (0-4095, linear on litres: DESIGN.md §8): 0 = empty, 4095 = this tank's configured
// capacity. Rounded and clamped so it always fits the register, even with a stale/overfull reading.
int raw_litres_of(double litres, double capacity) {
  if (!(capacity > 0)) return 0;
  double r = std::round(litres * kRawLitresMax / capacity);
  if (r < 0) return 0;
  if (r > kRawLitresMax) return kRawLitresMax;
  return int(r);
}

void write_status_fields(JsonOut& j, const DeviceStatus& s, StatusView view) {
  const bool full = view == StatusView::Full;
  double lpmm = s.tank.lpmm();
  double level = s.out.level_mm;
  double litres = level * lpmm;
  double capacity = s.tank.h_mm * lpmm;
  double fill = capacity > 0 ? 100.0 * litres / capacity : 0.0;
  if (fill < 0) fill = 0;
  j.str("fw", s.fw);
  if (full) j.str("tracker", s.tracker).str("calibration", s.calibration);
  j.str("variant", s.variant).str("chip", s.chip).str("mac", s.mac)
      .num("level_mm", level, 2).num("litres", litres, 2).num("capacity_l", capacity, 1).num("fill_pct", fill, 1)
      .uinteger("raw_litres", (unsigned)raw_litres_of(litres, capacity))
      .str("source", source_text(s.out.source));
  if (full) j.str("state", s.safe_mode ? "fault" : state_text(s.out.state));
  j.str("status", status_word_text(status_word(s)));
  if (full) j.boolean("moving", s.out.moving);
  j.num("age_s", s.age_s, 1);
  if (s.temp_valid) j.integer("temp_c", s.temp_c);
  else j.null("temp_c");
  j.uinteger("uptime_s", s.uptime_s).uinteger("boots", s.boots).uinteger("power_interruptions", s.power_interruptions);
  if (full) j.uinteger("settings_changes", s.settings_changes);   // tank changes + resets to defaults (never reset)
  j.begin_obj("tank")
      .num("l_mm", s.tank.l_mm, 1).num("w_mm", s.tank.w_mm, 1).num("h_mm", s.tank.h_mm, 1)
      .num("lpmm", lpmm, 5).boolean("custom", s.tank.custom)
      .end_obj();
  j.begin_obj("radar").str("link", s.safe_mode ? "off" : (s.radar_ok ? "ok" : "lost"));
  if (full) j.uinteger("frames", s.frames).uinteger("bad_packets", s.bad_packets).uinteger("gaps", s.gaps);
  j.end_obj();
  // fuel ledger (DESIGN.md §13): the customer data (litres filled / drained since installation or the last reset with the
  // current tank setting, event counts, the event in progress) in both views; the exact level-um totals (the server
  // recalculates litres), the reset generation and the DG-off hook (no DG input yet: null) only in the full view
  j.begin_obj("fuel")
      .num("filled_l", double(s.fuel.filled_um) * lpmm / 1000.0, 2).num("drained_l", double(s.fuel.drained_um) * lpmm / 1000.0, 2)
      .uinteger("fills", s.fuel.fills).uinteger("drains", s.fuel.drains)
      .str("active", fuel_active_text(s.fuel.active)).num("pending_l", double(s.fuel.pending_um) * lpmm / 1000.0, 2);
  if (full)
    j.integer("filled_um", s.fuel.filled_um).integer("drained_um", s.fuel.drained_um).uinteger("gen", s.fuel.gen)
        .null("drained_dg_off_l");
  j.end_obj();
  j.begin_obj("health")
      .boolean("safe_mode", s.safe_mode).boolean("trial", s.trial).str("reset_reason", s.reset_reason)
      .boolean("coredump", s.coredump);
  if (full)
    j.uinteger("heap_min", s.heap_min).uinteger("max_est_us", s.max_est_us)
        .str("raw_stream", s.raw_stream == 1 ? "ok" : (s.raw_stream == 2 ? "no_memory" : "off"));
  j.end_obj();
  // no "session" flag in any view (REV-2: it tells a sniffer when to look)
}

bool write_status_json(const DeviceStatus& s, char* buf, size_t cap, size_t* len, StatusView view) {
  JsonOut j(buf, cap);
  j.begin_obj();
  write_status_fields(j, s, view);
  j.end_obj();
  if (len) *len = j.len();
  return j.ok();
}

}  // namespace fs
