// Challenge / response wire formats (DESIGN.md §4). Pure C++, host-testable.
#include "challenge.h"

#include <cstring>

namespace fs {

namespace {

constexpr char kAlphabet[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";   // Crockford base32

int symbol_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'z') c = char(c - 'a' + 'A');
  switch (c) {
    case 'O': return 0;
    case 'I': case 'L': return 1;
    case 'U': return -1;
    default: break;
  }
  for (int v = 10; v < 32; ++v)
    if (kAlphabet[v] == c) return v;
  return -1;
}

bool is_separator(char c) { return c == '-' || c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

size_t symbols_for(size_t nbytes) { return (nbytes * 8 + 4) / 5; }

// Copies the symbols of `text` (separators dropped) into buf; returns the count, or SIZE_MAX on a bad character.
size_t collect_symbols(const char* text, char* buf, size_t cap) {
  size_t k = 0;
  for (const char* p = text; *p; ++p) {
    if (is_separator(*p)) continue;
    if (symbol_value(*p) < 0) return SIZE_MAX;   // 'F' and 'S' (the prefix) are ordinary symbols
    if (k >= cap) return SIZE_MAX;
    buf[k++] = *p;
  }
  return k;
}

size_t grouped(const char* syms, size_t n, size_t group, const char* prefix, char* out, size_t cap) {
  size_t plen = std::strlen(prefix);
  size_t need = plen + n + (n ? (n - 1) / group : 0) + 1;
  if (cap < need) return 0;
  size_t k = 0;
  for (size_t i = 0; i < plen; ++i) out[k++] = prefix[i];
  for (size_t i = 0; i < n; ++i) {
    if (i && i % group == 0) out[k++] = '-';
    out[k++] = syms[i];
  }
  out[k] = '\0';
  return k;
}

void put24(uint8_t* p, uint32_t v) { p[0] = uint8_t(v >> 16); p[1] = uint8_t(v >> 8); p[2] = uint8_t(v); }
uint32_t get24(const uint8_t* p) { return uint32_t(p[0]) << 16 | uint32_t(p[1]) << 8 | p[2]; }

}  // namespace

uint8_t Grant::byte() const {
  uint8_t h = hours < 1 ? 1 : (hours > 15 ? 15 : hours);
  return uint8_t((settings ? 1 : 0) | (firmware ? 2 : 0) | (calibration ? 4 : 0) | (older ? 8 : 0) | (h << 4));
}

Grant Grant::from_byte(uint8_t b) {
  Grant g;
  g.settings = b & 1;
  g.firmware = b & 2;
  g.calibration = b & 4;
  g.older = b & 8;
  g.hours = uint8_t(b >> 4);
  if (g.hours == 0) g.hours = 1;
  return g;
}

uint8_t Grant::service_byte() const {
  uint8_t h = hours < 1 ? 1 : (hours > 15 ? 15 : hours);
  return uint8_t((service ? 1 : 0) | (h << 4));
}

Grant Grant::from_service_byte(uint8_t b) {
  Grant g;
  g.service = (b & 1) && !(b & 0x0E);           // bits 1-3 are reserved: a grant that sets them grants nothing
  g.hours = uint8_t(b >> 4);
  if (g.hours == 0) g.hours = 1;
  return g;
}

uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

size_t b32_encode(const uint8_t* in, size_t n, char* out, size_t cap) {
  size_t nsym = symbols_for(n);
  if (cap < nsym + 1) return 0;
  uint32_t acc = 0;
  int bits = 0;
  size_t k = 0;
  for (size_t i = 0; i < n; ++i) {
    acc = (acc << 8) | in[i];
    bits += 8;
    while (bits >= 5) {
      out[k++] = kAlphabet[(acc >> (bits - 5)) & 31];
      bits -= 5;
    }
  }
  if (bits > 0) out[k++] = kAlphabet[(acc << (5 - bits)) & 31];
  out[k] = '\0';
  return k;
}

bool b32_decode(const char* text, uint8_t* out, size_t n) {
  size_t nsym = symbols_for(n);
  uint32_t acc = 0;
  int bits = 0;
  size_t k = 0, nbytes = 0;
  for (const char* p = text; *p; ++p) {
    if (is_separator(*p)) continue;
    int v = symbol_value(*p);
    if (v < 0) return false;
    if (++k > nsym) return false;
    acc = (acc << 5) | uint32_t(v);
    bits += 5;
    if (bits >= 8) {
      if (nbytes >= n) return false;
      out[nbytes++] = uint8_t(acc >> (bits - 8));
      bits -= 8;
    }
    acc &= (1u << bits) - 1u;         // keep only the unconsumed bits
  }
  if (k != nsym || nbytes != n) return false;
  return acc == 0;                    // padding bits must be zero
}

void challenge_pack(const ChallengeFields& f, uint8_t out[kChallengeBytes]) {
  out[0] = uint8_t(kChallengeFormat << 5 | (f.calibrated ? 0x10 : 0) | (f.last_ok ? 0x08 : 0) |
                   (f.last_known ? 0x04 : 0) | (f.locked ? 0x02 : 0));
  std::memcpy(out + 1, f.mac, 6);
  out[7] = uint8_t(f.ok_count >> 8);
  out[8] = uint8_t(f.ok_count);
  out[9] = f.fails;
  put24(out + 10, f.last_attempt_min & 0xFFFFFFu);
  put24(out + 13, f.phone_min & 0xFFFFFFu);
  out[16] = uint8_t(f.fw_version >> 8);
  out[17] = uint8_t(f.fw_version);
  put24(out + 18, f.nonce & 0xFFFFFFu);
  uint16_t crc = crc16_ccitt_false(out, 21);
  out[21] = uint8_t(crc >> 8);
  out[22] = uint8_t(crc);
}

bool challenge_unpack(const uint8_t in[kChallengeBytes], ChallengeFields* f) {
  if ((in[0] >> 5) != kChallengeFormat || (in[0] & 1)) return false;
  if (crc16_ccitt_false(in, 21) != (uint16_t(in[21]) << 8 | in[22])) return false;
  f->calibrated = in[0] & 0x10;
  f->last_ok = in[0] & 0x08;
  f->last_known = in[0] & 0x04;
  f->locked = in[0] & 0x02;
  std::memcpy(f->mac, in + 1, 6);
  f->ok_count = uint16_t(in[7] << 8 | in[8]);
  f->fails = in[9];
  f->last_attempt_min = get24(in + 10);
  f->phone_min = get24(in + 13);
  f->fw_version = uint16_t(in[16] << 8 | in[17]);
  f->nonce = get24(in + 18);
  return true;
}

size_t challenge_to_text(const uint8_t b[kChallengeBytes], char* out, size_t cap) {
  char syms[40];
  size_t n = b32_encode(b, kChallengeBytes, syms, sizeof syms);
  return grouped(syms, n, 5, "FS-", out, cap);
}

bool challenge_from_text(const char* text, uint8_t out[kChallengeBytes]) {
  char syms[64];
  size_t n = collect_symbols(text, syms, sizeof syms - 1);
  if (n == SIZE_MAX) return false;
  size_t start = 0;
  size_t want = symbols_for(kChallengeBytes);
  if (n == want + 2 && (syms[0] == 'F' || syms[0] == 'f') && (syms[1] == 'S' || syms[1] == 's')) start = 2;
  else if (n != want) return false;
  syms[n] = '\0';
  uint8_t tmp[kChallengeBytes];
  if (!b32_decode(syms + start, tmp, kChallengeBytes)) return false;
  ChallengeFields f;
  if (!challenge_unpack(tmp, &f)) return false;
  std::memcpy(out, tmp, kChallengeBytes);
  return true;
}

size_t response_to_text(const uint8_t r[kResponseBytes], char* out, size_t cap) {
  char syms[112];
  size_t n = b32_encode(r, kResponseBytes, syms, sizeof syms);
  return grouped(syms, n, 4, "", out, cap);
}

bool response_from_text(const char* text, uint8_t out[kResponseBytes]) {
  return b32_decode(text, out, kResponseBytes);
}

void response_message(const uint8_t chal[kChallengeBytes], uint8_t grant, uint8_t out[kResponseMessageBytes],
                      const char* domain) {
  std::memcpy(out, domain, kResponseDomainLen);
  std::memcpy(out + kResponseDomainLen, chal, kChallengeBytes);
  out[kResponseDomainLen + kChallengeBytes] = grant;
}

uint32_t unix_to_min2025(uint32_t unix_s) {
  if (unix_s < kEpoch2025Unix) return 0;
  uint32_t m = (unix_s - kEpoch2025Unix) / 60u;
  return m > 0xFFFFFFu ? 0xFFFFFFu : m;
}

}  // namespace fs
