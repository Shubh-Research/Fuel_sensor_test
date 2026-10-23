// Challenge / response wire formats (DESIGN.md §4.1, §4.2): byte layouts, CRC-16, Crockford base32 text forms.
#pragma once
#include <cstddef>
#include <cstdint>

namespace fs {

constexpr size_t kChallengeBytes = 23;
constexpr size_t kResponseBytes = 65;
constexpr size_t kChallengeTextMax = 48;     // "FS-" + 37 symbols + 7 dashes + NUL
constexpr size_t kResponseTextMax = 136;     // 104 symbols + 25 dashes + NUL
constexpr char kResponseDomain[] = "MRM-FUEL-RESP-1";   // 15 bytes, no NUL in the signed message: maintenance
constexpr size_t kResponseDomainLen = sizeof(kResponseDomain) - 1;
// Service responses (the /debug page, DESIGN.md §4.2) sign the same message under their own domain, so a maintenance
// signature can never be read as a service one or the other way round.
constexpr char kServiceDomain[] = "MRM-FUEL-SERV-1";
static_assert(sizeof(kServiceDomain) == sizeof(kResponseDomain), "both domains have 15 bytes");
constexpr size_t kResponseMessageBytes = kResponseDomainLen + kChallengeBytes + 1;
constexpr uint32_t kEpoch2025Unix = 1735689600u;          // 2025-01-01T00:00:00Z
constexpr uint8_t kChallengeFormat = 1;

struct ChallengeFields {
  bool calibrated = false;          // site calibration done
  bool last_ok = false;             // the last attempt succeeded
  bool last_known = false;          // a last attempt time is recorded
  bool locked = false;              // currently locked out
  uint8_t mac[6] = {};
  uint16_t ok_count = 0;            // successful logins so far (saturating)
  uint8_t fails = 0;                // failed attempts since the last success (saturating)
  uint32_t last_attempt_min = 0;    // minutes since 2025-01-01Z, phone-reported (0 = unknown), u24
  uint32_t phone_min = 0;           // phone time when issued, minutes since 2025-01-01Z (0 = none), u24
  uint16_t fw_version = 0;          // major(5) minor(6) patch(5)
  uint32_t nonce = 0;               // 24 bits
};

// What a session may do (DESIGN.md §4.2). A maintenance response (customer key; lab key on debug builds) grants
// settings / firmware / calibration / older; a service response (Procom service key; lab key on debug builds) grants
// service only: reading the technical status, the event log and the device log on /debug.
struct Grant {
  bool settings = false, firmware = false, calibration = false, older = false;
  bool service = false;
  uint8_t hours = 1;                // 1..15
  uint8_t byte() const;                          // maintenance grant byte (bit0 settings ... bit3 older, hours << 4)
  static Grant from_byte(uint8_t b);
  uint8_t service_byte() const;                  // service grant byte: bit0 service, bits 1-3 0, hours << 4
  static Grant from_service_byte(uint8_t b);     // reserved bits 1-3 set: no scope (the device refuses it)
  bool maintenance() const { return settings || firmware || calibration; }
  bool any_scope() const { return maintenance() || service; }
  // LOGIN_OK event arg: the grant byte; bit 8 marks a service login.
  uint16_t event_arg() const { return service ? uint16_t(0x100 | service_byte()) : byte(); }
};

uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

// Crockford base32, MSB-first, last group zero-padded. encode returns the number of symbols written (no NUL counted).
size_t b32_encode(const uint8_t* in, size_t n, char* out, size_t cap);
// Lenient decode of exactly `n` bytes: case-insensitive, ignores '-', ' ', '\t', '\r', '\n'; O->0, I/L->1; rejects U and
// anything else; requires exactly ceil(8n/5) symbols and zero padding bits.
bool b32_decode(const char* text, uint8_t* out, size_t n);

void challenge_pack(const ChallengeFields& f, uint8_t out[kChallengeBytes]);    // fills the CRC
bool challenge_unpack(const uint8_t in[kChallengeBytes], ChallengeFields* f);  // checks format and CRC

// "FS-XXXXX-...-XX"; returns the length (0 if cap is too small).
size_t challenge_to_text(const uint8_t b[kChallengeBytes], char* out, size_t cap);
bool challenge_from_text(const char* text, uint8_t out[kChallengeBytes]);      // accepts with or without "FS"

size_t response_to_text(const uint8_t r[kResponseBytes], char* out, size_t cap); // groups of 4
bool response_from_text(const char* text, uint8_t out[kResponseBytes]);

// The signed message: domain || challenge || grant (domain: kResponseDomain or kServiceDomain).
void response_message(const uint8_t chal[kChallengeBytes], uint8_t grant, uint8_t out[kResponseMessageBytes],
                      const char* domain = kResponseDomain);

// Phone unix seconds -> minutes since 2025-01-01Z, clamped to u24; 0 when absent or before 2025.
uint32_t unix_to_min2025(uint32_t unix_s);

}  // namespace fs
