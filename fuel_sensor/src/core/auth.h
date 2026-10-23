// Maintenance and service authorisation (DESIGN.md §4.2, §4.3): one outstanding challenge, response check against the
// device's own copy, attempt counters, lockout and a single session. Storage, clock and crypto are injected; pure C++.
#pragma once
#include <cstddef>
#include <cstdint>
#include "challenge.h"

namespace fs {

// What survives reboots (NVS namespace fs_auth on the device).
struct AuthPersist {
  uint16_t ok_count = 0;
  uint8_t fails = 0;
  uint32_t last_min = 0;          // phone minutes of the last attempt (0 = none)
  bool last_ok = false;
  bool have_chal = false;
  uint8_t chal[kChallengeBytes] = {};
};

class AuthStore {
 public:
  virtual ~AuthStore() = default;
  virtual bool load(AuthPersist* p) = 0;
  virtual bool save(const AuthPersist& p) = 0;
};

struct AuthConfig {
  const uint8_t (*keys)[32] = nullptr;   // maintenance responses: authority public keys accepted by this build
  size_t n_keys = 0;
  const uint8_t (*service_keys)[32] = nullptr;   // service responses (domain kServiceDomain): Procom service key (+ lab
  size_t n_service_keys = 0;                     // on debug builds). Never the customer keys.
  uint8_t mac[6] = {};
  uint16_t fw_version = 0;
  bool calibrated = false;               // site calibration done (set by the device; false until v7.0 calibration)
  bool warm_boot = false;                // the forced-new-code window starts at begin(): a reset loop cannot replace the
                                         // stored code (3 NVS entries) at every boot (the device sets it at every boot)
};

enum class AuthResult : uint8_t {
  Ok = 0, BadFormat, NoChallenge, LockedOut, BadSignature, NoScope, StoreError,
};
const char* auth_result_text(AuthResult r);

class Auth {
 public:
  static constexpr uint32_t kMaxFailsBeforeLock = 5;
  static constexpr uint64_t kLockoutMs = 15ull * 60 * 1000;
  static constexpr uint64_t kIdleMs = 15ull * 60 * 1000;
  // A forced "new challenge" replaces the outstanding one at most this often (device uptime). Bounds NVS writes from
  // unauthenticated clients and makes a repeat of an old 24-bit nonce take centuries instead of ~2^24 requests.
  static constexpr uint64_t kNewChallengeMinMs = 10ull * 60 * 1000;

  // now_ms: monotonic device time (ms since boot).
  bool begin(AuthStore* store, const AuthConfig& cfg, uint64_t now_ms);
  void set_calibrated(bool c) { cfg_.calibrated = c; }

  // Returns the outstanding challenge text, creating one (with the phone time) if none, or if force_new and the last
  // forced replacement is at least kNewChallengeMinMs ago. lockout_s > 0 means wrong responses are refused for that long.
  bool challenge(uint32_t phone_unix, bool force_new, uint64_t now_ms, char* text, size_t cap, uint32_t* lockout_s);

  // Checks a response; on success opens the session and writes the 32-hex token (+NUL) into token. A maintenance
  // signature (kResponseDomain) is checked against cfg.keys, a service signature (kServiceDomain) against
  // cfg.service_keys; the grant comes from the domain that verified (Grant::from_byte / from_service_byte). A new
  // session replaces the previous one (one session at a time, either kind).
  AuthResult respond(const char* response_text, uint32_t phone_unix, uint32_t client_ip, uint64_t now_ms,
                     char token[33], Grant* grant, uint32_t* lockout_s);

  // True if the token is the live session's and from the same client; refreshes the idle timer.
  bool check(const char* token, uint32_t client_ip, uint64_t now_ms, Grant* grant);
  void logout();
  bool session_active(uint64_t now_ms) const;
  uint32_t session_remaining_s(uint64_t now_ms) const;
  uint32_t lockout_remaining_s(uint64_t now_ms) const;
  uint32_t new_challenge_wait_s(uint64_t now_ms) const;
  const AuthPersist& persist() const { return p_; }

 private:
  bool make_challenge(uint32_t phone_unix);
  void expire(uint64_t now_ms);

  AuthStore* store_ = nullptr;
  AuthConfig cfg_;
  AuthPersist p_;
  uint64_t last_fail_ms_ = 0;            // failed attempt (or boot) that starts the lockout window
  uint64_t last_new_ms_ = 0;             // last forced replacement (valid when new_used_)
  bool new_used_ = false;
  bool session_ = false;
  char token_[33] = {};
  uint32_t session_ip_ = 0;
  uint64_t session_start_ms_ = 0, session_last_ms_ = 0, session_max_ms_ = 0;
  Grant grant_;
};

}  // namespace fs
