// Maintenance authorisation (DESIGN.md §4.3).
#include "auth.h"

#include <cstdio>
#include <cstring>
#include "crypto.h"

namespace fs {

const char* auth_result_text(AuthResult r) {
  switch (r) {
    case AuthResult::Ok: return "ok";
    case AuthResult::BadFormat: return "The code is not a valid response. Check that it was copied completely.";
    case AuthResult::NoChallenge: return "There is no open request. Get a new code first.";
    case AuthResult::LockedOut: return "Too many wrong codes. Wait before trying again.";
    case AuthResult::BadSignature: return "This response does not match this device's current code.";
    case AuthResult::NoScope: return "The response does not allow any change.";
    case AuthResult::StoreError: return "Could not save to the device memory.";
  }
  return "error";
}

bool Auth::begin(AuthStore* store, const AuthConfig& cfg, uint64_t now_ms) {
  store_ = store;
  cfg_ = cfg;
  p_ = AuthPersist{};
  bool ok = store_ && store_->load(&p_);
  if (p_.have_chal) {                          // drop a stored challenge that is corrupt or from another device
    ChallengeFields f;
    if (!challenge_unpack(p_.chal, &f) || std::memcmp(f.mac, cfg_.mac, 6) != 0) p_.have_chal = false;
  }
  last_fail_ms_ = now_ms;                      // after a reboot a locked device waits a full window again
  new_used_ = cfg_.warm_boot;                  // a reset loop with an attacker: ~1.2M NVS erases in 15 years otherwise
  last_new_ms_ = now_ms;
  session_ = false;
  return ok;
}

bool Auth::make_challenge(uint32_t phone_unix) {
  ChallengeFields f;
  f.calibrated = cfg_.calibrated;
  f.last_ok = p_.last_ok;
  f.last_known = p_.last_min != 0;
  f.locked = p_.fails >= kMaxFailsBeforeLock;
  std::memcpy(f.mac, cfg_.mac, 6);
  f.ok_count = p_.ok_count;
  f.fails = p_.fails;
  f.last_attempt_min = p_.last_min;
  f.phone_min = unix_to_min2025(phone_unix);
  f.fw_version = cfg_.fw_version;
  uint8_t r[3];
  random_bytes(r, 3);
  f.nonce = uint32_t(r[0]) << 16 | uint32_t(r[1]) << 8 | r[2];
  challenge_pack(f, p_.chal);
  p_.have_chal = true;
  return store_ && store_->save(p_);
}

uint32_t Auth::lockout_remaining_s(uint64_t now_ms) const {
  if (p_.fails < kMaxFailsBeforeLock) return 0;
  uint64_t elapsed = now_ms - last_fail_ms_;
  return elapsed >= kLockoutMs ? 0 : uint32_t((kLockoutMs - elapsed + 999) / 1000);
}

uint32_t Auth::new_challenge_wait_s(uint64_t now_ms) const {
  if (!new_used_ || now_ms - last_new_ms_ >= kNewChallengeMinMs) return 0;
  return uint32_t((kNewChallengeMinMs - (now_ms - last_new_ms_) + 999) / 1000);
}

bool Auth::challenge(uint32_t phone_unix, bool force_new, uint64_t now_ms, char* text, size_t cap,
                     uint32_t* lockout_s) {
  if (lockout_s) *lockout_s = lockout_remaining_s(now_ms);
  bool ok = true;
  if (!p_.have_chal) {
    ok = make_challenge(phone_unix);
  } else if (force_new && new_challenge_wait_s(now_ms) == 0) {
    ok = make_challenge(phone_unix);
    new_used_ = true;
    last_new_ms_ = now_ms;
  }
  if (challenge_to_text(p_.chal, text, cap) == 0) return false;
  return ok;
}

AuthResult Auth::respond(const char* response_text, uint32_t phone_unix, uint32_t client_ip, uint64_t now_ms,
                         char token[33], Grant* grant, uint32_t* lockout_s) {
  if (lockout_s) *lockout_s = 0;
  if (!p_.have_chal) return AuthResult::NoChallenge;
  uint8_t resp[kResponseBytes];
  bool parsed = response_from_text(response_text, resp);
  bool valid = false;
  Grant g;
  if (parsed) {
    uint8_t msg[kResponseMessageBytes];
    response_message(p_.chal, resp[0], msg, kResponseDomain);
    for (size_t k = 0; k < cfg_.n_keys && !valid; ++k)
      valid = ed25519_verify(resp + 1, msg, sizeof msg, cfg_.keys[k]);
    if (valid) {
      g = Grant::from_byte(resp[0]);
    } else {                                   // a service response: only the service keys may have signed it
      response_message(p_.chal, resp[0], msg, kServiceDomain);
      for (size_t k = 0; k < cfg_.n_service_keys && !valid; ++k)
        valid = ed25519_verify(resp + 1, msg, sizeof msg, cfg_.service_keys[k]);
      if (valid) g = Grant::from_service_byte(resp[0]);
    }
  }
  // The lockout only throttles WRONG responses: a valid signature cannot be guessed, so refusing it would only let
  // anyone on the Wi-Fi keep the operator out (5 junk codes, then one every 15 min).
  uint32_t wait = lockout_remaining_s(now_ms);
  if (wait && !(valid && g.any_scope())) {
    if (lockout_s) *lockout_s = wait;
    return AuthResult::LockedOut;
  }
  // Every submission counts as an attempt (also a malformed one), so the next challenge shows it to IT.
  p_.last_min = unix_to_min2025(phone_unix);
  AuthResult result;
  if (valid && g.any_scope()) {
    p_.ok_count = p_.ok_count == 0xFFFF ? 0xFFFF : uint16_t(p_.ok_count + 1);
    p_.fails = 0;
    p_.last_ok = true;
    p_.have_chal = false;                      // this challenge is used up
    result = AuthResult::Ok;
  } else {
    p_.fails = p_.fails == 0xFF ? 0xFF : uint8_t(p_.fails + 1);
    p_.last_ok = false;
    last_fail_ms_ = now_ms;
    result = !parsed ? AuthResult::BadFormat : (valid ? AuthResult::NoScope : AuthResult::BadSignature);
  }
  if (!store_ || !store_->save(p_)) {
    if (result == AuthResult::Ok) return AuthResult::StoreError;   // never open a session whose count was not stored
  }
  if (result != AuthResult::Ok) {
    if (lockout_s) *lockout_s = lockout_remaining_s(now_ms);
    return result;
  }
  uint8_t raw[16];
  random_bytes(raw, sizeof raw);
  for (int i = 0; i < 16; ++i) std::snprintf(token_ + 2 * i, 3, "%02x", raw[i]);
  std::memcpy(token, token_, 33);
  session_ = true;
  session_ip_ = client_ip;
  session_start_ms_ = session_last_ms_ = now_ms;
  session_max_ms_ = uint64_t(g.hours) * 3600ull * 1000ull;
  grant_ = g;
  if (grant) *grant = g;
  return AuthResult::Ok;
}

void Auth::expire(uint64_t now_ms) {
  if (!session_) return;
  if (now_ms - session_last_ms_ >= kIdleMs || now_ms - session_start_ms_ >= session_max_ms_) logout();
}

bool Auth::check(const char* token, uint32_t client_ip, uint64_t now_ms, Grant* grant) {
  expire(now_ms);
  if (!session_ || !token || std::strlen(token) != 32 || client_ip != session_ip_) return false;
  uint8_t diff = 0;                            // constant-time compare
  for (int i = 0; i < 32; ++i) diff |= uint8_t(token[i] ^ token_[i]);
  if (diff) return false;
  session_last_ms_ = now_ms;
  if (grant) *grant = grant_;
  return true;
}

void Auth::logout() {
  session_ = false;
  std::memset(token_, 0, sizeof token_);
  session_ip_ = 0;
}

bool Auth::session_active(uint64_t now_ms) const {
  return session_ && now_ms - session_last_ms_ < kIdleMs && now_ms - session_start_ms_ < session_max_ms_;
}

uint32_t Auth::session_remaining_s(uint64_t now_ms) const {
  if (!session_active(now_ms)) return 0;
  uint64_t by_max = session_max_ms_ - (now_ms - session_start_ms_);
  uint64_t by_idle = kIdleMs - (now_ms - session_last_ms_);
  return uint32_t((by_max < by_idle ? by_max : by_idle) / 1000);
}

}  // namespace fs
