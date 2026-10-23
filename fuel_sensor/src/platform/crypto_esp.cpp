// Device backend for src/core/crypto.h: mbedtls SHA-256 (hardware-accelerated where available) and ECDSA P-256 signing,
// libsodium Ed25519
// (both inside the unchanged Arduino ESP32 core 2.0.14) and the hardware RNG.
#include "../core/crypto.h"

#include <esp_system.h>
#include <mbedtls/ecdsa.h>
#include <mbedtls/sha256.h>
#include <sodium.h>
#include <new>

namespace fs {

static_assert(sizeof(mbedtls_sha256_context) <= sizeof(Sha256Ctx::opaque), "Sha256Ctx too small for mbedtls");
static_assert(alignof(mbedtls_sha256_context) <= 8, "Sha256Ctx alignment");

static mbedtls_sha256_context* ctx_of(Sha256Ctx* c) { return reinterpret_cast<mbedtls_sha256_context*>(c->opaque); }

void sha256_begin(Sha256Ctx* ctx) {
  mbedtls_sha256_context* m = new (ctx->opaque) mbedtls_sha256_context;
  mbedtls_sha256_init(m);
  mbedtls_sha256_starts_ret(m, 0);
}

void sha256_update(Sha256Ctx* ctx, const uint8_t* data, size_t len) { mbedtls_sha256_update_ret(ctx_of(ctx), data, len); }

void sha256_finish(Sha256Ctx* ctx, uint8_t out[32]) {
  mbedtls_sha256_finish_ret(ctx_of(ctx), out);
  mbedtls_sha256_free(ctx_of(ctx));
}

bool ed25519_verify(const uint8_t sig[64], const uint8_t* msg, size_t len, const uint8_t public_key[32]) {
  static const bool ready = sodium_init() >= 0;   // thread-safe static init; sodium_init is idempotent
  return ready && crypto_sign_verify_detached(sig, msg, len, public_key) == 0;
}

// Deterministic ECDSA (RFC 6979, MBEDTLS_ECDSA_DETERMINISTIC is on in core 2.0.14's mbedtls 2.28): the nonce comes from
// d and the digest, so a weak RNG can never leak the key. The RNG below only blinds the scalar multiplication. The MPIs
// and the curve live on the heap for the call (~2-4 KB, freed at return).
static int rng_blind(void*, unsigned char* out, size_t len) {
  esp_fill_random(out, len);
  return 0;
}

bool ecdsa_p256_sign(const uint8_t d[32], const uint8_t digest[32], uint8_t sig[64]) {
  mbedtls_ecp_group grp;
  mbedtls_mpi md, r, s;
  mbedtls_ecp_group_init(&grp);
  mbedtls_mpi_init(&md);
  mbedtls_mpi_init(&r);
  mbedtls_mpi_init(&s);
  bool ok = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1) == 0 && mbedtls_mpi_read_binary(&md, d, 32) == 0 &&
            mbedtls_ecdsa_sign_det_ext(&grp, &r, &s, &md, digest, 32, MBEDTLS_MD_SHA256, rng_blind, nullptr) == 0 &&
            mbedtls_mpi_write_binary(&r, sig, 32) == 0 && mbedtls_mpi_write_binary(&s, sig + 32, 32) == 0;
  mbedtls_mpi_free(&md);   // (zeroises the scalar copy)
  mbedtls_mpi_free(&r);
  mbedtls_mpi_free(&s);
  mbedtls_ecp_group_free(&grp);
  return ok;
}

// esp_fill_random gives true random numbers while the radio is on; Wi-Fi is started before any challenge or token is
// made (net_begin runs in setup), so every nonce and session token comes from the full-entropy source.
void random_bytes(uint8_t* out, size_t len) { esp_fill_random(out, len); }

}  // namespace fs
