// Crypto primitives the core needs. Implemented by src/platform/crypto_esp.cpp on the device (libsodium Ed25519,
// mbedtls SHA-256 and ECDSA P-256, hardware RNG) and by test/host/crypto_openssl.cpp on the host (OpenSSL 3).
#pragma once
#include <cstddef>
#include <cstdint>

namespace fs {

// Incremental SHA-256. The context is opaque storage large enough for every backend (checked by static_assert there).
struct Sha256Ctx { alignas(8) uint8_t opaque[256]; };
void sha256_begin(Sha256Ctx* ctx);
void sha256_update(Sha256Ctx* ctx, const uint8_t* data, size_t len);
void sha256_finish(Sha256Ctx* ctx, uint8_t out[32]);

// Ed25519 (RFC 8032, pure) signature check. Returns true only for a valid signature.
bool ed25519_verify(const uint8_t sig[64], const uint8_t* msg, size_t len, const uint8_t public_key[32]);

// ECDSA P-256 signature (fill reports, DESIGN.md §14) over a 32-byte SHA-256 digest with the private scalar d (32 bytes,
// big-endian). sig = r || s (32 + 32 bytes, big-endian: the WebCrypto / IEEE P1363 form). The device signs
// deterministically (RFC 6979, no RNG needed for the nonce); the host backend uses OpenSSL. False on any error.
bool ecdsa_p256_sign(const uint8_t d[32], const uint8_t digest[32], uint8_t sig[64]);

// Cryptographically secure random bytes (hardware RNG on the device; the radio must be on for full entropy).
void random_bytes(uint8_t* out, size_t len);

}  // namespace fs
