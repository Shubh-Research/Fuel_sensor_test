// Signed firmware package (DESIGN.md §5): header checks before anything is erased, then a streaming image check.
// The flash writer is injected (ImageSink), so the whole flow runs on the host in tests.
#pragma once
#include <cstddef>
#include <cstdint>
#include "crypto.h"

namespace fs {

constexpr size_t kPkgHeaderBytes = 128;
constexpr size_t kPkgSigBytes = 64;
constexpr size_t kPkgPrefixBytes = kPkgHeaderBytes + kPkgSigBytes;
constexpr char kPkgMagic[8] = {'M', 'R', 'M', 'F', 'W', 'P', 'K', '1'};
constexpr char kPkgProduct[] = "MRM-FUEL";
constexpr uint8_t kChipEsp32 = 0, kChipEsp32C3 = 5;          // = ESP image header chip_id
constexpr uint8_t kVariantProduction = 0, kVariantDebug = 1;

struct PkgHeader {
  uint8_t chip = 0, variant = 0;
  uint32_t version = 0, image_size = 0, build_time = 0;
  uint8_t sha256[32] = {};
  char git[9] = {};
};

enum class PkgError : uint8_t {
  Ok = 0, Short, Magic, HeaderVersion, Product, Reserved, Chip, Variant, Downgrade, TooBig, Signature,
  ImageHeader, Hash, SinkBegin, SinkWrite, SinkEnd, Length, State,
};
const char* pkg_error_text(PkgError e);

struct PkgPolicy {
  uint8_t chip = kChipEsp32;
  uint32_t running_version = 0;
  bool allow_older = false;           // session grant bit3
  uint32_t slot_size = 0;             // bytes of the target OTA partition
  const uint8_t (*keys)[32] = nullptr;
  size_t n_keys = 0;
};

// Parses and checks the 128-byte header and its signature (steps 1-6 of §5).
PkgError pkg_check_prefix(const uint8_t prefix[kPkgPrefixBytes], const PkgPolicy& policy, PkgHeader* out);

// Where the image goes (an OTA partition on the device, memory in tests).
class ImageSink {
 public:
  virtual ~ImageSink() = default;
  virtual bool begin(uint32_t image_size) = 0;   // erase / prepare; called only after the prefix checks pass
  virtual bool write(const uint8_t* data, size_t len) = 0;
  virtual bool end() = 0;                         // finalise (esp_ota_end + set boot partition on the device)
  virtual void abort() = 0;
};

// Feeds a package in arbitrary chunks. The sink sees image bytes only after the prefix has been verified, and end()
// is called only after the whole image hash matched the signed header.
class PkgReader {
 public:
  void begin(const PkgPolicy& policy, ImageSink* sink, uint32_t total_len);
  PkgError feed(const uint8_t* data, size_t len);
  PkgError finish();                 // call once after the last chunk
  void abort();
  const PkgHeader& header() const { return hdr_; }
  uint32_t received() const { return received_; }
  bool prefix_done() const { return prefix_used_ == kPkgPrefixBytes && sink_started_; }

 private:
  PkgError fail(PkgError e);
  PkgPolicy policy_;
  ImageSink* sink_ = nullptr;
  uint32_t total_ = 0, received_ = 0, image_got_ = 0;
  uint8_t prefix_[kPkgPrefixBytes] = {};
  size_t prefix_used_ = 0;
  uint8_t img_head_[16] = {};
  size_t img_head_used_ = 0;
  bool sink_started_ = false, failed_ = false;
  bool sha_open_ = false;            // a hash context is live and must be finished (frees it; on the ESP32 it may
                                     // hold the SHA-256 hardware engine until mbedtls_sha256_free)
  void close_sha();
  PkgError error_ = PkgError::Ok;
  PkgHeader hdr_;
  Sha256Ctx sha_;
};

}  // namespace fs
