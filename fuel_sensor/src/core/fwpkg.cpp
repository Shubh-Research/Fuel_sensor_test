// Signed firmware package (DESIGN.md §5).
#include "fwpkg.h"

#include <cstring>

namespace fs {

namespace {
uint16_t le16(const uint8_t* p) { return uint16_t(p[0] | p[1] << 8); }
uint32_t le32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
}  // namespace

const char* pkg_error_text(PkgError e) {
  switch (e) {
    case PkgError::Ok: return "ok";
    case PkgError::Short: return "The file is incomplete.";
    case PkgError::Magic: return "This is not a fuel sensor firmware package (.fwpkg).";
    case PkgError::HeaderVersion: return "This package format is not supported by the installed firmware.";
    case PkgError::Product: return "This firmware is for a different product.";
    case PkgError::Reserved: return "The package header is invalid.";
    case PkgError::Chip: return "This firmware is for a different chip (WROVER vs C3).";
    case PkgError::Variant: return "Unknown firmware variant.";
    case PkgError::Downgrade: return "This firmware is older than the installed one (not allowed by the approval).";
    case PkgError::TooBig: return "The firmware is too large for this device.";
    case PkgError::Signature: return "The firmware signature is not valid. Use an official package.";
    case PkgError::ImageHeader: return "The firmware image inside the package is invalid.";
    case PkgError::Hash: return "The firmware was damaged in transfer (checksum mismatch). Try again.";
    case PkgError::SinkBegin: return "Could not prepare the update area in flash.";
    case PkgError::SinkWrite: return "Writing the update to flash failed.";
    case PkgError::SinkEnd: return "The device could not activate the new firmware.";
    case PkgError::Length: return "The upload length does not match the package.";
    case PkgError::State: return "Update in an invalid state.";
  }
  return "error";
}

PkgError pkg_check_prefix(const uint8_t prefix[kPkgPrefixBytes], const PkgPolicy& policy, PkgHeader* out) {
  const uint8_t* h = prefix;
  if (std::memcmp(h, kPkgMagic, 8) != 0) return PkgError::Magic;
  if (le16(h + 8) != 1 || le16(h + 10) != kPkgHeaderBytes) return PkgError::HeaderVersion;
  uint8_t product[16] = {};
  std::memcpy(product, kPkgProduct, sizeof kPkgProduct - 1);
  if (std::memcmp(h + 12, product, 16) != 0) return PkgError::Product;
  if (le16(h + 30) != 0) return PkgError::Reserved;
  for (size_t i = 84; i < kPkgHeaderBytes; ++i)
    if (h[i]) return PkgError::Reserved;
  PkgHeader hd;
  hd.chip = h[28];
  hd.variant = h[29];
  hd.version = le32(h + 32);
  hd.image_size = le32(h + 36);
  std::memcpy(hd.sha256, h + 40, 32);
  hd.build_time = le32(h + 72);
  std::memcpy(hd.git, h + 76, 8);
  hd.git[8] = '\0';
  if (hd.chip != policy.chip) return PkgError::Chip;
  if (hd.variant > kVariantDebug) return PkgError::Variant;      // any board takes debug AND production (Procom decides; §5)
  if (hd.version < policy.running_version && !policy.allow_older) return PkgError::Downgrade;
  if (hd.image_size < 64 || hd.image_size > policy.slot_size) return PkgError::TooBig;
  bool sig_ok = false;
  for (size_t k = 0; k < policy.n_keys && !sig_ok; ++k)
    sig_ok = ed25519_verify(prefix + kPkgHeaderBytes, h, kPkgHeaderBytes, policy.keys[k]);
  if (!sig_ok) return PkgError::Signature;
  if (out) *out = hd;
  return PkgError::Ok;
}

void PkgReader::close_sha() {
  if (!sha_open_) return;
  uint8_t discard[32];
  sha256_finish(&sha_, discard);
  sha_open_ = false;
}

void PkgReader::begin(const PkgPolicy& policy, ImageSink* sink, uint32_t total_len) {
  close_sha();
  policy_ = policy;
  sink_ = sink;
  total_ = total_len;
  received_ = image_got_ = 0;
  prefix_used_ = img_head_used_ = 0;
  sink_started_ = failed_ = false;
  error_ = PkgError::Ok;
  hdr_ = PkgHeader{};
}

PkgError PkgReader::fail(PkgError e) {
  if (!failed_) {
    failed_ = true;
    error_ = e;
    close_sha();
    if (sink_started_ && sink_) sink_->abort();
  }
  return error_;
}

void PkgReader::abort() { fail(PkgError::State); }

PkgError PkgReader::feed(const uint8_t* data, size_t len) {
  if (failed_) return error_;
  if (uint64_t(received_) + len > total_) return fail(PkgError::Length);
  received_ += uint32_t(len);
  while (len) {
    if (prefix_used_ < kPkgPrefixBytes) {
      size_t n = kPkgPrefixBytes - prefix_used_;
      if (n > len) n = len;
      std::memcpy(prefix_ + prefix_used_, data, n);
      prefix_used_ += n;
      data += n;
      len -= n;
      if (prefix_used_ == kPkgPrefixBytes) {
        PkgError e = pkg_check_prefix(prefix_, policy_, &hdr_);
        if (e != PkgError::Ok) return fail(e);
        if (uint64_t(kPkgPrefixBytes) + hdr_.image_size != total_) return fail(PkgError::Length);
        if (!sink_ || !sink_->begin(hdr_.image_size)) return fail(PkgError::SinkBegin);
        sink_started_ = true;
        sha256_begin(&sha_);
        sha_open_ = true;
      }
      continue;
    }
    size_t n = len;
    if (image_got_ + n > hdr_.image_size) return fail(PkgError::Length);
    if (img_head_used_ < sizeof img_head_) {        // keep the first image bytes for the image-header check
      size_t m = sizeof img_head_ - img_head_used_;
      if (m > n) m = n;
      std::memcpy(img_head_ + img_head_used_, data, m);
      img_head_used_ += m;
      if (img_head_used_ == sizeof img_head_) {
        // ESP image header: magic 0xE9; chip_id (u16 LE) at offset 12 must equal the package chip.
        if (img_head_[0] != 0xE9 || le16(img_head_ + 12) != hdr_.chip) return fail(PkgError::ImageHeader);
      }
    }
    sha256_update(&sha_, data, n);
    if (!sink_->write(data, n)) return fail(PkgError::SinkWrite);
    image_got_ += uint32_t(n);
    data += n;
    len -= n;
  }
  return PkgError::Ok;
}

PkgError PkgReader::finish() {
  if (failed_) return error_;
  if (!sink_started_ || image_got_ != hdr_.image_size || received_ != total_) return fail(PkgError::Short);
  uint8_t digest[32];
  sha256_finish(&sha_, digest);
  sha_open_ = false;
  uint8_t diff = 0;
  for (int i = 0; i < 32; ++i) diff |= uint8_t(digest[i] ^ hdr_.sha256[i]);
  if (diff) return fail(PkgError::Hash);
  if (!sink_->end()) {
    failed_ = true;
    error_ = PkgError::SinkEnd;
    return error_;
  }
  return PkgError::Ok;
}

}  // namespace fs
