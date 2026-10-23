// Firmware update into the spare OTA slot, and the trial-boot / rollback state (DESIGN.md §5, §10).
#pragma once
#include <cstddef>
#include <cstdint>
#include "../core/challenge.h"
#include "../core/fwpkg.h"

namespace fs {

struct OtaBootInfo {
  bool trial = false;                 // this firmware is on trial (PENDING_VERIFY): it must pass the self-test
  bool rolled_back = false;           // the previous boot's new firmware failed and the bootloader came back here
  char running_version[24] = {};
  uint32_t rejected_version = 0;      // the version that was rolled back (0 = unknown)
};

// At boot, before anything else uses the OTA state. pending = the version an update installed (Store::pending()).
void ota_boot_check(uint32_t pending, OtaBootInfo* info);
bool ota_mark_valid();                           // trial passed: keep this firmware (false: not stored)
void ota_rollback_now();                         // trial failed: back to the previous firmware (reboots)
uint32_t ota_slot_size();                        // size of the spare slot (1,966,080 on the min_spiffs table)

// One upload, fed by the HTTP handler. Only one at a time.
class OtaUpload {
 public:
  bool start(uint32_t total_len, const Grant& grant);   // false if another upload is running
  PkgError feed(const uint8_t* data, size_t len);
  PkgError finish();
  void abort();
  bool active() const { return active_; }
  const PkgHeader& header() const { return reader_.header(); }

 private:
  PkgReader reader_;
  bool active_ = false;
};

OtaUpload& ota_upload();

}  // namespace fs
