// Firmware update and trial boot (DESIGN.md §5, §10). The image streams into the spare slot only after the signed
// header has been verified; the boot slot changes only after the whole image hash matched. The bootloader (rollback
// enabled) keeps the new image on trial until ota_mark_valid(); a crash, hang or failed self-test returns to the old one.
#include "ota.h"

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_task_wdt.h>
#include <cstdio>
#include <cstring>
#include "../core/version.h"
#include "board.h"
#include "events.h"
#include "keys_gen.h"

namespace fs {

namespace {

class OtaSink final : public ImageSink {
 public:
  bool begin(uint32_t image_size) override {
    part_ = esp_ota_get_next_update_partition(nullptr);
    if (!part_ || image_size > part_->size) return false;
    // Sequential mode erases each sector just before it is written, so no single call blocks the other tasks for
    // the seconds a 1 MB erase takes (size and bounds were checked from the signed header already).
    return esp_ota_begin(part_, OTA_WITH_SEQUENTIAL_WRITES, &handle_) == ESP_OK;
  }
  bool write(const uint8_t* data, size_t len) override {
    esp_task_wdt_reset();
    return esp_ota_write(handle_, data, len) == ESP_OK;
  }
  bool end() override {
    esp_err_t e = esp_ota_end(handle_);                         // checks the image (segments, SHA-256 appended)
    handle_ = 0;                                                // IDF 4.4 frees the handle on success and on failure
    if (e != ESP_OK) return false;
    return esp_ota_set_boot_partition(part_) == ESP_OK;
  }
  void abort() override {
    if (handle_) esp_ota_abort(handle_);
    handle_ = 0;
  }

 private:
  const esp_partition_t* part_ = nullptr;
  esp_ota_handle_t handle_ = 0;
};

OtaSink g_sink;
OtaUpload g_upload;

}  // namespace

OtaUpload& ota_upload() { return g_upload; }

uint32_t ota_slot_size() {
  const esp_partition_t* p = esp_ota_get_next_update_partition(nullptr);
  return p ? p->size : 0;
}

void ota_boot_check(uint32_t pending, OtaBootInfo* info) {
  *info = OtaBootInfo{};
  const esp_partition_t* running = esp_ota_get_running_partition();
  std::snprintf(info->running_version, sizeof info->running_version, "%s", FW_VERSION_STRING);
  esp_ota_img_states_t st;
  if (running && esp_ota_get_state_partition(running, &st) == ESP_OK) info->trial = st == ESP_OTA_IMG_PENDING_VERIFY;
  // An update recorded `pending` just before switching slots. If we are not that version, the bootloader came back
  // to us because the new firmware crashed, hung or failed its self-test.
  if (pending && pending != uint32_t(kFwVersionU32) && esp_ota_get_last_invalid_partition() != nullptr) {
    info->rolled_back = true;
    info->rejected_version = pending;
  }
}

bool ota_mark_valid() { return esp_ota_mark_app_valid_cancel_rollback() == ESP_OK; }

void ota_rollback_now() {
  log_event(Ev::RolledBack, 1, kFwVersionU32);
  delay(200);
  esp_ota_mark_app_invalid_rollback_and_reboot();   // does not return if a previous image exists
  esp_restart();
}

bool OtaUpload::start(uint32_t total_len, const Grant& grant) {
  if (active_) return false;
  PkgPolicy pol;
  pol.chip = board::kChipId;
  pol.running_version = uint32_t(kFwVersionU32);
  pol.allow_older = grant.older;
  pol.slot_size = ota_slot_size();
  pol.keys = kFirmwareKeys;
  pol.n_keys = kFirmwareKeyCount;
  reader_.begin(pol, &g_sink, total_len);
  active_ = true;
  return true;
}

PkgError OtaUpload::feed(const uint8_t* data, size_t len) {
  PkgError e = reader_.feed(data, len);
  if (e != PkgError::Ok) active_ = false;
  return e;
}

PkgError OtaUpload::finish() {
  PkgError e = reader_.finish();
  active_ = false;
  return e;
}

void OtaUpload::abort() {
  if (active_) reader_.abort();
  active_ = false;
}

}  // namespace fs
