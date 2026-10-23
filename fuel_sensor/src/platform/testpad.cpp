// TEST BUILDS ONLY (tools/build.py --test-pad-kb N): adds N KB of read-only data so the image is close to the
// 1,966,080-byte OTA slot. Installing this image over Wi-Fi proves that the update path and the bootloader handle a
// future firmware of that size. Normal builds compile this file to nothing.
#include "testpad.h"

#if defined(FS_TEST_PAD_KB) && FS_TEST_PAD_KB > 0
namespace {
// ESP images store .rodata uncompressed, so a zero-filled array occupies its full size in flash. It is read below,
// so --gc-sections keeps it.
const uint8_t kPad[FS_TEST_PAD_KB * 1024] = {1};
}  // namespace

namespace fs {
uint32_t test_pad_checksum() {
  uint32_t s = 0;
  for (size_t i = 0; i < sizeof kPad; i += 4093) s = s * 31u + *static_cast<const volatile uint8_t*>(&kPad[i]);
  return s;
}
size_t test_pad_bytes() { return sizeof kPad; }
}  // namespace fs
#else
namespace fs {
uint32_t test_pad_checksum() { return 0; }
size_t test_pad_bytes() { return 0; }
}  // namespace fs
#endif
