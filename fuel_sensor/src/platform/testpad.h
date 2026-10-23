// Size-test padding (see testpad.cpp). Both functions return 0 in normal builds.
#pragma once
#include <cstddef>
#include <cstdint>

namespace fs {
uint32_t test_pad_checksum();
size_t test_pad_bytes();
}  // namespace fs
