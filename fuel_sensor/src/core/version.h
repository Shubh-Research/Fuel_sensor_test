// Firmware and tracker versions (shown on the dashboard, in the challenge, in packages, later on RS-485).
#pragma once

#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 5
#define FW_VERSION_STRING "1.2.5"
#define TRACKER_VERSION "v6.11"

namespace fs {
// Package/anti-downgrade form: major<<16 | minor<<8 | patch (DESIGN.md §5).
constexpr unsigned long kFwVersionU32 = (unsigned long)FW_VERSION_MAJOR << 16 | FW_VERSION_MINOR << 8 | FW_VERSION_PATCH;
// Challenge form: major(5) minor(6) patch(5) (DESIGN.md §4.1).
constexpr unsigned kFwVersionU16 = (FW_VERSION_MAJOR & 31) << 11 | (FW_VERSION_MINOR & 63) << 5 | (FW_VERSION_PATCH & 31);
static_assert(FW_VERSION_MAJOR < 32 && FW_VERSION_MINOR < 64 && FW_VERSION_PATCH < 32, "version must fit the challenge");
}  // namespace fs
