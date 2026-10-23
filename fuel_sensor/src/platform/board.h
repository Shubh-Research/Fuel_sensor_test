// Per-chip pins and identity (DESIGN.md §6). The ONLY place besides radar_*.cpp that differs between boards.
#pragma once
#include <cstdint>

#if defined(FS_CHIP_ESP32) == defined(FS_CHIP_ESP32C3)
#error "Build with exactly one of -DFS_CHIP_ESP32=1 or -DFS_CHIP_ESP32C3=1 (tools/build.py does this)"
#endif
#if defined(FS_VARIANT_DEBUG) == defined(FS_VARIANT_PRODUCTION)
#error "Build with exactly one of -DFS_VARIANT_DEBUG=1 or -DFS_VARIANT_PRODUCTION=1 (tools/build.py does this)"
#endif

namespace fs {
namespace board {

#if defined(FS_CHIP_ESP32)
// ESP32-WROVER-B dev board: the A121 sits on a separate sensor MCU that streams I/Q packets over UART.
constexpr const char* kChipName = "esp32";
constexpr uint8_t kChipId = 0;                     // ESP image header chip_id
constexpr int kRadarUart = 2;
constexpr int kRadarRxPin = 19;                    // sensor TX -> GPIO19
constexpr int kRadarTxPin = 18;                    // wired but not driven
constexpr uint32_t kRadarBaud = 921600;
#else
// ESP32-C3 product board (schematic/fuel_board): the A121 is on SPI2 and the C3 runs Acconeer RSS itself.
constexpr const char* kChipName = "esp32c3";
constexpr uint8_t kChipId = 5;
constexpr int kSpiMiso = 2, kSpiSclk = 6, kSpiMosi = 7, kA121Ss = 10, kA121Enable = 3, kA121Int = 4;
constexpr uint32_t kSpiHz = 40000000;              // A121 supports up to 50 MHz
constexpr int kAccelCs = 9;                         // ADXL362 (later step)
constexpr int kRs485Tx = 18, kRs485Rx = 19, kRs485En = 5;   // later step
#endif

#if defined(FS_VARIANT_DEBUG)
constexpr const char* kVariantName = "debug";
constexpr bool kDebug = true;
#else
constexpr const char* kVariantName = "production";
constexpr bool kDebug = false;
#endif

// Wi-Fi (DESIGN.md §8): SSID "MRM-Fuel-<12 hex of the base MAC>", one simple password for every unit, fixed IP.
constexpr const char* kSsidPrefix = "MRM-Fuel-";
constexpr const char* kWifiPassword = "fuelsensor";
constexpr uint8_t kApIp[4] = {192, 168, 4, 1};
constexpr int kApChannel = 1;
constexpr int kApMaxClients = 4;
constexpr uint16_t kHttpPort = 80;
// Service page (web/service.html): technical details, event log, live debug log, with its own challenge-response login
// (Procom service key; the lab key on debug builds). Not linked from the dashboard. Its data needs a service session.
constexpr const char* kServicePath = "/debug";
constexpr uint16_t kRawStreamPort = 3333;          // debug builds only

}  // namespace board
}  // namespace fs
