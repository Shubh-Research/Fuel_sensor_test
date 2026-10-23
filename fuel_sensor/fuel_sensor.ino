// MRM fuel sensor firmware (fw 1.0.0, tracker v6.10). ESP32-WROVER-B (dev board) and ESP32-C3 (product) from one
// codebase. Build with tools/build.py, which sets -DFS_CHIP_* and -DFS_VARIANT_*. Design: docs/DESIGN.md.
#include "src/platform/app.h"
#include "src/platform/logs.h"

// Keep a newly installed firmware on trial: the app marks it valid only after its self-test (src/platform/app.cpp);
// a crash, hang or failed self-test makes the bootloader return to the previous firmware.
extern "C" bool verifyRollbackLater() { return true; }

void setup() {
  Serial.begin(115200);
  fs::logs_begin();                                       // UART0 always; the Wi-Fi ring (/api/logs) on demand
  fs::app_setup();
}

void loop() { fs::app_loop(); }
