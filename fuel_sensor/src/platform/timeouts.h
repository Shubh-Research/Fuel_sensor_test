// Every blocking wait in the firmware, in one place, checked against the task watchdog (ESP32-Smart-Devices practice:
// the watchdog must be longer than any blocking call, or a slow-but-healthy path would reset the device).
#pragma once
#include <cstdint>

namespace fs {
namespace timeouts {

constexpr uint32_t kTaskWdtS = 10;                  // task watchdog (panic + reset)
constexpr uint32_t kRadarReadMs = 500;              // radar task: one read() call
constexpr uint32_t kFrameQueueWaitMs = 500;         // tracker task: waiting for a frame
constexpr uint32_t kStatusLockMs = 200;             // any task: status mutex
constexpr uint32_t kLogReaderHoldMs = 120000;      // /api/logs: info lines go to the Wi-Fi ring this long after a poll
constexpr uint32_t kHeartbeatMs = 60000;           // the "hb" status line
constexpr uint32_t kLogBootKeepMs = 60000;         // every log line goes to the ring during the first minute after boot
constexpr uint32_t kEventLockMs = 1000;             // event-log mutex (a LittleFS write)
constexpr uint32_t kHttpRecvS = 8;                  // esp_http_server receive timeout per call
constexpr uint32_t kHttpSendS = 8;
constexpr uint32_t kOtaChunkMs = 4000;              // one flash erase/write step during an update
constexpr uint32_t kStreamSendMs = 200;             // raw stream: one send()

static_assert(kRadarReadMs < kTaskWdtS * 1000 / 2, "radar read must leave margin under the watchdog");
static_assert(kFrameQueueWaitMs < kTaskWdtS * 1000 / 2, "frame wait must leave margin");
static_assert(kEventLockMs < kTaskWdtS * 1000 / 2, "event lock must leave margin");
static_assert(kHttpRecvS < kTaskWdtS && kHttpSendS < kTaskWdtS, "HTTP socket timeouts must be shorter than the watchdog");
static_assert(kOtaChunkMs < kTaskWdtS * 1000, "flash step must be shorter than the watchdog");

// Supervisor thresholds (DESIGN.md §10).
constexpr uint32_t kRadarSilentReinitMs = 5000;
constexpr uint32_t kRadarSilentRestartMs = 60000;
constexpr uint32_t kNoEstimateRestartMs = 30000;
constexpr uint32_t kHeapFloorBytes = 24 * 1024;
constexpr uint32_t kHeapFloorHoldS = 10;            // ... for this many supervisor ticks in a row before a restart
constexpr uint32_t kRadarReinitEveryMs = 60000;     // while the radar stays silent: re-open the link this often
constexpr uint32_t kSupervisorRestarts = 2;         // radar-lost / no-estimate restarts in a row (then stay up)
constexpr uint32_t kSupervisorHealthyMs = 600000;   // estimates flowing this long re-arm the supervisor restarts
constexpr uint32_t kTrialSelfTestMs = 90000;
constexpr uint32_t kTrialMinFrames = 20;
constexpr uint32_t kTrialMinEstimates = 3;          // the tracker must work too, not only the radar link
constexpr uint32_t kSelfCheckConnectMs = 2000;      // trial self-test: non-blocking connect to the AP address
constexpr uint32_t kSafeModeWindowS = 120;          // abnormal reset within this uptime counts towards safe mode
constexpr uint32_t kSafeModeResets = 3;
constexpr uint32_t kSafeModeClearS = 600;
constexpr uint32_t kLevelSaveMinS = 60;             // flash save cadence (user, 13 Oct 2026)
constexpr double kLevelSaveMinChangeMm = 0.05;      // v6.10 rule
constexpr uint32_t kCountersFlushS = 600;           // boot counters reach NVS after this much uptime (reset loops: never)

}  // namespace timeouts
}  // namespace fs
