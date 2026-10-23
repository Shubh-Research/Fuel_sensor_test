// Device event log backend on LittleFS (the existing 128 KB "spiffs" partition), DESIGN.md §9.2.
// Segments /ev/<n> of at most 512 bytes: LittleFS keeps files that small inline in its metadata, so one event is one
// append + fsync = one metadata commit (no 4 KB data block is copied and erased). The fw 1.0.0 layout rewrote an
// 8 KB file per event (5.5 block erases, ~250 ms with the flash cache off); see dev/regression/work/rev6.
// EventBudget (RTC memory) bounds what faults and unauthenticated clients can write: at most 288 per class a day.
#include "events.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <dirent.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <fcntl.h>
#include <freertos/semphr.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "logs.h"
#include "timeouts.h"
#include <cerrno>

namespace fs {

namespace {

constexpr const char* kBase = "/littlefs";
constexpr const char* kDir = "/littlefs/ev";
constexpr const char* kLegacy = "/littlefs/events.bin";

void seg_path(char* out, size_t cap, uint32_t id) { std::snprintf(out, cap, "%s/%u", kDir, unsigned(id)); }

// A failing file operation is logged (with errno) the first 3 times and then every 100th: a flash fault must show in the
// device log without flooding it.
uint32_t g_fs_errors = 0;
void fs_error(const char* op, uint32_t id) {
  if (++g_fs_errors <= 3 || g_fs_errors % 100 == 0)
    FS_LOGE("evlog", "%s segment %u failed: errno %d (%u errors so far)", op, unsigned(id), errno, unsigned(g_fs_errors));
}

class LfsSegments final : public SegmentStore {
 public:
  bool scan(uint32_t* oldest, uint32_t* newest) override {
    DIR* d = opendir(kDir);
    if (!d) return false;
    bool any = false;
    while (struct dirent* e = readdir(d)) {
      char* end = nullptr;
      unsigned long id = std::strtoul(e->d_name, &end, 10);
      if (end == e->d_name || *end) continue;
      if (!any || id < *oldest) *oldest = uint32_t(id);
      if (!any || id > *newest) *newest = uint32_t(id);
      any = true;
    }
    closedir(d);
    return any;
  }
  uint32_t bytes(uint32_t id) override {
    char p[40];
    seg_path(p, sizeof p, id);
    struct stat st;
    return stat(p, &st) == 0 ? uint32_t(st.st_size) : 0;
  }
  // Open, seek to the end, write, close for every record. The core's esp_littlefs does not honour O_APPEND, and a writer
  // handle kept open did not make records visible to other opens (hardware test 14 Oct 2026: reopened segments were
  // written from offset 0, overwriting their oldest records; the count ran past the file and reads failed).
  bool append(uint32_t id, const uint8_t* data, size_t len) override {
    char p[40];
    seg_path(p, sizeof p, id);
    int fd = open(p, O_WRONLY | O_CREAT, 0644);
    if (fd < 0) { fs_error("open for append", id); return false; }
    bool ok = lseek(fd, 0, SEEK_END) >= 0 && write(fd, data, len) == ssize_t(len);
    if (!ok) fs_error("write", id);
    if (close(fd) != 0) {                                  // close() commits
      if (ok) fs_error("close", id);
      ok = false;
    }
    return ok;
  }
  bool read(uint32_t id, uint32_t offset, uint8_t* out, size_t len) override {
    char p[40];
    seg_path(p, sizeof p, id);
    int fd = open(p, O_RDONLY);
    if (fd < 0) { fs_error("open for read", id); return false; }
    bool ok = lseek(fd, off_t(offset), SEEK_SET) == off_t(offset) && ::read(fd, out, len) == ssize_t(len);
    if (!ok) fs_error("read", id);
    close(fd);
    return ok;
  }
  bool remove(uint32_t id) override {
    char p[40];
    seg_path(p, sizeof p, id);
    bool ok = unlink(p) == 0;
    if (!ok) fs_error("remove", id);
    return ok;
  }

};

class LegacyFile final : public ByteStore {
 public:
  explicit LegacyFile(int fd) : fd_(fd) {}
  bool read(uint32_t offset, uint8_t* out, size_t len) override {
    return lseek(fd_, off_t(offset), SEEK_SET) == off_t(offset) && ::read(fd_, out, len) == ssize_t(len);
  }

 private:
  int fd_;
};

LfsSegments g_segs;
EventLog g_log;
EventBudget g_budget;
RTC_NOINIT_ATTR EventBudgetState rtc_budget;              // survives warm resets: reboot loops cannot refill it
SemaphoreHandle_t g_lock = nullptr;
StaticSemaphore_t g_lock_buf;
uint32_t g_boot = 0;
bool g_ready = false;

// 512 appends are ~1,000 LittleFS commits and ~120 block erases (~6 s, simulated with littlefs 2.8.1 in review
// round 2) inside setup(), on the trial boot after the update from fw 1.0.0: feed the 10-s task watchdog, or a slow
// flash rolls the update back.
bool add_imported(const EventRecord& r, void* ctx) {
  esp_task_wdt_reset();
  return static_cast<EventLog*>(ctx)->add(r);
}

}  // namespace

bool events_begin(uint32_t boot_count, bool cold_boot) {
  g_boot = boot_count;
  g_lock = xSemaphoreCreateMutexStatic(&g_lock_buf);
  if (cold_boot) rtc_budget.magic = 0;                    // RTC content is random after a power-on
  g_budget.begin(&rtc_budget);
  // 4 descriptors: the open segment, a reader, the tank mirror and the one-time import of the old ring file.
  if (!LittleFS.begin(true, kBase, 4, "spiffs")) {      // formats once if it held the old SPIFFS
    FS_LOGE("evlog", "LittleFS mount (and format) of the spiffs partition failed: no event log this run");
    return false;
  }
  struct stat st;
  if (stat(kDir, &st) != 0 && mkdir(kDir, 0755) != 0) {
    FS_LOGE("evlog", "cannot create %s: errno %d", kDir, errno);
    return false;
  }
  g_ready = g_log.begin(&g_segs);
  FS_LOGI("evlog", "LittleFS %u of %u bytes used; log segments %u..%u, %u records", unsigned(LittleFS.usedBytes()),
          unsigned(LittleFS.totalBytes()), unsigned(g_log.oldest_id()), unsigned(g_log.newest_id()), unsigned(g_log.count()));
  int fd = open(kLegacy, O_RDONLY);                        // fw 1.0.0 ring: import once, then delete
  if (fd >= 0) {
    LegacyFile old(fd);
    if (g_log.count() == 0) import_legacy_ring(&old, add_imported, &g_log);
    close(fd);
    unlink(kLegacy);
  }
  EventRecord last;                                        // setup only: no other task logs yet
  if (g_ready && g_log.get_newest(0, &last) && last.boot >= g_boot && last.boot < UINT32_MAX) g_boot = last.boot + 1;
  return g_ready;
}

uint32_t events_boot() { return g_boot; }

void log_event(Ev type, uint16_t arg, uint32_t value) {
  if (!g_ready || !g_lock) {
    static bool warned = false;
    if (!warned) FS_LOGW("evlog", "event %s dropped: event log not ready this run", event_name(uint16_t(type)));
    warned = true;
    return;
  }
  if (xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeouts::kEventLockMs)) != pdTRUE) {
    FS_LOGW("evlog", "event %s not logged: log busy", event_name(uint16_t(type)));
    return;
  }
  uint16_t dropped = 0;
  uint64_t now_ms = uint64_t(esp_timer_get_time() / 1000);
  if (g_budget.allow(uint16_t(type), now_ms, &dropped)) {
    EventRecord r;
    r.boot = g_boot;
    r.uptime_s = uint32_t(now_ms / 1000);
    r.type = uint16_t(uint16_t(type) | uint16_t(dropped << kEvDroppedShift));
    r.arg = arg;
    r.value = value;
    if (!g_log.add(r)) FS_LOGE("evlog", "event %s (arg %u) not stored", event_name(uint16_t(type)), unsigned(arg));
  }
  xSemaphoreGive(g_lock);
}

void events_tick() {
  if (!g_ready || !g_lock) return;
  if (xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeouts::kEventLockMs)) != pdTRUE) return;
  uint64_t now_ms = uint64_t(esp_timer_get_time() / 1000);
  uint16_t mask = 0;
  uint32_t total = 0;
  g_budget.tick(now_ms);
  if (g_budget.take_summary(now_ms, &mask, &total)) {
    EventRecord r;
    r.boot = g_boot;
    r.uptime_s = uint32_t(now_ms / 1000);
    r.type = uint16_t(Ev::EventsDropped);
    r.arg = mask;
    r.value = total;
    g_log.add(r);
  }
  xSemaphoreGive(g_lock);
}

uint16_t events_count() { return g_ready ? g_log.count() : 0; }

bool events_get(uint16_t newest_index, EventRecord* r) {
  if (!g_ready || xSemaphoreTake(g_lock, pdMS_TO_TICKS(timeouts::kEventLockMs)) != pdTRUE) return false;
  bool ok = g_log.get_newest(newest_index, r);
  xSemaphoreGive(g_lock);
  return ok;
}

// Small files (<= 512 B stay inline in LittleFS metadata), written whole: create + write + fsync + close.
bool fs_file_write(const char* name, const uint8_t* data, size_t len) {
  if (!g_ready) return false;
  char p[48];
  std::snprintf(p, sizeof p, "%s/%s", kBase, name);
  int fd = open(p, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) return false;
  bool ok = write(fd, data, len) == ssize_t(len) && fsync(fd) == 0;
  return close(fd) == 0 && ok;
}

bool fs_file_read_n(const char* name, uint8_t* out, size_t cap, size_t* got) {
  *got = 0;
  if (!g_ready) return false;
  char p[48];
  std::snprintf(p, sizeof p, "%s/%s", kBase, name);
  int fd = open(p, O_RDONLY);
  if (fd < 0) return false;
  const ssize_t n = ::read(fd, out, cap);
  close(fd);
  if (n < 0) return false;
  *got = size_t(n);
  return true;
}

bool fs_file_read(const char* name, uint8_t* out, size_t len) {
  if (!g_ready) return false;
  char p[48];
  std::snprintf(p, sizeof p, "%s/%s", kBase, name);
  int fd = open(p, O_RDONLY);
  if (fd < 0) return false;
  bool ok = ::read(fd, out, len) == ssize_t(len);
  close(fd);
  return ok;
}

// Append (open + lseek(SEEK_END) + write + close: esp_littlefs ignores O_APPEND, see SegStore::append).
bool fs_file_append(const char* name, const uint8_t* data, size_t len) {
  if (!g_ready) return false;
  char p[48];
  std::snprintf(p, sizeof p, "%s/%s", kBase, name);
  int fd = open(p, O_WRONLY | O_CREAT, 0644);
  if (fd < 0) return false;
  bool ok = lseek(fd, 0, SEEK_END) >= 0 && write(fd, data, len) == ssize_t(len);
  return close(fd) == 0 && ok;                           // close() commits
}

long fs_file_size(const char* name) {
  if (!g_ready) return -1;
  char p[48];
  std::snprintf(p, sizeof p, "%s/%s", kBase, name);
  struct stat st;
  return stat(p, &st) == 0 ? long(st.st_size) : -1;
}

bool fs_file_read_at(const char* name, uint32_t offset, uint8_t* out, size_t len) {
  if (!g_ready) return false;
  char p[48];
  std::snprintf(p, sizeof p, "%s/%s", kBase, name);
  int fd = open(p, O_RDONLY);
  if (fd < 0) return false;
  bool ok = lseek(fd, off_t(offset), SEEK_SET) == off_t(offset) && ::read(fd, out, len) == ssize_t(len);
  close(fd);
  return ok;
}

bool fs_file_remove(const char* name) {
  if (!g_ready) return false;
  char p[48];
  std::snprintf(p, sizeof p, "%s/%s", kBase, name);
  struct stat st;
  return stat(p, &st) != 0 || unlink(p) == 0;            // true when it is gone
}

bool fs_file_rename(const char* from, const char* to) {
  if (!g_ready) return false;
  char a[48], b[48];
  std::snprintf(a, sizeof a, "%s/%s", kBase, from);
  std::snprintf(b, sizeof b, "%s/%s", kBase, to);
  return rename(a, b) == 0;                              // LittleFS: atomic, replaces `to`
}

}  // namespace fs
