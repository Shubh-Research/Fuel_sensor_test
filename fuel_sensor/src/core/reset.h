// "Reset to defaults" in Maintenance (DESIGN.md §9.3): the list of customer data it sets back, and a roll-forward
// journal. The reset runs at the NEXT BOOT, before any of its items is used: the request writes the journal (the commit
// point) and the device restarts about 2 s later; the boot finishes it however often it is cut. Pure C++; the device
// keeps the journal in NVS (fs_cfg/rsj) and the items in their own stores.
#pragma once
#include <cstdint>
#include "ledger.h"

namespace fs {

// One bit per item. The journal stores the set being reset.
enum ResetItem : uint16_t {
  kResetTank = 1u << 0,              // tank L x W x H + custom flag -> the calibrated Eicher tank (fs_cfg/tank, /tank.bin)
  kResetTankCalibration = 1u << 1,   // RESERVED: the per-tank (site) calibration of v7.0
  kResetLedger = 1u << 2,            // the fuel totals (filled / drained, counts) -> 0, a new reset generation; the old
                                     // totals stay in the event log (FUEL_RESET)
  kResetLevel = 1u << 3,             // the remembered level (fs_lvl/rec and its RTC copy): the next start-up runs without
                                     // memory (up to ~2 min to settle)
  kResetFillReports = 1u << 4,       // the fill-report list (/fills.bin), the since-reset report number and an open report;
                                     // the lifetime report serial is kept (DESIGN.md §14)
};
// THE LIST: what this firmware resets. The v7.0 calibration adds kResetTankCalibration and its case in the device's reset
// target (platform/app.cpp).
constexpr uint16_t kResetItems = kResetTank | kResetLedger | kResetLevel | kResetFillReports;
constexpr uint16_t kResetKnownItems = kResetTank | kResetTankCalibration | kResetLedger | kResetLevel | kResetFillReports;
// Never reset (DESIGN.md §9.3): authority and firmware keys, the firmware and its boot/OTA/trial state, the radar's
// A121 calibration, the compiled-in tracker calibration, the device identity (MAC, SSID, boot and power counters), the
// attempt counters of the challenge, the event log (the audit trail), the lifetime fill-report serial and the
// settings-changed counter (fs_sys/chg: +1 per tank change and per reset to defaults; the future RS-485 register).

// Confirmation the API requires in the body: {"confirm":"RESET"} (the page's "Reset now" button sends it).
constexpr const char* kResetConfirmWord = "RESET";
bool reset_confirm_ok(const char* word);     // exactly kResetConfirmWord

// The journal record (u32): the items (low 16 bits) and the generation the reset gives the fuel totals (high 16 bits:
// the stored generation + 1 at the request). Once the stored record holds that generation the reset is complete (the
// ledger goes last), so a journal that could not be cleared never repeats it over later bookings or settings.
constexpr uint32_t reset_record(uint16_t items, uint16_t gen) { return uint32_t(items) | uint32_t(gen) << 16; }

class ResetJournal {                 // one durable record, written atomically (one NVS entry on the device)
 public:
  virtual ~ResetJournal() = default;
  virtual bool read(uint32_t* rec) = 0;      // true (and the record) while a reset is pending
  virtual bool write(uint32_t rec) = 0;
  virtual bool clear() = 0;
};

class ResetTarget {
 public:
  virtual ~ResetTarget() = default;
  // one bit; idempotent: puts that item to its default (gen: the ledger's target generation)
  virtual bool reset_item(uint16_t item, uint16_t gen) = 0;
  // the stored fuel record already holds the target generation: an earlier boot finished every item
  virtual bool finished(uint16_t gen) = 0;
  virtual void committed(uint16_t items, bool at_boot) = 0;    // log RESET_TO_DEFAULTS (arg items, value 0 request / 1 done)
};

enum class ResetResult : uint8_t {
  Ok,              // the journal is written: the next boot resets the items
  JournalError,    // the journal could not be written: nothing will change
};
const char* reset_result_text(ResetResult r);

// The request: writes the journal (an unfinished earlier request is merged in, keeping its target generation) and logs
// committed(items, false). Nothing is reset now: the caller restarts the device.
ResetResult request_reset(ResetJournal& journal, ResetTarget& target, uint16_t items, uint16_t gen);
// At boot, before any reset item is used: performs a pending reset (every item is attempted even after one fails; the
// ledger last). Only when it is complete: committed(items, true) and the journal is cleared (else the next boot retries,
// without a second event). Returns the items of a pending reset, 0 if none.
uint16_t resume_reset(ResetJournal& journal, ResetTarget& target);

// The ledger item at boot (platform: the copies of the record; core: the order, testable on the host).
class LedgerResetStore {
 public:
  virtual ~LedgerResetStore() = default;
  virtual bool newest(LedgerRecord* r, uint8_t* tokens) = 0;   // the newest stored record (NVS, /fuel.bin, RTC after a warm boot)
  virtual void log_old(const LedgerRecord& old) = 0;           // FUEL_RESET (old filled, old drained)
  virtual void drop_rtc() = 0;                                 // the RTC record and slow account are not restored
  virtual bool write_nvs(const LedgerRecord& z, uint8_t tokens) = 0;
  virtual bool write_mirror(const LedgerRecord& z, uint8_t tokens) = 0;
  virtual bool nvs_usable() = 0;
};
// No-op (true) once the newest record holds gen. Else: FUEL_RESET with the old totals FIRST (only when they are not 0: a
// cut cannot lose the evidence), the RTC copies dropped, the zero record of generation gen written to NVS (its save
// tokens kept), then to /fuel.bin. False when NVS (or, NVS unusable, the mirror) was not written: the next boot retries.
bool ledger_zero_at_boot(LedgerResetStore& s, uint16_t gen);

}  // namespace fs
