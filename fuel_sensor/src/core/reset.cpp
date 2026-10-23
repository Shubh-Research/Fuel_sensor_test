// Reset to defaults (DESIGN.md §9.3).
#include "reset.h"

namespace fs {

namespace {

bool reset_all(ResetTarget& target, uint16_t items, uint16_t gen) {
  bool ok = true;
  for (uint16_t bit = 1; bit && bit <= kResetKnownItems; bit = uint16_t(bit << 1))   // every item is tried, also after a
    if ((items & bit) && bit != kResetLedger) ok = target.reset_item(bit, gen) && ok;  // failure; the ledger LAST: its
  if (items & kResetLedger) ok = target.reset_item(kResetLedger, gen) && ok;          // generation marks a finished reset
  return ok;
}

}  // namespace

bool reset_confirm_ok(const char* word) {
  if (!word) return false;
  const char* w = kResetConfirmWord;
  while (*w && *word == *w) ++word, ++w;
  return *w == '\0' && *word == '\0';
}

const char* reset_result_text(ResetResult r) {
  switch (r) {
    case ResetResult::Ok: return "ok";
    case ResetResult::JournalError: return "Could not save to the device memory. Nothing was changed.";
  }
  return "error";
}

ResetResult request_reset(ResetJournal& journal, ResetTarget& target, uint16_t items, uint16_t gen) {
  uint32_t pending = 0;
  if (journal.read(&pending)) {                                   // an unfinished earlier request: one reset, its target
    items = uint16_t(items | (pending & 0xFFFF));
    gen = uint16_t(pending >> 16);
  }
  items = uint16_t(items & kResetKnownItems);
  if (!journal.write(reset_record(items, gen))) return ResetResult::JournalError;   // nothing will change
  target.committed(items, false);                                 // from here the reset happens, at the next boot
  return ResetResult::Ok;
}

uint16_t resume_reset(ResetJournal& journal, ResetTarget& target) {
  uint32_t rec = 0;
  if (!journal.read(&rec)) return 0;
  const uint16_t items = uint16_t(rec & kResetKnownItems);
  const uint16_t gen = uint16_t(rec >> 16);
  const bool done = (items & kResetLedger) && target.finished(gen);   // an earlier boot finished it (cut before the clear)
  if (!done && !reset_all(target, items, gen)) return items;          // the journal stays: the next boot retries
  if (items) target.committed(items, true);                           // the audit trail shows the reset done at boot
  journal.clear();
  return items;
}

bool ledger_zero_at_boot(LedgerResetStore& s, uint16_t gen) {
  LedgerRecord old;
  uint8_t tok = 0;
  const bool have = s.newest(&old, &tok);
  if (have && old.gen == gen) return true;                    // done before: the bookings since are kept
  if (have && (old.filled_um || old.drained_um)) s.log_old(old);   // first: a cut cannot lose the evidence
  s.drop_rtc();
  LedgerRecord z;
  z.gen = gen;
  const bool n = s.write_nvs(z, tok);
  const bool m = s.write_mirror(z, tok);
  return s.nvs_usable() ? n : m;
}

}  // namespace fs
