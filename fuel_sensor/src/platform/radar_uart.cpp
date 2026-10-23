// WROVER board: radar packets from the separate sensor MCU over UART2 (DESIGN.md §6, §7.1).
#include "board.h"
#if defined(FS_CHIP_ESP32)

#include <Arduino.h>
#include <driver/uart.h>
#include "../core/iq_packet.h"
#include "radar_source.h"

namespace fs {

namespace {

constexpr uart_port_t kPort = UART_NUM_2;
constexpr int kRxBufferBytes = 16384;     // 21 packets; the decoder drains it every few ms

class UartRadar final : public RadarSource {
 public:
  bool begin() override {
    uart_config_t cfg = {};
    cfg.baud_rate = int(board::kRadarBaud);
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;
    if (uart_driver_install(kPort, kRxBufferBytes, 0, 0, nullptr, 0) != ESP_OK) return false;
    if (uart_param_config(kPort, &cfg) != ESP_OK) return false;
    // TX is wired to the sensor MCU but never driven by this firmware (DESIGN.md §6).
    if (uart_set_pin(kPort, UART_PIN_NO_CHANGE, board::kRadarRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
      return false;
    started_ = true;
    return true;
  }

  bool read(Frame& out, uint32_t timeout_ms) override {
    if (!started_) return false;
    uint32_t t0 = millis();
    for (;;) {
      while (pos_ < len_) {                                  // bytes left over from the previous read first
        uint8_t b = chunk_[pos_++];
        if (decoder_.feed(b, rx_ms_, &out)) {
          ++stats_.frames;
          if (out.flags & kFrameGapBefore) ++stats_.gaps;
          sync_stats();
          return true;
        }
      }
      uint32_t elapsed = millis() - t0;
      if (elapsed >= timeout_ms) { sync_stats(); return false; }
      uint32_t wait = timeout_ms - elapsed;
      if (wait > 20) wait = 20;                              // short waits keep the caller responsive
      int n = uart_read_bytes(kPort, chunk_, sizeof chunk_, pdMS_TO_TICKS(wait));
      rx_ms_ = millis();
      if (n > 0) { len_ = size_t(n); pos_ = 0; }
      else { len_ = pos_ = 0; }
    }
  }

  void drop_input() override { uart_flush_input(kPort); }
  void reset_link() override {
    uart_flush_input(kPort);
    decoder_.reset();
    len_ = pos_ = 0;
    ++stats_.link_resets;
  }

  RadarStats stats() const override { return stats_; }
  const char* name() const override { return "uart"; }

 private:
  void sync_stats() {
    const PacketStats& p = decoder_.stats();
    stats_.bad = p.bad;
    stats_.config_rejects = p.config_rejects;
    stats_.recalibrations = p.epoch_changes;
  }
  PacketDecoder decoder_;
  RadarStats stats_;
  uint8_t chunk_[512] = {};
  size_t len_ = 0, pos_ = 0;
  uint32_t rx_ms_ = 0;
  bool started_ = false;
};

UartRadar g_radar;

}  // namespace

RadarSource& radar_source() { return g_radar; }

}  // namespace fs

#endif  // FS_CHIP_ESP32
