// ESP32-C3 product board: the A121 on SPI2, Acconeer RSS (a121 v1.13, sparse I/Q service) running on the C3
// (DESIGN.md §6). The configuration is the one every recording and the calibration used (setup.json session_config):
// start -8, 169 points, step 1, profile 1, HWAAS 183, receiver gain 10, TX on, loopback off, phase enhancement on,
// IQ-imbalance compensation on, PRF 19.5 MHz, 1 sweep per frame, frames triggered every 100 ms by this firmware.
// HAL after esp32c3_xe121/source/acc_hal_integration_espidf_xe121.c (Acconeer, BSD-3-Clause) with our board pins.
// NOT YET RUN ON HARDWARE: compile- and size-checked only until a C3 board exists.
#include "board.h"
#if defined(FS_CHIP_ESP32C3)

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <freertos/semphr.h>

extern "C" {
#include "acc_config.h"
#include "acc_definitions_a121.h"
#include "acc_hal_definitions_a121.h"
#include "acc_processing.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
}
#include "radar_source.h"
#include "logs.h"

namespace fs {

namespace {

constexpr acc_sensor_id_t kSensorId = 1;
constexpr uint32_t kFramePeriodMs = 100;           // 10 Hz, as in every recording
constexpr uint32_t kSensorTimeoutMs = 1000;
constexpr size_t kMaxSpiTransfer = 8192;

spi_device_handle_t g_spi = nullptr;
SemaphoreHandle_t g_isr_sem = nullptr;
StaticSemaphore_t g_isr_sem_buf;

void IRAM_ATTR on_sensor_interrupt(void*) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(g_isr_sem, &woken);
  if (woken == pdTRUE) portYIELD_FROM_ISR();      // no-argument form: valid on both the RISC-V and Xtensa ports
}

void hal_transfer(acc_sensor_id_t, uint8_t* buffer, size_t size) {
  spi_transaction_t t = {};
  t.length = size * 8;
  t.tx_buffer = buffer;
  t.rx_buffer = buffer;
  spi_device_polling_transmit(g_spi, &t);
}

void hal_log(acc_log_level_t level, const char* module, const char* format, ...) {
  if (level > ACC_LOG_LEVEL_WARNING) return;
  char line[160];
  va_list ap;
  va_start(ap, format);
  vsnprintf(line, sizeof line, format, ap);
  va_end(ap);
  FS_LOGW("rss", "%s: %s", module, line);
}

bool wait_interrupt(uint32_t timeout_ms) {
  if (gpio_get_level(gpio_num_t(board::kA121Int)) == 1) {
    xSemaphoreTake(g_isr_sem, 0);
    return true;
  }
  return xSemaphoreTake(g_isr_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

const acc_hal_a121_t g_hal = {
    kMaxSpiTransfer,                 // max_spi_transfer_size
    malloc,                          // mem_alloc (used at setup only)
    free,                            // mem_free
    hal_transfer,                    // transfer
    hal_log,                         // log
    {nullptr},                       // optimization.transfer16
};

class A121Radar final : public RadarSource {
 public:
  bool begin() override {
    // IO3/IO4 to the GPIO function first (IO4 is MTMS); as Acconeer's HAL does for its interrupt pin.
    esp_rom_gpio_pad_select_gpio(board::kA121Enable);
    esp_rom_gpio_pad_select_gpio(board::kA121Int);
    gpio_set_direction(gpio_num_t(board::kA121Enable), GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num_t(board::kA121Enable), 0);
    g_isr_sem = xSemaphoreCreateBinaryStatic(&g_isr_sem_buf);
    gpio_set_direction(gpio_num_t(board::kA121Int), GPIO_MODE_INPUT);
    gpio_pulldown_en(gpio_num_t(board::kA121Int));
    gpio_pullup_dis(gpio_num_t(board::kA121Int));
    gpio_set_intr_type(gpio_num_t(board::kA121Int), GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio_num_t(board::kA121Int), on_sensor_interrupt, nullptr);
    spi_bus_config_t bus = {};
    bus.miso_io_num = board::kSpiMiso;
    bus.mosi_io_num = board::kSpiMosi;
    bus.sclk_io_num = board::kSpiSclk;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = int(kMaxSpiTransfer);
    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = int(board::kSpiHz);
    dev.mode = 0;
    dev.spics_io_num = board::kA121Ss;
    dev.queue_size = 1;
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;
    if (spi_bus_add_device(SPI2_HOST, &dev, &g_spi) != ESP_OK) return false;
    if (!acc_rss_hal_register(&g_hal)) return false;
    if (!open()) FS_LOGW("radar", "A121 not ready at boot; the radar task keeps retrying");
    return true;                            // bus and HAL are up; read() re-opens the sensor as needed
  }

  bool read(Frame& out, uint32_t timeout_ms) override {
    if (!sensor_ && !open()) { delay(timeout_ms < 200 ? timeout_ms : 200); return false; }
    // Keep the 100 ms cadence: wait for the next slot (bounded by the caller's timeout).
    int64_t now = esp_timer_get_time();
    int64_t due = next_us_;
    if (due > now) {
      uint32_t wait_ms = uint32_t((due - now) / 1000);
      if (wait_ms > timeout_ms) { delay(timeout_ms); return false; }
      delay(wait_ms);
    }
    int64_t t_meas = esp_timer_get_time();
    next_us_ = (next_us_ == 0 ? t_meas : next_us_) + int64_t(kFramePeriodMs) * 1000;
    if (next_us_ < t_meas) { next_us_ = t_meas + int64_t(kFramePeriodMs) * 1000; gap_ = true; }   // fell behind
    if (!acc_sensor_measure(sensor_) || !wait_interrupt(kSensorTimeoutMs) || !acc_sensor_read(sensor_, buffer_, buffer_size_)) {
      ++stats_.bad;
      close();
      gap_ = true;
      return false;
    }
    acc_processing_result_t res;
    acc_processing_execute(processing_, buffer_, &res);
    if (res.calibration_needed) {
      // Temperature drifted 15 C since the calibration. Acconeer (Sparse IQ guide 6.3, example_service.c): reset the
      // sensor, calibrate and prepare again, and do not use this frame. res.frame points INTO buffer_, which
      // calibrate() overwrites (and close() frees), so the frame could not be copied afterwards anyway.
      ++stats_.recalibrations;
      if (!calibrate()) close();
      gap_ = true;
      return false;
    }
    out.seq = seq_++;
    out.tick_ms = uint32_t(t_meas / 1000);
    out.rx_ms = millis();
    out.temperature_c = res.temperature;
    out.flags = uint16_t((res.data_saturated ? kFrameSaturated : 0) | (res.frame_delayed ? kFrameDelayed : 0) |
                         (gap_ ? kFrameGapBefore : 0));
    for (int i = 0; i < kPoints; ++i) {
      out.iq[i].i = res.frame[i].real;
      out.iq[i].q = res.frame[i].imag;
    }
    if (gap_) ++stats_.gaps;
    gap_ = false;
    ++stats_.frames;
    return true;
  }

  void reset_link() override {
    close();
    ++stats_.link_resets;
  }
  RadarStats stats() const override { return stats_; }
  const char* name() const override { return "a121-spi"; }

 private:
  bool open() {
    config_ = acc_config_create();
    if (!config_) return false;
    acc_config_start_point_set(config_, -8);
    acc_config_num_points_set(config_, kPoints);
    acc_config_step_length_set(config_, 1);
    acc_config_profile_set(config_, ACC_CONFIG_PROFILE_1);
    acc_config_hwaas_set(config_, 183);
    acc_config_receiver_gain_set(config_, 10);
    acc_config_sweeps_per_frame_set(config_, 1);
    acc_config_enable_tx_set(config_, true);
    acc_config_enable_loopback_set(config_, false);
    acc_config_phase_enhancement_set(config_, true);
    acc_config_iq_imbalance_compensation_set(config_, true);
    acc_config_prf_set(config_, ACC_CONFIG_PRF_19_5_MHZ);
    acc_config_inter_frame_idle_state_set(config_, ACC_CONFIG_IDLE_STATE_DEEP_SLEEP);
    acc_config_inter_sweep_idle_state_set(config_, ACC_CONFIG_IDLE_STATE_READY);
    acc_processing_metadata_t meta;
    processing_ = acc_processing_create(config_, &meta);
    if (!processing_ || meta.frame_data_length != kPoints) { close(); ++stats_.config_rejects; return false; }
    if (!acc_rss_get_buffer_size(config_, &buffer_size_)) { close(); return false; }
    buffer_ = malloc(buffer_size_);          // allocated per (re)open, never per frame
    if (!buffer_) { close(); return false; }
    sensor_enable();
    sensor_ = acc_sensor_create(kSensorId);
    if (!sensor_ || !calibrate()) { close(); return false; }
    gap_ = true;
    next_us_ = 0;
    return true;
  }

  // acc_hal_integration_sensor_enable(): ENABLE high, 2 ms for the crystal, then drop a stale interrupt.
  void sensor_enable() {
    gpio_set_level(gpio_num_t(board::kA121Enable), 1);
    delay(2);
    xSemaphoreTake(g_isr_sem, 0);
  }

  // do_sensor_calibration_and_prepare() of Acconeer's example_service.c: the sensor is reset (ENABLE low/high) before
  // EVERY attempt, including the first and the re-calibration after "calibration needed"; one retry.
  bool calibrate() {
    bool ok = false;
    for (int tries = 0; tries < 2 && !ok; ++tries) {
      gpio_set_level(gpio_num_t(board::kA121Enable), 0);
      delay(2);
      sensor_enable();
      bool complete = false;
      do {
        ok = acc_sensor_calibrate(sensor_, &complete, &cal_, buffer_, buffer_size_);
        if (ok && !complete) ok = wait_interrupt(kSensorTimeoutMs);
      } while (ok && !complete);
    }
    return ok && acc_sensor_prepare(sensor_, config_, &cal_, buffer_, buffer_size_);
  }

  void close() {
    if (sensor_) { acc_sensor_destroy(sensor_); sensor_ = nullptr; }
    if (processing_) { acc_processing_destroy(processing_); processing_ = nullptr; }
    if (config_) { acc_config_destroy(config_); config_ = nullptr; }
    if (buffer_) { free(buffer_); buffer_ = nullptr; }
    gpio_set_level(gpio_num_t(board::kA121Enable), 0);
  }

  acc_config_t* config_ = nullptr;
  acc_processing_t* processing_ = nullptr;
  acc_sensor_t* sensor_ = nullptr;
  acc_cal_result_t cal_ = {};
  void* buffer_ = nullptr;
  uint32_t buffer_size_ = 0;
  uint32_t seq_ = 0;
  int64_t next_us_ = 0;
  bool gap_ = true;
  RadarStats stats_;
};

A121Radar g_radar;

}  // namespace

RadarSource& radar_source() { return g_radar; }

}  // namespace fs

#endif  // FS_CHIP_ESP32C3
