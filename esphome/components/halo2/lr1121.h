#pragma once

// Semtech LR1121 (e.g. Waveshare Core1121-HF) driver for the HALO 2 protocol.
// Port of src/lr1121/__init__.py: GFSK 125 kbps on 2405 MHz, the lamp address as
// 32-bit sync word, no hardware CRC (the packet is built in software).

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"
#include "protocol.h"

namespace esphome {
namespace halo2 {

using Halo2SPIDevice = spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                      spi::DATA_RATE_4MHZ>;

class LR1121Radio {
 public:
  void set_spi(Halo2SPIDevice *spi) { this->spi_ = spi; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_busy_pin(GPIOPin *pin) { this->busy_pin_ = pin; }
  void set_irq_pin(GPIOPin *pin) { this->irq_pin_ = pin; }

  /// Reset and configure the chip. Returns false if it is missing or not responding.
  bool init(const Address &sync_word);
  /// Change the sync word (lamp address) and restart RX.
  void set_sync_word(const Address &sync_word);
  /// Start continuous RX, capturing SNIFF_LEN bytes after each sync word.
  void listen();

  /// Read a captured packet if one is waiting. Returns its length, 0 if none.
  size_t read_raw(uint8_t *buf, size_t max_len);

  /// Send a packet and wait briefly for the lamp's reply. Returns true and fills
  /// `reply` with a CRC-valid packet, as send_with_ack() + receive_packet().
  bool transact(const Address &address, const RawPacket &packet, Packet *reply);

  /// Firmware version bytes read during init (hardware, use case, major, minor).
  const std::array<uint8_t, 4> &version() const { return this->version_; }
  bool is_failed() const { return this->failed_; }
  /// Why the radio failed, empty if it didn't.
  const char *failure() const { return this->failure_; }
  /// LR1121 GetErrors flags read after calibration (0 = none).
  uint16_t errors() const { return this->errors_; }
  uint32_t tx_count() const { return this->tx_count_; }
  uint32_t tx_done_count() const { return this->tx_done_count_; }
  uint32_t reply_count() const { return this->reply_count_; }

 protected:
  bool wait_busy_(uint32_t timeout_ms = 1000);  // Same as the MicroPython driver
  void write_cmd_(std::initializer_list<uint8_t> opcode_and_params);
  void write_cmd_(uint16_t opcode, const uint8_t *params, size_t len);
  bool read_cmd_(std::initializer_list<uint8_t> opcode_and_params, uint8_t *out, size_t len);
  void set_packet_length_(uint8_t len);
  void write_sync_word_(const Address &sync_word);
  uint32_t read_irq_();
  size_t read_packet_(uint32_t irq, uint8_t *buf, size_t max_len);
  void restart_rx_();

  Halo2SPIDevice *spi_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *busy_pin_{nullptr};
  GPIOPin *irq_pin_{nullptr};
  uint8_t pkt_len_{0};
  bool failed_{false};
  uint16_t last_cmd_{0};
  uint16_t errors_{0};
  uint32_t tx_count_{0};
  uint32_t tx_done_count_{0};
  uint32_t reply_count_{0};
  char failure_[80]{};
  std::array<uint8_t, 4> version_{};
};

}  // namespace halo2
}  // namespace esphome
