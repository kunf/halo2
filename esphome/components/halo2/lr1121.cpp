#include "lr1121.h"

#include <algorithm>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cstdio>

namespace esphome {
namespace halo2 {

static const char *const TAG = "halo2.lr1121";

// Opcodes (Semtech LR1121 user manual), as used by src/lr1121/__init__.py
static constexpr uint16_t CMD_GET_STATUS = 0x0100;
static constexpr uint16_t CMD_GET_VERSION = 0x0101;
static constexpr uint16_t CMD_WRITE_BUFFER = 0x0109;
static constexpr uint16_t CMD_READ_BUFFER = 0x010A;
static constexpr uint16_t CMD_CLEAR_RX_BUFFER = 0x010B;
static constexpr uint16_t CMD_GET_ERRORS = 0x010D;
static constexpr uint16_t CMD_CLEAR_ERRORS = 0x010E;
static constexpr uint16_t CMD_CALIBRATE = 0x010F;
static constexpr uint16_t CMD_SET_REG_MODE = 0x0110;
static constexpr uint16_t CMD_SET_DIO_AS_RF_SWITCH = 0x0112;
static constexpr uint16_t CMD_SET_DIO_IRQ_PARAMS = 0x0113;
static constexpr uint16_t CMD_CLEAR_IRQ = 0x0114;
static constexpr uint16_t CMD_SET_TCXO_MODE = 0x0117;
static constexpr uint16_t CMD_SET_STANDBY = 0x011C;
static constexpr uint16_t CMD_SET_FS = 0x011D;
static constexpr uint8_t STDBY_RC = 0x00;
static constexpr uint8_t STDBY_XOSC = 0x01;  // Keeps the 32 MHz oscillator running
static constexpr uint8_t FALLBACK_STDBY_XOSC = 0x02;
static constexpr uint16_t ERR_HF_XOSC_START = 0x0020;
// TCXO start-up timeout in 30.52 us steps: ~31 ms (the MicroPython driver used ~9 ms)
static constexpr uint32_t TCXO_TIMEOUT = 0x000400;
static constexpr uint16_t CMD_GET_RX_BUFFER_STATUS = 0x0203;
static constexpr uint16_t CMD_SET_GFSK_SYNC_WORD = 0x0206;
static constexpr uint16_t CMD_SET_RX = 0x0209;
static constexpr uint16_t CMD_SET_TX = 0x020A;
static constexpr uint16_t CMD_SET_RF_FREQUENCY = 0x020B;
static constexpr uint16_t CMD_AUTO_TX_RX = 0x020C;
static constexpr uint16_t CMD_SET_PKT_TYPE = 0x020E;
static constexpr uint16_t CMD_SET_MOD_PARAMS = 0x020F;
static constexpr uint16_t CMD_SET_PKT_PARAMS = 0x0210;
static constexpr uint16_t CMD_SET_TX_PARAMS = 0x0211;
static constexpr uint16_t CMD_SET_RX_TX_FALLBACK = 0x0213;
static constexpr uint16_t CMD_SET_PA_CONFIG = 0x0215;
static constexpr uint16_t CMD_SET_RX_BOOSTED = 0x0227;

static constexpr uint32_t IRQ_TX_DONE = 0x00000004;
static constexpr uint32_t IRQ_RX_DONE = 0x00000008;
static constexpr uint32_t RF_FREQUENCY_HZ = 2405000000UL;  // Channel 1
static constexpr uint32_t BITRATE = 125000;
static constexpr uint32_t FREQ_DEVIATION = 160000;
static constexpr uint32_t REPLY_TIMEOUT_MS = 20;

#define HI(op) static_cast<uint8_t>((op) >> 8)
#define LO(op) static_cast<uint8_t>((op) & 0xFF)
#define BE32(v) \
  static_cast<uint8_t>((v) >> 24), static_cast<uint8_t>((v) >> 16), static_cast<uint8_t>((v) >> 8), \
      static_cast<uint8_t>(v)

bool LR1121Radio::wait_busy_(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (this->busy_pin_->digital_read()) {
    if (millis() - start > timeout_ms) {
      if (!this->failed_) {
        snprintf(this->failure_, sizeof(this->failure_), "BUSY stayed high for %" PRIu32 " ms after command 0x%04X",
                 timeout_ms, this->last_cmd_);
        ESP_LOGE(TAG, "%s", this->failure_);
      }
      this->failed_ = true;
      return false;
    }
    delayMicroseconds(10);
  }
  return true;
}

// Every command transaction returns Stat1 in its first byte. Its bits 3:1 are the status of
// the previous command: 0 CMD_FAIL, 1 CMD_PERR (bad parameters), 2 CMD_OK, 3 CMD_DAT.
void LR1121Radio::check_stat1_(uint8_t stat1) {
  const uint8_t status = (stat1 >> 1) & 0x07;
  if (this->last_cmd_ != 0 && status <= 1) {
    if (this->cmd_failure_count_ < this->cmd_failures_.size()) {
      this->cmd_failures_[this->cmd_failure_count_] = {this->last_cmd_, status};
    }
    this->cmd_failure_count_++;
    if (this->cmd_failure_count_ <= 20) {
      ESP_LOGW(TAG, "Command 0x%04X (params %s) was rejected: %s", this->last_cmd_, this->last_params_,
               status == 0 ? "CMD_FAIL" : "CMD_PERR (bad parameters)");
    }
  }
}

void LR1121Radio::begin_cmd_(const uint8_t *data, size_t len) {
  this->spi_->enable();
  this->check_stat1_(this->spi_->transfer_byte(data[0]));
  for (size_t i = 1; i < len; i++)
    this->spi_->write_byte(data[i]);
  this->spi_->disable();
  this->last_cmd_ = static_cast<uint16_t>((data[0] << 8) | data[1]);
  to_hex(data + 2, len > 2 ? (len - 2 > 12 ? 12 : len - 2) : 0, this->last_params_, sizeof(this->last_params_));
}

void LR1121Radio::write_cmd_(std::initializer_list<uint8_t> data) {
  if (!this->wait_busy_())
    return;
  this->begin_cmd_(data.begin(), data.size());
  delayMicroseconds(50);
}

void LR1121Radio::write_cmd_(uint16_t opcode, const uint8_t *params, size_t len) {
  if (!this->wait_busy_())
    return;
  this->spi_->enable();
  this->check_stat1_(this->spi_->transfer_byte(HI(opcode)));
  this->spi_->write_byte(LO(opcode));
  this->spi_->write_array(params, len);
  this->spi_->disable();
  this->last_cmd_ = opcode;
  to_hex(params, len > 12 ? 12 : len, this->last_params_, sizeof(this->last_params_));
  delayMicroseconds(50);
}

bool LR1121Radio::read_cmd_(std::initializer_list<uint8_t> data, uint8_t *out, size_t len) {
  if (!this->wait_busy_())
    return false;
  this->begin_cmd_(data.begin(), data.size());
  if (!this->wait_busy_())
    return false;
  this->spi_->enable();
  this->spi_->read_array(out, len);
  this->spi_->disable();
  this->last_cmd_ = 0;  // Stat1 of the next command describes this read, not a command
  return true;
}

void LR1121Radio::set_packet_length_(uint8_t len) {
  this->write_cmd_({HI(CMD_SET_PKT_PARAMS), LO(CMD_SET_PKT_PARAMS),
                    0x00, 0x08,  // Preamble length: 8 bits
                    0x00,        // Preamble detector: off
                    0x20,        // Sync word length: 32 bits
                    0x00,        // No address filtering
                    0x00,        // Fixed-length packets
                    len,         // Payload length
                    0x01,        // No CRC (done in software)
                    0x00});      // No whitening
  this->pkt_len_ = len;
}

bool LR1121Radio::init(const Address &sync_word) {
  this->failed_ = false;
  this->failure_[0] = '\0';
  this->last_cmd_ = 0;
  this->cmd_failure_count_ = 0;
  this->reset_pin_->setup();
  this->busy_pin_->setup();
  this->irq_pin_->setup();

  this->reset_pin_->digital_write(false);
  delay(20);
  this->reset_pin_->digital_write(true);
  delay(50);
  if (!this->wait_busy_(1000))
    return false;

  uint8_t version[5] = {};
  if (!this->read_cmd_({HI(CMD_GET_VERSION), LO(CMD_GET_VERSION)}, version, sizeof(version)))
    return false;
  std::copy(version + 1, version + 5, this->version_.begin());
  bool all_zero = true, all_ff = true;
  for (uint8_t b : this->version_) {
    all_zero &= (b == 0x00);
    all_ff &= (b == 0xFF);
  }
  if (all_zero || all_ff) {
    snprintf(this->failure_, sizeof(this->failure_), "LR1121 not found (version %02X %02X %02X %02X), check wiring",
             version_[0], version_[1], version_[2], version_[3]);
    ESP_LOGE(TAG, "%s", this->failure_);
    this->failed_ = true;
    return false;
  }

  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), STDBY_RC});
  this->write_cmd_({HI(CMD_SET_REG_MODE), LO(CMD_SET_REG_MODE), 0x01});
  this->write_cmd_({HI(CMD_SET_DIO_AS_RF_SWITCH), LO(CMD_SET_DIO_AS_RF_SWITCH), 0x03, 0x01, 0x01, 0x01, 0x01, 0x01,
                    0x00, 0x00});
  if (this->tcxo_voltage_code_ <= 7) {
    this->write_cmd_({HI(CMD_SET_TCXO_MODE), LO(CMD_SET_TCXO_MODE), this->tcxo_voltage_code_,
                      static_cast<uint8_t>(TCXO_TIMEOUT >> 16), static_cast<uint8_t>(TCXO_TIMEOUT >> 8),
                      static_cast<uint8_t>(TCXO_TIMEOUT)});
  }
  this->write_cmd_({HI(CMD_CLEAR_ERRORS), LO(CMD_CLEAR_ERRORS)});
  this->write_cmd_({HI(CMD_CALIBRATE), LO(CMD_CALIBRATE), 0x3F});  // All blocks, as Semtech's reference driver
  delay(10);
  this->errors_ = this->read_errors_();
  this->write_cmd_({HI(CMD_CLEAR_ERRORS), LO(CMD_CLEAR_ERRORS)});

  this->write_cmd_({HI(CMD_SET_PKT_TYPE), LO(CMD_SET_PKT_TYPE), 0x01});  // GFSK
  this->write_cmd_({HI(CMD_SET_RF_FREQUENCY), LO(CMD_SET_RF_FREQUENCY), BE32(RF_FREQUENCY_HZ)});
  this->write_cmd_(
      {HI(CMD_SET_MOD_PARAMS), LO(CMD_SET_MOD_PARAMS), BE32(BITRATE), 0x09, 0x09, BE32(FREQ_DEVIATION)});
  this->set_packet_length_(SNIFF_LEN);
  // Stay in standby until everything is configured: settings sent while in RX can be rejected
  this->write_sync_word_(sync_word);
  // 2.4 GHz (HF) PA. Semtech's reference driver uses duty cycle 0 and HP select 0 for the
  // HF PA; the MicroPython driver's duty cycle of 4 is out of range for it.
  this->write_cmd_({HI(CMD_SET_PA_CONFIG), LO(CMD_SET_PA_CONFIG), 0x02, 0x00, 0x00, 0x00});
  this->write_cmd_({HI(CMD_SET_TX_PARAMS), LO(CMD_SET_TX_PARAMS), 0x00, 0x02});
  this->write_cmd_({HI(CMD_SET_RX_TX_FALLBACK), LO(CMD_SET_RX_TX_FALLBACK), FALLBACK_STDBY_XOSC});
  this->write_cmd_({HI(CMD_SET_RX_BOOSTED), LO(CMD_SET_RX_BOOSTED), 0x01});
  this->write_cmd_({HI(CMD_AUTO_TX_RX), LO(CMD_AUTO_TX_RX), 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00});
  // RX_DONE on DIO9 (the IRQ status register latches the other IRQs too)
  this->write_cmd_({HI(CMD_SET_DIO_IRQ_PARAMS), LO(CMD_SET_DIO_IRQ_PARAMS), BE32(IRQ_RX_DONE), 0x00, 0x00, 0x00, 0x00});
  this->errors_ |= this->read_errors_();

  // Clock check: frequency synthesis needs the 32 MHz oscillator and the PLL
  this->write_cmd_({HI(CMD_CLEAR_ERRORS), LO(CMD_CLEAR_ERRORS)});
  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), STDBY_XOSC});
  this->write_cmd_({HI(CMD_SET_FS), LO(CMD_SET_FS)});
  delay(5);
  this->clock_check_errors_ = this->read_errors_();
  if (this->clock_check_errors_ & ERR_HF_XOSC_START)
    ESP_LOGE(TAG, "32 MHz oscillator did not start; check tcxo_voltage");
  this->write_cmd_({HI(CMD_CLEAR_ERRORS), LO(CMD_CLEAR_ERRORS)});
  this->listen();
  return !this->failed_;
}

void LR1121Radio::write_sync_word_(const Address &sync_word) {
  this->write_cmd_({HI(CMD_SET_GFSK_SYNC_WORD), LO(CMD_SET_GFSK_SYNC_WORD), sync_word[0], sync_word[1], sync_word[2],
                    sync_word[3], 0x00, 0x00, 0x00, 0x00});
}

void LR1121Radio::set_sync_word(const Address &sync_word) {
  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), STDBY_XOSC});
  this->write_sync_word_(sync_word);
  this->listen();
}

uint32_t LR1121Radio::read_irq_() {
  // GetStatus returns Stat1, Stat2 and IrqStatus during the command itself
  if (!this->wait_busy_())
    return 0;
  uint8_t buf[6] = {HI(CMD_GET_STATUS), LO(CMD_GET_STATUS), 0x00, 0x00, 0x00, 0x00};
  this->spi_->enable();
  this->spi_->transfer_array(buf, sizeof(buf));
  this->spi_->disable();
  this->check_stat1_(buf[0]);
  this->last_cmd_ = CMD_GET_STATUS;
  this->last_params_[0] = '\0';
  return (uint32_t(buf[2]) << 24) | (uint32_t(buf[3]) << 16) | (uint32_t(buf[4]) << 8) | uint32_t(buf[5]);
}

uint16_t LR1121Radio::read_errors_() {
  uint8_t errors[3] = {};
  if (!this->read_cmd_({HI(CMD_GET_ERRORS), LO(CMD_GET_ERRORS)}, errors, sizeof(errors)))
    return 0;
  return static_cast<uint16_t>((errors[1] << 8) | errors[2]);
}

void LR1121Radio::restart_rx_() {
  this->write_cmd_({HI(CMD_AUTO_TX_RX), LO(CMD_AUTO_TX_RX), 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00});
  this->write_cmd_({HI(CMD_CLEAR_IRQ), LO(CMD_CLEAR_IRQ), 0xFF, 0xFF, 0xFF, 0xFF});
  this->write_cmd_({HI(CMD_CLEAR_RX_BUFFER), LO(CMD_CLEAR_RX_BUFFER)});
  this->write_cmd_({HI(CMD_SET_RX), LO(CMD_SET_RX), 0x00, 0x00, 0x00});
}

void LR1121Radio::listen() {
  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), STDBY_XOSC});
  if (this->pkt_len_ != SNIFF_LEN)
    this->set_packet_length_(SNIFF_LEN);
  this->restart_rx_();
}

size_t LR1121Radio::read_raw(uint8_t *buf, size_t max_len) {
  if (this->failed_ || !this->irq_pin_->digital_read())
    return 0;
  return this->read_packet_(IRQ_RX_DONE, buf, max_len);  // DIO9 only signals RX_DONE
}

size_t LR1121Radio::read_packet_(uint32_t irq, uint8_t *buf, size_t max_len) {
  size_t len = 0;
  if (irq & IRQ_RX_DONE) {
    uint8_t rx_status[3] = {};
    if (!this->read_cmd_({HI(CMD_GET_RX_BUFFER_STATUS), LO(CMD_GET_RX_BUFFER_STATUS)}, rx_status, sizeof(rx_status)))
      return 0;
    len = rx_status[1];
    const uint8_t offset = rx_status[2];
    if (len > max_len)
      len = max_len;
    uint8_t data[SNIFF_LEN + 1] = {};
    if (len > SNIFF_LEN)
      len = SNIFF_LEN;
    if (!this->read_cmd_({HI(CMD_READ_BUFFER), LO(CMD_READ_BUFFER), offset, static_cast<uint8_t>(len)}, data,
                         len + 1))
      return 0;
    std::copy(data + 1, data + 1 + len, buf);  // First byte is the status
  }
  // Back to sniffing (also clears any IRQ that isn't RX_DONE so DIO9 can't stay high)
  if (this->pkt_len_ != SNIFF_LEN)
    this->set_packet_length_(SNIFF_LEN);
  this->restart_rx_();
  return len;
}

bool LR1121Radio::transact(const Address &address, const RawPacket &packet, Packet *reply) {
  if (this->failed_)
    return false;
  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), STDBY_XOSC});
  this->set_packet_length_(PACKET_LEN);
  this->write_cmd_({HI(CMD_CLEAR_IRQ), LO(CMD_CLEAR_IRQ), 0xFF, 0xFF, 0xFF, 0xFF});
  this->write_cmd_({HI(CMD_CLEAR_RX_BUFFER), LO(CMD_CLEAR_RX_BUFFER)});
  // Switch to RX right after TX to catch the lamp's reply
  this->write_cmd_({HI(CMD_AUTO_TX_RX), LO(CMD_AUTO_TX_RX), 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00});
  this->write_cmd_(CMD_WRITE_BUFFER, packet.data(), packet.size());
  this->write_cmd_({HI(CMD_SET_TX), LO(CMD_SET_TX), 0x00, 0x00, 0x00});

  const uint32_t start = millis();
  while (!this->irq_pin_->digital_read() && millis() - start <= REPLY_TIMEOUT_MS)
    delayMicroseconds(10);

  const bool rx_done = this->irq_pin_->digital_read();
  uint32_t irq = this->read_irq_();
  this->tx_count_++;
  if (irq & IRQ_TX_DONE)
    this->tx_done_count_++;
  uint8_t buf[SNIFF_LEN];
  const size_t len = this->read_packet_(rx_done ? IRQ_RX_DONE : 0, buf, sizeof(buf));  // Also restarts sniffing
  if (len < PACKET_LEN) {
    const uint16_t errors = this->read_errors_();
    ESP_LOGD(TAG,
             "No reply from lamp (TX %s, IRQ 0x%08" PRIX32 ", errors 0x%04X, %" PRIu32 "/%" PRIu32
             " transmissions completed)",
             (irq & IRQ_TX_DONE) ? "done" : "NOT done", irq, errors, this->tx_done_count_, this->tx_count_);
    if (errors != 0)
      this->write_cmd_({HI(CMD_CLEAR_ERRORS), LO(CMD_CLEAR_ERRORS)});
    this->listen();
    return false;
  }
  Packet p = parse_packet(address, buf, len);
  if (!p.crc_valid || p.pkt_len != PAYLOAD_LEN) {
    char hex[SNIFF_LEN * 3];
    to_hex(buf, len, hex, sizeof(hex));
    ESP_LOGD(TAG, "Reply with bad CRC: %s", hex);
    return false;
  }
  this->reply_count_++;
  *reply = p;
  return true;
}

}  // namespace halo2
}  // namespace esphome
