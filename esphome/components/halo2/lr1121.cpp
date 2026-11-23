#include "lr1121.h"

#include <algorithm>

#include "esphome/core/log.h"

namespace esphome {
namespace halo2 {

static const char *const TAG = "halo2.lr1121";

// Opcodes (Semtech LR1121 user manual), as used by src/lr1121/__init__.py
static constexpr uint16_t CMD_GET_STATUS = 0x0100;
static constexpr uint16_t CMD_GET_VERSION = 0x0101;
static constexpr uint16_t CMD_WRITE_BUFFER = 0x0109;
static constexpr uint16_t CMD_READ_BUFFER = 0x010A;
static constexpr uint16_t CMD_CLEAR_RX_BUFFER = 0x010B;
static constexpr uint16_t CMD_CLEAR_ERRORS = 0x010E;
static constexpr uint16_t CMD_CALIBRATE = 0x010F;
static constexpr uint16_t CMD_SET_REG_MODE = 0x0110;
static constexpr uint16_t CMD_SET_DIO_AS_RF_SWITCH = 0x0112;
static constexpr uint16_t CMD_SET_DIO_IRQ_PARAMS = 0x0113;
static constexpr uint16_t CMD_CLEAR_IRQ = 0x0114;
static constexpr uint16_t CMD_SET_TCXO_MODE = 0x0117;
static constexpr uint16_t CMD_SET_STANDBY = 0x011C;
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
      if (!this->failed_)
        ESP_LOGE(TAG, "BUSY timeout: transceiver not responding");
      this->failed_ = true;
      return false;
    }
    delayMicroseconds(10);
  }
  return true;
}

void LR1121Radio::write_cmd_(std::initializer_list<uint8_t> data) {
  if (!this->wait_busy_())
    return;
  this->spi_->enable();
  for (uint8_t b : data)
    this->spi_->write_byte(b);
  this->spi_->disable();
  delayMicroseconds(50);
}

void LR1121Radio::write_cmd_(uint16_t opcode, const uint8_t *params, size_t len) {
  if (!this->wait_busy_())
    return;
  this->spi_->enable();
  this->spi_->write_byte(HI(opcode));
  this->spi_->write_byte(LO(opcode));
  this->spi_->write_array(params, len);
  this->spi_->disable();
  delayMicroseconds(50);
}

bool LR1121Radio::read_cmd_(std::initializer_list<uint8_t> data, uint8_t *out, size_t len) {
  if (!this->wait_busy_())
    return false;
  this->spi_->enable();
  for (uint8_t b : data)
    this->spi_->write_byte(b);
  this->spi_->disable();
  if (!this->wait_busy_())
    return false;
  this->spi_->enable();
  this->spi_->read_array(out, len);
  this->spi_->disable();
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
    ESP_LOGE(TAG, "LR1121 not found (version %02X %02X %02X %02X), check wiring", version_[0], version_[1],
             version_[2], version_[3]);
    this->failed_ = true;
    return false;
  }

  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), 0x00});
  this->write_cmd_({HI(CMD_SET_REG_MODE), LO(CMD_SET_REG_MODE), 0x01});
  this->write_cmd_({HI(CMD_SET_DIO_AS_RF_SWITCH), LO(CMD_SET_DIO_AS_RF_SWITCH), 0x03, 0x01, 0x01, 0x01, 0x01, 0x01,
                    0x00, 0x00});
  this->write_cmd_({HI(CMD_SET_TCXO_MODE), LO(CMD_SET_TCXO_MODE), 0x06, 0x00, 0x01, 0x2C});
  this->write_cmd_({HI(CMD_CLEAR_ERRORS), LO(CMD_CLEAR_ERRORS)});
  this->write_cmd_({HI(CMD_CALIBRATE), LO(CMD_CALIBRATE)});
  delay(10);

  this->write_cmd_({HI(CMD_SET_PKT_TYPE), LO(CMD_SET_PKT_TYPE), 0x01});  // GFSK
  this->write_cmd_({HI(CMD_SET_RF_FREQUENCY), LO(CMD_SET_RF_FREQUENCY), BE32(RF_FREQUENCY_HZ)});
  this->write_cmd_(
      {HI(CMD_SET_MOD_PARAMS), LO(CMD_SET_MOD_PARAMS), BE32(BITRATE), 0x09, 0x09, BE32(FREQ_DEVIATION)});
  this->set_packet_length_(SNIFF_LEN);
  this->set_sync_word(sync_word);
  this->write_cmd_({HI(CMD_SET_PA_CONFIG), LO(CMD_SET_PA_CONFIG), 0x02, 0x00, 0x04, 0x00});
  this->write_cmd_({HI(CMD_SET_TX_PARAMS), LO(CMD_SET_TX_PARAMS), 0x00, 0x02});
  this->write_cmd_({HI(CMD_SET_RX_TX_FALLBACK), LO(CMD_SET_RX_TX_FALLBACK), 0x01});
  this->write_cmd_({HI(CMD_SET_RX_BOOSTED), LO(CMD_SET_RX_BOOSTED), 0x01});
  this->write_cmd_({HI(CMD_AUTO_TX_RX), LO(CMD_AUTO_TX_RX), 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00});
  // Route RX_DONE to DIO9
  this->write_cmd_({HI(CMD_SET_DIO_IRQ_PARAMS), LO(CMD_SET_DIO_IRQ_PARAMS), BE32(IRQ_RX_DONE), 0x00, 0x00, 0x00, 0x00});
  this->listen();
  return !this->failed_;
}

void LR1121Radio::set_sync_word(const Address &sync_word) {
  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), 0x00});
  this->write_cmd_({HI(CMD_SET_GFSK_SYNC_WORD), LO(CMD_SET_GFSK_SYNC_WORD), sync_word[0], sync_word[1], sync_word[2],
                    sync_word[3], 0x00, 0x00, 0x00, 0x00});
  this->listen();
}

void LR1121Radio::restart_rx_() {
  this->write_cmd_({HI(CMD_AUTO_TX_RX), LO(CMD_AUTO_TX_RX), 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00});
  this->write_cmd_({HI(CMD_CLEAR_IRQ), LO(CMD_CLEAR_IRQ), 0xFF, 0xFF, 0xFF, 0xFF});
  this->write_cmd_({HI(CMD_CLEAR_RX_BUFFER), LO(CMD_CLEAR_RX_BUFFER)});
  this->write_cmd_({HI(CMD_SET_RX), LO(CMD_SET_RX), 0x00, 0x00, 0x00});
}

void LR1121Radio::listen() {
  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), 0x00});
  if (this->pkt_len_ != SNIFF_LEN)
    this->set_packet_length_(SNIFF_LEN);
  this->restart_rx_();
}

size_t LR1121Radio::read_raw(uint8_t *buf, size_t max_len) {
  if (this->failed_ || !this->irq_pin_->digital_read())
    return 0;

  uint8_t status[6] = {};
  if (!this->read_cmd_({HI(CMD_GET_STATUS), LO(CMD_GET_STATUS)}, status, sizeof(status)))
    return 0;
  const uint32_t irq = (uint32_t(status[2]) << 24) | (uint32_t(status[3]) << 16) | (uint32_t(status[4]) << 8) |
                       uint32_t(status[5]);
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
  this->write_cmd_({HI(CMD_SET_STANDBY), LO(CMD_SET_STANDBY), 0x00});
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

  uint8_t buf[SNIFF_LEN];
  const size_t len = this->read_raw(buf, sizeof(buf));  // Also restarts sniffing
  if (len < PACKET_LEN) {
    this->listen();
    return false;
  }
  Packet p = parse_packet(address, buf, len);
  if (!p.crc_valid || p.pkt_len != PAYLOAD_LEN)
    return false;
  *reply = p;
  return true;
}

}  // namespace halo2
}  // namespace esphome
