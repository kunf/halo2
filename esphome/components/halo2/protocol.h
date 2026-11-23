#pragma once

// BenQ ScreenBar HALO 2 radio protocol: packet framing, CRC and payload encoding.
// Plain C++17 with no ESPHome dependencies, so it can be unit tested on the host
// (see tests/test_protocol.cpp). Ported from src/benq_halo/utils.py and
// src/benq_halo/__init__.py.
//
// On-air packet (Enhanced ShockBurst style, 125 kbps GFSK, MSB first):
//   preamble | 4-byte address | 9-bit PCF | 10-byte payload | CRC-CCITT
//   9-bit PCF = 6-bit payload length (10) | 2-bit PID | 1-bit NO_ACK
//   CRC: poly 0x1021, init 0xFFFF, over address + PCF (9 bits) + payload.

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace halo2 {

// Lamp address in on-air byte order: this is the LR1121 sync word, and the
// reverse of HALO2_ADDRESS in the MicroPython project's halo2_address.py.
using Address = std::array<uint8_t, 4>;
using Payload = std::array<uint8_t, 10>;
using RawPacket = std::array<uint8_t, 14>;

static constexpr size_t PAYLOAD_LEN = 10;
static constexpr size_t PACKET_LEN = 14;          // 9-bit PCF + payload + CRC = 105 bits, padded
static constexpr size_t PACKET_BITS = 9 + 80 + 16;
static constexpr size_t SNIFF_LEN = 36;           // Room for a remote request and the lamp reply

// Sync word used while discovering the address: back lamp brightness 10% (0x0A)
// followed by colour temperature 3925 K (0x0F55) and the fixed 0x01 trailer byte,
// i.e. the remote's payload when set to 10% / 3925 K.
static constexpr Address DISCOVERY_SYNC_WORD{0x0A, 0x0F, 0x55, 0x01};

// Payload commands
static constexpr uint8_t CMD_WAKE = 0x00;
static constexpr uint8_t CMD_POWER = 0x02;
static constexpr uint8_t CMD_SET = 0x03;
static constexpr uint8_t CMD_SYNC = 0x04;

static constexpr uint8_t BRIGHTNESS_MIN = 1;
static constexpr uint8_t BRIGHTNESS_MAX = 100;
static constexpr uint16_t COLOR_TEMP_MIN = 2700;
static constexpr uint16_t COLOR_TEMP_MAX = 6500;

struct Packet {
  bool crc_valid{false};
  uint8_t pkt_len{0};
  uint8_t pid{0};
  bool no_ack{false};
  Payload payload{};
  uint16_t crc{0};
};

struct LampState {
  bool power{true};
  bool ultrasonic{true};
  bool front{true};
  bool back{false};
  uint8_t front_brightness{1};
  uint8_t back_brightness{1};
  uint16_t color_temp{4000};

  bool operator==(const LampState &o) const {
    return power == o.power && ultrasonic == o.ultrasonic && front == o.front && back == o.back &&
           front_brightness == o.front_brightness && back_brightness == o.back_brightness &&
           color_temp == o.color_temp;
  }
  bool operator!=(const LampState &o) const { return !(*this == o); }
};

/// CRC of a packet: address, a 0 bit (the PCF's top length bit), PCF byte, payload.
uint16_t packet_crc(const Address &address, uint8_t pcf, const uint8_t *payload);

/// PCF byte (the low 8 of the 9 PCF bits) for a 10-byte payload and the given PID.
inline uint8_t make_pcf(uint8_t pid, bool no_ack = false) {
  return static_cast<uint8_t>(0x50 | ((pid & 0x03) << 1) | (no_ack ? 1 : 0));
}

/// Bytes to transmit after the sync word (address), as compile_packet() in utils.py.
RawPacket compile_packet(const Address &address, uint8_t pid, const Payload &payload);

/// Decode one packet starting at bit `bit_offset` of `raw` (needs PACKET_BITS bits).
Packet parse_packet(const Address &address, const uint8_t *raw, size_t len, size_t bit_offset = 0);

/// Find the packet that follows the first one in a SNIFF_LEN capture: searches for the
/// address at any bit position after the first packet. Returns true for a CRC-valid packet.
bool find_second_packet(const Address &address, const uint8_t *raw, size_t len, Packet *out);

/// Address discovery (see find_halo2_address.py): with DISCOVERY_SYNC_WORD as sync word,
/// a capture contains the next packet's preamble, address and PCF.
bool extract_address(const uint8_t *raw, size_t len, Address *out);

/// Payload bytes for a command, from the lamp state.
Payload build_payload(uint8_t command, const LampState &state, bool auto_mode = false);

/// Sanity checks on a received payload (validate_packet() in benq_halo/__init__.py).
bool validate_payload(const Payload &payload);

/// Lamp state carried by a payload (parse_lamp_status()).
LampState parse_payload(const Payload &payload);

}  // namespace halo2
}  // namespace esphome
