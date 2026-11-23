#include "protocol.h"

#include <cstring>

namespace esphome {
namespace halo2 {

namespace {

inline uint8_t get_bit(const uint8_t *data, size_t bit) { return (data[bit >> 3] >> (7 - (bit & 7))) & 1; }

inline void set_bit(uint8_t *data, size_t bit, uint8_t value) {
  if (value) {
    data[bit >> 3] |= static_cast<uint8_t>(0x80 >> (bit & 7));
  } else {
    data[bit >> 3] &= static_cast<uint8_t>(~(0x80 >> (bit & 7)));
  }
}

inline uint16_t crc_bit(uint16_t crc, uint8_t bit) {
  const bool feedback = ((crc >> 15) & 1) ^ (bit & 1);
  crc = static_cast<uint16_t>(crc << 1);
  return feedback ? static_cast<uint16_t>(crc ^ 0x1021) : crc;
}

inline uint16_t crc_byte(uint16_t crc, uint8_t byte) {
  for (int i = 7; i >= 0; --i)
    crc = crc_bit(crc, (byte >> i) & 1);
  return crc;
}

uint16_t address_crc(const Address &address) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : address)
    crc = crc_byte(crc, b);
  return crc;
}

}  // namespace

uint16_t packet_crc(const Address &address, uint8_t pcf, const uint8_t *payload) {
  uint16_t crc = address_crc(address);
  crc = crc_bit(crc, 0);
  crc = crc_byte(crc, pcf);
  for (size_t i = 0; i < PAYLOAD_LEN; i++)
    crc = crc_byte(crc, payload[i]);
  return crc;
}

RawPacket compile_packet(const Address &address, uint8_t pid, const Payload &payload) {
  RawPacket out{};
  const uint8_t pcf = make_pcf(pid);
  const uint16_t crc = packet_crc(address, pcf, payload.data());
  size_t bit = 1;  // Bit 0 is the PCF's top length bit, always 0 for a 10-byte payload
  auto put = [&](uint8_t byte) {
    for (int i = 7; i >= 0; --i)
      set_bit(out.data(), bit++, (byte >> i) & 1);
  };
  put(pcf);
  for (uint8_t b : payload)
    put(b);
  put(static_cast<uint8_t>(crc >> 8));
  put(static_cast<uint8_t>(crc & 0xFF));
  return out;
}

Packet parse_packet(const Address &address, const uint8_t *raw, size_t len, size_t bit_offset) {
  Packet p;
  if (raw == nullptr || bit_offset + PACKET_BITS > len * 8)
    return p;
  auto byte_at = [&](size_t bit) {
    uint8_t v = 0;
    for (size_t i = 0; i < 8; i++)
      v = static_cast<uint8_t>((v << 1) | get_bit(raw, bit + i));
    return v;
  };
  const uint8_t len_msb = get_bit(raw, bit_offset);
  const uint8_t pcf = byte_at(bit_offset + 1);
  p.pkt_len = static_cast<uint8_t>((len_msb << 5) | (pcf >> 3));
  p.pid = (pcf >> 1) & 0x03;
  p.no_ack = pcf & 0x01;
  for (size_t i = 0; i < PAYLOAD_LEN; i++)
    p.payload[i] = byte_at(bit_offset + 9 + i * 8);
  p.crc = static_cast<uint16_t>((byte_at(bit_offset + 89) << 8) | byte_at(bit_offset + 97));

  uint16_t crc = address_crc(address);
  for (size_t i = 0; i < 89; i++)
    crc = crc_bit(crc, get_bit(raw, bit_offset + i));
  p.crc_valid = (crc == p.crc);
  return p;
}

bool find_second_packet(const Address &address, const uint8_t *raw, size_t len, Packet *out) {
  const size_t total_bits = len * 8;
  if (total_bits < PACKET_BITS + 32 + PACKET_BITS)
    return false;
  uint32_t want = 0;
  for (uint8_t b : address)
    want = (want << 8) | b;
  uint32_t window = 0;
  for (size_t bit = PACKET_BITS; bit + PACKET_BITS <= total_bits; bit++) {
    window = (window << 1) | get_bit(raw, bit);
    if (bit < PACKET_BITS + 31 || window != want)
      continue;
    Packet p = parse_packet(address, raw, len, bit + 1);
    if (p.crc_valid && p.pkt_len == PAYLOAD_LEN) {
      *out = p;
      return true;
    }
  }
  return false;
}

bool extract_address(const uint8_t *raw, size_t len, Address *out) {
  if (raw == nullptr || len < 10 || len > SNIFF_LEN)
    return false;
  bool all_zero = true;
  for (size_t i = 0; i < len; i++)
    all_zero &= (raw[i] == 0);
  if (all_zero)
    return false;

  // Shift right by one bit, as utils.extract_address() does
  uint8_t buf[SNIFF_LEN];
  uint8_t carry = 0;
  for (size_t i = 0; i < len; i++) {
    buf[i] = static_cast<uint8_t>((raw[i] >> 1) | (carry << 7));
    carry = raw[i] & 1;
  }
  // The preamble is 0xAA or 0x55 depending on the first address bit. It is followed by
  // the 4-byte address and the PCF, whose first 6 bits (payload length 10) give 0x28.
  const size_t end = (len - 5 < 10) ? len - 5 : 10;
  for (uint8_t preamble : {static_cast<uint8_t>(0xAA), static_cast<uint8_t>(0x55)}) {
    for (size_t pos = 4; pos < end; pos++) {
      if (buf[pos] == preamble && (buf[pos + 5] & 0xFC) == 0x28) {
        std::memcpy(out->data(), &buf[pos + 1], 4);
        return true;
      }
    }
  }
  return false;
}

Payload build_payload(uint8_t command, const LampState &state, bool auto_mode) {
  uint8_t lamp_mode = 0;  // 0 = front, 1 = back, 2 = both
  if (state.back)
    lamp_mode = state.front ? 2 : 1;
  uint8_t control =
      static_cast<uint8_t>((state.ultrasonic ? 0x20 : 0) | (lamp_mode << 3) | (state.power ? 0x01 : 0));
  if (auto_mode)
    control |= 0x02;
  const uint8_t t1 = static_cast<uint8_t>(state.color_temp >> 8);
  const uint8_t t2 = static_cast<uint8_t>(state.color_temp & 0xFF);
  return Payload{command, control, state.front_brightness, t1, t2, state.back_brightness, t1, t2, 0x01, 0x02};
}

bool validate_payload(const Payload &p) {
  if (p[0] > 5)
    return false;
  if (p[2] > BRIGHTNESS_MAX || p[5] > BRIGHTNESS_MAX)
    return false;
  if (p[8] != 0x01 || p[9] != 0x02)
    return false;
  const uint16_t t1 = static_cast<uint16_t>((p[3] << 8) | p[4]);
  const uint16_t t2 = static_cast<uint16_t>((p[6] << 8) | p[7]);
  return t1 >= COLOR_TEMP_MIN && t1 <= COLOR_TEMP_MAX && t2 >= COLOR_TEMP_MIN && t2 <= COLOR_TEMP_MAX;
}

LampState parse_payload(const Payload &p) {
  LampState s;
  const uint8_t control = p[1];
  s.power = control & 0x01;
  s.ultrasonic = (control >> 5) & 0x01;
  const uint8_t lamp_mode = (control >> 3) & 0x03;
  s.front = (lamp_mode == 0) || (lamp_mode == 2);
  s.back = (lamp_mode == 1) || (lamp_mode == 2);
  s.front_brightness = p[2];
  s.back_brightness = p[5];
  s.color_temp = static_cast<uint16_t>((p[3] << 8) | p[4]);
  return s;
}

}  // namespace halo2
}  // namespace esphome
