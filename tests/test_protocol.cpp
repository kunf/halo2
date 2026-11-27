// Host unit tests for esphome/components/halo2/protocol.cpp.
// Build and run from the repository root:
//   g++ -std=c++17 -Wall -Wextra -Iesphome/components/halo2 tests/test_protocol.cpp
//       esphome/components/halo2/protocol.cpp -o test_protocol && ./test_protocol
#include <cstdio>
#include <cstring>

#include "protocol.h"
#include "vectors.h"

using namespace esphome::halo2;

static int failures = 0;
#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                      \
    }                                                                  \
  } while (0)

static Address vec_address() {
  Address a;
  std::memcpy(a.data(), VEC_ADDRESS, 4);
  return a;
}

static void test_compile_matches_micropython() {
  const Address addr = vec_address();
  for (const auto &v : COMPILE_VECTORS) {
    Payload payload;
    std::memcpy(payload.data(), v.payload, 10);
    RawPacket pkt = compile_packet(addr, v.pid, payload);
    CHECK(std::memcmp(pkt.data(), v.packet, 14) == 0);
    CHECK(packet_crc(addr, make_pcf(v.pid), payload.data()) == v.crc);

    Packet p = parse_packet(addr, v.packet, 14);
    CHECK(p.crc_valid);
    CHECK(p.pkt_len == 10);
    CHECK(p.pid == v.pid);
    CHECK(!p.no_ack);
    CHECK(std::memcmp(p.payload.data(), v.payload, 10) == 0);
  }
}

static void test_corrupt_packet_rejected() {
  const Address addr = vec_address();
  uint8_t pkt[14];
  std::memcpy(pkt, COMPILE_VECTORS[0].packet, 14);
  pkt[5] ^= 0x10;
  CHECK(!parse_packet(addr, pkt, 14).crc_valid);
  Address other = addr;
  other[0] ^= 1;
  CHECK(!parse_packet(other, COMPILE_VECTORS[0].packet, 14).crc_valid);
}

// Independent captures from github.com/Termina1/benq-screenbar-halo2-esphome. That project
// reads the frame one bit late, so its "address 86 BB EA 9C" is 43 5D F5 4E here and its
// PCF bytes are our PCF bytes. Its CRCs must match ours.
static void test_third_party_captures() {
  const Address addr{0x43, 0x5D, 0xF5, 0x4E};
  const Payload stock{0x04, 0x10, 0x0C, 0x0F, 0x55, 0x5B, 0x0F, 0x55, 0x01, 0x02};
  const Payload on{0x02, 0x11, 0x0C, 0x0F, 0x55, 0x5B, 0x0F, 0x55, 0x01, 0x02};
  const Payload off{0x02, 0x10, 0x0C, 0x0F, 0x55, 0x5B, 0x0F, 0x55, 0x01, 0x02};
  CHECK(packet_crc(addr, 0x54, stock.data()) == 0x20B9);
  CHECK(packet_crc(addr, 0x50, on.data()) == 0xE962);
  CHECK(packet_crc(addr, 0x50, off.data()) == 0x0241);
}

static void test_second_packet_any_alignment() {
  const Address addr = vec_address();
  Payload request{0x04, 0x10, 0x0C, 0x0F, 0x55, 0x5B, 0x0F, 0x55, 0x01, 0x02};
  Payload reply{0x04, 0x31, 0x50, 0x10, 0x68, 0x20, 0x10, 0x68, 0x01, 0x02};
  RawPacket first = compile_packet(addr, 2, request);
  RawPacket second = compile_packet(addr, 3, reply);
  for (size_t gap = 0; gap < 24; gap++) {
    uint8_t buf[SNIFF_LEN] = {};
    size_t bit = 0;
    auto put_bits = [&](const uint8_t *data, size_t nbits) {
      for (size_t i = 0; i < nbits; i++, bit++)
        if ((data[i >> 3] >> (7 - (i & 7))) & 1)
          buf[bit >> 3] |= static_cast<uint8_t>(0x80 >> (bit & 7));
    };
    const uint8_t pre = 0xAA;
    put_bits(first.data(), PACKET_BITS);
    bit += gap;
    put_bits(&pre, 8);
    put_bits(addr.data(), 32);
    put_bits(second.data(), PACKET_BITS);
    if (bit > SNIFF_LEN * 8)
      break;
    CHECK(parse_packet(addr, buf, sizeof(buf)).crc_valid);
    Packet p;
    CHECK(find_second_packet(addr, buf, sizeof(buf), &p));
    CHECK(p.pid == 3);
    CHECK(p.payload == reply);
  }
}

static void test_extract_address() {
  Address found{};
  CHECK(extract_address(DISCOVERY_CAPTURE, 36, &found));
  CHECK(found == vec_address());
  uint8_t zeros[36] = {};
  CHECK(!extract_address(zeros, 36, &found));
}

static void test_payload_roundtrip() {
  LampState s;
  s.power = true;
  s.ultrasonic = false;
  s.front = false;
  s.back = true;
  s.front_brightness = 42;
  s.back_brightness = 7;
  s.color_temp = 5100;
  Payload p = build_payload(CMD_SET, s);
  CHECK(p[0] == CMD_SET);
  CHECK(p[1] == 0x09);
  CHECK(validate_payload(p));
  CHECK(parse_payload(p) == s);

  s.front = true;  // both
  p = build_payload(CMD_SET, s, true);
  CHECK(p[1] == (0x10 | 0x02 | 0x01));
  LampState parsed = parse_payload(p);
  CHECK(parsed.front && parsed.back);

  s.back = false;  // front only
  CHECK(build_payload(CMD_SET, s)[1] == 0x01);

  p[3] = 0x05;  // colour temperature out of range
  CHECK(!validate_payload(p));
}

static void test_to_hex() {
  const uint8_t data[] = {0x00, 0xAB, 0x7F};
  char out[9];
  to_hex(data, 3, out, sizeof(out));
  CHECK(std::strcmp(out, "00 AB 7F") == 0);
  char small[4];
  to_hex(data, 3, small, sizeof(small));
  CHECK(std::strlen(small) < sizeof(small));
}

int main() {
  test_compile_matches_micropython();
  test_corrupt_packet_rejected();
  test_third_party_captures();
  test_second_packet_any_alignment();
  test_extract_address();
  test_payload_roundtrip();
  test_to_hex();
  if (failures) {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("All protocol tests passed\n");
  return 0;
}
