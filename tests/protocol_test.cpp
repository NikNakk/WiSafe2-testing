#include <cstdint>
#include <cstdio>
#include <cstring>

#include "protocol.h"

using esphome::wisafe2::DecodedPacket;
using esphome::wisafe2::decode_packet;

namespace {

int failures = 0;
int checks = 0;

#define CHECK(condition)                                                                                              \
  do {                                                                                                                \
    ++checks;                                                                                                         \
    if (!(condition)) {                                                                                               \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                                      \
      ++failures;                                                                                                     \
    }                                                                                                                 \
  } while (false)

#define CHECK_STRING(actual, expected) CHECK(std::strcmp((actual), (expected)) == 0)

template<size_t N> DecodedPacket decode(const uint8_t (&packet)[N]) {
  DecodedPacket decoded{};
  CHECK(decode_packet(packet, N, &decoded));
  CHECK(decoded.recognized);
  return decoded;
}

template<size_t N> void check_rejected(const uint8_t (&packet)[N]) {
  DecodedPacket decoded{};
  CHECK(!decode_packet(packet, N, &decoded));
}

void test_observed_status_packets() {
  const uint8_t off_base[] = {0x71, 0x68, 0x96, 0x1A, 0x11, 0x04, 0x01, 0x07, 0x05, 0x7E};
  DecodedPacket decoded = decode(off_base);
  CHECK(decoded.device_id == 0x68961A);
  CHECK(decoded.model_id == 0x1104);
  CHECK_STRING(decoded.device, "68961A");
  CHECK_STRING(decoded.model, "1104");
  CHECK_STRING(decoded.event, "STATUS");
  CHECK(decoded.has_model);
  CHECK(decoded.has_base);
  CHECK(decoded.has_battery);
  CHECK(!decoded.alarm);
  CHECK(decoded.base_problem);
  CHECK(!decoded.battery_low);
  CHECK_STRING(decoded.base, "OFF");
  CHECK_STRING(decoded.battery, "OK");

  const uint8_t on_base[] = {0x71, 0x68, 0x96, 0x1A, 0x11, 0x04, 0x05, 0x07, 0x06, 0x7E};
  decoded = decode(on_base);
  CHECK(!decoded.base_problem);
  CHECK(!decoded.battery_low);
  CHECK_STRING(decoded.base, "ON");
}

void test_status_battery_bits() {
  const uint8_t low_bit_1[] = {0x71, 0x01, 0x02, 0x03, 0xED, 0x08, 0x46, 0x7E};
  DecodedPacket decoded = decode(low_bit_1);
  CHECK(!decoded.base_problem);
  CHECK(decoded.battery_low);
  CHECK_STRING(decoded.battery, "LOW");

  const uint8_t low_bit_2[] = {0x71, 0x01, 0x02, 0x03, 0xED, 0x08, 0x02, 0x7E};
  decoded = decode(low_bit_2);
  CHECK(decoded.base_problem);
  CHECK(decoded.battery_low);
}

void test_fire_test_pass() {
  const uint8_t packet[] = {0x70, 0x12, 0x34, 0x56, 0x81, 0x01, 0xED, 0x08, 0x7E};
  DecodedPacket decoded = decode(packet);
  CHECK(decoded.device_id == 0x123456);
  CHECK(decoded.model_id == 0xED08);
  CHECK_STRING(decoded.event, "FIRE TEST");
  CHECK_STRING(decoded.result, "PASS");
  CHECK_STRING(decoded.base, "ON");
  CHECK_STRING(decoded.battery, "OK");
  CHECK(decoded.has_result);
  CHECK(decoded.has_battery);
  CHECK(!decoded.alarm);
}

void test_co_test_failure() {
  const uint8_t packet[] = {0x70, 0xAB, 0xCD, 0xEF, 0x41, 0x00, 0x78, 0x03, 0x7E};
  DecodedPacket decoded = decode(packet);
  CHECK_STRING(decoded.event, "CARBON MONOXIDE TEST");
  CHECK_STRING(decoded.result, "FAIL");
  CHECK(!decoded.has_battery);
  CHECK_STRING(decoded.battery, "UNKNOWN");
}

void test_emergencies() {
  const uint8_t co[] = {0x50, 0x10, 0x20, 0x30, 0x41, 0x00, 0x7E};
  DecodedPacket decoded = decode(co);
  CHECK(decoded.alarm);
  CHECK(decoded.has_base);
  CHECK(!decoded.base_problem);
  CHECK_STRING(decoded.event, "CARBON MONOXIDE EMERGENCY");

  const uint8_t fire_82[] = {0x50, 0x10, 0x20, 0x30, 0x82, 0x00, 0x7E};
  decoded = decode(fire_82);
  CHECK(decoded.alarm);
  CHECK_STRING(decoded.event, "FIRE EMERGENCY");

  const uint8_t unknown[] = {0x50, 0x10, 0x20, 0x30, 0x99, 0x00, 0x7E};
  decoded = decode(unknown);
  CHECK(decoded.alarm);
  CHECK_STRING(decoded.event, "UNKNOWN EMERGENCY 0x99");
}

void test_silence_and_missing() {
  const uint8_t silence[] = {0x61, 0xAA, 0xBB, 0xCC, 0x81, 0x00, 0x7E};
  DecodedPacket decoded = decode(silence);
  CHECK(decoded.device_id == 0xAABBCC);
  CHECK(!decoded.alarm);
  CHECK_STRING(decoded.event, "SILENCE");
  CHECK_STRING(decoded.base, "ON");

  const uint8_t missing[] = {0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0xAD, 0x01, 0x7E};
  decoded = decode(missing);
  CHECK(decoded.device_id == 0xDEAD01);
  CHECK(!decoded.alarm);
  CHECK(decoded.base_problem);
  CHECK(decoded.battery_low);
  CHECK_STRING(decoded.event, "MISSING");
  CHECK_STRING(decoded.base, "MISSING");
  CHECK_STRING(decoded.battery, "MISSING");
}

void test_malformed_frames() {
  const uint8_t terminator_only[] = {0x7E};
  check_rejected(terminator_only);

  const uint8_t unknown_type[] = {0x99, 0x00, 0x7E};
  check_rejected(unknown_type);

  const uint8_t status_too_short[] = {0x71, 0x01, 0x02, 0x03, 0x11, 0x04, 0x7E};
  check_rejected(status_too_short);

  const uint8_t test_too_short[] = {0x70, 0x01, 0x02, 0x03, 0x81, 0x01, 0x11, 0x7E};
  check_rejected(test_too_short);

  const uint8_t emergency_too_short[] = {0x50, 0x01, 0x02, 0x03, 0x81, 0x7E};
  check_rejected(emergency_too_short);

  const uint8_t silence_too_short[] = {0x61, 0x01, 0x02, 0x03, 0x81, 0x7E};
  check_rejected(silence_too_short);

  const uint8_t missing_too_short[] = {0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0xAD, 0x7E};
  check_rejected(missing_too_short);

  const uint8_t no_terminator[] = {0x71, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x00};
  check_rejected(no_terminator);

  DecodedPacket decoded{};
  CHECK(!decode_packet(nullptr, 8, &decoded));
  const uint8_t valid[] = {0x71, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x7E};
  CHECK(!decode_packet(valid, sizeof(valid), nullptr));
}

}  // namespace

int main() {
  test_observed_status_packets();
  test_status_battery_bits();
  test_fire_test_pass();
  test_co_test_failure();
  test_emergencies();
  test_silence_and_missing();
  test_malformed_frames();

  if (failures != 0) {
    std::fprintf(stderr, "%d of %d checks failed\n", failures, checks);
    return 1;
  }
  std::printf("All %d WiSafe2 protocol checks passed\n", checks);
  return 0;
}
