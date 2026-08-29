#include <cstdint>
#include <cstdio>
#include <cstring>

#include "protocol.h"

using esphome::wisafe2::DecodedPacket;
using esphome::wisafe2::CommandFrames;
using esphome::wisafe2::DetectorType;
using esphome::wisafe2::ManagementCommand;
using esphome::wisafe2::RadioDiagnostic;
using esphome::wisafe2::decode_packet;
using esphome::wisafe2::decode_radio_diagnostic;
using esphome::wisafe2::decode_sid_map;
using esphome::wisafe2::detector_model_name;
using esphome::wisafe2::detector_type_for_model;
using esphome::wisafe2::encode_identity_response;
using esphome::wisafe2::encode_management_command;
using esphome::wisafe2::is_identity_request;
using esphome::wisafe2::management_command_name;

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

template<size_t N> void check_frame(const uint8_t *actual, size_t actual_length, const uint8_t (&expected)[N]) {
  CHECK(actual_length == N);
  CHECK(std::memcmp(actual, expected, N) == 0);
}

void test_management_command_frames() {
  constexpr uint32_t device = 0xA5B813;
  constexpr uint16_t model = 0x1103;
  CommandFrames frames{};

  CHECK(encode_management_command(ManagementCommand::SOUND_CO, device, model, &frames));
  const uint8_t co_announce[] = {0x70, 0xA5, 0xB8, 0x13, 0x41, 0x01, 0x11, 0x03, 0x7E};
  const uint8_t co_transmit[] = {0x91, 0xA5, 0xB8, 0x13, 0x11, 0x03, 0x41, 0x05, 0x00, 0x02, 0x7E};
  check_frame(frames.primary, frames.primary_length, co_announce);
  check_frame(frames.secondary, frames.secondary_length, co_transmit);

  CHECK(encode_management_command(ManagementCommand::SOUND_FIRE, device, model, &frames));
  const uint8_t fire_announce[] = {0x70, 0xA5, 0xB8, 0x13, 0x81, 0x01, 0x11, 0x03, 0x7E};
  const uint8_t fire_transmit[] = {0x91, 0xA5, 0xB8, 0x13, 0x11, 0x03, 0x81, 0x05, 0x00, 0x02, 0x7E};
  check_frame(frames.primary, frames.primary_length, fire_announce);
  check_frame(frames.secondary, frames.secondary_length, fire_transmit);

  CHECK(encode_management_command(ManagementCommand::SOUND_COMBINED, device, model, &frames));
  const uint8_t combined_announce[] = {0x70, 0xA5, 0xB8, 0x13, 0xFF, 0x01, 0x11, 0x03, 0x7E};
  const uint8_t combined_transmit[] = {0x91, 0xA5, 0xB8, 0x13, 0x11, 0x03, 0xFF, 0x05, 0x00, 0x02, 0x7E};
  check_frame(frames.primary, frames.primary_length, combined_announce);
  check_frame(frames.secondary, frames.secondary_length, combined_transmit);

  CHECK(encode_management_command(ManagementCommand::SILENCE_CO, device, model, &frames));
  const uint8_t silence_co[] = {0x61, 0xA5, 0xB8, 0x13, 0x40, 0x01, 0x7E};
  check_frame(frames.primary, frames.primary_length, silence_co);
  CHECK(frames.secondary_length == 0);

  CHECK(encode_management_command(ManagementCommand::SILENCE_FIRE, device, model, &frames));
  const uint8_t silence_fire[] = {0x61, 0xA5, 0xB8, 0x13, 0x80, 0x01, 0x7E};
  check_frame(frames.primary, frames.primary_length, silence_fire);

  CHECK(encode_management_command(ManagementCommand::QUERY_PAIRING, device, model, &frames));
  const uint8_t query_pairing[] = {0xD3, 0x03, 0x7E};
  check_frame(frames.primary, frames.primary_length, query_pairing);

  CHECK(encode_management_command(ManagementCommand::START_PAIRING, device, model, &frames));
  const uint8_t start_pairing[] = {0xD3, 0x12, 0x01, 0x7E};
  const uint8_t pairing_identity[] = {0x91, 0xA5, 0xB8, 0x13, 0x11, 0x03, 0xFF, 0x05, 0x01, 0x01, 0x7E};
  check_frame(frames.primary, frames.primary_length, start_pairing);
  check_frame(frames.secondary, frames.secondary_length, pairing_identity);

  CHECK(!encode_management_command(ManagementCommand::SOUND_FIRE, 0x1000000, model, &frames));
  CHECK(!encode_management_command(ManagementCommand::SOUND_FIRE, device, model, nullptr));
  CHECK_STRING(management_command_name(ManagementCommand::SOUND_FIRE), "Sound fire test");
}

void test_radio_identity_frames() {
  const uint8_t request[] = {0x41, 0x7E};
  CHECK(is_identity_request(request, sizeof(request)));
  const uint8_t extended_request[] = {0x41, 0x00, 0x7E};
  CHECK(!is_identity_request(extended_request, sizeof(extended_request)));
  CHECK(!is_identity_request(nullptr, 0));

  uint8_t response[11]{};
  size_t length = 0;
  CHECK(encode_identity_response(0xA5B813, 0x1103, response, sizeof(response), &length));
  const uint8_t smoke_response[] = {0x91, 0xA5, 0xB8, 0x13, 0x11, 0x03,
                                    0x81, 0x05, 0x01, 0x01, 0x7E};
  check_frame(response, length, smoke_response);

  CHECK(encode_identity_response(0xA5B813, 0x7803, response, sizeof(response), &length));
  const uint8_t co_response[] = {0x91, 0xA5, 0xB8, 0x13, 0x78, 0x03,
                                 0x41, 0x05, 0x01, 0x01, 0x7E};
  check_frame(response, length, co_response);
  CHECK(!encode_identity_response(0x1000000, 0x1103, response, sizeof(response), &length));
  CHECK(!encode_identity_response(0xA5B813, 0x1103, response, sizeof(response) - 1, &length));

  CHECK(encode_identity_response(0xA5B813, 0xFFFF, response, sizeof(response), &length));
  CHECK(response[6] == 0xFF);
}

void test_sid_map() {
  const uint8_t response[] = {0xD4, 0x03, 0x88, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x7E};
  uint64_t sid_map = 0;
  CHECK(decode_sid_map(response, sizeof(response), &sid_map));
  CHECK(sid_map == 0x88);

  const uint8_t high_sid[] = {0xD4, 0x03, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x80, 0x7E};
  CHECK(decode_sid_map(high_sid, sizeof(high_sid), &sid_map));
  CHECK(sid_map == 0x8000000000000000ULL);

  const uint8_t unknown_subtype[] = {0xD4, 0x10, 0x88, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x7E};
  CHECK(!decode_sid_map(unknown_subtype, sizeof(unknown_subtype), &sid_map));
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

void test_silence() {
  const uint8_t silence[] = {0x61, 0xAA, 0xBB, 0xCC, 0x81, 0x00, 0x7E};
  DecodedPacket decoded = decode(silence);
  CHECK(decoded.device_id == 0xAABBCC);
  CHECK(!decoded.alarm);
  CHECK_STRING(decoded.event, "SILENCE");
  CHECK_STRING(decoded.base, "ON");
}

void test_radio_diagnostic() {
  const uint8_t packet[] = {0xD2, 0x2A, 0x38, 0x41, 0x00, 0xEF, 0xA5,
                            0xB8, 0x13, 0x00, 0x00, 0x09, 0x40, 0x7E};
  DecodedPacket decoded{};
  CHECK(!decode_packet(packet, sizeof(packet), &decoded));

  RadioDiagnostic diagnostic{};
  CHECK(decode_radio_diagnostic(packet, sizeof(packet), &diagnostic));
  CHECK(diagnostic.battery_primary == 0x2A);
  CHECK(diagnostic.battery_radio == 0x38);
  CHECK(diagnostic.unknown_1 == 0x41);
  CHECK(diagnostic.rssi == 0x00);
  CHECK(diagnostic.firmware_version == 0xEF);
  CHECK(diagnostic.device_id == 0xA5B813);
  CHECK(diagnostic.flags == 0x00);
  CHECK(diagnostic.radio_fault_count == 0x00);
  CHECK(diagnostic.sid == 0x09);
  CHECK(diagnostic.unknown_2 == 0x40);
  CHECK(diagnostic.connected);

  const uint8_t disconnected[] = {0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5,
                                  0xB8, 0x13, 0x00, 0x00, 0x40, 0x00, 0x7E};
  CHECK(decode_radio_diagnostic(disconnected, sizeof(disconnected), &diagnostic));
  CHECK(!diagnostic.connected);

  const uint8_t short_packet[] = {0xD2, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0xA5, 0xB8, 0x13, 0x7E};
  CHECK(!decode_radio_diagnostic(short_packet, sizeof(short_packet), &diagnostic));
}

void test_model_catalogue() {
  CHECK_STRING(detector_model_name(0xED08), "FP2620W2");
  CHECK_STRING(detector_model_name(0x1104), "FP1720W2");
  CHECK_STRING(detector_model_name(0x1103), "WST-630");
  CHECK_STRING(detector_model_name(0x340E), "WST-630N");
  CHECK_STRING(detector_model_name(0x7803), "W2-CO-10X");
  CHECK_STRING(detector_model_name(0xC304), "W2-SVP-630");
  CHECK_STRING(detector_model_name(0xFFFF), "Unknown FireAngel alarm");
  CHECK_STRING(detector_model_name(0xED08, false), "Unknown FireAngel alarm");

  CHECK(detector_type_for_model(0xED08) == DetectorType::SMOKE);
  CHECK(detector_type_for_model(0x1103) == DetectorType::SMOKE);
  CHECK(detector_type_for_model(0x340E) == DetectorType::SMOKE);
  CHECK(detector_type_for_model(0x1104) == DetectorType::HEAT);
  CHECK(detector_type_for_model(0x7803) == DetectorType::CARBON_MONOXIDE);
  CHECK(detector_type_for_model(0xC304) == DetectorType::UNKNOWN);
  CHECK(detector_type_for_model(0xFFFF) == DetectorType::UNKNOWN);
  CHECK(detector_type_for_model(0x7803, false) == DetectorType::UNKNOWN);
}

void test_extended_frames() {
  // Known message types use minimum lengths. Preserve forward compatibility by
  // decoding established offsets while ignoring trailing extension bytes.
  const uint8_t fire_test[] = {0x70, 0x12, 0x34, 0x56, 0x82, 0x01, 0xED, 0x08, 0xAA, 0xBB, 0x7E};
  DecodedPacket decoded = decode(fire_test);
  CHECK_STRING(decoded.event, "FIRE TEST");
  CHECK_STRING(decoded.result, "PASS");
  CHECK(decoded.model_id == 0xED08);

  const uint8_t emergency[] = {0x50, 0x12, 0x34, 0x56, 0x41, 0x00, 0xAA, 0x7E};
  decoded = decode(emergency);
  CHECK_STRING(decoded.event, "CARBON MONOXIDE EMERGENCY");
  CHECK(decoded.alarm);

  const uint8_t silence[] = {0x61, 0x12, 0x34, 0x56, 0x80, 0x01, 0xAA, 0x7E};
  decoded = decode(silence);
  CHECK_STRING(decoded.event, "SILENCE");
  CHECK(!decoded.alarm);

  const uint8_t extended_diagnostic[] = {0xD2, 0x2A, 0x38, 0x41, 0x00, 0xEF, 0x92, 0xBF,
                                         0x1A, 0x00, 0x00, 0x09, 0x40, 0xAA, 0x7E};
  RadioDiagnostic diagnostic{};
  CHECK(decode_radio_diagnostic(extended_diagnostic, sizeof(extended_diagnostic), &diagnostic));
  CHECK(diagnostic.device_id == 0x92BF1A);
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

  const uint8_t diagnostic_not_detector[] = {0xD2, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0xDE, 0xAD, 0x01, 0x00, 0x00, 0x09, 0x40, 0x7E};
  check_rejected(diagnostic_not_detector);

  const uint8_t no_terminator[] = {0x71, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x00};
  check_rejected(no_terminator);

  DecodedPacket decoded{};
  CHECK(!decode_packet(nullptr, 8, &decoded));
  const uint8_t valid[] = {0x71, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x7E};
  CHECK(!decode_packet(valid, sizeof(valid), nullptr));
}

}  // namespace

int main() {
  test_management_command_frames();
  test_radio_identity_frames();
  test_sid_map();
  test_model_catalogue();
  test_extended_frames();
  test_observed_status_packets();
  test_status_battery_bits();
  test_fire_test_pass();
  test_co_test_failure();
  test_emergencies();
  test_silence();
  test_radio_diagnostic();
  test_malformed_frames();

  if (failures != 0) {
    std::fprintf(stderr, "%d of %d checks failed\n", failures, checks);
    return 1;
  }
  std::printf("All %d WiSafe2 protocol checks passed\n", checks);
  return 0;
}
