#include <cstdint>
#include <cstdio>
#include <cstring>

#include "protocol.h"

using esphome::wisafe2::DecodedPacket;
using esphome::wisafe2::CommandFrames;
using esphome::wisafe2::DetectorType;
using esphome::wisafe2::ManagementCommand;
using esphome::wisafe2::RadioDiagnostic;
using esphome::wisafe2::RemoteDiagnostic;
using esphome::wisafe2::decode_packet;
using esphome::wisafe2::decode_new_device;
using esphome::wisafe2::decode_radio_diagnostic;
using esphome::wisafe2::decode_remote_diagnostic;
using esphome::wisafe2::decode_sid_map;
using esphome::wisafe2::detector_model_name;
using esphome::wisafe2::detector_type_for_model;
using esphome::wisafe2::encode_identity_response;
using esphome::wisafe2::encode_management_command;
using esphome::wisafe2::encode_remote_diagnostic_request;
using esphome::wisafe2::escape_frame;
using esphome::wisafe2::is_identity_request;
using esphome::wisafe2::management_command_name;
using esphome::wisafe2::unescape_frame;

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

void test_frame_escaping() {
  const uint8_t plain[] = {0xD3, 0x03, 0x7E};
  uint8_t wire[32]{};
  size_t wire_length = 0;
  CHECK(escape_frame(plain, sizeof(plain), wire, sizeof(wire), &wire_length));
  check_frame(wire, wire_length, plain);

  const uint8_t logical[] = {0x71, 0x7D, 0x7E, 0x01, 0x7E};
  const uint8_t expected_wire[] = {0x71, 0x7D, 0x02, 0x7D, 0x01, 0x01, 0x7E};
  CHECK(escape_frame(logical, sizeof(logical), wire, sizeof(wire), &wire_length));
  check_frame(wire, wire_length, expected_wire);

  uint8_t round_trip[16]{};
  size_t round_trip_length = 0;
  CHECK(unescape_frame(wire, wire_length, round_trip, sizeof(round_trip), &round_trip_length));
  check_frame(round_trip, round_trip_length, logical);

  size_t ignored = 0;
  CHECK(!escape_frame(logical, sizeof(logical), wire, sizeof(expected_wire) - 1, &ignored));
  CHECK(!unescape_frame(expected_wire, sizeof(expected_wire), round_trip, sizeof(logical) - 1, &ignored));

  const uint8_t bad_code[] = {0x71, 0x7D, 0x03, 0x7E};
  CHECK(!unescape_frame(bad_code, sizeof(bad_code), round_trip, sizeof(round_trip), &ignored));
  const uint8_t dangling_escape[] = {0x71, 0x7D, 0x7E};
  CHECK(!unescape_frame(dangling_escape, sizeof(dangling_escape), round_trip, sizeof(round_trip), &ignored));
  const uint8_t embedded_terminator[] = {0x71, 0x7E, 0x00, 0x7E};
  CHECK(!unescape_frame(embedded_terminator, sizeof(embedded_terminator), round_trip, sizeof(round_trip), &ignored));
  const uint8_t no_terminator[] = {0x71, 0x00};
  CHECK(!escape_frame(no_terminator, sizeof(no_terminator), wire, sizeof(wire), &ignored));
  CHECK(!unescape_frame(no_terminator, sizeof(no_terminator), round_trip, sizeof(round_trip), &ignored));

  // Reserved bytes may occur inside a detector ID. Unescaping must restore the
  // original offsets before the packet decoder sees the frame.
  const uint8_t escaped_status[] = {0x71, 0x12, 0x7D, 0x01, 0x7D, 0x02,
                                    0x11, 0x04, 0x05, 0x7E};
  CHECK(unescape_frame(escaped_status, sizeof(escaped_status), round_trip, sizeof(round_trip),
                       &round_trip_length));
  DecodedPacket decoded{};
  CHECK(decode_packet(round_trip, round_trip_length, &decoded));
  CHECK(decoded.device_id == 0x127E7D);
  CHECK(decoded.model_id == 0x1104);
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
  CHECK_STRING(management_command_name(ManagementCommand::REFRESH_DETECTORS), "Refresh detector diagnostics");
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

void test_remote_discovery_frames() {
  uint8_t request[5]{};
  size_t request_length = 0;
  CHECK(encode_remote_diagnostic_request(9, true, request, sizeof(request), &request_length));
  const uint8_t identity_request[] = {0xD3, 0x06, 0x09, 0x01, 0x7E};
  check_frame(request, request_length, identity_request);
  CHECK(encode_remote_diagnostic_request(9, false, request, sizeof(request), &request_length));
  const uint8_t status_request[] = {0xD3, 0x06, 0x09, 0x00, 0x7E};
  check_frame(request, request_length, status_request);
  CHECK(!encode_remote_diagnostic_request(64, true, request, sizeof(request), &request_length));

  const uint8_t identity[] = {0xC4, 0x12, 0x34, 0x56, 0x81, 0x11, 0x03, 0x09, 0x01, 0x7E};
  DecodedPacket decoded = decode(identity);
  CHECK(decoded.device_id == 0x123456);
  CHECK(decoded.model_id == 0x1103);
  CHECK(decoded.has_model);
  CHECK(decoded.has_device_type);
  CHECK(decoded.device_type == 0x81);
  CHECK(decoded.has_sid);
  CHECK(decoded.sid == 9);
  CHECK(!decoded.has_alarm);
  CHECK_STRING(decoded.event, "REMOTE IDENTITY");

  const uint8_t new_device[] = {0xD4, 0x09, 0x09, 0x12, 0x34, 0x56, 0x7E};
  uint8_t sid = 0;
  uint32_t device_id = 0;
  CHECK(decode_new_device(new_device, sizeof(new_device), &sid, &device_id));
  CHECK(sid == 9);
  CHECK(device_id == 0x123456);

  const uint8_t status[] = {0xD4, 0x06, 0x09, 0x2A, 0x38, 0x41, 0xA0,
                            0xEF, 0x12, 0x34, 0x56, 0x05, 0x02, 0x7E};
  RemoteDiagnostic diagnostic{};
  CHECK(decode_remote_diagnostic(status, sizeof(status), &diagnostic));
  CHECK(diagnostic.sid == 9);
  CHECK(diagnostic.battery_primary == 0x2A);
  CHECK(diagnostic.battery_radio == 0x38);
  CHECK(diagnostic.rssi == 0xA0);
  CHECK(diagnostic.firmware_version == 0xEF);
  CHECK(diagnostic.device_id == 0x123456);
  CHECK(diagnostic.flags == 0x05);
  CHECK(diagnostic.radio_fault_count == 2);
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
  CHECK(!decoded.has_alarm);
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
  CHECK(!decoded.has_alarm);
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

  const uint8_t cleared[] = {0x51, 0x10, 0x20, 0x30, 0x81, 0x09, 0x01, 0x7E};
  decoded = decode(cleared);
  CHECK(decoded.has_alarm);
  CHECK(!decoded.alarm);
  CHECK_STRING(decoded.event, "FIRE ALARM CLEARED");
}

void test_silence() {
  const uint8_t silence[] = {0x61, 0xAA, 0xBB, 0xCC, 0x81, 0x00, 0x7E};
  DecodedPacket decoded = decode(silence);
  CHECK(decoded.device_id == 0xAABBCC);
  CHECK(!decoded.has_alarm);
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

  const uint8_t alarm_off_too_short[] = {0x51, 0x01, 0x02, 0x03, 0x81, 0x01, 0x7E};
  check_rejected(alarm_off_too_short);

  const uint8_t remote_identity_bad_sid[] = {0xC4, 0x01, 0x02, 0x03, 0x81,
                                             0x11, 0x03, 0x40, 0x01, 0x7E};
  check_rejected(remote_identity_bad_sid);

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
  test_frame_escaping();
  test_management_command_frames();
  test_radio_identity_frames();
  test_sid_map();
  test_remote_discovery_frames();
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
