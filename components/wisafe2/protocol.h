#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::wisafe2 {

// 0x71 status flags, named by captured FireAngel Connected Gateway firmware.
static constexpr uint8_t STATUS_FLAG_CALIBRATED = 0x01;
static constexpr uint8_t STATUS_FLAG_FAULTY = 0x02;
static constexpr uint8_t STATUS_FLAG_ON_BASE = 0x04;
static constexpr uint8_t STATUS_FLAG_SD_BATTERY_FAULT = 0x08;
static constexpr uint8_t STATUS_FLAG_AC_FAILED = 0x10;
static constexpr uint8_t STATUS_FLAG_RM_BATTERY_FAULT = 0x20;

enum class DetectorType : uint8_t { UNKNOWN, SMOKE, HEAT, CARBON_MONOXIDE };

enum class ManagementCommand : uint8_t {
  SOUND_CO,
  SOUND_FIRE,
  SOUND_COMBINED,
  SILENCE_CO,
  SILENCE_FIRE,
  QUERY_PAIRING,
  START_PAIRING,
  REFRESH_DETECTORS,
};

struct CommandFrames {
  uint8_t primary[11];
  size_t primary_length;
  uint8_t secondary[11];
  size_t secondary_length;
};

struct DecodedPacket {
  bool recognized;
  uint32_t device_id;
  uint16_t model_id;
  bool has_model;
  bool has_device_type;
  bool has_event;
  bool has_result;
  bool has_base;
  bool has_battery;
  bool has_alarm;
  bool has_sid;
  bool has_status_flags;
  bool alarm;
  bool base_problem;
  bool battery_low;
  bool calibrated;
  bool device_fault;
  bool sensor_battery_fault;
  bool ac_power_fault;
  bool radio_battery_fault;
  uint8_t sid;
  uint8_t device_type;
  uint8_t status_flags;
  char device[12];
  char model[12];
  char event[32];
  char result[12];
  char base[12];
  char battery[12];
};

struct RemoteDiagnostic {
  uint8_t sid;
  uint8_t battery_primary;
  uint8_t battery_radio;
  uint8_t unknown_1;
  uint8_t rssi;
  uint8_t firmware_version;
  uint32_t device_id;
  uint8_t flags;
  uint8_t radio_fault_count;
};

struct RadioDiagnostic {
  uint8_t battery_primary;
  uint8_t battery_radio;
  uint8_t unknown_1;
  uint8_t rssi;
  uint8_t firmware_version;
  uint32_t device_id;
  uint8_t flags;
  uint8_t radio_fault_count;
  uint8_t sid;
  uint8_t unknown_2;
  bool connected;
};

const char *detector_model_name(uint16_t model_id, bool has_model = true);
DetectorType detector_type_for_model(uint16_t model_id, bool has_model = true);
DetectorType infer_detector_type(uint16_t model_id, bool has_model, uint8_t device_type,
                                 bool has_device_type, const char *event);
bool escape_frame(const uint8_t *frame, size_t length, uint8_t *escaped, size_t capacity,
                  size_t *escaped_length);
bool unescape_frame(const uint8_t *frame, size_t length, uint8_t *unescaped, size_t capacity,
                    size_t *unescaped_length);
bool decode_packet(const uint8_t *packet, size_t length, DecodedPacket *decoded);
bool decode_radio_diagnostic(const uint8_t *packet, size_t length, RadioDiagnostic *diagnostic);
bool decode_sid_map(const uint8_t *packet, size_t length, uint64_t *sid_map);
bool decode_new_device(const uint8_t *packet, size_t length, uint8_t *sid, uint32_t *device_id);
bool decode_remote_diagnostic(const uint8_t *packet, size_t length, RemoteDiagnostic *diagnostic);
bool encode_remote_diagnostic_request(uint8_t sid, bool identity, uint8_t *frame, size_t capacity,
                                      size_t *length);
bool is_identity_request(const uint8_t *packet, size_t length);
bool encode_identity_response(uint32_t bridge_device_id, uint16_t bridge_model_id, uint8_t *frame,
                              size_t capacity, size_t *length);
bool encode_management_command(ManagementCommand command, uint32_t bridge_device_id, uint16_t bridge_model_id,
                               CommandFrames *frames);
const char *management_command_name(ManagementCommand command);

}  // namespace esphome::wisafe2
