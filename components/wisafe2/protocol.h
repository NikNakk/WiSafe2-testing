#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::wisafe2 {

enum class ManagementCommand : uint8_t {
  SOUND_CO,
  SOUND_FIRE,
  SOUND_COMBINED,
  SILENCE_CO,
  SILENCE_FIRE,
  QUERY_PAIRING,
  START_PAIRING,
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
  bool has_event;
  bool has_result;
  bool has_base;
  bool has_battery;
  bool alarm;
  bool base_problem;
  bool battery_low;
  char device[12];
  char model[12];
  char event[32];
  char result[12];
  char base[12];
  char battery[12];
};

bool decode_packet(const uint8_t *packet, size_t length, DecodedPacket *decoded);
bool encode_management_command(ManagementCommand command, uint32_t bridge_device_id, uint16_t bridge_model_id,
                               CommandFrames *frames);
const char *management_command_name(ManagementCommand command);

}  // namespace esphome::wisafe2
