#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::wisafe2 {

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

}  // namespace esphome::wisafe2
