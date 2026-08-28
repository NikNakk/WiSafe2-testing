#include "protocol.h"

#include <cstdio>
#include <cstring>

namespace esphome::wisafe2 {

bool decode_packet(const uint8_t *packet, size_t length, DecodedPacket *decoded) {
  if (packet == nullptr || decoded == nullptr || length < 2 || packet[length - 1] != 0x7E)
    return false;

  memset(decoded, 0, sizeof(*decoded));
  snprintf(decoded->device, sizeof(decoded->device), "UNKNOWN");
  snprintf(decoded->model, sizeof(decoded->model), "N/A");
  snprintf(decoded->event, sizeof(decoded->event), "UNKNOWN");
  snprintf(decoded->result, sizeof(decoded->result), "N/A");
  snprintf(decoded->base, sizeof(decoded->base), "UNKNOWN");
  snprintf(decoded->battery, sizeof(decoded->battery), "UNKNOWN");

  if (packet[0] == 0x70 && length >= 9) {
    decoded->device_id = (static_cast<uint32_t>(packet[1]) << 16) | (static_cast<uint32_t>(packet[2]) << 8) |
                         packet[3];
    decoded->model_id = (static_cast<uint16_t>(packet[6]) << 8) | packet[7];
    decoded->has_model = true;
    decoded->has_event = true;
    decoded->has_result = true;
    decoded->has_base = true;
    decoded->has_battery = packet[5] == 0x01;
    decoded->alarm = false;
    decoded->base_problem = false;
    decoded->battery_low = false;
    snprintf(decoded->device, sizeof(decoded->device), "%02X%02X%02X", packet[1], packet[2], packet[3]);
    snprintf(decoded->model, sizeof(decoded->model), "%02X%02X", packet[6], packet[7]);
    if (packet[4] == 0x81 || packet[4] == 0x82)
      snprintf(decoded->event, sizeof(decoded->event), "FIRE TEST");
    else if (packet[4] == 0x41)
      snprintf(decoded->event, sizeof(decoded->event), "CARBON MONOXIDE TEST");
    else
      snprintf(decoded->event, sizeof(decoded->event), "UNKNOWN TEST 0x%02X", packet[4]);
    snprintf(decoded->result, sizeof(decoded->result), "%s", packet[5] == 0x01 ? "PASS" : "FAIL");
    snprintf(decoded->base, sizeof(decoded->base), "ON");
    if (packet[5] == 0x01)
      snprintf(decoded->battery, sizeof(decoded->battery), "OK");
  } else if (packet[0] == 0x71 && length >= 8) {
    decoded->device_id = (static_cast<uint32_t>(packet[1]) << 16) | (static_cast<uint32_t>(packet[2]) << 8) |
                         packet[3];
    decoded->model_id = (static_cast<uint16_t>(packet[4]) << 8) | packet[5];
    decoded->has_model = true;
    decoded->has_event = true;
    decoded->has_base = true;
    decoded->has_battery = true;
    decoded->alarm = false;
    decoded->base_problem = (packet[6] & 0x04) == 0;
    decoded->battery_low = (packet[6] & 0x42) != 0;
    snprintf(decoded->device, sizeof(decoded->device), "%02X%02X%02X", packet[1], packet[2], packet[3]);
    snprintf(decoded->model, sizeof(decoded->model), "%02X%02X", packet[4], packet[5]);
    snprintf(decoded->event, sizeof(decoded->event), "STATUS");
    snprintf(decoded->base, sizeof(decoded->base), "%s", decoded->base_problem ? "OFF" : "ON");
    snprintf(decoded->battery, sizeof(decoded->battery), "%s", decoded->battery_low ? "LOW" : "OK");
  } else if (packet[0] == 0x50 && length >= 7) {
    decoded->device_id = (static_cast<uint32_t>(packet[1]) << 16) | (static_cast<uint32_t>(packet[2]) << 8) |
                         packet[3];
    decoded->has_event = true;
    decoded->has_base = true;
    decoded->alarm = true;
    decoded->base_problem = false;
    snprintf(decoded->device, sizeof(decoded->device), "%02X%02X%02X", packet[1], packet[2], packet[3]);
    if (packet[4] == 0x81 || packet[4] == 0x82)
      snprintf(decoded->event, sizeof(decoded->event), "FIRE EMERGENCY");
    else if (packet[4] == 0x41)
      snprintf(decoded->event, sizeof(decoded->event), "CARBON MONOXIDE EMERGENCY");
    else
      snprintf(decoded->event, sizeof(decoded->event), "UNKNOWN EMERGENCY 0x%02X", packet[4]);
    snprintf(decoded->base, sizeof(decoded->base), "ON");
  } else if (packet[0] == 0x61 && length >= 7) {
    decoded->device_id = (static_cast<uint32_t>(packet[1]) << 16) | (static_cast<uint32_t>(packet[2]) << 8) |
                         packet[3];
    decoded->has_event = true;
    decoded->has_base = true;
    decoded->alarm = false;
    decoded->base_problem = false;
    snprintf(decoded->device, sizeof(decoded->device), "%02X%02X%02X", packet[1], packet[2], packet[3]);
    snprintf(decoded->event, sizeof(decoded->event), "SILENCE");
    snprintf(decoded->base, sizeof(decoded->base), "ON");
  } else if (packet[0] == 0xD2 && length >= 10) {
    decoded->device_id = (static_cast<uint32_t>(packet[6]) << 16) | (static_cast<uint32_t>(packet[7]) << 8) |
                         packet[8];
    decoded->has_event = true;
    decoded->has_base = true;
    decoded->has_battery = true;
    decoded->alarm = false;
    decoded->base_problem = true;
    decoded->battery_low = true;
    snprintf(decoded->device, sizeof(decoded->device), "%02X%02X%02X", packet[6], packet[7], packet[8]);
    snprintf(decoded->event, sizeof(decoded->event), "MISSING");
    snprintf(decoded->base, sizeof(decoded->base), "MISSING");
    snprintf(decoded->battery, sizeof(decoded->battery), "MISSING");
  } else {
    return false;
  }

  decoded->recognized = true;
  return true;
}

}  // namespace esphome::wisafe2
