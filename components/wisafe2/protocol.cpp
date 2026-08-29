#include "protocol.h"

#include <cstdio>
#include <cstring>

namespace esphome::wisafe2 {

const char *detector_model_name(uint16_t model_id, bool has_model) {
  if (!has_model)
    return "Unknown FireAngel alarm";
  switch (model_id) {
    case 0xED08: return "FP2620W2";
    case 0x1104: return "FP1720W2";
    case 0x1103: return "WST-630";
    case 0x340E: return "WST-630N";
    case 0x7803: return "W2-CO-10X";
    case 0xC304: return "W2-SVP-630";
    default: return "Unknown FireAngel alarm";
  }
}

DetectorType detector_type_for_model(uint16_t model_id, bool has_model) {
  if (!has_model)
    return DetectorType::UNKNOWN;
  switch (model_id) {
    case 0xED08:
    case 0x1103:
    case 0x340E: return DetectorType::SMOKE;
    case 0x1104: return DetectorType::HEAT;
    case 0x7803: return DetectorType::CARBON_MONOXIDE;
    default: return DetectorType::UNKNOWN;
  }
}

const char *management_command_name(ManagementCommand command) {
  switch (command) {
    case ManagementCommand::SOUND_CO: return "Sound CO test";
    case ManagementCommand::SOUND_FIRE: return "Sound fire test";
    case ManagementCommand::SOUND_COMBINED: return "Sound combined test";
    case ManagementCommand::SILENCE_CO: return "Silence CO alarms";
    case ManagementCommand::SILENCE_FIRE: return "Silence fire alarms";
    case ManagementCommand::QUERY_PAIRING: return "Check pairing";
    case ManagementCommand::START_PAIRING: return "Start pairing";
  }
  return "Unknown command";
}

bool encode_management_command(ManagementCommand command, uint32_t bridge_device_id, uint16_t bridge_model_id,
                               CommandFrames *frames) {
  if (frames == nullptr || bridge_device_id > 0xFFFFFF)
    return false;

  memset(frames, 0, sizeof(*frames));
  const uint8_t id0 = (bridge_device_id >> 16) & 0xFF;
  const uint8_t id1 = (bridge_device_id >> 8) & 0xFF;
  const uint8_t id2 = bridge_device_id & 0xFF;
  const uint8_t model0 = (bridge_model_id >> 8) & 0xFF;
  const uint8_t model1 = bridge_model_id & 0xFF;

  uint8_t event_code = 0;
  if (command == ManagementCommand::SOUND_CO || command == ManagementCommand::SOUND_FIRE ||
      command == ManagementCommand::SOUND_COMBINED) {
    event_code = command == ManagementCommand::SOUND_CO        ? 0x41
                 : command == ManagementCommand::SOUND_FIRE    ? 0x81
                                                               : 0xFF;
    const uint8_t primary[] = {0x70, id0, id1, id2, event_code, 0x01, model0, model1, 0x7E};
    const uint8_t secondary[] = {0x91, id0, id1, id2, model0, model1, event_code, 0x05, 0x00, 0x02, 0x7E};
    memcpy(frames->primary, primary, sizeof(primary));
    memcpy(frames->secondary, secondary, sizeof(secondary));
    frames->primary_length = sizeof(primary);
    frames->secondary_length = sizeof(secondary);
    return true;
  }

  if (command == ManagementCommand::SILENCE_CO || command == ManagementCommand::SILENCE_FIRE) {
    event_code = command == ManagementCommand::SILENCE_CO ? 0x40 : 0x80;
    const uint8_t primary[] = {0x61, id0, id1, id2, event_code, 0x01, 0x7E};
    memcpy(frames->primary, primary, sizeof(primary));
    frames->primary_length = sizeof(primary);
    return true;
  }

  if (command == ManagementCommand::QUERY_PAIRING) {
    const uint8_t primary[] = {0xD3, 0x03, 0x7E};
    memcpy(frames->primary, primary, sizeof(primary));
    frames->primary_length = sizeof(primary);
    return true;
  }

  if (command == ManagementCommand::START_PAIRING) {
    const uint8_t primary[] = {0xD3, 0x12, 0x01, 0x7E};
    const uint8_t secondary[] = {0x91, id0, id1, id2, model0, model1, 0xFF, 0x05, 0x01, 0x01, 0x7E};
    memcpy(frames->primary, primary, sizeof(primary));
    memcpy(frames->secondary, secondary, sizeof(secondary));
    frames->primary_length = sizeof(primary);
    frames->secondary_length = sizeof(secondary);
    return true;
  }

  return false;
}

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
    // Both 0x81 and the observed 0x82 variant are fire-family values; captures
    // do not establish a reliable smoke-versus-heat distinction.
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
