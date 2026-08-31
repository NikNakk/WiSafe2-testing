#include "protocol.h"

#include <cstdio>
#include <cstring>

namespace esphome::wisafe2 {

bool escape_frame(const uint8_t *frame, size_t length, uint8_t *escaped, size_t capacity,
                  size_t *escaped_length) {
  if (frame == nullptr || escaped == nullptr || escaped_length == nullptr || length == 0 ||
      frame[length - 1] != 0x7E)
    return false;

  size_t used = 0;
  for (size_t i = 0; i + 1 < length; ++i) {
    if (frame[i] == 0x7E || frame[i] == 0x7D) {
      if (used + 2 > capacity)
        return false;
      escaped[used++] = 0x7D;
      escaped[used++] = frame[i] == 0x7E ? 0x01 : 0x02;
    } else {
      if (used + 1 > capacity)
        return false;
      escaped[used++] = frame[i];
    }
  }
  if (used + 1 > capacity)
    return false;
  escaped[used++] = 0x7E;
  *escaped_length = used;
  return true;
}

bool unescape_frame(const uint8_t *frame, size_t length, uint8_t *unescaped, size_t capacity,
                    size_t *unescaped_length) {
  if (frame == nullptr || unescaped == nullptr || unescaped_length == nullptr || length == 0 ||
      frame[length - 1] != 0x7E)
    return false;

  size_t used = 0;
  for (size_t i = 0; i + 1 < length; ++i) {
    uint8_t value = frame[i];
    if (value == 0x7E)
      return false;
    if (value == 0x7D) {
      if (++i + 1 >= length)
        return false;
      if (frame[i] == 0x01)
        value = 0x7E;
      else if (frame[i] == 0x02)
        value = 0x7D;
      else
        return false;
    }
    if (used + 1 > capacity)
      return false;
    unescaped[used++] = value;
  }
  if (used + 1 > capacity)
    return false;
  unescaped[used++] = 0x7E;
  *unescaped_length = used;
  return true;
}

const char *detector_model_name(uint16_t model_id, bool has_model) {
  if (!has_model)
    return "Unknown FireAngel alarm";
  switch (model_id) {
    case 0xED08: return "FP2620W2";
    case 0x1104: return "FP1720W2";
    case 0x1103: return "WST-630";
    case 0x340E: return "WST-630N";
    case 0x7C04: return "ST-630-DE";
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
    case 0x340E:
    case 0x7C04: return DetectorType::SMOKE;
    case 0x1104: return DetectorType::HEAT;
    case 0x7803: return DetectorType::CARBON_MONOXIDE;
    default: return DetectorType::UNKNOWN;
  }
}

static uint8_t device_type_for_model(uint16_t model_id) {
  switch (detector_type_for_model(model_id)) {
    case DetectorType::SMOKE:
    case DetectorType::HEAT: return 0x81;
    case DetectorType::CARBON_MONOXIDE: return 0x41;
    case DetectorType::UNKNOWN: return 0xFF;
  }
  return 0xFF;
}

bool is_identity_request(const uint8_t *packet, size_t length) {
  return packet != nullptr && length == 2 && packet[0] == 0x41 && packet[1] == 0x7E;
}

bool encode_identity_response(uint32_t bridge_device_id, uint16_t bridge_model_id, uint8_t *frame,
                              size_t capacity, size_t *length) {
  static constexpr size_t FRAME_LENGTH = 11;
  if (frame == nullptr || length == nullptr || capacity < FRAME_LENGTH || bridge_device_id > 0xFFFFFF)
    return false;

  const uint8_t response[FRAME_LENGTH] = {
      0x91,
      static_cast<uint8_t>((bridge_device_id >> 16) & 0xFF),
      static_cast<uint8_t>((bridge_device_id >> 8) & 0xFF),
      static_cast<uint8_t>(bridge_device_id & 0xFF),
      static_cast<uint8_t>((bridge_model_id >> 8) & 0xFF),
      static_cast<uint8_t>(bridge_model_id & 0xFF),
      device_type_for_model(bridge_model_id),
      0x05,
      0x01,
      0x01,
      0x7E,
  };
  memcpy(frame, response, sizeof(response));
  *length = sizeof(response);
  return true;
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
    case ManagementCommand::REFRESH_DETECTORS: return "Refresh detector diagnostics";
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
    decoded->has_status_flags = true;
    decoded->alarm = false;
    decoded->status_flags = packet[6];
    decoded->calibrated = (packet[6] & STATUS_FLAG_CALIBRATED) != 0;
    decoded->device_fault = (packet[6] & STATUS_FLAG_FAULTY) != 0;
    decoded->base_problem = (packet[6] & STATUS_FLAG_ON_BASE) == 0;
    decoded->sensor_battery_fault = (packet[6] & STATUS_FLAG_SD_BATTERY_FAULT) != 0;
    decoded->ac_power_fault = (packet[6] & STATUS_FLAG_AC_FAILED) != 0;
    decoded->radio_battery_fault = (packet[6] & STATUS_FLAG_RM_BATTERY_FAULT) != 0;
    decoded->battery_low = decoded->sensor_battery_fault || decoded->radio_battery_fault;
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
    decoded->has_alarm = true;
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
  } else if (packet[0] == 0x51 && length >= 8) {
    decoded->device_id = (static_cast<uint32_t>(packet[1]) << 16) | (static_cast<uint32_t>(packet[2]) << 8) |
                         packet[3];
    decoded->has_event = true;
    decoded->has_alarm = true;
    decoded->alarm = false;
    snprintf(decoded->device, sizeof(decoded->device), "%02X%02X%02X", packet[1], packet[2], packet[3]);
    if (packet[4] == 0x81 || packet[4] == 0x82)
      snprintf(decoded->event, sizeof(decoded->event), "FIRE ALARM CLEARED");
    else if (packet[4] == 0x41)
      snprintf(decoded->event, sizeof(decoded->event), "CARBON MONOXIDE ALARM CLEARED");
    else
      snprintf(decoded->event, sizeof(decoded->event), "ALARM CLEARED 0x%02X", packet[4]);
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
  } else if (packet[0] == 0xC4 && length >= 10 && packet[7] < 64) {
    decoded->device_id = (static_cast<uint32_t>(packet[1]) << 16) | (static_cast<uint32_t>(packet[2]) << 8) |
                         packet[3];
    decoded->model_id = (static_cast<uint16_t>(packet[5]) << 8) | packet[6];
    decoded->has_model = true;
    decoded->has_device_type = true;
    decoded->has_sid = true;
    decoded->sid = packet[7];
    decoded->device_type = packet[4];
    snprintf(decoded->device, sizeof(decoded->device), "%02X%02X%02X", packet[1], packet[2], packet[3]);
    snprintf(decoded->model, sizeof(decoded->model), "%02X%02X", packet[5], packet[6]);
    snprintf(decoded->event, sizeof(decoded->event), "REMOTE IDENTITY");
  } else {
    return false;
  }

  decoded->recognized = true;
  return true;
}

bool decode_radio_diagnostic(const uint8_t *packet, size_t length, RadioDiagnostic *diagnostic) {
  // D2 is the response to the local D1 diagnostic request. Earlier bridge
  // firmware treated every D2 frame as a missing-detector event, but observed
  // 14-byte frames contain the attached radio's own identity and SID instead.
  if (packet == nullptr || diagnostic == nullptr || length < 14 || packet[0] != 0xD2 ||
      packet[length - 1] != 0x7E)
    return false;

  memset(diagnostic, 0, sizeof(*diagnostic));
  diagnostic->battery_primary = packet[1];
  diagnostic->battery_radio = packet[2];
  diagnostic->unknown_1 = packet[3];
  diagnostic->rssi = packet[4];
  diagnostic->firmware_version = packet[5];
  diagnostic->device_id = (static_cast<uint32_t>(packet[6]) << 16) |
                          (static_cast<uint32_t>(packet[7]) << 8) | packet[8];
  diagnostic->flags = packet[9];
  diagnostic->radio_fault_count = packet[10];
  diagnostic->sid = packet[11];
  diagnostic->unknown_2 = packet[12];
  diagnostic->connected = diagnostic->device_id != 0 && diagnostic->sid < 0x40;
  return true;
}

bool decode_sid_map(const uint8_t *packet, size_t length, uint64_t *sid_map) {
  if (packet == nullptr || sid_map == nullptr || length < 11 || packet[0] != 0xD4 || packet[1] != 0x03 ||
      packet[length - 1] != 0x7E)
    return false;

  *sid_map = 0;
  for (size_t i = 0; i < 8; ++i)
    *sid_map |= static_cast<uint64_t>(packet[i + 2]) << (i * 8);
  return true;
}

bool decode_new_device(const uint8_t *packet, size_t length, uint8_t *sid, uint32_t *device_id) {
  if (packet == nullptr || sid == nullptr || device_id == nullptr || length < 7 || packet[0] != 0xD4 ||
      packet[1] != 0x09 || packet[length - 1] != 0x7E || packet[2] >= 64)
    return false;

  *sid = packet[2];
  *device_id = (static_cast<uint32_t>(packet[3]) << 16) |
               (static_cast<uint32_t>(packet[4]) << 8) | packet[5];
  return true;
}

bool decode_remote_diagnostic(const uint8_t *packet, size_t length, RemoteDiagnostic *diagnostic) {
  if (packet == nullptr || diagnostic == nullptr || length < 14 || packet[0] != 0xD4 || packet[1] != 0x06 ||
      packet[length - 1] != 0x7E || packet[2] >= 64)
    return false;

  memset(diagnostic, 0, sizeof(*diagnostic));
  diagnostic->sid = packet[2];
  diagnostic->battery_primary = packet[3];
  diagnostic->battery_radio = packet[4];
  diagnostic->unknown_1 = packet[5];
  diagnostic->rssi = packet[6];
  diagnostic->firmware_version = packet[7];
  diagnostic->device_id = (static_cast<uint32_t>(packet[8]) << 16) |
                          (static_cast<uint32_t>(packet[9]) << 8) | packet[10];
  diagnostic->flags = packet[11];
  diagnostic->radio_fault_count = packet[12];
  return true;
}

bool encode_remote_diagnostic_request(uint8_t sid, bool identity, uint8_t *frame, size_t capacity,
                                      size_t *length) {
  static constexpr size_t FRAME_LENGTH = 5;
  if (frame == nullptr || length == nullptr || capacity < FRAME_LENGTH || sid >= 64)
    return false;
  const uint8_t request[FRAME_LENGTH] = {0xD3, 0x06, sid, static_cast<uint8_t>(identity ? 0x01 : 0x00), 0x7E};
  memcpy(frame, request, sizeof(request));
  *length = sizeof(request);
  return true;
}

}  // namespace esphome::wisafe2
