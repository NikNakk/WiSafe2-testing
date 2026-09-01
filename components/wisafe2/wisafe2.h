#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/spi_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "protocol.h"

namespace esphome::wisafe2 {

class WiSafe2CommandButton;

class WiSafe2Component : public Component {
 public:
  void set_pins(int sclk, int mosi, int miso, int cs, int irq);
  void set_initialized_sensor(binary_sensor::BinarySensor *sensor) { this->initialized_sensor_ = sensor; }
  void set_last_packet_sensor(text_sensor::TextSensor *sensor) { this->last_packet_sensor_ = sensor; }
  void set_last_device_sensor(text_sensor::TextSensor *sensor) { this->last_device_sensor_ = sensor; }
  void set_last_model_sensor(text_sensor::TextSensor *sensor) { this->last_model_sensor_ = sensor; }
  void set_last_event_sensor(text_sensor::TextSensor *sensor) { this->last_event_sensor_ = sensor; }
  void set_last_result_sensor(text_sensor::TextSensor *sensor) { this->last_result_sensor_ = sensor; }
  void set_last_base_sensor(text_sensor::TextSensor *sensor) { this->last_base_sensor_ = sensor; }
  void set_last_battery_sensor(text_sensor::TextSensor *sensor) { this->last_battery_sensor_ = sensor; }
  void set_paired_sensor(binary_sensor::BinarySensor *sensor) { this->paired_sensor_ = sensor; }
  void set_command_busy_sensor(binary_sensor::BinarySensor *sensor) { this->command_busy_sensor_ = sensor; }
  void set_last_command_sensor(text_sensor::TextSensor *sensor) { this->last_command_sensor_ = sensor; }
  void set_discovery_prefix(const std::string &prefix) { this->discovery_prefix_ = prefix; }
  void set_max_detectors(uint8_t max_detectors) { this->max_detectors_ = max_detectors; }
  void set_bridge_device_id(uint32_t device_id) { this->bridge_device_id_ = device_id; }
  void set_bridge_model_id(uint16_t model_id) { this->bridge_model_id_ = model_id; }
  void set_time(time::RealTimeClock *time) { this->time_ = time; }
  void request_command(ManagementCommand command);

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  static constexpr size_t PACKET_MAX = 64;
  // Every logical payload byte can expand to a two-byte escape sequence. The
  // terminating 0x7E remains unescaped, so twice the logical limit is ample.
  static constexpr size_t WIRE_PACKET_MAX = PACKET_MAX * 2;
  // Capacity is 16 bits because this radio has occasionally clocked 9-10 bits
  // for one byte. The protocol still expects 8; trans_len records what occurred.
  static constexpr size_t SPI_SLOT_BITS = 16;
  static constexpr uint32_t BYTE_TIMEOUT_MS = 1000;
  static constexpr uint32_t RX_ACK_PULSE_US = 8;
  static constexpr unsigned INIT_MAX_ATTEMPTS = 50;
  static constexpr uint32_t INIT_RETRY_DELAY_MS = 500;
  static constexpr uint32_t INCOMPLETE_FRAME_TIMEOUT_MS = 100;
  static constexpr uint32_t COMMAND_POLL_INTERVAL_MS = 100;
  static constexpr uint32_t COMMAND_RESPONSE_TIMEOUT_MS = 1000;
  static constexpr uint32_t COMMAND_RETRY_DELAY_MS = 500;
  static constexpr uint32_t RADIO_COMMAND_GAP_MS = 500;
  static constexpr uint32_t REMOTE_RESPONSE_TIMEOUT_MS = 20000;
  static constexpr unsigned COMMAND_MAX_ATTEMPTS = 6;
  static constexpr uint32_t PAIRING_WINDOW_MS = 21000;
  static constexpr uint32_t LOCAL_DIAGNOSTIC_INTERVAL_MS = 60000;
  static constexpr size_t MAX_DETECTORS = 32;
  static constexpr uint32_t INVENTORY_MAGIC = 0x57533231;
  static constexpr uint32_t TEST_HISTORY_MAGIC = 0x57535431;

  struct SpiSlot {
    uint32_t tx_word;
    uint32_t rx_word;
    spi_slave_transaction_t transaction;
  };

  struct ExchangeResult {
    bool spi_reset;
    size_t tx_count;
    size_t tx_expected;
    size_t tx_bits[WIRE_PACKET_MAX];
    uint8_t simultaneous_rx[WIRE_PACKET_MAX];
    size_t response_len;
    uint8_t response[PACKET_MAX];
    size_t response_bits[PACKET_MAX];
  };

  ExchangeResult exchange_scratch_{};

  enum class EventType : uint8_t { INITIALIZED, ERROR, PACKET };

  enum class RemoteRequestType : uint8_t { NONE, IDENTITY, STATUS };

  enum class CommandOutcome : uint8_t {
    ACCEPTED,
    TIMEOUT,
    PAIRED,
    UNPAIRED,
    ALREADY_PAIRED,
    NO_DETECTORS,
    ERROR,
  };

  struct RadioEvent {
    EventType type;
    size_t length;
    uint8_t packet[PACKET_MAX];
  };

  struct CommandResult {
    ManagementCommand command;
    CommandOutcome outcome;
  };

  struct StoredDetector {
    uint32_t device_id;
    uint16_t model_id;
    // Bit 0 is has_model, bit 1 is has_device_type, and bits 2..3 encode
    // 0x41 (1), 0x81 (2), or another device type (3). Old 0/1 values remain
    // valid after this formerly boolean byte is upgraded to metadata.
    uint8_t metadata;
    // Zero means unknown; values 1..64 encode SIDs 0..63. This reuses the
    // formerly reserved byte without invalidating existing inventories.
    uint8_t sid_plus_one;
  };

  struct StoredInventory {
    uint32_t magic;
    uint8_t count;
    uint8_t reserved[3];
    StoredDetector detectors[MAX_DETECTORS];
  };

  struct StoredTestRecord {
    uint32_t device_id;
    uint32_t last_test_epoch;
    uint8_t result;
    uint8_t reserved[3];
  };

  struct StoredTestHistory {
    uint32_t magic;
    StoredTestRecord detectors[MAX_DETECTORS];
  };

  struct DetectorState {
    uint32_t device_id;
    uint16_t model_id;
    bool has_model;
    bool has_device_type;
    uint8_t device_type;
    int8_t sid;
    int8_t alarm;
    int8_t base_problem;
    int8_t battery_low;
    int8_t calibrated;
    int8_t device_fault;
    int8_t sensor_battery_fault;
    int8_t ac_power_fault;
    int8_t radio_battery_fault;
    int16_t status_flags;
    int8_t network_member;
    bool has_remote_diagnostic;
    uint8_t battery_primary;
    uint8_t battery_radio;
    uint8_t rssi;
    uint8_t firmware_version;
    uint8_t diagnostic_flags;
    uint8_t radio_fault_count;
    uint32_t last_test_epoch;
    char event[32];
    char result[12];
    char raw_frame[PACKET_MAX * 3];
  };

  static void radio_task_entry(void *parameter);
  void radio_task_();
  bool initialize_radio_();
  void raw_receive_loop_();
  void execute_command_(ManagementCommand command);
  bool send_expect_(const uint8_t *data, size_t length, const uint8_t *expected, size_t expected_length,
                    unsigned expected_packets, unsigned attempts);
  bool transmit_packet_(const uint8_t *data, size_t length);
  bool query_pairing_status_(bool *paired);
  CommandOutcome start_pairing_();
  bool respond_to_identity_request_();
  void poll_local_radio_();
  bool discover_sid_(uint8_t sid, bool *awaiting_response);
  bool poll_remote_status_(uint8_t sid, bool *awaiting_response);
  bool remote_request_ready_() const;
  bool has_remote_work_() const;
  void service_remote_requests_();
  bool note_discovery_packet_(const uint8_t *packet, size_t length);
  void emit_exchange_packets_(const ExchangeResult *result);
  esp_err_t receive_window_(uint32_t window_ms);

  esp_err_t init_gpio_();
  esp_err_t init_spi_slave_();
  void irq_set_(bool asserted);
  void acknowledge_received_byte_();
  void prepare_slot_(SpiSlot *slot, uint8_t tx_value);
  uint8_t slot_rx_byte_(const SpiSlot *slot) const;
  esp_err_t queue_slot_(SpiSlot *slot, uint8_t tx_value);
  esp_err_t wait_for_slot_(SpiSlot *expected, TickType_t timeout);
  esp_err_t reset_spi_slave_();
  void wait_for_radio_command_gap_();
  void note_radio_frame_complete_();
  esp_err_t abort_exchange_(esp_err_t cause, ExchangeResult *result);
  esp_err_t exchange_packet_(const uint8_t *data, size_t length, ExchangeResult *result,
                             unsigned response_packets, uint32_t response_timeout_ms);
  bool response_contains_packet_(const ExchangeResult *result, const uint8_t *expected, size_t expected_len) const;
  bool response_pairing_state_(const ExchangeResult *result, bool *paired) const;
  void log_exchange_diagnostics_(const ExchangeResult *result) const;
  void log_packet_(const char *prefix, const uint8_t *data, size_t length) const;
  void emit_event_(EventType type, const uint8_t *packet = nullptr, size_t length = 0);
  void emit_command_result_(ManagementCommand command, CommandOutcome outcome);
  void load_inventory_();
  void save_inventory_();
  void load_test_history_();
  void save_test_history_();
  DetectorState *find_or_create_detector_(const DecodedPacket &decoded);
  void update_detector_(const DecodedPacket &decoded, const char *raw_frame);
  void update_remote_diagnostic_(const RemoteDiagnostic &diagnostic, const char *raw_frame);
  void update_detector_membership_(uint64_t sid_map);
  void service_mqtt_();
  bool publish_detector_discovery_(const DetectorState &detector);
  bool publish_detector_state_(const DetectorState &detector);
  bool publish_detector_trigger_discovery_(const DetectorState &detector, const char *key, const char *type,
                                           const char *subtype, const char *payload);
  bool publish_detector_event_(const DetectorState &detector, const char *payload);
  bool publish_discovery_entity_(const DetectorState &detector, const char *component, const char *key,
                                 const char *name, const char *value_template, const char *device_class,
                                 const char *entity_category, const char *icon,
                                 const char *state_class = nullptr);
  const char *model_name_(const DetectorState &detector) const;
  const char *alarm_device_class_(const DetectorState &detector) const;
  bool format_last_test_(const DetectorState &detector, char *buffer, size_t length) const;
  void format_detector_topic_(char *buffer, size_t length, const DetectorState &detector, const char *suffix) const;

  gpio_num_t sclk_pin_{GPIO_NUM_NC};
  gpio_num_t mosi_pin_{GPIO_NUM_NC};
  gpio_num_t miso_pin_{GPIO_NUM_NC};
  gpio_num_t cs_pin_{GPIO_NUM_NC};
  gpio_num_t irq_pin_{GPIO_NUM_NC};
  gpio_glitch_filter_handle_t sclk_filter_{nullptr};
  QueueHandle_t event_queue_{nullptr};
  QueueHandle_t command_queue_{nullptr};
  QueueHandle_t command_result_queue_{nullptr};
  TaskHandle_t radio_task_handle_{nullptr};
  binary_sensor::BinarySensor *initialized_sensor_{nullptr};
  text_sensor::TextSensor *last_packet_sensor_{nullptr};
  text_sensor::TextSensor *last_device_sensor_{nullptr};
  text_sensor::TextSensor *last_model_sensor_{nullptr};
  text_sensor::TextSensor *last_event_sensor_{nullptr};
  text_sensor::TextSensor *last_result_sensor_{nullptr};
  text_sensor::TextSensor *last_base_sensor_{nullptr};
  text_sensor::TextSensor *last_battery_sensor_{nullptr};
  binary_sensor::BinarySensor *paired_sensor_{nullptr};
  binary_sensor::BinarySensor *command_busy_sensor_{nullptr};
  text_sensor::TextSensor *last_command_sensor_{nullptr};
  std::string discovery_prefix_{"homeassistant"};
  uint8_t max_detectors_{16};
  uint32_t bridge_device_id_{0xA5B813};
  uint16_t bridge_model_id_{0x1103};
  time::RealTimeClock *time_{nullptr};
  DetectorState detectors_[MAX_DETECTORS]{};
  uint8_t detector_count_{0};
  // Seeded from persisted inventory before the radio task starts; after that,
  // discovery scheduling and SID lookup are owned exclusively by that task.
  uint8_t own_sid_{0xFF};
  uint64_t known_sid_map_{0};
  uint64_t latest_sid_map_{0};
  uint64_t pending_discovery_sid_map_{0};
  uint64_t pending_status_sid_map_{0};
  uint32_t sid_device_ids_[64]{};
  RemoteRequestType remote_request_type_{RemoteRequestType::NONE};
  uint8_t remote_request_sid_{0xFF};
  TickType_t remote_request_started_tick_{0};
  TickType_t last_radio_frame_tick_{0};
  bool has_completed_radio_frame_{false};
  ESPPreferenceObject inventory_pref_{};
  ESPPreferenceObject test_history_pref_{};
  bool mqtt_was_connected_{false};
  bool mqtt_resync_pending_{false};
  uint8_t mqtt_resync_index_{0};
  uint32_t mqtt_next_sync_ms_{0};
  std::atomic<bool> radio_ready_{false};
  bool command_busy_{false};
};

class WiSafe2CommandButton : public button::Button {
 public:
  void set_parent(WiSafe2Component *parent) { this->parent_ = parent; }
  void set_command(ManagementCommand command) { this->command_ = command; }

 protected:
  void press_action() override;

  WiSafe2Component *parent_{nullptr};
  ManagementCommand command_{ManagementCommand::QUERY_PAIRING};
};

}  // namespace esphome::wisafe2
