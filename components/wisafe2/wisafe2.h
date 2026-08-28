#pragma once

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
#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "protocol.h"

namespace esphome::wisafe2 {

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
  void set_discovery_prefix(const std::string &prefix) { this->discovery_prefix_ = prefix; }
  void set_max_detectors(uint8_t max_detectors) { this->max_detectors_ = max_detectors; }
  void set_bridge_device_id(uint32_t device_id) { this->bridge_device_id_ = device_id; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  static constexpr size_t PACKET_MAX = 64;
  static constexpr size_t SPI_SLOT_BITS = 16;
  static constexpr uint32_t BYTE_TIMEOUT_MS = 1000;
  static constexpr uint32_t RX_ACK_PULSE_US = 8;
  static constexpr unsigned INIT_MAX_ATTEMPTS = 50;
  static constexpr uint32_t INIT_RETRY_DELAY_MS = 500;
  static constexpr uint32_t INCOMPLETE_FRAME_TIMEOUT_MS = 100;
  static constexpr size_t MAX_DETECTORS = 32;
  static constexpr uint32_t INVENTORY_MAGIC = 0x57533231;

  struct SpiSlot {
    uint32_t tx_word;
    uint32_t rx_word;
    spi_slave_transaction_t transaction;
  };

  struct ExchangeResult {
    bool spi_reset;
    size_t tx_count;
    size_t tx_bits[PACKET_MAX];
    uint8_t simultaneous_rx[PACKET_MAX];
    size_t response_len;
    uint8_t response[PACKET_MAX];
    size_t response_bits[PACKET_MAX];
  };

  enum class EventType : uint8_t { INITIALIZED, ERROR, PACKET };

  struct RadioEvent {
    EventType type;
    size_t length;
    uint8_t packet[PACKET_MAX];
  };

  struct StoredDetector {
    uint32_t device_id;
    uint16_t model_id;
    uint8_t has_model;
    uint8_t reserved;
  };

  struct StoredInventory {
    uint32_t magic;
    uint8_t count;
    uint8_t reserved[3];
    StoredDetector detectors[MAX_DETECTORS];
  };

  struct DetectorState {
    uint32_t device_id;
    uint16_t model_id;
    bool has_model;
    int8_t alarm;
    int8_t base_problem;
    int8_t battery_low;
    char event[32];
    char result[12];
    char raw_frame[PACKET_MAX * 3];
  };

  static void radio_task_entry(void *parameter);
  void radio_task_();
  bool initialize_radio_();
  void raw_receive_loop_();

  esp_err_t init_gpio_();
  esp_err_t init_spi_slave_();
  void irq_set_(bool asserted);
  void acknowledge_received_byte_();
  void prepare_slot_(SpiSlot *slot, uint8_t tx_value);
  uint8_t slot_rx_byte_(const SpiSlot *slot) const;
  esp_err_t queue_slot_(SpiSlot *slot, uint8_t tx_value);
  esp_err_t wait_for_slot_(SpiSlot *expected, TickType_t timeout);
  esp_err_t reset_spi_slave_();
  esp_err_t abort_exchange_(esp_err_t cause, ExchangeResult *result);
  esp_err_t exchange_packet_(const uint8_t *data, size_t length, ExchangeResult *result,
                             unsigned response_packets, uint32_t response_timeout_ms);
  bool response_contains_packet_(const ExchangeResult *result, const uint8_t *expected, size_t expected_len) const;
  void log_exchange_diagnostics_(const ExchangeResult *result) const;
  void log_packet_(const char *prefix, const uint8_t *data, size_t length) const;
  void emit_event_(EventType type, const uint8_t *packet = nullptr, size_t length = 0);
  void load_inventory_();
  void save_inventory_();
  DetectorState *find_or_create_detector_(const DecodedPacket &decoded);
  void update_detector_(const DecodedPacket &decoded, const char *raw_frame);
  void service_mqtt_();
  bool publish_detector_discovery_(const DetectorState &detector);
  bool publish_detector_state_(const DetectorState &detector);
  bool publish_discovery_entity_(const DetectorState &detector, const char *component, const char *key,
                                 const char *name, const char *value_template, const char *device_class,
                                 const char *entity_category, const char *icon);
  const char *model_name_(const DetectorState &detector) const;
  const char *alarm_device_class_(const DetectorState &detector) const;
  void format_detector_topic_(char *buffer, size_t length, const DetectorState &detector, const char *suffix) const;

  gpio_num_t sclk_pin_{GPIO_NUM_NC};
  gpio_num_t mosi_pin_{GPIO_NUM_NC};
  gpio_num_t miso_pin_{GPIO_NUM_NC};
  gpio_num_t cs_pin_{GPIO_NUM_NC};
  gpio_num_t irq_pin_{GPIO_NUM_NC};
  gpio_glitch_filter_handle_t sclk_filter_{nullptr};
  QueueHandle_t event_queue_{nullptr};
  TaskHandle_t radio_task_handle_{nullptr};
  binary_sensor::BinarySensor *initialized_sensor_{nullptr};
  text_sensor::TextSensor *last_packet_sensor_{nullptr};
  text_sensor::TextSensor *last_device_sensor_{nullptr};
  text_sensor::TextSensor *last_model_sensor_{nullptr};
  text_sensor::TextSensor *last_event_sensor_{nullptr};
  text_sensor::TextSensor *last_result_sensor_{nullptr};
  text_sensor::TextSensor *last_base_sensor_{nullptr};
  text_sensor::TextSensor *last_battery_sensor_{nullptr};
  std::string discovery_prefix_{"homeassistant"};
  uint8_t max_detectors_{16};
  uint32_t bridge_device_id_{0xA5B813};
  DetectorState detectors_[MAX_DETECTORS]{};
  uint8_t detector_count_{0};
  ESPPreferenceObject inventory_pref_{};
  bool mqtt_was_connected_{false};
  bool mqtt_resync_pending_{false};
  uint8_t mqtt_resync_index_{0};
  uint32_t mqtt_next_sync_ms_{0};
};

}  // namespace esphome::wisafe2
