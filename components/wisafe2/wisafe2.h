#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/spi_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome::wisafe2 {

class WiSafe2Component : public Component {
 public:
  void set_pins(int sclk, int mosi, int miso, int cs, int irq);
  void set_initialized_sensor(binary_sensor::BinarySensor *sensor) { this->initialized_sensor_ = sensor; }
  void set_last_packet_sensor(text_sensor::TextSensor *sensor) { this->last_packet_sensor_ = sensor; }

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
};

}  // namespace esphome::wisafe2

