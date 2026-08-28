#include "wisafe2.h"

#include <cstdio>
#include <cstring>
#include <string>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_private/spi_slave_internal.h"
#include "esp_rom_sys.h"
#include "esphome/core/log.h"

namespace esphome::wisafe2 {

static const char *const TAG = "wisafe2";
static constexpr spi_host_device_t SPI_HOST_USED = SPI2_HOST;
#if CONFIG_FREERTOS_UNICORE
static constexpr BaseType_t RADIO_TASK_CORE = 0;
#else
static constexpr BaseType_t RADIO_TASK_CORE = 1;
#endif

void WiSafe2Component::set_pins(int sclk, int mosi, int miso, int cs, int irq) {
  this->sclk_pin_ = static_cast<gpio_num_t>(sclk);
  this->mosi_pin_ = static_cast<gpio_num_t>(mosi);
  this->miso_pin_ = static_cast<gpio_num_t>(miso);
  this->cs_pin_ = static_cast<gpio_num_t>(cs);
  this->irq_pin_ = static_cast<gpio_num_t>(irq);
}

void WiSafe2Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up WiSafe2 radio...");
  if (this->initialized_sensor_ != nullptr)
    this->initialized_sensor_->publish_initial_state(false);

  this->event_queue_ = xQueueCreate(8, sizeof(RadioEvent));
  if (this->event_queue_ == nullptr) {
    ESP_LOGE(TAG, "Could not allocate radio event queue");
    this->mark_failed();
    return;
  }

  // Keep SPI initialization and all subsequent driver calls on one core. This
  // also keeps the WiSafe timing path independent of ESPHome's main loop.
  if (xTaskCreatePinnedToCore(WiSafe2Component::radio_task_entry, "wisafe2_radio", 8192, this, 5,
                              &this->radio_task_handle_, RADIO_TASK_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Could not create radio task");
    vQueueDelete(this->event_queue_);
    this->event_queue_ = nullptr;
    this->mark_failed();
  }
}

void WiSafe2Component::loop() {
  if (this->event_queue_ == nullptr)
    return;

  RadioEvent event{};
  while (xQueueReceive(this->event_queue_, &event, 0) == pdTRUE) {
    if (event.type == EventType::INITIALIZED) {
      if (this->initialized_sensor_ != nullptr)
        this->initialized_sensor_->publish_state(true);
      this->status_clear_error();
    } else if (event.type == EventType::ERROR) {
      if (this->initialized_sensor_ != nullptr)
        this->initialized_sensor_->publish_state(false);
      this->status_set_error();
    } else if (event.type == EventType::PACKET) {
      char line[PACKET_MAX * 3]{};
      size_t pos = 0;
      for (size_t i = 0; i < event.length && pos + 3 < sizeof(line); ++i) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X%s", event.packet[i],
                        i + 1 < event.length ? " " : "");
      }
      if (this->last_packet_sensor_ != nullptr)
        this->last_packet_sensor_->publish_state(line);
    }
  }
}

void WiSafe2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "WiSafe2 radio:");
  ESP_LOGCONFIG(TAG, "  Mode: ESP32 SPI slave (radio is master)");
  ESP_LOGCONFIG(TAG, "  SCLK: GPIO%d", this->sclk_pin_);
  ESP_LOGCONFIG(TAG, "  MOSI: GPIO%d", this->mosi_pin_);
  ESP_LOGCONFIG(TAG, "  MISO: GPIO%d", this->miso_pin_);
  ESP_LOGCONFIG(TAG, "  CS: GPIO%d", this->cs_pin_);
  ESP_LOGCONFIG(TAG, "  IRQ: GPIO%d", this->irq_pin_);
}

void WiSafe2Component::radio_task_entry(void *parameter) {
  static_cast<WiSafe2Component *>(parameter)->radio_task_();
}

void WiSafe2Component::radio_task_() {
  esp_err_t err = this->init_gpio_();
  if (err == ESP_OK)
    err = this->init_spi_slave_();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Radio hardware setup failed: %s", esp_err_to_name(err));
    this->emit_event_(EventType::ERROR);
    this->radio_task_handle_ = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG, "Waiting 5 seconds for the radio to stabilise...");
  vTaskDelay(pdMS_TO_TICKS(5000));
  if (!this->initialize_radio_()) {
    this->emit_event_(EventType::ERROR);
    this->radio_task_handle_ = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  this->emit_event_(EventType::INITIALIZED);
  ESP_LOGI(TAG, "Entering receive mode");
  this->raw_receive_loop_();
  this->emit_event_(EventType::ERROR);
  this->radio_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

esp_err_t WiSafe2Component::init_gpio_() {
  gpio_config_t irq_config{};
  irq_config.pin_bit_mask = 1ULL << this->irq_pin_;
  irq_config.mode = GPIO_MODE_OUTPUT;
  irq_config.pull_up_en = GPIO_PULLUP_DISABLE;
  irq_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  irq_config.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&irq_config), TAG, "IRQ GPIO config failed");
  this->irq_set_(false);

  gpio_pin_glitch_filter_config_t filter_config{};
  filter_config.clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT;
  filter_config.gpio_num = this->sclk_pin_;
  ESP_RETURN_ON_ERROR(gpio_new_pin_glitch_filter(&filter_config, &this->sclk_filter_), TAG,
                      "SCLK glitch filter allocation failed");
  ESP_RETURN_ON_ERROR(gpio_glitch_filter_enable(this->sclk_filter_), TAG,
                      "SCLK glitch filter enable failed");
  return ESP_OK;
}

esp_err_t WiSafe2Component::init_spi_slave_() {
  spi_bus_config_t bus_config{};
  bus_config.mosi_io_num = this->mosi_pin_;
  bus_config.miso_io_num = this->miso_pin_;
  bus_config.sclk_io_num = this->sclk_pin_;
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.data4_io_num = -1;
  bus_config.data5_io_num = -1;
  bus_config.data6_io_num = -1;
  bus_config.data7_io_num = -1;
  bus_config.max_transfer_sz = SPI_SLOT_BITS / 8;
  bus_config.flags = SPICOMMON_BUSFLAG_SLAVE;

  spi_slave_interface_config_t slave_config{};
  slave_config.spics_io_num = this->cs_pin_;
  slave_config.queue_size = 3;
  slave_config.mode = 0;

  ESP_RETURN_ON_ERROR(spi_slave_initialize(SPI_HOST_USED, &bus_config, &slave_config, SPI_DMA_DISABLED),
                      TAG, "SPI slave init failed");
  ESP_RETURN_ON_ERROR(gpio_set_drive_capability(this->miso_pin_, GPIO_DRIVE_CAP_0), TAG,
                      "MISO drive-strength configuration failed");
  return ESP_OK;
}

void WiSafe2Component::irq_set_(bool asserted) { gpio_set_level(this->irq_pin_, asserted ? 1 : 0); }

void WiSafe2Component::acknowledge_received_byte_() {
  this->irq_set_(true);
  esp_rom_delay_us(RX_ACK_PULSE_US);
  this->irq_set_(false);
}

void WiSafe2Component::prepare_slot_(SpiSlot *slot, uint8_t tx_value) {
  memset(slot, 0, sizeof(*slot));
  reinterpret_cast<uint8_t *>(&slot->tx_word)[0] = tx_value;
  slot->transaction.length = SPI_SLOT_BITS;
  slot->transaction.tx_buffer = &slot->tx_word;
  slot->transaction.rx_buffer = &slot->rx_word;
}

uint8_t WiSafe2Component::slot_rx_byte_(const SpiSlot *slot) const {
  return reinterpret_cast<const uint8_t *>(&slot->rx_word)[0];
}

esp_err_t WiSafe2Component::queue_slot_(SpiSlot *slot, uint8_t tx_value) {
  this->prepare_slot_(slot, tx_value);
  return spi_slave_queue_trans(SPI_HOST_USED, &slot->transaction, pdMS_TO_TICKS(BYTE_TIMEOUT_MS));
}

esp_err_t WiSafe2Component::wait_for_slot_(SpiSlot *expected, TickType_t timeout) {
  spi_slave_transaction_t *completed = nullptr;
  esp_err_t err = spi_slave_get_trans_result(SPI_HOST_USED, &completed, timeout);
  if (err != ESP_OK)
    return err;
  if (completed != &expected->transaction) {
    ESP_LOGE(TAG, "Unexpected SPI transaction completed");
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t WiSafe2Component::reset_spi_slave_() {
  this->irq_set_(false);
  for (int i = 0; i < 100 && gpio_get_level(this->cs_pin_) == 0; ++i)
    esp_rom_delay_us(100);
  if (gpio_get_level(this->cs_pin_) == 0) {
    ESP_LOGE(TAG, "Cannot reset SPI slave while CS remains asserted");
    return ESP_ERR_INVALID_STATE;
  }
  return spi_slave_queue_reset(SPI_HOST_USED);
}

esp_err_t WiSafe2Component::abort_exchange_(esp_err_t cause, ExchangeResult *result) {
  esp_err_t reset_error = this->reset_spi_slave_();
  result->spi_reset = reset_error == ESP_OK;
  return reset_error == ESP_OK ? cause : reset_error;
}

esp_err_t WiSafe2Component::exchange_packet_(const uint8_t *data, size_t length, ExchangeResult *result,
                                             unsigned response_packets, uint32_t response_timeout_ms) {
  if (data == nullptr || result == nullptr || length == 0 || length > PACKET_MAX || response_packets == 0)
    return ESP_ERR_INVALID_ARG;

  memset(result, 0, sizeof(*result));
  SpiSlot tx_slot{};
  SpiSlot rx_slots[2]{};
  bool first_rx_queued = false;

  for (size_t i = 0; i < length; ++i) {
    esp_err_t err = this->queue_slot_(&tx_slot, data[i]);
    if (err != ESP_OK)
      return this->abort_exchange_(err, result);

    if (i + 1 == length) {
      err = this->queue_slot_(&rx_slots[0], 0x00);
      if (err != ESP_OK)
        return this->abort_exchange_(err, result);
      first_rx_queued = true;
    }

    this->irq_set_(true);
    err = this->wait_for_slot_(&tx_slot, pdMS_TO_TICKS(BYTE_TIMEOUT_MS));
    this->irq_set_(false);
    if (err != ESP_OK)
      return this->abort_exchange_(err, result);

    result->tx_bits[i] = tx_slot.transaction.trans_len;
    result->simultaneous_rx[i] = this->slot_rx_byte_(&tx_slot);
    result->tx_count = i + 1;
  }

  if (!first_rx_queued)
    return ESP_FAIL;

  TickType_t start = xTaskGetTickCount();
  TickType_t overall = pdMS_TO_TICKS(response_timeout_ms);
  unsigned current = 0;
  unsigned packets_received = 0;
  while (result->response_len < PACKET_MAX) {
    TickType_t elapsed = xTaskGetTickCount() - start;
    if (elapsed >= overall)
      return this->abort_exchange_(ESP_ERR_TIMEOUT, result);

    esp_err_t err = this->wait_for_slot_(&rx_slots[current], overall - elapsed);
    if (err != ESP_OK)
      return this->abort_exchange_(err, result);

    uint8_t value = this->slot_rx_byte_(&rx_slots[current]);
    size_t index = result->response_len++;
    result->response[index] = value;
    result->response_bits[index] = rx_slots[current].transaction.trans_len;
    if (value == 0x7E && ++packets_received == response_packets) {
      this->acknowledge_received_byte_();
      return ESP_OK;
    }

    unsigned next = current ^ 1U;
    err = this->queue_slot_(&rx_slots[next], 0x00);
    if (err != ESP_OK)
      return this->abort_exchange_(err, result);
    this->acknowledge_received_byte_();
    current = next;
  }
  return this->abort_exchange_(ESP_ERR_NO_MEM, result);
}

bool WiSafe2Component::response_contains_packet_(const ExchangeResult *result, const uint8_t *expected,
                                                 size_t expected_len) const {
  size_t packet_start = 0;
  for (size_t i = 0; i < result->response_len; ++i) {
    if (result->response[i] != 0x7E)
      continue;
    size_t packet_len = i - packet_start + 1;
    if (packet_len == expected_len && memcmp(&result->response[packet_start], expected, expected_len) == 0)
      return true;
    packet_start = i + 1;
  }
  return false;
}

bool WiSafe2Component::initialize_radio_() {
  static const uint8_t init_command[] = {0xD3, 0x19, 0x50, 0x00, 0x7E};
  static const uint8_t init_ok[] = {0x46, 0x7E};
  this->log_packet_("INIT TX: ", init_command, sizeof(init_command));

  static ExchangeResult result;
  for (unsigned attempt = 1; attempt <= INIT_MAX_ATTEMPTS; ++attempt) {
    ESP_LOGI(TAG, "Radio initialisation attempt %u/%u", attempt, INIT_MAX_ATTEMPTS);
    esp_err_t err = this->exchange_packet_(init_command, sizeof(init_command), &result, 4, BYTE_TIMEOUT_MS);
    this->log_exchange_diagnostics_(&result);
    if (result.response_len > 0)
      this->log_packet_("INIT RX: ", result.response, result.response_len);
    if (this->response_contains_packet_(&result, init_ok, sizeof(init_ok))) {
      ESP_LOGI(TAG, "Radio initialised: received 46 7E");
      return true;
    }
    if (!result.spi_reset && this->reset_spi_slave_() != ESP_OK)
      return false;
    if (attempt < INIT_MAX_ATTEMPTS)
      vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
    (void) err;
  }
  ESP_LOGE(TAG, "Radio initialisation failed after %u attempts", INIT_MAX_ATTEMPTS);
  return false;
}

void WiSafe2Component::raw_receive_loop_() {
  SpiSlot slots[2]{};
  unsigned current = 0;
  if (this->queue_slot_(&slots[current], 0x00) != ESP_OK)
    return;

  uint8_t packet[PACKET_MAX]{};
  size_t used = 0;
  while (true) {
    esp_err_t err = this->wait_for_slot_(&slots[current], pdMS_TO_TICKS(BYTE_TIMEOUT_MS));
    if (err == ESP_ERR_TIMEOUT)
      continue;
    if (err != ESP_OK) {
      this->reset_spi_slave_();
      return;
    }

    uint8_t value = this->slot_rx_byte_(&slots[current]);
    size_t bits = slots[current].transaction.trans_len;
    unsigned next = current ^ 1U;
    if (this->queue_slot_(&slots[next], 0x00) != ESP_OK) {
      this->reset_spi_slave_();
      return;
    }
    this->acknowledge_received_byte_();
    current = next;

    if (bits != 8)
      ESP_LOGW(TAG, "RX transaction used %u clocks (expected 8)", static_cast<unsigned>(bits));
    if (used < sizeof(packet))
      packet[used++] = value;
    else
      used = 0;

    if (value == 0x7E) {
      this->log_packet_("PACKET: ", packet, used);
      this->emit_event_(EventType::PACKET, packet, used);
      used = 0;
    }
  }
}

void WiSafe2Component::log_exchange_diagnostics_(const ExchangeResult *result) const {
  for (size_t i = 0; i < result->tx_count; ++i) {
    if (result->tx_bits[i] != 8)
      ESP_LOGW(TAG, "TX byte %u used %u clocks (expected 8)", static_cast<unsigned>(i),
               static_cast<unsigned>(result->tx_bits[i]));
  }
  for (size_t i = 0; i < result->response_len; ++i) {
    if (result->response_bits[i] != 8)
      ESP_LOGW(TAG, "RX byte %u used %u clocks (expected 8)", static_cast<unsigned>(i),
               static_cast<unsigned>(result->response_bits[i]));
  }
}

void WiSafe2Component::log_packet_(const char *prefix, const uint8_t *data, size_t length) const {
  char line[PACKET_MAX * 3]{};
  size_t pos = 0;
  for (size_t i = 0; i < length && pos + 3 < sizeof(line); ++i)
    pos += snprintf(line + pos, sizeof(line) - pos, "%02X%s", data[i], i + 1 < length ? " " : "");
  ESP_LOGI(TAG, "%s%s", prefix, line);
}

void WiSafe2Component::emit_event_(EventType type, const uint8_t *packet, size_t length) {
  if (this->event_queue_ == nullptr)
    return;
  RadioEvent event{};
  event.type = type;
  event.length = length > PACKET_MAX ? PACKET_MAX : length;
  if (packet != nullptr && event.length > 0)
    memcpy(event.packet, packet, event.length);
  if (xQueueSend(this->event_queue_, &event, 0) != pdTRUE)
    ESP_LOGW(TAG, "Radio event queue full; dropping event");
}

}  // namespace esphome::wisafe2
