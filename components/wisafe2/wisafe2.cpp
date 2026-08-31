#include "wisafe2.h"

#include <cstdio>
#include <cstring>
#include <string>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_private/spi_slave_internal.h"
#include "esp_rom_sys.h"
#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::wisafe2 {

static const char *const TAG = "wisafe2";
static constexpr spi_host_device_t SPI_HOST_USED = SPI2_HOST;
static constexpr uint8_t MAX_EVENTS_PER_LOOP = 4;
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
  this->load_inventory_();
  this->load_test_history_();
  if (this->initialized_sensor_ != nullptr)
    this->initialized_sensor_->publish_initial_state(false);
  if (this->command_busy_sensor_ != nullptr)
    this->command_busy_sensor_->publish_initial_state(false);

  // Initial SID discovery can produce an identity and a status frame for every
  // configured detector before ESPHome's main loop gets a chance to drain it.
  this->event_queue_ = xQueueCreate(this->max_detectors_ * 2 + 8, sizeof(RadioEvent));
  this->command_queue_ = xQueueCreate(1, sizeof(ManagementCommand));
  if (this->event_queue_ == nullptr || this->command_queue_ == nullptr) {
    ESP_LOGE(TAG, "Could not allocate radio queues");
    if (this->event_queue_ != nullptr)
      vQueueDelete(this->event_queue_);
    if (this->command_queue_ != nullptr)
      vQueueDelete(this->command_queue_);
    this->event_queue_ = nullptr;
    this->command_queue_ = nullptr;
    this->mark_failed();
    return;
  }

  // Keep SPI initialization and all subsequent driver calls on one core. This
  // also keeps the WiSafe timing path independent of ESPHome's main loop.
  if (xTaskCreatePinnedToCore(WiSafe2Component::radio_task_entry, "wisafe2_radio", 8192, this, 5,
                              &this->radio_task_handle_, RADIO_TASK_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Could not create radio task");
    vQueueDelete(this->event_queue_);
    vQueueDelete(this->command_queue_);
    this->event_queue_ = nullptr;
    this->command_queue_ = nullptr;
    this->mark_failed();
  }
}

void WiSafe2Component::request_command(ManagementCommand command) {
  const char *name = management_command_name(command);
  if (!this->radio_ready_ || this->command_queue_ == nullptr) {
    ESP_LOGW(TAG, "%s rejected: radio is not ready", name);
    if (this->last_command_sensor_ != nullptr)
      this->last_command_sensor_->publish_state(std::string(name) + ": RADIO NOT READY");
    return;
  }
  if (this->command_busy_ || xQueueSend(this->command_queue_, &command, 0) != pdTRUE) {
    ESP_LOGW(TAG, "%s rejected: another command is running", name);
    if (this->last_command_sensor_ != nullptr)
      this->last_command_sensor_->publish_state(std::string(name) + ": BUSY");
    return;
  }
  this->command_busy_ = true;
  if (this->command_busy_sensor_ != nullptr)
    this->command_busy_sensor_->publish_state(true);
  if (this->last_command_sensor_ != nullptr)
    this->last_command_sensor_->publish_state(std::string(name) + ": QUEUED");
}

void WiSafe2CommandButton::press_action() {
  if (this->parent_ != nullptr)
    this->parent_->request_command(this->command_);
}

void WiSafe2Component::loop() {
  if (this->event_queue_ == nullptr)
    return;

  RadioEvent event{};
  uint8_t processed = 0;
  // Yield to ESPHome after a small batch so packet bursts cannot monopolize
  // the main loop. The dedicated radio task continues servicing SPI and the
  // queue retains the remaining events for the next loop invocation.
  while (processed < MAX_EVENTS_PER_LOOP && xQueueReceive(this->event_queue_, &event, 0) == pdTRUE) {
    ++processed;
    if (event.type == EventType::INITIALIZED) {
      if (this->initialized_sensor_ != nullptr)
        this->initialized_sensor_->publish_state(true);
      this->status_clear_error();
    } else if (event.type == EventType::ERROR) {
      if (this->initialized_sensor_ != nullptr)
        this->initialized_sensor_->publish_state(false);
      this->status_set_error();
      this->command_busy_ = false;
      if (this->command_busy_sensor_ != nullptr)
        this->command_busy_sensor_->publish_state(false);
    } else if (event.type == EventType::COMMAND_RESULT) {
      const char *result = "ERROR";
      switch (event.outcome) {
        case CommandOutcome::ACCEPTED: result = "ACCEPTED"; break;
        case CommandOutcome::TIMEOUT: result = "TIMEOUT"; break;
        case CommandOutcome::PAIRED: result = "PAIRED"; break;
        case CommandOutcome::UNPAIRED: result = "UNPAIRED"; break;
        case CommandOutcome::ALREADY_PAIRED: result = "ALREADY PAIRED"; break;
        case CommandOutcome::NO_DETECTORS: result = "NO DETECTORS"; break;
        case CommandOutcome::ERROR: result = "ERROR"; break;
      }
      const char *name = management_command_name(event.command);
      ESP_LOGI(TAG, "%s: %s", name, result);
      if (this->last_command_sensor_ != nullptr)
        this->last_command_sensor_->publish_state(std::string(name) + ": " + result);
      if (event.outcome == CommandOutcome::PAIRED || event.outcome == CommandOutcome::ALREADY_PAIRED) {
        if (this->paired_sensor_ != nullptr)
          this->paired_sensor_->publish_state(true);
      } else if (event.outcome == CommandOutcome::UNPAIRED) {
        if (this->paired_sensor_ != nullptr)
          this->paired_sensor_->publish_state(false);
      }
      this->command_busy_ = false;
      if (this->command_busy_sensor_ != nullptr)
        this->command_busy_sensor_->publish_state(false);
    } else if (event.type == EventType::PACKET) {
      char line[PACKET_MAX * 3]{};
      size_t pos = 0;
      for (size_t i = 0; i < event.length && pos + 3 < sizeof(line); ++i) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X%s", event.packet[i],
                        i + 1 < event.length ? " " : "");
      }
      if (this->last_packet_sensor_ != nullptr)
        this->last_packet_sensor_->publish_state(line);

      DecodedPacket decoded{};
      if (decode_packet(event.packet, event.length, &decoded)) {
        ESP_LOGI(TAG, "Decoded device=%s model=%s event=%s result=%s base=%s battery=%s", decoded.device,
                 decoded.model, decoded.event, decoded.result, decoded.base, decoded.battery);
        if (this->last_device_sensor_ != nullptr)
          this->last_device_sensor_->publish_state(decoded.device);
        if (this->last_model_sensor_ != nullptr)
          this->last_model_sensor_->publish_state(decoded.model);
        if (this->last_event_sensor_ != nullptr)
          this->last_event_sensor_->publish_state(decoded.event);
        if (this->last_result_sensor_ != nullptr)
          this->last_result_sensor_->publish_state(decoded.result);
        if (this->last_base_sensor_ != nullptr)
          this->last_base_sensor_->publish_state(decoded.base);
        if (this->last_battery_sensor_ != nullptr)
          this->last_battery_sensor_->publish_state(decoded.battery);
        this->update_detector_(decoded, line);
      } else {
        RadioDiagnostic diagnostic{};
        if (decode_radio_diagnostic(event.packet, event.length, &diagnostic)) {
          ESP_LOGI(TAG,
                   "Radio diagnostic device=%06X sid=%u connected=%s flags=0x%02X faults=%u "
                   "battery=%02X/%02X rssi=0x%02X firmware=0x%02X",
                   static_cast<unsigned>(diagnostic.device_id), diagnostic.sid,
                   diagnostic.connected ? "yes" : "no", diagnostic.flags, diagnostic.radio_fault_count,
                   diagnostic.battery_primary, diagnostic.battery_radio, diagnostic.rssi,
                   diagnostic.firmware_version);
          char device[7]{};
          snprintf(device, sizeof(device), "%06X", static_cast<unsigned>(diagnostic.device_id));
          if (this->last_device_sensor_ != nullptr)
            this->last_device_sensor_->publish_state(device);
          if (this->last_model_sensor_ != nullptr)
            this->last_model_sensor_->publish_state("N/A");
          if (this->last_event_sensor_ != nullptr)
            this->last_event_sensor_->publish_state("RADIO DIAGNOSTIC");
          if (this->last_result_sensor_ != nullptr)
            this->last_result_sensor_->publish_state("N/A");
          if (this->last_base_sensor_ != nullptr)
            this->last_base_sensor_->publish_state("N/A");
          if (this->last_battery_sensor_ != nullptr)
            this->last_battery_sensor_->publish_state("N/A");
          if (this->paired_sensor_ != nullptr)
            this->paired_sensor_->publish_state(diagnostic.connected);
        } else {
          uint64_t sid_map = 0;
          if (decode_sid_map(event.packet, event.length, &sid_map)) {
            ESP_LOGI(TAG, "Radio SID map: 0x%016llX", static_cast<unsigned long long>(sid_map));
            if (this->paired_sensor_ != nullptr)
              this->paired_sensor_->publish_state(sid_map != 0);
            this->update_detector_membership_(sid_map);
          } else {
            RemoteDiagnostic remote{};
            if (decode_remote_diagnostic(event.packet, event.length, &remote)) {
              ESP_LOGI(TAG,
                       "Remote diagnostic device=%06X sid=%u flags=0x%02X faults=%u "
                       "battery=%02X/%02X rssi=0x%02X firmware=0x%02X",
                       static_cast<unsigned>(remote.device_id), remote.sid, remote.flags,
                       remote.radio_fault_count, remote.battery_primary, remote.battery_radio,
                       remote.rssi, remote.firmware_version);
              this->update_remote_diagnostic_(remote, line);
            } else {
              uint8_t sid = 0;
              uint32_t device_id = 0;
              if (decode_new_device(event.packet, event.length, &sid, &device_id))
                ESP_LOGI(TAG, "Radio announced new device %06X at SID %u",
                         static_cast<unsigned>(device_id), sid);
            }
          }
        }
      }
    }
  }
  this->service_mqtt_();
}

void WiSafe2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "WiSafe2 radio:");
  ESP_LOGCONFIG(TAG, "  Mode: ESP32 SPI slave (radio is master)");
  ESP_LOGCONFIG(TAG, "  SCLK: GPIO%d", this->sclk_pin_);
  ESP_LOGCONFIG(TAG, "  MOSI: GPIO%d", this->mosi_pin_);
  ESP_LOGCONFIG(TAG, "  MISO: GPIO%d", this->miso_pin_);
  ESP_LOGCONFIG(TAG, "  CS: GPIO%d", this->cs_pin_);
  ESP_LOGCONFIG(TAG, "  IRQ: GPIO%d", this->irq_pin_);
  ESP_LOGCONFIG(TAG, "  MQTT discovery prefix: %s", this->discovery_prefix_.c_str());
  ESP_LOGCONFIG(TAG, "  Discovered detectors: %u/%u", this->detector_count_, this->max_detectors_);
  ESP_LOGCONFIG(TAG, "  Bridge identity: %06X model %04X", static_cast<unsigned>(this->bridge_device_id_),
                this->bridge_model_id_);
  ESP_LOGCONFIG(TAG, "  Last-test clock: %s", this->time_ != nullptr ? "configured" : "missing");
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

  this->radio_ready_ = true;
  this->emit_event_(EventType::INITIALIZED);
  ESP_LOGI(TAG, "Entering receive mode");
  this->raw_receive_loop_();
  this->radio_ready_ = false;
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

void WiSafe2Component::wait_for_radio_command_gap_() {
  if (!this->has_completed_radio_frame_)
    return;
  const TickType_t gap = pdMS_TO_TICKS(RADIO_COMMAND_GAP_MS);
  const TickType_t elapsed = xTaskGetTickCount() - this->last_radio_frame_tick_;
  if (elapsed < gap)
    vTaskDelay(gap - elapsed);
}

void WiSafe2Component::note_radio_frame_complete_() {
  this->last_radio_frame_tick_ = xTaskGetTickCount();
  this->has_completed_radio_frame_ = true;
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
  uint8_t wire_data[WIRE_PACKET_MAX]{};
  size_t wire_length = 0;
  if (!escape_frame(data, length, wire_data, sizeof(wire_data), &wire_length))
    return ESP_ERR_INVALID_ARG;
  result->tx_expected = wire_length;
  this->wait_for_radio_command_gap_();
  SpiSlot tx_slot{};
  SpiSlot rx_slots[2]{};
  bool first_rx_queued = false;

  for (size_t i = 0; i < wire_length; ++i) {
    esp_err_t err = this->queue_slot_(&tx_slot, wire_data[i]);
    if (err != ESP_OK)
      return this->abort_exchange_(err, result);

    if (i + 1 == wire_length) {
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
  this->note_radio_frame_complete_();

  if (!first_rx_queued)
    return ESP_FAIL;

  TickType_t start = xTaskGetTickCount();
  TickType_t overall = pdMS_TO_TICKS(response_timeout_ms);
  unsigned current = 0;
  unsigned packets_received = 0;
  bool escape_pending = false;
  size_t escape_bits = 0;
  while (result->response_len < PACKET_MAX) {
    TickType_t elapsed = xTaskGetTickCount() - start;
    if (elapsed >= overall)
      return this->abort_exchange_(ESP_ERR_TIMEOUT, result);

    esp_err_t err = this->wait_for_slot_(&rx_slots[current], overall - elapsed);
    if (err != ESP_OK)
      return this->abort_exchange_(err, result);

    uint8_t value = this->slot_rx_byte_(&rx_slots[current]);
    const bool terminator = !escape_pending && value == 0x7E;
    const size_t bits = rx_slots[current].transaction.trans_len;
    bool append = true;
    if (escape_pending) {
      if (value == 0x01)
        value = 0x7E;
      else if (value == 0x02)
        value = 0x7D;
      else
        return this->abort_exchange_(ESP_ERR_INVALID_RESPONSE, result);
      escape_pending = false;
    } else if (value == 0x7D) {
      escape_pending = true;
      escape_bits = bits;
      append = false;
    }
    if (append) {
      size_t index = result->response_len++;
      result->response[index] = value;
      result->response_bits[index] = escape_bits > bits ? escape_bits : bits;
      escape_bits = 0;
    }
    if (terminator && ++packets_received == response_packets) {
      this->note_radio_frame_complete_();
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
    size_t compare_start = packet_start;
    while (compare_start < i && result->response[compare_start] == 0x00)
      ++compare_start;
    size_t packet_len = i - compare_start + 1;
    if (packet_len == expected_len && memcmp(&result->response[compare_start], expected, expected_len) == 0)
      return true;
    packet_start = i + 1;
  }
  return false;
}

bool WiSafe2Component::response_pairing_state_(const ExchangeResult *result, bool *paired) const {
  if (result == nullptr || paired == nullptr)
    return false;
  size_t packet_start = 0;
  for (size_t i = 0; i < result->response_len; ++i) {
    if (result->response[i] != 0x7E)
      continue;
    while (packet_start < i && result->response[packet_start] == 0x00)
      ++packet_start;
    const size_t packet_length = i - packet_start + 1;
    uint64_t sid_map = 0;
    if (decode_sid_map(&result->response[packet_start], packet_length, &sid_map)) {
      *paired = sid_map != 0;
      return true;
    }
    packet_start = i + 1;
  }
  return false;
}

bool WiSafe2Component::send_expect_(const uint8_t *data, size_t length, const uint8_t *expected,
                                    size_t expected_length, unsigned expected_packets, unsigned attempts) {
  ExchangeResult &result = this->exchange_scratch_;
  for (unsigned attempt = 0; attempt < attempts; ++attempt) {
    esp_err_t err = this->exchange_packet_(data, length, &result, expected_packets, COMMAND_RESPONSE_TIMEOUT_MS);
    this->log_exchange_diagnostics_(&result);
    if (result.response_len > 0)
      this->log_packet_("COMMAND RX: ", result.response, result.response_len);
    if (err == ESP_OK && this->response_contains_packet_(&result, expected, expected_length)) {
      return true;
    }
    if (!result.spi_reset && this->reset_spi_slave_() != ESP_OK)
      break;
    if (attempt + 1 < attempts)
      vTaskDelay(pdMS_TO_TICKS(COMMAND_RETRY_DELAY_MS));
  }
  return false;
}

bool WiSafe2Component::transmit_packet_(const uint8_t *data, size_t length) {
  ExchangeResult &result = this->exchange_scratch_;
  esp_err_t err = this->exchange_packet_(data, length, &result, 1, 250);
  this->log_exchange_diagnostics_(&result);
  if (result.response_len > 0)
    this->log_packet_("COMMAND TX RESPONSE: ", result.response, result.response_len);
  // Broadcast commands do not produce a response from remote detectors. Once
  // every byte was clocked by the donor radio, a receive timeout is expected.
  const bool transmitted = result.tx_count == result.tx_expected;
  if (err != ESP_OK && !result.spi_reset)
    (void) this->reset_spi_slave_();
  return transmitted;
}

bool WiSafe2Component::query_pairing_status_(bool *paired) {
  CommandFrames frames{};
  if (!encode_management_command(ManagementCommand::QUERY_PAIRING, this->bridge_device_id_,
                                 this->bridge_model_id_, &frames))
    return false;

  ExchangeResult &result = this->exchange_scratch_;
  for (unsigned attempt = 0; attempt < COMMAND_MAX_ATTEMPTS; ++attempt) {
    esp_err_t err = this->exchange_packet_(frames.primary, frames.primary_length, &result, 1,
                                           COMMAND_RESPONSE_TIMEOUT_MS);
    this->log_exchange_diagnostics_(&result);
    if (result.response_len > 0)
      this->log_packet_("PAIR STATUS RX: ", result.response, result.response_len);
    if (err == ESP_OK && this->response_pairing_state_(&result, paired))
      return true;
    if (!result.spi_reset && this->reset_spi_slave_() != ESP_OK)
      return false;
    if (attempt + 1 < COMMAND_MAX_ATTEMPTS)
      vTaskDelay(pdMS_TO_TICKS(COMMAND_RETRY_DELAY_MS));
  }
  return false;
}

WiSafe2Component::CommandOutcome WiSafe2Component::start_pairing_() {
  bool paired = false;
  if (!this->query_pairing_status_(&paired))
    return CommandOutcome::TIMEOUT;
  if (paired)
    return CommandOutcome::ALREADY_PAIRED;

  CommandFrames frames{};
  if (!encode_management_command(ManagementCommand::START_PAIRING, this->bridge_device_id_,
                                 this->bridge_model_id_, &frames))
    return CommandOutcome::ERROR;

  static const uint8_t accepted[] = {0x46, 0x7E};
  static const uint8_t ready[] = {0x41, 0x7E};
  bool activated = false;
  ExchangeResult &result = this->exchange_scratch_;
  for (unsigned attempt = 0; attempt < COMMAND_MAX_ATTEMPTS; ++attempt) {
    esp_err_t err = this->exchange_packet_(frames.primary, frames.primary_length, &result, 2,
                                           COMMAND_RESPONSE_TIMEOUT_MS);
    this->log_exchange_diagnostics_(&result);
    if (result.response_len > 0)
      this->log_packet_("PAIR START RX: ", result.response, result.response_len);
    if (err == ESP_OK && this->response_contains_packet_(&result, accepted, sizeof(accepted)) &&
        this->response_contains_packet_(&result, ready, sizeof(ready))) {
      activated = true;
      break;
    }
    if (!result.spi_reset && this->reset_spi_slave_() != ESP_OK)
      return CommandOutcome::ERROR;
    if (attempt + 1 < COMMAND_MAX_ATTEMPTS)
      vTaskDelay(pdMS_TO_TICKS(COMMAND_RETRY_DELAY_MS));
  }
  if (!activated || !this->transmit_packet_(frames.secondary, frames.secondary_length))
    return CommandOutcome::TIMEOUT;

  ESP_LOGI(TAG, "Pairing active for 21 seconds; press the test/pair button on another alarm");
  if (this->receive_window_(PAIRING_WINDOW_MS) != ESP_OK)
    return CommandOutcome::ERROR;
  if (!this->query_pairing_status_(&paired))
    return CommandOutcome::TIMEOUT;
  return paired ? CommandOutcome::PAIRED : CommandOutcome::UNPAIRED;
}

bool WiSafe2Component::respond_to_identity_request_() {
  uint8_t response[11]{};
  size_t length = 0;
  if (!encode_identity_response(this->bridge_device_id_, this->bridge_model_id_, response, sizeof(response),
                                &length))
    return false;
  this->log_packet_("IDENTITY TX: ", response, length);
  return this->transmit_packet_(response, length);
}

void WiSafe2Component::emit_exchange_packets_(const ExchangeResult *result) {
  if (result == nullptr)
    return;
  size_t packet_start = 0;
  for (size_t i = 0; i < result->response_len; ++i) {
    if (result->response[i] != 0x7E)
      continue;
    while (packet_start < i && result->response[packet_start] == 0x00)
      ++packet_start;
    if (packet_start <= i)
      this->emit_event_(EventType::PACKET, &result->response[packet_start], i - packet_start + 1);
    packet_start = i + 1;
  }
}

bool WiSafe2Component::poll_remote_status_(uint8_t sid, bool *awaiting_response) {
  if (awaiting_response == nullptr)
    return false;
  *awaiting_response = false;
  uint8_t request[5]{};
  size_t request_length = 0;
  if (!encode_remote_diagnostic_request(sid, false, request, sizeof(request), &request_length))
    return false;

  ExchangeResult &result = this->exchange_scratch_;
  ESP_LOGI(TAG, "Querying diagnostic status for SID %u", sid);
  esp_err_t err = this->exchange_packet_(request, request_length, &result, 1, COMMAND_RESPONSE_TIMEOUT_MS);
  this->log_exchange_diagnostics_(&result);
  if (result.response_len > 0) {
    this->log_packet_("REMOTE STATUS RX: ", result.response, result.response_len);
    this->emit_exchange_packets_(&result);
  }
  if (err != ESP_OK)
    return false;

  size_t packet_start = 0;
  for (size_t i = 0; i < result.response_len; ++i) {
    if (result.response[i] != 0x7E)
      continue;
    while (packet_start < i && result.response[packet_start] == 0x00)
      ++packet_start;
    RemoteDiagnostic diagnostic{};
    if (packet_start <= i &&
        decode_remote_diagnostic(&result.response[packet_start], i - packet_start + 1, &diagnostic) &&
        diagnostic.sid == sid) {
      this->known_sid_map_ |= uint64_t{1} << sid;
      this->sid_device_ids_[sid] = diagnostic.device_id;
      return true;
    }
    packet_start = i + 1;
  }

  static const uint8_t accepted[] = {0x46, 0x7E};
  *awaiting_response = this->response_contains_packet_(&result, accepted, sizeof(accepted));
  return *awaiting_response;
}

bool WiSafe2Component::discover_sid_(uint8_t sid, bool *awaiting_response) {
  if (awaiting_response == nullptr)
    return false;
  *awaiting_response = false;
  if (sid >= 64 || sid == this->own_sid_)
    return true;
  const uint64_t bit = uint64_t{1} << sid;
  if ((this->known_sid_map_ & bit) != 0)
    return true;

  uint8_t request[5]{};
  size_t request_length = 0;
  if (!encode_remote_diagnostic_request(sid, true, request, sizeof(request), &request_length))
    return false;

  ExchangeResult &result = this->exchange_scratch_;
  ESP_LOGI(TAG, "Querying identity for SID %u", sid);
  esp_err_t err = this->exchange_packet_(request, request_length, &result, 1, COMMAND_RESPONSE_TIMEOUT_MS);
  this->log_exchange_diagnostics_(&result);
  if (result.response_len > 0) {
    this->log_packet_("REMOTE ID RX: ", result.response, result.response_len);
    this->emit_exchange_packets_(&result);
  }
  if (err != ESP_OK)
    return false;

  size_t packet_start = 0;
  for (size_t i = 0; i < result.response_len; ++i) {
    if (result.response[i] != 0x7E)
      continue;
    while (packet_start < i && result.response[packet_start] == 0x00)
      ++packet_start;
    DecodedPacket identity{};
    if (packet_start <= i && decode_packet(&result.response[packet_start], i - packet_start + 1, &identity) &&
        result.response[packet_start] == 0xC4 && identity.has_sid && identity.sid == sid &&
        identity.device_id != 0 && identity.device_id != this->bridge_device_id_) {
      this->known_sid_map_ |= bit;
      this->sid_device_ids_[sid] = identity.device_id;
      this->pending_status_sid_map_ |= bit;
      return true;
    }
    packet_start = i + 1;
  }
  static const uint8_t accepted[] = {0x46, 0x7E};
  if (this->response_contains_packet_(&result, accepted, sizeof(accepted))) {
    ESP_LOGI(TAG, "Identity request for SID %u accepted; awaiting asynchronous C4 response", sid);
    *awaiting_response = true;
    return true;
  }

  ESP_LOGW(TAG, "Identity query for SID %u returned neither acceptance nor a matching C4 frame", sid);
  return false;
}

bool WiSafe2Component::has_remote_work_() const {
  return this->remote_request_type_ != RemoteRequestType::NONE || this->pending_discovery_sid_map_ != 0 ||
         this->pending_status_sid_map_ != 0;
}

bool WiSafe2Component::remote_request_ready_() const {
  if (this->remote_request_type_ == RemoteRequestType::NONE)
    return this->pending_discovery_sid_map_ != 0 || this->pending_status_sid_map_ != 0;
  return xTaskGetTickCount() - this->remote_request_started_tick_ >= pdMS_TO_TICKS(REMOTE_RESPONSE_TIMEOUT_MS);
}

void WiSafe2Component::service_remote_requests_() {
  if (this->own_sid_ < 64) {
    const uint64_t own_bit = uint64_t{1} << this->own_sid_;
    this->pending_discovery_sid_map_ &= ~own_bit;
    this->pending_status_sid_map_ &= ~own_bit;
  }

  if (this->remote_request_type_ != RemoteRequestType::NONE) {
    if (!this->remote_request_ready_())
      return;
    ESP_LOGW(TAG, "Timed out waiting for asynchronous %s response from SID %u",
             this->remote_request_type_ == RemoteRequestType::IDENTITY ? "identity" : "diagnostic",
             this->remote_request_sid_);
    this->remote_request_type_ = RemoteRequestType::NONE;
    this->remote_request_sid_ = 0xFF;
  }

  RemoteRequestType request_type = RemoteRequestType::NONE;
  uint8_t request_sid = 0xFF;
  for (uint8_t sid = 0; sid < 64; ++sid) {
    const uint64_t bit = uint64_t{1} << sid;
    if ((this->pending_status_sid_map_ & bit) != 0) {
      this->pending_status_sid_map_ &= ~bit;
      request_type = RemoteRequestType::STATUS;
      request_sid = sid;
      break;
    }
  }
  if (request_type == RemoteRequestType::NONE) {
    uint8_t known_remote_count = 0;
    for (uint8_t sid = 0; sid < 64; ++sid) {
      if (sid != this->own_sid_ && (this->known_sid_map_ & (uint64_t{1} << sid)) != 0)
        ++known_remote_count;
    }
    if (known_remote_count >= this->max_detectors_ && this->pending_discovery_sid_map_ != 0) {
      ESP_LOGW(TAG, "Ignoring undiscovered SIDs: configured detector limit of %u reached", this->max_detectors_);
      this->pending_discovery_sid_map_ = 0;
    }
    for (uint8_t sid = 0; sid < 64; ++sid) {
      const uint64_t bit = uint64_t{1} << sid;
      if ((this->pending_discovery_sid_map_ & bit) != 0) {
        this->pending_discovery_sid_map_ &= ~bit;
        request_type = RemoteRequestType::IDENTITY;
        request_sid = sid;
        break;
      }
    }
  }
  if (request_type == RemoteRequestType::NONE)
    return;

  bool awaiting_response = false;
  const bool accepted = request_type == RemoteRequestType::STATUS
                            ? this->poll_remote_status_(request_sid, &awaiting_response)
                            : this->discover_sid_(request_sid, &awaiting_response);
  if (!accepted) {
    ESP_LOGW(TAG, "Could not submit %s request for SID %u",
             request_type == RemoteRequestType::IDENTITY ? "identity" : "diagnostic", request_sid);
    return;
  }
  if (awaiting_response) {
    this->remote_request_type_ = request_type;
    this->remote_request_sid_ = request_sid;
    this->remote_request_started_tick_ = xTaskGetTickCount();
  }
}

bool WiSafe2Component::note_discovery_packet_(const uint8_t *packet, size_t length) {
  uint8_t sid = 0;
  uint32_t device_id = 0;
  if (decode_new_device(packet, length, &sid, &device_id)) {
    const uint64_t bit = uint64_t{1} << sid;
    if ((this->known_sid_map_ & bit) != 0 && this->sid_device_ids_[sid] == device_id) {
      ESP_LOGI(TAG, "Known device %06X announced again at SID %u; identity refresh not required",
               static_cast<unsigned>(device_id), sid);
      return false;
    }
    this->known_sid_map_ &= ~bit;
    this->sid_device_ids_[sid] = device_id;
    this->pending_discovery_sid_map_ |= bit;
    return this->remote_request_ready_();
  }

  DecodedPacket identity{};
  if (decode_packet(packet, length, &identity) && packet[0] == 0xC4 && identity.has_sid && identity.sid < 64) {
    const uint64_t bit = uint64_t{1} << identity.sid;
    if (this->remote_request_type_ == RemoteRequestType::IDENTITY && this->remote_request_sid_ == identity.sid) {
      this->remote_request_type_ = RemoteRequestType::NONE;
      this->remote_request_sid_ = 0xFF;
    }
    this->known_sid_map_ |= bit;
    this->sid_device_ids_[identity.sid] = identity.device_id;
    this->pending_status_sid_map_ |= bit;
    return this->remote_request_ready_();
  }

  RemoteDiagnostic diagnostic{};
  if (decode_remote_diagnostic(packet, length, &diagnostic)) {
    const uint64_t bit = uint64_t{1} << diagnostic.sid;
    if (this->remote_request_type_ == RemoteRequestType::STATUS && this->remote_request_sid_ == diagnostic.sid) {
      this->remote_request_type_ = RemoteRequestType::NONE;
      this->remote_request_sid_ = 0xFF;
    }
    this->known_sid_map_ |= bit;
    this->sid_device_ids_[diagnostic.sid] = diagnostic.device_id;
    return this->remote_request_ready_();
  }
  return false;
}

void WiSafe2Component::poll_local_radio_() {
  static const uint8_t diagnostic_request[] = {0xD1, 0x7E};
  static const uint8_t sid_map_request[] = {0xD3, 0x03, 0x7E};
  const struct {
    const char *name;
    const uint8_t *data;
    size_t length;
  } requests[] = {
      {"RADIO DIAGNOSTIC", diagnostic_request, sizeof(diagnostic_request)},
      {"SID MAP", sid_map_request, sizeof(sid_map_request)},
  };

  ExchangeResult &result = this->exchange_scratch_;
  for (const auto &request : requests) {
    esp_err_t err = this->exchange_packet_(request.data, request.length, &result, 1,
                                           COMMAND_RESPONSE_TIMEOUT_MS);
    this->log_exchange_diagnostics_(&result);
    if (result.response_len > 0) {
      char prefix[32]{};
      snprintf(prefix, sizeof(prefix), "%s RX: ", request.name);
      this->log_packet_(prefix, result.response, result.response_len);
      this->emit_exchange_packets_(&result);

      size_t packet_start = 0;
      for (size_t i = 0; i < result.response_len; ++i) {
        if (result.response[i] != 0x7E)
          continue;
        while (packet_start < i && result.response[packet_start] == 0x00)
          ++packet_start;
        const uint8_t *packet = &result.response[packet_start];
        const size_t packet_length = i - packet_start + 1;
        RadioDiagnostic local{};
        if (decode_radio_diagnostic(packet, packet_length, &local))
          this->own_sid_ = local.connected ? local.sid : 0xFF;
        uint64_t sid_map = 0;
        if (decode_sid_map(packet, packet_length, &sid_map)) {
          this->latest_sid_map_ = sid_map;
          this->known_sid_map_ &= sid_map;
          this->pending_discovery_sid_map_ |= sid_map & ~this->known_sid_map_;
        }
        packet_start = i + 1;
      }
    }
    if (err != ESP_OK && !result.spi_reset && this->reset_spi_slave_() != ESP_OK)
      return;
  }
  this->service_remote_requests_();
}

void WiSafe2Component::execute_command_(ManagementCommand command) {
  CommandOutcome outcome = CommandOutcome::ERROR;
  CommandFrames frames{};
  static const uint8_t accepted[] = {0x46, 0x7E};
  static const uint8_t ready[] = {0x41, 0x7E};

  ESP_LOGI(TAG, "Executing %s", management_command_name(command));
  if (command == ManagementCommand::QUERY_PAIRING) {
    bool paired = false;
    outcome = this->query_pairing_status_(&paired) ? (paired ? CommandOutcome::PAIRED : CommandOutcome::UNPAIRED)
                                                   : CommandOutcome::TIMEOUT;
  } else if (command == ManagementCommand::START_PAIRING) {
    outcome = this->start_pairing_();
  } else if (command == ManagementCommand::REFRESH_DETECTORS) {
    // Reconcile the attached radio's current SID map before deciding that no
    // queryable detectors exist. Unknown SIDs start asynchronous C4 discovery;
    // their D4 06 status query is scheduled when that identity arrives.
    this->poll_local_radio_();
    uint64_t remote_sid_map = this->latest_sid_map_;
    if (this->own_sid_ < 64)
      remote_sid_map &= ~(uint64_t{1} << this->own_sid_);
    const uint64_t queryable = remote_sid_map & this->known_sid_map_;
    this->pending_status_sid_map_ |= queryable;
    if (this->remote_request_type_ == RemoteRequestType::STATUS && this->remote_request_sid_ < 64)
      this->pending_status_sid_map_ &= ~(uint64_t{1} << this->remote_request_sid_);
    unsigned queued = 0;
    for (uint8_t sid = 0; sid < 64; ++sid) {
      if ((queryable & (uint64_t{1} << sid)) != 0)
        ++queued;
    }
    ESP_LOGI(TAG, "Manual detector diagnostic refresh queued %u known detector(s)", queued);
    outcome = remote_sid_map != 0 ? CommandOutcome::ACCEPTED : CommandOutcome::NO_DETECTORS;
    this->service_remote_requests_();
  } else if (!encode_management_command(command, this->bridge_device_id_, this->bridge_model_id_, &frames)) {
    outcome = CommandOutcome::ERROR;
  } else if (command == ManagementCommand::SOUND_CO || command == ManagementCommand::SOUND_FIRE ||
             command == ManagementCommand::SOUND_COMBINED) {
    if (this->send_expect_(frames.primary, frames.primary_length, ready, sizeof(ready), 1, 4) &&
        this->transmit_packet_(frames.secondary, frames.secondary_length))
      outcome = CommandOutcome::ACCEPTED;
    else
      outcome = CommandOutcome::TIMEOUT;
  } else {
    outcome = this->send_expect_(frames.primary, frames.primary_length, accepted, sizeof(accepted), 1,
                                 COMMAND_MAX_ATTEMPTS)
                  ? CommandOutcome::ACCEPTED
                  : CommandOutcome::TIMEOUT;
  }
  this->emit_command_result_(command, outcome);
}

bool WiSafe2Component::initialize_radio_() {
  static const uint8_t init_command[] = {0xD3, 0x19, 0x50, 0x00, 0x7E};
  static const uint8_t init_ok[] = {0x46, 0x7E};
  this->log_packet_("INIT TX: ", init_command, sizeof(init_command));

  ExchangeResult &result = this->exchange_scratch_;
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

  uint8_t wire_packet[WIRE_PACKET_MAX]{};
  size_t used = 0;
  bool discarding_overflow = false;
  TickType_t last_local_poll = xTaskGetTickCount();
  while (true) {
    ManagementCommand command{};
    if (!this->has_remote_work_() && xQueueReceive(this->command_queue_, &command, 0) == pdTRUE) {
      if (this->reset_spi_slave_() != ESP_OK)
        return;
      this->execute_command_(command);
      this->service_remote_requests_();
      memset(slots, 0, sizeof(slots));
      current = 0;
      used = 0;
      discarding_overflow = false;
      if (this->queue_slot_(&slots[current], 0x00) != ESP_OK)
        return;
      continue;
    }

    uint32_t timeout_ms = used > 0 ? INCOMPLETE_FRAME_TIMEOUT_MS : COMMAND_POLL_INTERVAL_MS;
    esp_err_t err = this->wait_for_slot_(&slots[current], pdMS_TO_TICKS(timeout_ms));
    if (err == ESP_ERR_TIMEOUT) {
      if (used > 0) {
        ESP_LOGW(TAG, "Discarding incomplete %u-byte radio frame", static_cast<unsigned>(used));
        used = 0;
      } else if (this->remote_request_ready_()) {
        if (this->reset_spi_slave_() != ESP_OK)
          return;
        this->service_remote_requests_();
        memset(slots, 0, sizeof(slots));
        current = 0;
        discarding_overflow = false;
        if (this->queue_slot_(&slots[current], 0x00) != ESP_OK)
          return;
      } else if (!this->has_remote_work_() &&
                 xTaskGetTickCount() - last_local_poll >= pdMS_TO_TICKS(LOCAL_DIAGNOSTIC_INTERVAL_MS)) {
        if (this->reset_spi_slave_() != ESP_OK)
          return;
        this->poll_local_radio_();
        memset(slots, 0, sizeof(slots));
        current = 0;
        discarding_overflow = false;
        last_local_poll = xTaskGetTickCount();
        if (this->queue_slot_(&slots[current], 0x00) != ESP_OK)
          return;
      }
      continue;
    }
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
    if (discarding_overflow) {
      if (value == 0x7E)
        discarding_overflow = false;
      continue;
    }

    if (used >= sizeof(wire_packet)) {
      ESP_LOGW(TAG, "Radio frame exceeded %u bytes; discarding through terminator",
               static_cast<unsigned>(sizeof(wire_packet)));
      used = 0;
      discarding_overflow = value != 0x7E;
      continue;
    }
    wire_packet[used++] = value;

    if (value == 0x7E) {
      this->note_radio_frame_complete_();
      uint8_t packet[PACKET_MAX]{};
      size_t packet_length = 0;
      if (!unescape_frame(wire_packet, used, packet, sizeof(packet), &packet_length)) {
        ESP_LOGW(TAG, "Discarding malformed escaped radio frame");
        used = 0;
        continue;
      }
      this->log_packet_("PACKET: ", packet, packet_length);
      this->emit_event_(EventType::PACKET, packet, packet_length);
      const bool identity_requested = is_identity_request(packet, packet_length);
      const bool discovery_requested = this->note_discovery_packet_(packet, packet_length);
      used = 0;
      if (identity_requested || discovery_requested) {
        if (this->reset_spi_slave_() != ESP_OK)
          return;
        if (identity_requested && !this->respond_to_identity_request_())
          ESP_LOGW(TAG, "Radio identity response was not fully transmitted");
        if (discovery_requested)
          this->service_remote_requests_();
        memset(slots, 0, sizeof(slots));
        current = 0;
        if (this->queue_slot_(&slots[current], 0x00) != ESP_OK)
          return;
      }
    }
  }
}

esp_err_t WiSafe2Component::receive_window_(uint32_t window_ms) {
  SpiSlot slots[2]{};
  unsigned current = 0;
  if (this->queue_slot_(&slots[current], 0x00) != ESP_OK)
    return ESP_FAIL;

  uint8_t wire_packet[WIRE_PACKET_MAX]{};
  size_t used = 0;
  bool discarding_overflow = false;
  const TickType_t start = xTaskGetTickCount();
  const TickType_t overall = pdMS_TO_TICKS(window_ms);
  while (xTaskGetTickCount() - start < overall) {
    TickType_t elapsed = xTaskGetTickCount() - start;
    TickType_t remaining = overall - elapsed;
    esp_err_t err = this->wait_for_slot_(&slots[current], remaining);
    if (err == ESP_ERR_TIMEOUT)
      break;
    if (err != ESP_OK) {
      (void) this->reset_spi_slave_();
      return err;
    }

    const uint8_t value = this->slot_rx_byte_(&slots[current]);
    const size_t bits = slots[current].transaction.trans_len;
    const unsigned next = current ^ 1U;
    err = this->queue_slot_(&slots[next], 0x00);
    if (err != ESP_OK) {
      (void) this->reset_spi_slave_();
      return err;
    }
    this->acknowledge_received_byte_();
    current = next;

    if (bits != 8)
      ESP_LOGW(TAG, "Pairing RX used %u clocks (expected 8)", static_cast<unsigned>(bits));
    if (discarding_overflow) {
      if (value == 0x7E)
        discarding_overflow = false;
      continue;
    }
    if (used >= sizeof(wire_packet)) {
      ESP_LOGW(TAG, "Pairing frame overflow; discarding through terminator");
      used = 0;
      discarding_overflow = value != 0x7E;
      continue;
    }
    wire_packet[used++] = value;
    if (value == 0x7E) {
      this->note_radio_frame_complete_();
      uint8_t packet[PACKET_MAX]{};
      size_t packet_length = 0;
      if (!unescape_frame(wire_packet, used, packet, sizeof(packet), &packet_length)) {
        ESP_LOGW(TAG, "Discarding malformed escaped pairing frame");
        used = 0;
        continue;
      }
      this->log_packet_("PAIRING EVENT: ", packet, packet_length);
      this->emit_event_(EventType::PACKET, packet, packet_length);
      const bool identity_requested = is_identity_request(packet, packet_length);
      (void) this->note_discovery_packet_(packet, packet_length);
      used = 0;
      if (identity_requested) {
        if (this->reset_spi_slave_() != ESP_OK)
          return ESP_FAIL;
        if (!this->respond_to_identity_request_())
          ESP_LOGW(TAG, "Pairing identity response was not fully transmitted");
        memset(slots, 0, sizeof(slots));
        current = 0;
        if (this->queue_slot_(&slots[current], 0x00) != ESP_OK)
          return ESP_FAIL;
      }
    }
  }
  return this->reset_spi_slave_();
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
  // Packet bursts may consume all but one slot. Reserve the final slot for a
  // non-blocking command result so the main loop can always clear busy state.
  if (type == EventType::PACKET && uxQueueSpacesAvailable(this->event_queue_) <= 1) {
    ESP_LOGW(TAG, "Radio event queue nearly full; dropping packet");
    return;
  }
  RadioEvent event{};
  event.type = type;
  event.length = length > PACKET_MAX ? PACKET_MAX : length;
  if (packet != nullptr && event.length > 0)
    memcpy(event.packet, packet, event.length);
  if (xQueueSend(this->event_queue_, &event, 0) != pdTRUE)
    ESP_LOGW(TAG, "Radio event queue full; dropping event");
}

void WiSafe2Component::emit_command_result_(ManagementCommand command, CommandOutcome outcome) {
  if (this->event_queue_ == nullptr)
    return;
  RadioEvent event{};
  event.type = EventType::COMMAND_RESULT;
  event.command = command;
  event.outcome = outcome;
  if (xQueueSend(this->event_queue_, &event, 0) != pdTRUE)
    ESP_LOGW(TAG, "Radio event queue full; dropping command result");
}

void WiSafe2Component::load_inventory_() {
  this->inventory_pref_ = global_preferences->make_preference<StoredInventory>(0x8C33B6A1, true);
  StoredInventory stored{};
  if (!this->inventory_pref_.load(&stored) || stored.magic != INVENTORY_MAGIC || stored.count > MAX_DETECTORS) {
    ESP_LOGI(TAG, "No valid stored detector inventory");
    return;
  }
  this->detector_count_ = stored.count > this->max_detectors_ ? this->max_detectors_ : stored.count;
  for (uint8_t i = 0; i < this->detector_count_; ++i) {
    this->detectors_[i].device_id = stored.detectors[i].device_id;
    this->detectors_[i].model_id = stored.detectors[i].model_id;
    this->detectors_[i].has_model = (stored.detectors[i].metadata & 0x01) != 0;
    this->detectors_[i].has_device_type = (stored.detectors[i].metadata & 0x02) != 0;
    if (this->detectors_[i].has_device_type) {
      switch ((stored.detectors[i].metadata >> 2) & 0x03) {
        case 1: this->detectors_[i].device_type = 0x41; break;
        case 2: this->detectors_[i].device_type = 0x81; break;
        default: this->detectors_[i].device_type = 0xFF; break;
      }
    }
    this->detectors_[i].sid = stored.detectors[i].sid_plus_one == 0
                                  ? -1
                                  : static_cast<int8_t>(stored.detectors[i].sid_plus_one - 1);
    if (this->detectors_[i].sid >= 0) {
      this->known_sid_map_ |= uint64_t{1} << this->detectors_[i].sid;
      this->sid_device_ids_[this->detectors_[i].sid] = this->detectors_[i].device_id;
    }
    this->detectors_[i].alarm = -1;
    this->detectors_[i].base_problem = -1;
    this->detectors_[i].battery_low = -1;
    this->detectors_[i].network_member = -1;
  }
  ESP_LOGI(TAG, "Restored %u detector(s) from flash", this->detector_count_);
}

void WiSafe2Component::load_test_history_() {
  this->test_history_pref_ = global_preferences->make_preference<StoredTestHistory>(0x7A80D9C2, true);
  StoredTestHistory stored{};
  if (!this->test_history_pref_.load(&stored) || stored.magic != TEST_HISTORY_MAGIC) {
    ESP_LOGI(TAG, "No valid stored detector test history");
    return;
  }
  for (uint8_t i = 0; i < this->detector_count_; ++i) {
    for (const auto &record : stored.detectors) {
      if (record.device_id != this->detectors_[i].device_id)
        continue;
      this->detectors_[i].last_test_epoch = record.last_test_epoch;
      if (record.result == 1)
        snprintf(this->detectors_[i].result, sizeof(this->detectors_[i].result), "PASS");
      else if (record.result == 2)
        snprintf(this->detectors_[i].result, sizeof(this->detectors_[i].result), "FAIL");
      break;
    }
  }
}

void WiSafe2Component::save_test_history_() {
  StoredTestHistory stored{};
  stored.magic = TEST_HISTORY_MAGIC;
  for (uint8_t i = 0; i < this->detector_count_; ++i) {
    stored.detectors[i].device_id = this->detectors_[i].device_id;
    stored.detectors[i].last_test_epoch = this->detectors_[i].last_test_epoch;
    if (strcmp(this->detectors_[i].result, "PASS") == 0)
      stored.detectors[i].result = 1;
    else if (strcmp(this->detectors_[i].result, "FAIL") == 0)
      stored.detectors[i].result = 2;
  }
  if (!this->test_history_pref_.save(&stored))
    ESP_LOGW(TAG, "Failed to persist detector test history");
}

void WiSafe2Component::save_inventory_() {
  StoredInventory stored{};
  stored.magic = INVENTORY_MAGIC;
  stored.count = this->detector_count_;
  for (uint8_t i = 0; i < this->detector_count_; ++i) {
    stored.detectors[i].device_id = this->detectors_[i].device_id;
    stored.detectors[i].model_id = this->detectors_[i].model_id;
    stored.detectors[i].metadata = this->detectors_[i].has_model ? 0x01 : 0x00;
    if (this->detectors_[i].has_device_type) {
      const uint8_t type_code = this->detectors_[i].device_type == 0x41 ? 1
                                : this->detectors_[i].device_type == 0x81 ? 2
                                                                         : 3;
      stored.detectors[i].metadata |= 0x02 | (type_code << 2);
    }
    stored.detectors[i].sid_plus_one = this->detectors_[i].sid >= 0
                                           ? static_cast<uint8_t>(this->detectors_[i].sid + 1)
                                           : 0;
  }
  if (!this->inventory_pref_.save(&stored))
    ESP_LOGW(TAG, "Failed to persist detector inventory");
}

WiSafe2Component::DetectorState *WiSafe2Component::find_or_create_detector_(const DecodedPacket &decoded) {
  if (decoded.device_id == 0 || decoded.device_id == this->bridge_device_id_)
    return nullptr;
  for (uint8_t i = 0; i < this->detector_count_; ++i) {
    if (this->detectors_[i].device_id == decoded.device_id)
      return &this->detectors_[i];
  }
  if (this->detector_count_ >= this->max_detectors_) {
    ESP_LOGW(TAG, "Detector %06X ignored: configured limit of %u reached",
             static_cast<unsigned>(decoded.device_id), this->max_detectors_);
    return nullptr;
  }
  DetectorState *detector = &this->detectors_[this->detector_count_++];
  memset(detector, 0, sizeof(*detector));
  detector->device_id = decoded.device_id;
  detector->sid = -1;
  detector->alarm = -1;
  detector->base_problem = -1;
  detector->battery_low = -1;
  detector->network_member = -1;
  ESP_LOGI(TAG, "Discovered new detector %06X (%u/%u)", static_cast<unsigned>(decoded.device_id),
           this->detector_count_, this->max_detectors_);
  this->save_inventory_();
  return detector;
}

void WiSafe2Component::update_detector_(const DecodedPacket &decoded, const char *raw_frame) {
  DetectorState *detector = this->find_or_create_detector_(decoded);
  if (detector == nullptr)
    return;
  bool first_update = detector->raw_frame[0] == '\0';
  bool inventory_changed = false;
  if (decoded.has_model && (!detector->has_model || detector->model_id != decoded.model_id)) {
    detector->model_id = decoded.model_id;
    detector->has_model = true;
    inventory_changed = true;
  }
  if (decoded.has_device_type &&
      (!detector->has_device_type || detector->device_type != decoded.device_type)) {
    detector->device_type = decoded.device_type;
    detector->has_device_type = true;
    inventory_changed = true;
  }
  if (decoded.has_sid && detector->sid != static_cast<int8_t>(decoded.sid)) {
    detector->sid = static_cast<int8_t>(decoded.sid);
    this->sid_device_ids_[decoded.sid] = decoded.device_id;
    inventory_changed = true;
  }
  if (decoded.has_sid)
    detector->network_member = 1;
  if (decoded.has_alarm)
    detector->alarm = decoded.alarm ? 1 : 0;
  if (decoded.has_base)
    detector->base_problem = decoded.base_problem ? 1 : 0;
  if (decoded.has_battery)
    detector->battery_low = decoded.battery_low ? 1 : 0;
  if (decoded.has_event)
    snprintf(detector->event, sizeof(detector->event), "%s", decoded.event);
  if (decoded.has_result) {
    snprintf(detector->result, sizeof(detector->result), "%s", decoded.result);
    detector->last_test_epoch = 0;
    if (this->time_ != nullptr) {
      ESPTime now = this->time_->utcnow();
      if (now.is_valid())
        detector->last_test_epoch = static_cast<uint32_t>(now.timestamp);
      else
        ESP_LOGW(TAG, "Test received from %06X before time synchronization",
                 static_cast<unsigned>(detector->device_id));
    }
    this->save_test_history_();
  }
  snprintf(detector->raw_frame, sizeof(detector->raw_frame), "%s", raw_frame);
  if (inventory_changed)
    this->save_inventory_();
  if (mqtt::global_mqtt_client != nullptr && mqtt::global_mqtt_client->is_connected()) {
    bool published = true;
    if (first_update || inventory_changed)
      published = this->publish_detector_discovery_(*detector);
    published &= this->publish_detector_state_(*detector);
    if (!published) {
      this->mqtt_resync_pending_ = true;
      this->mqtt_resync_index_ = 0;
      this->mqtt_next_sync_ms_ = millis() + 5000;
    }

    // Device-trigger events are deliberately non-retained and are emitted only
    // for live radio traffic. Do not retry them after an MQTT failure: doing so
    // could turn a stale alarm or test into a new Home Assistant automation.
    const char *event_payload = nullptr;
    if (decoded.has_alarm)
      event_payload = decoded.alarm ? "alarm_on" : "alarm_off";
    else if (decoded.has_result)
      event_payload = strcmp(decoded.result, "PASS") == 0 ? "test_pass" : "test_fail";
    if (event_payload != nullptr && !this->publish_detector_event_(*detector, event_payload))
      ESP_LOGW(TAG, "Failed to publish live event %s for detector %06X", event_payload,
               static_cast<unsigned>(detector->device_id));
  }
}

void WiSafe2Component::update_remote_diagnostic_(const RemoteDiagnostic &diagnostic, const char *raw_frame) {
  DecodedPacket decoded{};
  decoded.device_id = diagnostic.device_id;
  decoded.has_sid = true;
  decoded.sid = diagnostic.sid;
  DetectorState *detector = this->find_or_create_detector_(decoded);
  if (detector == nullptr)
    return;

  const bool first_update = detector->raw_frame[0] == '\0';
  const bool inventory_changed = detector->sid != static_cast<int8_t>(diagnostic.sid);
  detector->sid = static_cast<int8_t>(diagnostic.sid);
  detector->network_member = 1;
  detector->has_remote_diagnostic = true;
  detector->battery_primary = diagnostic.battery_primary;
  detector->battery_radio = diagnostic.battery_radio;
  detector->rssi = diagnostic.rssi;
  detector->firmware_version = diagnostic.firmware_version;
  detector->diagnostic_flags = diagnostic.flags;
  detector->radio_fault_count = diagnostic.radio_fault_count;
  snprintf(detector->raw_frame, sizeof(detector->raw_frame), "%s", raw_frame);
  if (inventory_changed)
    this->save_inventory_();

  if (mqtt::global_mqtt_client != nullptr && mqtt::global_mqtt_client->is_connected()) {
    bool published = true;
    if (first_update || inventory_changed)
      published = this->publish_detector_discovery_(*detector);
    published &= this->publish_detector_state_(*detector);
    if (!published) {
      this->mqtt_resync_pending_ = true;
      this->mqtt_resync_index_ = 0;
      this->mqtt_next_sync_ms_ = millis() + 5000;
    }
  }
}

void WiSafe2Component::update_detector_membership_(uint64_t sid_map) {
  const bool mqtt_connected = mqtt::global_mqtt_client != nullptr && mqtt::global_mqtt_client->is_connected();
  for (uint8_t i = 0; i < this->detector_count_; ++i) {
    DetectorState &detector = this->detectors_[i];
    const int8_t membership = detector.sid >= 0 && detector.sid < 64
                                  ? ((sid_map & (uint64_t{1} << detector.sid)) != 0 ? 1 : 0)
                                  : -1;
    if (detector.network_member == membership)
      continue;
    detector.network_member = membership;
    if (mqtt_connected && !this->publish_detector_state_(detector)) {
      this->mqtt_resync_pending_ = true;
      this->mqtt_resync_index_ = 0;
      this->mqtt_next_sync_ms_ = millis() + 5000;
    }
  }
}

void WiSafe2Component::service_mqtt_() {
  if (mqtt::global_mqtt_client == nullptr)
    return;
  bool connected = mqtt::global_mqtt_client->is_connected();
  if (connected && !this->mqtt_was_connected_) {
    ESP_LOGI(TAG, "MQTT connected; scheduling detector discovery refresh");
    this->mqtt_resync_pending_ = true;
    this->mqtt_resync_index_ = 0;
    this->mqtt_next_sync_ms_ = millis();
  }
  this->mqtt_was_connected_ = connected;
  if (!connected || !this->mqtt_resync_pending_)
    return;
  uint32_t now = millis();
  if (static_cast<int32_t>(now - this->mqtt_next_sync_ms_) < 0)
    return;
  if (this->mqtt_resync_index_ >= this->detector_count_) {
    this->mqtt_resync_pending_ = false;
    return;
  }
  const DetectorState &detector = this->detectors_[this->mqtt_resync_index_];
  bool published = this->publish_detector_discovery_(detector);
  // State topics are retained by the broker. Do not replace a retained state
  // with unknown values immediately after reboot; publish once this boot has
  // actually heard the detector.
  if (detector.raw_frame[0] != '\0')
    published &= this->publish_detector_state_(detector);
  if (published) {
    ++this->mqtt_resync_index_;
    this->mqtt_next_sync_ms_ = now + 250;
  } else {
    this->mqtt_next_sync_ms_ = now + 5000;
  }
}

void WiSafe2Component::format_detector_topic_(char *buffer, size_t length, const DetectorState &detector,
                                              const char *suffix) const {
  snprintf(buffer, length, "%s/wisafe2/%06x/%s", mqtt::global_mqtt_client->get_topic_prefix().c_str(),
           static_cast<unsigned>(detector.device_id), suffix);
}

const char *WiSafe2Component::model_name_(const DetectorState &detector) const {
  return detector_model_name(detector.model_id, detector.has_model);
}

const char *WiSafe2Component::alarm_device_class_(const DetectorState &detector) const {
  // Match MODEL_DEVICE_TYPES in fireangel-pro-connected-component. C304 is
  // deliberately not inferred because donor-module type may not match role.
  DetectorType type = detector_type_for_model(detector.model_id, detector.has_model);
  if ((detector.has_device_type && detector.device_type == 0x41) ||
      type == DetectorType::CARBON_MONOXIDE || strstr(detector.event, "CARBON MONOXIDE") != nullptr)
    return "carbon_monoxide";
  if (type == DetectorType::HEAT)
    return "heat";
  return "smoke";
}

bool WiSafe2Component::publish_discovery_entity_(const DetectorState &detector, const char *component,
                                                 const char *key, const char *name, const char *value_template,
                                                 const char *device_class, const char *entity_category,
                                                 const char *icon) {
  char discovery_topic[192];
  char state_topic[128];
  char unique_id[96];
  char object_id[96];
  char device_identifier[80];
  char detector_name[40];
  snprintf(unique_id, sizeof(unique_id), "wisafe2_%s_%06x_%s", App.get_name().c_str(),
           static_cast<unsigned>(detector.device_id), key);
  snprintf(object_id, sizeof(object_id), "%s_%06x_%s", App.get_name().c_str(),
           static_cast<unsigned>(detector.device_id), key);
  snprintf(device_identifier, sizeof(device_identifier), "wisafe2_%s_%06x", App.get_name().c_str(),
           static_cast<unsigned>(detector.device_id));
  snprintf(detector_name, sizeof(detector_name), "FireAngel %06X", static_cast<unsigned>(detector.device_id));
  snprintf(discovery_topic, sizeof(discovery_topic), "%s/%s/%s/config", this->discovery_prefix_.c_str(), component,
           unique_id);
  this->format_detector_topic_(state_topic, sizeof(state_topic), detector, "state");
  const mqtt::Availability &availability = mqtt::global_mqtt_client->get_availability();

  return mqtt::global_mqtt_client->publish_json(
      discovery_topic,
      [&](JsonObject root) {
        root["name"] = name;
        root["unique_id"] = unique_id;
        root["object_id"] = object_id;
        root["state_topic"] = state_topic;
        root["value_template"] = value_template;
        if (device_class != nullptr)
          root["device_class"] = device_class;
        if (entity_category != nullptr)
          root["entity_category"] = entity_category;
        if (icon != nullptr)
          root["icon"] = icon;
        if (strcmp(component, "binary_sensor") == 0) {
          root["payload_on"] = "ON";
          root["payload_off"] = "OFF";
        }
        if (!availability.topic.empty()) {
          root["availability_topic"] = availability.topic;
          root["payload_available"] = availability.payload_available;
          root["payload_not_available"] = availability.payload_not_available;
        }
        JsonObject device = root["device"].to<JsonObject>();
        JsonArray identifiers = device["identifiers"].to<JsonArray>();
        identifiers.add(device_identifier);
        device["name"] = detector_name;
        device["manufacturer"] = "FireAngel";
        device["model"] = this->model_name_(detector);
        JsonObject origin = root["origin"].to<JsonObject>();
        origin["name"] = "WiSafe2 ESPHome bridge";
        origin["sw_version"] = ESPHOME_VERSION;
      },
      0, true);
}

bool WiSafe2Component::publish_detector_trigger_discovery_(const DetectorState &detector, const char *key,
                                                           const char *type, const char *subtype,
                                                           const char *payload) {
  char discovery_topic[192];
  char event_topic[128];
  char trigger_id[96];
  char device_identifier[80];
  char detector_name[40];
  snprintf(trigger_id, sizeof(trigger_id), "wisafe2_%s_%06x_%s", App.get_name().c_str(),
           static_cast<unsigned>(detector.device_id), key);
  snprintf(device_identifier, sizeof(device_identifier), "wisafe2_%s_%06x", App.get_name().c_str(),
           static_cast<unsigned>(detector.device_id));
  snprintf(detector_name, sizeof(detector_name), "FireAngel %06X", static_cast<unsigned>(detector.device_id));
  snprintf(discovery_topic, sizeof(discovery_topic), "%s/device_automation/%s/config",
           this->discovery_prefix_.c_str(), trigger_id);
  this->format_detector_topic_(event_topic, sizeof(event_topic), detector, "event");

  return mqtt::global_mqtt_client->publish_json(
      discovery_topic,
      [&](JsonObject root) {
        root["automation_type"] = "trigger";
        root["type"] = type;
        root["subtype"] = subtype;
        root["topic"] = event_topic;
        root["payload"] = payload;
        JsonObject device = root["device"].to<JsonObject>();
        JsonArray identifiers = device["identifiers"].to<JsonArray>();
        identifiers.add(device_identifier);
        device["name"] = detector_name;
        device["manufacturer"] = "FireAngel";
        device["model"] = this->model_name_(detector);
        JsonObject origin = root["origin"].to<JsonObject>();
        origin["name"] = "WiSafe2 ESPHome bridge";
        origin["sw_version"] = ESPHOME_VERSION;
      },
      0, true);
}

bool WiSafe2Component::publish_detector_event_(const DetectorState &detector, const char *payload) {
  char topic[128];
  this->format_detector_topic_(topic, sizeof(topic), detector, "event");
  return mqtt::global_mqtt_client->publish(std::string(topic), std::string(payload), 0, false);
}

bool WiSafe2Component::format_last_test_(const DetectorState &detector, char *buffer, size_t length) const {
  if (detector.last_test_epoch == 0 || buffer == nullptr || length == 0)
    return false;
  ESPTime timestamp = ESPTime::from_epoch_utc(detector.last_test_epoch);
  return timestamp.is_valid() && timestamp.strftime(buffer, length, "%Y-%m-%dT%H:%M:%SZ") > 0;
}

bool WiSafe2Component::publish_detector_discovery_(const DetectorState &detector) {
  bool ok = true;
  ok &= this->publish_discovery_entity_(detector, "binary_sensor", "alarm", "Alarm", "{{ value_json.alarm }}",
                                        this->alarm_device_class_(detector), nullptr, nullptr);
  ok &= this->publish_discovery_entity_(detector, "binary_sensor", "battery_low", "Battery",
                                        "{{ value_json.battery_low }}", "battery", "diagnostic", nullptr);
  ok &= this->publish_discovery_entity_(detector, "binary_sensor", "base_problem", "Base",
                                        "{{ value_json.base_problem }}", "problem", "diagnostic", nullptr);
  ok &= this->publish_discovery_entity_(detector, "binary_sensor", "network_member", "Network member",
                                        "{{ value_json.network_member }}", "connectivity", "diagnostic", nullptr);
  ok &= this->publish_discovery_entity_(detector, "sensor", "model", "Model", "{{ value_json.model }}", nullptr,
                                        "diagnostic", "mdi:smoke-detector-variant");
  ok &= this->publish_discovery_entity_(detector, "sensor", "device_type", "Device type",
                                        "{{ value_json.device_type }}", nullptr, "diagnostic", "mdi:shape");
  ok &= this->publish_discovery_entity_(detector, "sensor", "sid", "SID", "{{ value_json.sid }}", nullptr,
                                        "diagnostic", "mdi:identifier");
  ok &= this->publish_discovery_entity_(detector, "sensor", "battery_primary_raw", "Sensor battery reading",
                                        "{{ value_json.battery_primary_raw }}", nullptr, "diagnostic",
                                        "mdi:battery-heart-variant");
  ok &= this->publish_discovery_entity_(detector, "sensor", "battery_radio_raw", "Radio battery reading",
                                        "{{ value_json.battery_radio_raw }}", nullptr, "diagnostic",
                                        "mdi:battery-heart-variant");
  ok &= this->publish_discovery_entity_(detector, "sensor", "rssi_raw", "Radio RSSI reading",
                                        "{{ value_json.rssi_raw }}", nullptr, "diagnostic", "mdi:signal");
  ok &= this->publish_discovery_entity_(detector, "sensor", "firmware", "Radio firmware",
                                        "{{ value_json.firmware }}", nullptr, "diagnostic", "mdi:chip");
  ok &= this->publish_discovery_entity_(detector, "sensor", "diagnostic_flags", "Diagnostic flags",
                                        "{{ value_json.diagnostic_flags }}", nullptr, "diagnostic",
                                        "mdi:flag-outline");
  ok &= this->publish_discovery_entity_(detector, "sensor", "radio_fault_count", "Radio fault count",
                                        "{{ value_json.radio_fault_count }}", nullptr, "diagnostic",
                                        "mdi:alert-circle-check-outline");
  ok &= this->publish_discovery_entity_(detector, "sensor", "last_event", "Last event",
                                        "{{ value_json.event }}", nullptr, nullptr, "mdi:message-alert-outline");
  ok &= this->publish_discovery_entity_(detector, "sensor", "test_result", "Last test result",
                                        "{{ value_json.test_result }}", nullptr, "diagnostic",
                                        "mdi:check-decagram-outline");
  ok &= this->publish_discovery_entity_(detector, "sensor", "last_test", "Last test",
                                        "{{ value_json.last_test }}", "timestamp", "diagnostic",
                                        "mdi:clock-check-outline");
  ok &= this->publish_discovery_entity_(detector, "sensor", "raw_frame", "Last raw frame",
                                        "{{ value_json.raw_frame }}", nullptr, "diagnostic", "mdi:code-tags");
  ok &= this->publish_detector_trigger_discovery_(detector, "alarm_detected", "alarm", "detected", "alarm_on");
  ok &= this->publish_detector_trigger_discovery_(detector, "alarm_cleared", "alarm", "cleared", "alarm_off");
  ok &= this->publish_detector_trigger_discovery_(detector, "test_passed", "test", "passed", "test_pass");
  ok &= this->publish_detector_trigger_discovery_(detector, "test_failed", "test", "failed", "test_fail");
  return ok;
}

bool WiSafe2Component::publish_detector_state_(const DetectorState &detector) {
  char topic[128];
  char model[16];
  char device_type[8];
  char firmware[8];
  char last_test[32];
  bool has_last_test = this->format_last_test_(detector, last_test, sizeof(last_test));
  this->format_detector_topic_(topic, sizeof(topic), detector, "state");
  if (detector.has_model)
    snprintf(model, sizeof(model), "%04X", detector.model_id);
  else
    snprintf(model, sizeof(model), "UNKNOWN");
  snprintf(device_type, sizeof(device_type), "%02X", detector.device_type);
  snprintf(firmware, sizeof(firmware), "%02X", detector.firmware_version);
  return mqtt::global_mqtt_client->publish_json(
      topic,
      [&](JsonObject root) {
        if (detector.alarm >= 0) root["alarm"] = detector.alarm ? "ON" : "OFF";
        else root["alarm"] = nullptr;
        if (detector.battery_low >= 0) root["battery_low"] = detector.battery_low ? "ON" : "OFF";
        else root["battery_low"] = nullptr;
        if (detector.base_problem >= 0) root["base_problem"] = detector.base_problem ? "ON" : "OFF";
        else root["base_problem"] = nullptr;
        if (detector.network_member >= 0) root["network_member"] = detector.network_member ? "ON" : "OFF";
        else root["network_member"] = nullptr;
        root["model"] = model;
        root["model_name"] = this->model_name_(detector);
        root["device_type"] = detector.has_device_type ? device_type : nullptr;
        if (detector.sid >= 0) root["sid"] = detector.sid;
        else root["sid"] = nullptr;
        if (detector.has_remote_diagnostic) {
          root["battery_primary_raw"] = detector.battery_primary;
          root["battery_radio_raw"] = detector.battery_radio;
          root["rssi_raw"] = detector.rssi;
          root["firmware"] = firmware;
          root["diagnostic_flags"] = detector.diagnostic_flags;
          root["radio_fault_count"] = detector.radio_fault_count;
        } else {
          root["battery_primary_raw"] = nullptr;
          root["battery_radio_raw"] = nullptr;
          root["rssi_raw"] = nullptr;
          root["firmware"] = nullptr;
          root["diagnostic_flags"] = nullptr;
          root["radio_fault_count"] = nullptr;
        }
        root["event"] = detector.event[0] != '\0' ? detector.event : nullptr;
        root["test_result"] = detector.result[0] != '\0' ? detector.result : nullptr;
        root["last_test"] = has_last_test ? last_test : nullptr;
        root["raw_frame"] = detector.raw_frame[0] != '\0' ? detector.raw_frame : nullptr;
      },
      0, true);
}

}  // namespace esphome::wisafe2
