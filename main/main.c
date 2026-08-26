#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * WiSafe2 radio <-> M5Stack AtomS3 Lite prototype wiring
 *
 * Radio is the SPI MASTER. ESP32-S3 is the SPI SLAVE.
 *
 * WiSafe2 2x5 header    AtomS3 Lite
 * ---------------------------------
 * CS                    GPIO8
 * IRQ                   GPIO38
 * MOSI (radio -> ESP)   GPIO6
 * SCK                   GPIO5
 * MISO (ESP -> radio)   GPIO7
 * GND                   GND
 * 3V3 (both pins)       3V3
 *
 * Separate antenna header:
 * GND                   GND
 * ANT                   quarter-wave wire
 */
#define PIN_SCLK  GPIO_NUM_5
#define PIN_MOSI  GPIO_NUM_6
#define PIN_MISO  GPIO_NUM_7
#define PIN_CS    GPIO_NUM_8
#define PIN_IRQ   GPIO_NUM_38

/*
 * The original Nano firmware is an SPI slave and uses a separate IRQ output
 * to request that the WiSafe2 radio clocks one byte. Each radio CS assertion
 * represents one byte transaction.
 */
#define SPI_HOST_USED SPI2_HOST
#define PACKET_MAX 64
#define BYTE_TIMEOUT_MS 1000

static const char *TAG = "wisafe2";

static void irq_set(bool asserted)
{
    gpio_set_level(PIN_IRQ, asserted ? 1 : 0);
}

static esp_err_t init_gpio(void)
{
    gpio_config_t irq_cfg = {
        .pin_bit_mask = 1ULL << PIN_IRQ,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&irq_cfg), TAG, "IRQ GPIO config failed");

    /* Match the Nano's idle state: IRQ is not asserted until we want to send. */
    irq_set(false);
    return ESP_OK;
}

static esp_err_t init_spi_slave(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 1,
        .flags = SPICOMMON_BUSFLAG_SLAVE,
        .intr_flags = 0,
    };

    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num = PIN_CS,
        .flags = 0,
        .queue_size = 3,
        .mode = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };

    ESP_RETURN_ON_ERROR(
        spi_slave_initialize(SPI_HOST_USED, &bus_cfg, &slave_cfg, SPI_DMA_DISABLED),
        TAG,
        "SPI slave init failed"
    );

    return ESP_OK;
}

/*
 * Wait for the radio to clock exactly one unsolicited byte.
 * A zero byte is exposed on MISO while the radio clocks MOSI.
 */
static esp_err_t receive_byte(uint8_t *value, TickType_t timeout)
{
    uint8_t tx = 0x00;
    uint8_t rx = 0x00;

    spi_slave_transaction_t trans = {
        .length = 8,
        .tx_buffer = &tx,
        .rx_buffer = &rx,
    };

    esp_err_t err = spi_slave_transmit(SPI_HOST_USED, &trans, timeout);
    if (err != ESP_OK) {
        return err;
    }

    if (trans.trans_len != 8) {
        ESP_LOGW(TAG, "Unexpected SPI transaction length: %u bits", (unsigned) trans.trans_len);
    }

    if (value != NULL) {
        *value = rx;
    }
    return ESP_OK;
}

/*
 * Send one byte using the WiSafe2/Nano handshake.
 *
 * The AVR code raises IRQ, waits for CS to fall, then loads SPDR. On ESP-IDF
 * it is safer to arm the SPI-slave transaction first, then raise IRQ. This
 * guarantees that MISO is ready before the radio is invited to assert CS and
 * clock the byte.
 *
 * Sequence:
 *   1. queue one 8-bit SPI-slave transaction
 *   2. assert IRQ
 *   3. radio asserts CS and clocks the byte
 *   4. wait for transaction completion
 *   5. deassert IRQ
 */
static esp_err_t send_byte(uint8_t value)
{
    uint8_t tx = value;
    uint8_t rx = 0x00;

    spi_slave_transaction_t trans = {
        .length = 8,
        .tx_buffer = &tx,
        .rx_buffer = &rx,
    };

    TickType_t timeout = pdMS_TO_TICKS(BYTE_TIMEOUT_MS);

    esp_err_t err = spi_slave_queue_trans(SPI_HOST_USED, &trans, timeout);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to arm TX 0x%02X: %s", value, esp_err_to_name(err));
        return err;
    }

    /* Only tell the radio to clock a byte after the SPI transaction is armed. */
    irq_set(true);

    spi_slave_transaction_t *completed = NULL;
    err = spi_slave_get_trans_result(SPI_HOST_USED, &completed, timeout);

    /* Always return IRQ to idle, including timeout/error paths. */
    irq_set(false);

    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Timeout sending 0x%02X: radio did not clock a byte", value);
        return err;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "SPI TX failed");

    if (completed != &trans) {
        ESP_LOGE(TAG, "Unexpected SPI transaction completed");
        return ESP_FAIL;
    }

    if (trans.trans_len != 8) {
        ESP_LOGW(TAG, "Unexpected TX transaction length: %u bits", (unsigned) trans.trans_len);
    }

    ESP_LOGI(TAG, "TX 0x%02X  simultaneous RX 0x%02X", value, rx);
    return ESP_OK;
}

static esp_err_t send_packet(const uint8_t *data, size_t length)
{
    ESP_LOGI(TAG, "Sending %u-byte packet", (unsigned) length);
    for (size_t i = 0; i < length; ++i) {
        ESP_RETURN_ON_ERROR(send_byte(data[i]), TAG, "Packet TX failed at byte %u", (unsigned) i);
    }
    return ESP_OK;
}

static size_t receive_packet(uint8_t *buffer, size_t capacity, uint32_t overall_timeout_ms)
{
    size_t used = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t overall = pdMS_TO_TICKS(overall_timeout_ms);

    while (used < capacity) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= overall) {
            break;
        }

        TickType_t remaining = overall - elapsed;
        uint8_t value = 0;
        esp_err_t err = receive_byte(&value, remaining);
        if (err == ESP_ERR_TIMEOUT) {
            break;
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI RX failed: %s", esp_err_to_name(err));
            break;
        }

        buffer[used++] = value;
        ESP_LOGI(TAG, "RX 0x%02X", value);

        if (value == 0x7E) {
            break;
        }
    }

    return used;
}

static void log_packet(const char *prefix, const uint8_t *data, size_t length)
{
    char line[PACKET_MAX * 3 + 1];
    size_t pos = 0;

    for (size_t i = 0; i < length && pos + 3 < sizeof(line); ++i) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X%s", data[i], i + 1 < length ? " " : "");
    }
    line[pos] = '\0';
    ESP_LOGI(TAG, "%s%s", prefix, line);
}

void app_main(void)
{
    ESP_LOGI(TAG, "WiSafe2 ESP32-S3 minimal radio test");
    ESP_LOGI(TAG, "Radio is SPI master; ESP32-S3 is SPI slave");
    ESP_LOGI(TAG, "Pins: SCLK=%d MOSI=%d MISO=%d CS=%d IRQ=%d",
             PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_IRQ);

    ESP_ERROR_CHECK(init_gpio());
    ESP_ERROR_CHECK(init_spi_slave());

    ESP_LOGI(TAG, "Waiting 5 seconds for radio to stabilise (matching Nano firmware)...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    const uint8_t init_cmd[] = {0xD3, 0x19, 0x50, 0x00, 0x7E};
    log_packet("INIT TX: ", init_cmd, sizeof(init_cmd));

    esp_err_t err = send_packet(init_cmd, sizeof(init_cmd));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initialisation command failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Check wiring, 3.3 V supply, CS/IRQ polarity and SPI timing.");
    } else {
        uint8_t response[PACKET_MAX] = {0};
        size_t response_len = receive_packet(response, sizeof(response), 1000);

        if (response_len > 0) {
            log_packet("INIT RX: ", response, response_len);
        } else {
            ESP_LOGW(TAG, "No response received after init command");
        }

        if (response_len >= 2 && response[0] == 0x46 && response[1] == 0x7E) {
            ESP_LOGI(TAG, "*** INIT OK: received expected 46 7E ***");
        } else {
            ESP_LOGW(TAG, "Init response did not match expected 46 7E");
        }
    }

    ESP_LOGI(TAG, "Entering raw receive mode. Trigger a FireAngel device and watch the log.");

    uint8_t packet[PACKET_MAX];
    size_t used = 0;

    while (true) {
        uint8_t value = 0;
        err = receive_byte(&value, pdMS_TO_TICKS(1000));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI receive error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGI(TAG, "RX 0x%02X", value);

        if (used < sizeof(packet)) {
            packet[used++] = value;
        } else {
            ESP_LOGW(TAG, "Packet buffer overflow; discarding partial packet");
            used = 0;
        }

        if (value == 0x7E) {
            log_packet("PACKET: ", packet, used);
            used = 0;
        }
    }
}
