#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/spi_slave.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_private/spi_slave_internal.h"
#include "esp_rom_sys.h"
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
#define RX_ACK_PULSE_US 8
#define SPI_SLOT_BITS 16
#define INIT_MAX_ATTEMPTS 50
#define INIT_RETRY_DELAY_MS 500
#define PAIRING_COMMAND_ATTEMPTS 5
#define PAIRING_WINDOW_MS 21000

/* Pseudo-device identity used by the original Nano bridge (WST-630 model).
 * Change the three ID bytes if this identity already exists on the network. */
#define BRIDGE_DEVICE_ID_0 0xA5
#define BRIDGE_DEVICE_ID_1 0xB8
#define BRIDGE_DEVICE_ID_2 0x13
#define BRIDGE_MODEL_0     0x11
#define BRIDGE_MODEL_1     0x03

static const char *TAG = "wisafe2";
static gpio_glitch_filter_handle_t s_sclk_filter;

/*
 * The radio normally transfers one byte per CS assertion, but early bench
 * captures showed 9- and 10-clock transactions. Use word-aligned, two-byte
 * buffers so those extra clocks cannot run past a one-byte stack object and
 * so trans_len reports the real number of clocks (up to 16).
 */
typedef struct {
    uint32_t tx_word;
    uint32_t rx_word;
    spi_slave_transaction_t trans;
} spi_slot_t;

typedef struct {
    bool spi_reset;
    size_t tx_count;
    size_t tx_bits[PACKET_MAX];
    uint8_t simultaneous_rx[PACKET_MAX];
    size_t response_len;
    uint8_t response[PACKET_MAX];
    size_t response_bits[PACKET_MAX];
} exchange_result_t;

static void log_packet(const char *prefix, const uint8_t *data, size_t length);
static void log_exchange_diagnostics(const exchange_result_t *result);

static void irq_set(bool asserted)
{
    gpio_set_level(PIN_IRQ, asserted ? 1 : 0);
}

static void acknowledge_received_byte(void)
{
    /*
     * FireAngelNano.ino's ReadByteFromRadio() pulses IRQ for 8 us whenever
     * SS/CS returns HIGH after a received byte. This appears to act as a
     * per-byte acknowledgement / flow-control signal to the radio.
     */
    irq_set(true);
    esp_rom_delay_us(RX_ACK_PULSE_US);
    irq_set(false);
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

    /* Direct wiring has much faster edges than the original MOSFET level
     * shifters. Reject sub-25 ns spikes/ringing on SCLK before it reaches the
     * SPI peripheral. Valid WiSafe2 clock pulses are substantially longer. */
    gpio_pin_glitch_filter_config_t filter_cfg = {
        .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
        .gpio_num = PIN_SCLK,
    };
    ESP_RETURN_ON_ERROR(gpio_new_pin_glitch_filter(&filter_cfg, &s_sclk_filter),
                        TAG, "SCLK glitch filter allocation failed");
    ESP_RETURN_ON_ERROR(gpio_glitch_filter_enable(s_sclk_filter),
                        TAG, "SCLK glitch filter enable failed");
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
        .max_transfer_sz = SPI_SLOT_BITS / 8,
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

    /* Reduce MISO edge energy and crosstalk into the adjacent SCLK wire. */
    ESP_RETURN_ON_ERROR(gpio_set_drive_capability(PIN_MISO, GPIO_DRIVE_CAP_0),
                        TAG, "MISO drive-strength configuration failed");

    return ESP_OK;
}

static void prepare_slot(spi_slot_t *slot, uint8_t tx_value)
{
    memset(slot, 0, sizeof(*slot));

    /* ESP32-S3 is little-endian; the first byte shifted is byte zero. */
    ((uint8_t *) &slot->tx_word)[0] = tx_value;
    slot->trans.length = SPI_SLOT_BITS;
    slot->trans.tx_buffer = &slot->tx_word;
    slot->trans.rx_buffer = &slot->rx_word;
}

static uint8_t slot_rx_byte(const spi_slot_t *slot)
{
    return ((const uint8_t *) &slot->rx_word)[0];
}

static esp_err_t queue_slot(spi_slot_t *slot, uint8_t tx_value)
{
    prepare_slot(slot, tx_value);
    return spi_slave_queue_trans(SPI_HOST_USED, &slot->trans, pdMS_TO_TICKS(BYTE_TIMEOUT_MS));
}

static esp_err_t wait_for_slot(spi_slot_t *expected, TickType_t timeout)
{
    spi_slave_transaction_t *completed = NULL;
    esp_err_t err = spi_slave_get_trans_result(SPI_HOST_USED, &completed, timeout);
    if (err != ESP_OK) {
        return err;
    }
    if (completed != &expected->trans) {
        ESP_LOGE(TAG, "Unexpected SPI transaction completed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t reset_spi_slave(void)
{
    /* A timed-out queued receive cannot be cancelled through the public queue
     * API. ESP-IDF's internal queue reset safely discards it at idle without
     * freeing/reallocating the SPI interrupt (which can assert on IDF 6.1). */
    irq_set(false);

    for (int i = 0; i < 100 && gpio_get_level(PIN_CS) == 0; ++i) {
        esp_rom_delay_us(100);
    }
    if (gpio_get_level(PIN_CS) == 0) {
        ESP_LOGE(TAG, "Cannot reset SPI slave while CS remains asserted");
        return ESP_ERR_INVALID_STATE;
    }

    return spi_slave_queue_reset(SPI_HOST_USED);
}

static esp_err_t abort_exchange(esp_err_t cause, exchange_result_t *result)
{
    /* Cancel any queued/current descriptor while exchange_packet's buffers are
     * still valid. Prefer a reset failure because continuing would be unsafe. */
    esp_err_t reset_err = reset_spi_slave();
    result->spi_reset = (reset_err == ESP_OK);
    return reset_err == ESP_OK ? cause : reset_err;
}

/*
 * Send a packet and receive its immediate reply without ever leaving the SPI
 * slave unarmed. The first receive slot is queued before IRQ requests the
 * final transmit byte. For multi-byte replies, the next slot is queued before
 * the 8 us acknowledgement pulse tells the radio it may continue.
 */
static esp_err_t exchange_packet(const uint8_t *data,
                                 size_t length,
                                 exchange_result_t *result,
                                 unsigned response_packets,
                                 uint32_t response_timeout_ms)
{
    if (data == NULL || result == NULL || length == 0 || length > PACKET_MAX ||
        response_packets == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));

    spi_slot_t tx_slot;
    spi_slot_t rx_slots[2];
    bool first_rx_queued = false;

    for (size_t i = 0; i < length; ++i) {
        esp_err_t err = queue_slot(&tx_slot, data[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to queue TX byte %u: %s",
                     (unsigned) i, esp_err_to_name(err));
            return abort_exchange(err, result);
        }

        if (i + 1 == length) {
            /* This closes the response race: RX is ready before the radio is
             * invited to clock the packet terminator. */
            err = queue_slot(&rx_slots[0], 0x00);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to pre-arm first response byte: %s",
                         esp_err_to_name(err));
                return abort_exchange(err, result);
            }
            first_rx_queued = true;
        }

        irq_set(true);
        err = wait_for_slot(&tx_slot, pdMS_TO_TICKS(BYTE_TIMEOUT_MS));
        irq_set(false);
        if (err != ESP_OK) {
            return abort_exchange(err, result);
        }

        result->tx_bits[i] = tx_slot.trans.trans_len;
        result->simultaneous_rx[i] = slot_rx_byte(&tx_slot);
        result->tx_count = i + 1;
    }

    if (!first_rx_queued) {
        return ESP_FAIL;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t overall = pdMS_TO_TICKS(response_timeout_ms);
    unsigned current = 0;
    unsigned packets_received = 0;

    while (result->response_len < PACKET_MAX) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= overall) {
            return abort_exchange(ESP_ERR_TIMEOUT, result);
        }

        esp_err_t err = wait_for_slot(&rx_slots[current], overall - elapsed);
        if (err != ESP_OK) {
            return abort_exchange(err, result);
        }

        uint8_t value = slot_rx_byte(&rx_slots[current]);
        size_t response_index = result->response_len++;
        result->response[response_index] = value;
        result->response_bits[response_index] = rx_slots[current].trans.trans_len;

        if (value == 0x7E) {
            ++packets_received;
            if (packets_received == response_packets) {
                acknowledge_received_byte();
                return ESP_OK;
            }
        }

        unsigned next = current ^ 1U;
        err = queue_slot(&rx_slots[next], 0x00);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to keep response RX armed: %s", esp_err_to_name(err));
            return abort_exchange(err, result);
        }
        acknowledge_received_byte();
        current = next;
    }

    /* A non-terminated maximum-length reply has one more receive slot armed. */
    return abort_exchange(ESP_ERR_NO_MEM, result);
}

static esp_err_t prepare_for_retry(const exchange_result_t *result)
{
    return result->spi_reset ? ESP_OK : reset_spi_slave();
}

static bool query_pairing_status(bool *paired)
{
    static const uint8_t status_cmd[] = {0xD3, 0x03, 0x7E};

    for (unsigned attempt = 1; attempt <= PAIRING_COMMAND_ATTEMPTS; ++attempt) {
        static exchange_result_t result;
        esp_err_t err = exchange_packet(status_cmd, sizeof(status_cmd), &result, 1,
                                        BYTE_TIMEOUT_MS);
        log_exchange_diagnostics(&result);
        if (result.response_len > 0) {
            log_packet("PAIR STATUS RX: ", result.response, result.response_len);
        }

        if (err == ESP_OK && result.response_len == 11 &&
            result.response[0] == 0xD4 && result.response[1] == 0x03 &&
            result.response[10] == 0x7E) {
            *paired = (result.response[2] != 0x00);
            return true;
        }

        ESP_LOGW(TAG, "Pairing-status query attempt %u/%u failed",
                 attempt, (unsigned) PAIRING_COMMAND_ATTEMPTS);
        if (prepare_for_retry(&result) != ESP_OK) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
    }

    return false;
}

static esp_err_t receive_pairing_window(uint32_t window_ms)
{
    spi_slot_t slots[2];
    unsigned current = 0;
    static uint8_t packet[PACKET_MAX];
    size_t used = 0;

    ESP_RETURN_ON_ERROR(queue_slot(&slots[current], 0x00), TAG,
                        "Failed to arm pairing receive window");

    TickType_t start = xTaskGetTickCount();
    TickType_t overall = pdMS_TO_TICKS(window_ms);

    while (xTaskGetTickCount() - start < overall) {
        TickType_t remaining = overall - (xTaskGetTickCount() - start);
        esp_err_t err = wait_for_slot(&slots[current], remaining);
        if (err == ESP_ERR_TIMEOUT) {
            break;
        }
        if (err != ESP_OK) {
            reset_spi_slave();
            return err;
        }

        uint8_t value = slot_rx_byte(&slots[current]);
        size_t bits = slots[current].trans.trans_len;
        unsigned next = current ^ 1U;

        err = queue_slot(&slots[next], 0x00);
        if (err != ESP_OK) {
            reset_spi_slave();
            return err;
        }
        acknowledge_received_byte();
        current = next;

        if (bits != 8) {
            ESP_LOGW(TAG, "Pairing RX used %u clocks (expected 8)", (unsigned) bits);
        }
        if (used < sizeof(packet)) {
            packet[used++] = value;
        } else {
            used = 0;
        }
        if (value == 0x7E) {
            log_packet("PAIRING EVENT: ", packet, used);
            used = 0;
        }
    }

    /* The window normally ends with an uncompleted receive slot queued. */
    return reset_spi_slave();
}

static bool start_pairing(void)
{
    bool paired = false;
    if (!query_pairing_status(&paired)) {
        ESP_LOGE(TAG, "Could not determine current pairing state");
        return false;
    }
    if (paired) {
        ESP_LOGI(TAG, "Radio is already paired; pairing mode not started");
        return true;
    }

    static const uint8_t start_cmd[] = {0xD3, 0x12, 0x01, 0x7E};
    bool activated = false;

    for (unsigned attempt = 1; attempt <= PAIRING_COMMAND_ATTEMPTS; ++attempt) {
        static exchange_result_t result;
        esp_err_t err = exchange_packet(start_cmd, sizeof(start_cmd), &result, 2,
                                        BYTE_TIMEOUT_MS);
        log_exchange_diagnostics(&result);
        if (result.response_len > 0) {
            log_packet("PAIR START RX: ", result.response, result.response_len);
        }

        if (err == ESP_OK && result.response_len == 4 &&
            result.response[0] == 0x46 && result.response[1] == 0x7E &&
            result.response[2] == 0x41 && result.response[3] == 0x7E) {
            activated = true;
            break;
        }

        ESP_LOGW(TAG, "Pairing activation attempt %u/%u failed",
                 attempt, (unsigned) PAIRING_COMMAND_ATTEMPTS);
        if (prepare_for_retry(&result) != ESP_OK) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
    }

    if (!activated) {
        ESP_LOGE(TAG, "Radio did not accept the pairing command");
        return false;
    }

    static const uint8_t identity_cmd[] = {
        0x91,
        BRIDGE_DEVICE_ID_0, BRIDGE_DEVICE_ID_1, BRIDGE_DEVICE_ID_2,
        BRIDGE_MODEL_0, BRIDGE_MODEL_1,
        0xFF, 0x05, 0x01, 0x01, 0x7E,
    };
    static exchange_result_t identity_result;
    esp_err_t identity_err = exchange_packet(identity_cmd, sizeof(identity_cmd),
                                              &identity_result, 1, 250);
    log_exchange_diagnostics(&identity_result);
    if (identity_result.response_len > 0) {
        log_packet("PAIR IDENTITY RX: ", identity_result.response,
                   identity_result.response_len);
    }
    if (identity_err != ESP_OK && identity_result.tx_count != sizeof(identity_cmd)) {
        ESP_LOGE(TAG, "Could not send pairing identity: %s", esp_err_to_name(identity_err));
        return false;
    }

    ESP_LOGI(TAG, "*** PAIRING MODE ACTIVE FOR 21 SECONDS ***");
    ESP_LOGI(TAG, "Press the physical test/pair button on the other FireAngel device now");
    if (receive_pairing_window(PAIRING_WINDOW_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Pairing receive window failed");
        return false;
    }

    if (!query_pairing_status(&paired)) {
        ESP_LOGE(TAG, "Could not verify pairing state");
        return false;
    }
    if (paired) {
        ESP_LOGI(TAG, "*** NETWORK IS NOW PAIRED ***");
        return true;
    }

    ESP_LOGW(TAG, "Network is still unpaired; reset and try the pairing window again");
    return false;
}

static void log_exchange_diagnostics(const exchange_result_t *result)
{
    for (size_t i = 0; i < result->tx_count; ++i) {
        if (result->tx_bits[i] != 8) {
            ESP_LOGW(TAG, "TX byte %u used %u clocks (expected 8)",
                     (unsigned) i, (unsigned) result->tx_bits[i]);
        }
        ESP_LOGI(TAG, "TX byte %u: simultaneous RX 0x%02X",
                 (unsigned) i, result->simultaneous_rx[i]);
    }

    for (size_t i = 0; i < result->response_len; ++i) {
        if (result->response_bits[i] != 8) {
            ESP_LOGW(TAG, "RX byte %u used %u clocks (expected 8)",
                     (unsigned) i, (unsigned) result->response_bits[i]);
        }
        ESP_LOGI(TAG, "RX 0x%02X", result->response[i]);
    }
}

static bool response_contains_packet(const exchange_result_t *result,
                                     const uint8_t *expected,
                                     size_t expected_len)
{
    size_t packet_start = 0;

    for (size_t i = 0; i < result->response_len; ++i) {
        if (result->response[i] != 0x7E) {
            continue;
        }

        size_t packet_len = i - packet_start + 1;
        if (packet_len == expected_len &&
            memcmp(&result->response[packet_start], expected, expected_len) == 0) {
            return true;
        }
        packet_start = i + 1;
    }

    return false;
}

/*
 * Raw receive mode uses two alternating slots. Once a slot completes, its
 * successor is queued before IRQ is pulsed and before any logging occurs.
 */
static void raw_receive_loop(void)
{
    spi_slot_t slots[2];
    unsigned current = 0;

    esp_err_t err = queue_slot(&slots[current], 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to arm raw receive mode: %s", esp_err_to_name(err));
        return;
    }

    static uint8_t packet[PACKET_MAX];
    size_t used = 0;

    while (true) {
        err = wait_for_slot(&slots[current], pdMS_TO_TICKS(1000));
        if (err == ESP_ERR_TIMEOUT) {
            /* The transaction remains queued; keep waiting for the same slot. */
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI receive error: %s", esp_err_to_name(err));
            reset_spi_slave();
            return;
        }

        uint8_t value = slot_rx_byte(&slots[current]);
        size_t bits = slots[current].trans.trans_len;
        unsigned next = current ^ 1U;

        err = queue_slot(&slots[next], 0x00);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to re-arm SPI receive: %s", esp_err_to_name(err));
            reset_spi_slave();
            return;
        }
        acknowledge_received_byte();
        current = next;

        if (bits != 8) {
            ESP_LOGW(TAG, "RX transaction used %u clocks (expected 8)", (unsigned) bits);
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

static void log_packet(const char *prefix, const uint8_t *data, size_t length)
{
    static char line[PACKET_MAX * 3 + 1];
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
    const uint8_t init_ok_packet[] = {0x46, 0x7E};
    log_packet("INIT TX: ", init_cmd, sizeof(init_cmd));

    bool init_ok = false;
    bool spi_ready = true;

    for (unsigned attempt = 1; attempt <= INIT_MAX_ATTEMPTS; ++attempt) {
        static exchange_result_t result;
        ESP_LOGI(TAG, "Radio initialisation attempt %u/%u",
                 attempt, (unsigned) INIT_MAX_ATTEMPTS);

        /* The radio can have a queued status/event packet ahead of its init
         * acknowledgement. Keep listening across packet boundaries and find
         * 46 7E in the stream instead of assuming the first packet is ours. */
        esp_err_t err = exchange_packet(init_cmd, sizeof(init_cmd), &result, 4,
                                        BYTE_TIMEOUT_MS);
        log_exchange_diagnostics(&result);

        if (result.response_len > 0) {
            log_packet("INIT RX: ", result.response, result.response_len);
        }

        if (response_contains_packet(&result, init_ok_packet,
                                     sizeof(init_ok_packet))) {
            ESP_LOGI(TAG, "*** INIT OK: received expected 46 7E ***");
            init_ok = true;
            break;
        }

        if (err == ESP_ERR_TIMEOUT && result.tx_count == sizeof(init_cmd)) {
            ESP_LOGW(TAG, "No response received after init command");
        } else if (err != ESP_OK) {
            ESP_LOGW(TAG, "Initialisation exchange failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGW(TAG, "Init response did not match expected 46 7E");
        }

        /* Protocol mismatches finish cleanly but still get a fresh peripheral
         * state. Error paths have already cancelled any pending descriptor. */
        if (!result.spi_reset) {
            err = reset_spi_slave();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Could not reset SPI for retry: %s", esp_err_to_name(err));
                spi_ready = false;
                break;
            }
        }

        if (attempt < INIT_MAX_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
        }
    }

    if (!spi_ready) {
        ESP_LOGE(TAG, "SPI stopped; check whether CS is stuck low");
        return;
    }

    if (!init_ok) {
        ESP_LOGE(TAG, "Radio initialisation failed after %u attempts",
                 (unsigned) INIT_MAX_ATTEMPTS);
        ESP_LOGE(TAG, "Check the captured clock counts with a logic analyser on IRQ, CS, SCLK, MOSI and MISO.");
    } else {
        start_pairing();
    }

    ESP_LOGI(TAG, "Entering raw receive mode. Trigger a FireAngel device and watch the log.");
    raw_receive_loop();
}
