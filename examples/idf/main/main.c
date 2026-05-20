#include <stdio.h>
#include <inttypes.h>

#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "canopen.h"
#include "canopen_console.h"
#include "motor_console.h"

#define EXAMPLE_UART_PORT UART_NUM_1
#define EXAMPLE_UART_TX_PIN 43
#define EXAMPLE_UART_RX_PIN 44
#define EXAMPLE_UART_BAUD_RATE 115200
#define EXAMPLE_UART_BUF_SIZE 256
#define EXAMPLE_CAN_TX_PIN 14
#define EXAMPLE_CAN_RX_PIN 15
#define EXAMPLE_CAN_BITRATE 1000000u
#define EXAMPLE_LED_PIN 48
#define EXAMPLE_LED_RMT_RESOLUTION_HZ 10000000

static const char *TAG = "epos_example";

static void led_set_color(rmt_channel_handle_t led_chan,
                          rmt_encoder_handle_t led_encoder,
                          uint8_t red,
                          uint8_t green,
                          uint8_t blue)
{
    uint8_t led_data[3] = { green, red, blue };
    const rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_data, sizeof(led_data), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, -1));
}

static void led_blink_task(void *arg)
{
    (void)arg;

    bool led_on = false;

    rmt_channel_handle_t led_chan = NULL;
    const rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = EXAMPLE_LED_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    rmt_encoder_handle_t led_encoder = NULL;
    const rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3,
            .level1 = 0,
            .duration1 = 9,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9,
            .level1 = 0,
            .duration1 = 3,
        },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    while (true) {
        if (led_on) {
            led_set_color(led_chan, led_encoder, 0, 32, 0);
        } else {
            led_set_color(led_chan, led_encoder, 0, 0, 0);
        }
        led_on = !led_on;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static esp_err_t example_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = EXAMPLE_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(EXAMPLE_UART_PORT,
                                            EXAMPLE_UART_BUF_SIZE,
                                            EXAMPLE_UART_BUF_SIZE,
                                            0,
                                            NULL,
                                            0),
                        TAG,
                        "install UART driver");
    ESP_RETURN_ON_ERROR(uart_param_config(EXAMPLE_UART_PORT, &uart_config),
                        TAG,
                        "configure UART");
    ESP_RETURN_ON_ERROR(uart_set_pin(EXAMPLE_UART_PORT,
                                     EXAMPLE_UART_TX_PIN,
                                     EXAMPLE_UART_RX_PIN,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG,
                        "set UART pins");

    return ESP_OK;
}

static void uart_periodic_task(void *arg)
{
    (void)arg;

    uint32_t count = 0;
    char line[80];

    while (true) {
        int len = snprintf(line,
                           sizeof(line),
                           "EPOS UART heartbeat %" PRIu32 "\r\n",
                           ++count);
        if (len > 0) {
            uart_write_bytes(EXAMPLE_UART_PORT, line, len);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, NULL);

    canopen_console_cfg_t console_cfg = CANOPEN_CONSOLE_DEFAULT();
    console_cfg.enable_tcp_console = false;
    console_cfg.dumb_mode = true;

    bool telemetry_uart_available = !(console_cfg.enable_uart_console &&
                                      console_cfg.uart_tx_pin == EXAMPLE_UART_TX_PIN &&
                                      console_cfg.uart_rx_pin == EXAMPLE_UART_RX_PIN);
    if (telemetry_uart_available) {
        ESP_ERROR_CHECK(example_uart_init());
        xTaskCreate(uart_periodic_task, "uart_periodic", 2048, NULL, 5, NULL);
    } else {
        ESP_LOGI(TAG, "Telemetry UART disabled because GPIO %d/%d are used by the console.",
                 EXAMPLE_UART_TX_PIN,
                 EXAMPLE_UART_RX_PIN);
    }

    canopen_console_init(&console_cfg);
    canopen_console_register_commands();
    motor_register_commands();

    ESP_LOGI(TAG, "Console initialized. Telemetry UART%d TX=%d RX=%d at %d baud.",
             EXAMPLE_UART_PORT,
             EXAMPLE_UART_TX_PIN,
             EXAMPLE_UART_RX_PIN,
             EXAMPLE_UART_BAUD_RATE);

    canopen_init_cfg_t canopen_cfg = CANOPEN_INIT_DEFAULT();
    canopen_cfg.can_tx_pin = EXAMPLE_CAN_TX_PIN;
    canopen_cfg.can_rx_pin = EXAMPLE_CAN_RX_PIN;
    canopen_cfg.can_bitrate = EXAMPLE_CAN_BITRATE;

    ESP_LOGI(TAG, "Starting CANopen/TWAI on TX=%d RX=%d bitrate=%" PRIu32,
             canopen_cfg.can_tx_pin,
             canopen_cfg.can_rx_pin,
             canopen_cfg.can_bitrate);
    ESP_ERROR_CHECK(canopen_initialize(&canopen_cfg));
    ESP_LOGI(TAG, "CANopen ready");
}
