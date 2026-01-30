#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"

#include "esp_log.h"
#include "esp_system.h"

#include "esp_spiffs.h"
#include "esp_vfs.h"

#define TAG "YERRA"

/* ===================== incoming uart ===================== */
#define UART_PORT UART_NUM_1
#define UART_RX_PIN 16
#define UART_TX_PIN 17
#define UART_BUF_SZ 256

/* ===================== outgoing i2s ===================== */
#define I2S_BCLK_PIN 10
#define I2S_LRCLK_PIN 11
#define I2S_DATA_PIN 9
#define AMP_EN_PIN 12

static i2s_chan_handle_t i2s_tx_chan;

/* ===================== initialize ===================== */

static void init_amp(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << AMP_EN_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_cfg);
    gpio_set_level(AMP_EN_PIN, 1); // amp enable
}

static void init_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT, UART_BUF_SZ * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void init_i2s(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 240;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_LRCLK_PIN,
            .dout = I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan));
}

static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    ESP_LOGI(TAG, "SPIFFS mounted");
}

/* ===================== WAV PLAYER ===================== */

static void play_wav(const char *path)
{
    ESP_LOGI(TAG, "Playing %s", path);

    FILE *f = fopen(path, "rb");
    if (!f)
    {
        ESP_LOGE(TAG, "Failed to open WAV");
        return;
    }

    uint8_t wav_header[44];
    fread(wav_header, 1, 44, f); // skip WAV header

    int16_t samples[512];
    size_t bytes_written;

    while (1)
    {
        size_t samples_read = fread(samples, sizeof(int16_t), 512, f);
        if (samples_read == 0)
            break;

        i2s_channel_write(
            i2s_tx_chan,
            samples,
            samples_read * sizeof(int16_t),
            &bytes_written,
            portMAX_DELAY);
    }

    fclose(f);
}

/* ===================== UART TASK ===================== */

static void uart_task(void *arg)
{
    uint8_t buf[UART_BUF_SZ];

    while (1)
    {
        int len = uart_read_bytes(
            UART_PORT,
            buf,
            UART_BUF_SZ - 1,
            20 / portTICK_PERIOD_MS);

        if (len > 0)
        {
            buf[len] = 0;
            ESP_LOGI(TAG, "UART RX: %s", buf);

            if (strstr((char *)buf, "PERSON"))
                play_wav("/spiffs/person.wav");

            else if (strstr((char *)buf, "CAR"))
                play_wav("/spiffs/car.wav");

            else if (strstr((char *)buf, "DOG"))
                play_wav("/spiffs/dog.wav");
        }
    }
}

/* ===================== MAIN ===================== */

void app_main(void)
{
    init_amp();
    init_uart();
    init_i2s();
    init_spiffs();

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System ready");
}
