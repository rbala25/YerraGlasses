#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"

#include "esp_log.h"
// #include "esp_system.h"
#include "esp_spiffs.h"
// #include "esp_vfs.h"

#define TAG "YERRA"

/* ==========label map========*/

typedef struct
{
    const char *label;
    const char *file;
} label_map_t;

static const label_map_t label_map[] = {

    {"person", "/spiffs/person.wav"},
    {"bicycle", "/spiffs/bicycle.wav"},
    {"car", "/spiffs/car.wav"},
    {"motorcycle", "/spiffs/motorcycle.wav"},
    {"airplane", "/spiffs/airplane.wav"},
    {"bus", "/spiffs/bus.wav"},
    {"train", "/spiffs/train.wav"},
    {"truck", "/spiffs/truck.wav"},
    {"boat", "/spiffs/boat.wav"},
    {"traffic light", "/spiffs/traffic_light.wav"},
    {"fire hydrant", "/spiffs/fire_hydrant.wav"},
    {"stop sign", "/spiffs/stop_sign.wav"},
    {"parking meter", "/spiffs/parking_meter.wav"},
    {"bench", "/spiffs/bench.wav"},
    {"bird", "/spiffs/bird.wav"},
    {"cat", "/spiffs/cat.wav"},
    {"dog", "/spiffs/dog.wav"},
    {"horse", "/spiffs/horse.wav"},
    {"sheep", "/spiffs/sheep.wav"},
    {"cow", "/spiffs/cow.wav"},
    {"elephant", "/spiffs/elephant.wav"},
    {"bear", "/spiffs/bear.wav"},
    {"zebra", "/spiffs/zebra.wav"},
    {"giraffe", "/spiffs/giraffe.wav"},
    {"backpack", "/spiffs/backpack.wav"},
    {"umbrella", "/spiffs/umbrella.wav"},
    {"handbag", "/spiffs/handbag.wav"},
    {"tie", "/spiffs/tie.wav"},
    {"suitcase", "/spiffs/suitcase.wav"},
    {"frisbee", "/spiffs/frisbee.wav"},
    {"skis", "/spiffs/skis.wav"},
    {"snowboard", "/spiffs/snowboard.wav"},
    {"sports ball", "/spiffs/sports_ball.wav"},
    {"kite", "/spiffs/kite.wav"},
    {"baseball bat", "/spiffs/baseball_bat.wav"},
    {"baseball glove", "/spiffs/baseball_glove.wav"},
    {"skateboard", "/spiffs/skateboard.wav"},
    {"surfboard", "/spiffs/surfboard.wav"},
    {"tennis racket", "/spiffs/tennis_racket.wav"},
    {"bottle", "/spiffs/bottle.wav"},
    {"wine glass", "/spiffs/wine_glass.wav"},
    {"cup", "/spiffs/cup.wav"},
    {"fork", "/spiffs/fork.wav"},
    {"knife", "/spiffs/knife.wav"},
    {"spoon", "/spiffs/spoon.wav"},
    {"bowl", "/spiffs/bowl.wav"},
    {"banana", "/spiffs/banana.wav"},
    {"apple", "/spiffs/apple.wav"},
    {"sandwich", "/spiffs/sandwich.wav"},
    {"orange", "/spiffs/orange.wav"},
    {"brocolli", "/spiffs/brocolli.wav"},
    {"carrot", "/spiffs/carrot.wav"},
    {"hot dog", "/spiffs/hot_dog.wav"},
    {"pizza", "/spiffs/pizza.wav"},
    {"donut", "/spiffs/donut.wav"},
    {"cake", "/spiffs/cake.wav"},
    {"chair", "/spiffs/chair.wav"},
    {"couch", "/spiffs/couch.wav"},
    {"potted plant", "/spiffs/potted_plant.wav"},
    {"bed", "/spiffs/bed.wav"},
    {"dining table", "/spiffs/dining_table.wav"},
    {"toilet", "/spiffs/toilet.wav"},
    {"tv", "/spiffs/tv.wav"},
    {"laptop", "/spiffs/laptop.wav"},
    {"mouse", "/spiffs/mouse.wav"},
    {"remote", "/spiffs/remote.wav"},
    {"keyboard", "/spiffs/keyboard.wav"},
    {"cell phone", "/spiffs/cell_phone.wav"},
    {"microwave", "/spiffs/microwave.wav"},
    {"oven", "/spiffs/oven.wav"},
    {"toaster", "/spiffs/toaster.wav"},
    {"sink", "/spiffs/sink.wav"},
    {"refrigerator", "/spiffs/refrigerator.wav"},
    {"book", "/spiffs/book.wav"},
    {"clock", "/spiffs/clock.wav"},
    {"vase", "/spiffs/vase.wav"},
    {"scissors", "/spiffs/scissors.wav"},
    {"teddy bear", "/spiffs/teddy_bear.wav"},
    {"hair drier", "/spiffs/hair_drier.wav"},
    {"toothbrush", "/spiffs/toothbrush.wav"}

};

#define NUM_LABELS (sizeof(label_map) / sizeof(label_map[0]))

/* ================= UART ================= */

#define UART_PORT   UART_NUM_1
#define UART_RX_PIN 4
#define UART_TX_PIN 5
#define UART_BAUD   115200
#define UART_BUF_SZ 256

/* ================= i2s ================= */
#define I2S_BCLK_PIN 10
#define I2S_LRCLK_PIN 11
#define I2S_DATA_PIN 9
#define AMP_EN_PIN 12

static i2s_chan_handle_t i2s_tx_chan;

/* ================= audio Queue ================= */
static QueueHandle_t audio_queue;
static volatile bool speaking = false;

/* ================= amp ================= */

static void init_amp(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << AMP_EN_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_cfg);
    gpio_set_level(AMP_EN_PIN, 1);
}

/* ================= UART config ================= */

static void init_uart(void)
{
    uart_config_t conf = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &conf));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SZ * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART initialized at %d baud", UART_BAUD);
}

/* ================= i2s ================= */

static void init_i2s(void)
{
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 240;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_MONO),
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

/* ================= spiffs ================= */

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

/* ================= wav player ================= */

static void play_wav(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return;
    }

    uint8_t wav_header[44];
    fread(wav_header, 1, 44, f);

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

/* ================= audio task ================= */

static void audio_task(void *arg)
{
    char filepath[64];

    while (1)
    {
        if (xQueueReceive(audio_queue, filepath, portMAX_DELAY))
        {
            speaking = true;
            ESP_LOGI(TAG, "Speaking: %s", filepath);
            play_wav(filepath);
            speaking = false;
        }
    }
}

// static void audio_task(void *arg)
// {
//     while (1)
//     {
//         speaking = true;
//         ESP_LOGI(TAG, "Looping person.wav");
//         play_wav("/spiffs/person.wav");
//     }
// }

/* ================= UART task ================= */

static void uart_task(void *arg)
{
    uint8_t buf[UART_BUF_SZ];
    char line[UART_BUF_SZ];
    int line_pos = 0;

    while (1)
    {
        int len = uart_read_bytes(UART_PORT, buf, UART_BUF_SZ - 1, pdMS_TO_TICKS(10));

        for (int i = 0; i < len; i++)
        {
            if (buf[i] == '\n')
            {
                line[line_pos] = 0;
                line_pos = 0;

                char label[32];
                float prob;
                if (sscanf(line, "%31[^:]:%f", label, &prob) == 2)
                {
                    if (prob < 0.5f || speaking)
                        continue;

                    char filepath[64];
                    bool found = false;
                    for (int j = 0; j < NUM_LABELS; j++)
                    {
                        if (strcmp(label, label_map[j].label) == 0)
                        {
                            strcpy(filepath, label_map[j].file);
                            found = true;
                            break;
                        }
                    }

                    if (!found)
                    {
                        ESP_LOGW(TAG, "Unknown label: %s", label);
                        continue;
                    }

                    if (xQueueSend(audio_queue, filepath, 0) != pdTRUE)
                    {
                        ESP_LOGW(TAG, "Audio queue full, dropping: %s", filepath);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Queued: %s (%.2f)", filepath, prob);
                    }
                }
            }
            else if (buf[i] != '\r')
            {
                if (line_pos < UART_BUF_SZ - 1)
                    line[line_pos++] = buf[i];
                else
                {
                    ESP_LOGW(TAG, "Line buffer overflow, resetting");
                    line_pos = 0;
                }
            }
        }
    }
}

/* ================= main ================= */

void app_main(void)
{
    init_amp();
    init_uart();
    init_i2s();
    init_spiffs();

    audio_queue = xQueueCreate(5, sizeof(char[64]));

    xTaskCreate(audio_task, "audio_task", 4096, NULL, 5, NULL);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System ready");
}
