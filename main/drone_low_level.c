#include "driver/usb_serial_jtag.h"
#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <stdint.h>

#define TAG "MAIN"

#define BUF_SIZE (1024)

#define ENC_LEFT_A 11
#define ENC_LEFT_B 12
#define ENC_RIGHT_A 13
#define ENC_RIGHT_B 14

static int l_pwm_speed = 0;
static int r_pwm_speed = 0;

static void parse_speed_cmd(char *buffer) {
    int l_sp;
    int r_sp;

    if (sscanf(buffer, "%d,%d", &l_sp, &r_sp) == 2) {
        ESP_LOGI(TAG, "Parsed: l_speed=%d, r_speed=%d", l_sp, r_sp);
        l_pwm_speed = l_sp;
        r_pwm_speed = r_sp;

        char reply[64];
        int reply_len = snprintf(reply, sizeof(reply),
                                 "Speed set to l:%d, r:%d\n", l_sp, r_sp);
        usb_serial_jtag_write_bytes((uint8_t *)reply, reply_len,
                                    100 / portTICK_PERIOD_MS);
    } else {
        const char *err = "Parse error\n";
        usb_serial_jtag_write_bytes((uint8_t *)err, strlen(err),
                                    100 / portTICK_PERIOD_MS);
        ESP_LOGW(TAG, "Parse error");
    }
}

void app_main(void) {
    encoder_t enc_left, enc_right;

    encoder_init(&enc_left, ENC_LEFT_A, ENC_LEFT_B, -1, "LEFT");
    encoder_init(&enc_right, ENC_RIGHT_A, ENC_RIGHT_B, +1, "RIGHT");

    int64_t last_left = 0, last_right = 0;

    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    usb_serial_jtag_driver_config_t usb_serial_conf = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_conf));

    static char buffer[256];
    static int index = 0;

    while (1) {
        int64_t left = encoder_get_count(&enc_left);
        int64_t right = encoder_get_count(&enc_right);

        if (left != last_left || right != last_right) {
            ESP_LOGI(TAG, "LEFT: %lld  RIGHT: %lld", left, right);
            last_left = left;
            last_right = right;
        }
        int len = usb_serial_jtag_read_bytes(data, BUF_SIZE - 1,
                                             100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            for (int i = 0; i < len; i++) {
                char c = data[i];
                if (c == '\n') {
                    buffer[index] = '\0';
                    ESP_LOGI(TAG, "Complete message: %s", buffer);
                    parse_speed_cmd(buffer);
                    index = 0;
                } else if (index < sizeof(buffer) - 1) {
                    buffer[index++] = c;
                } else {
                    ESP_LOGW(TAG, "Buffer overflow - resetting");
                    index = 0;
                }
            }
        }
    }
    free(data);
    vTaskDelay(pdMS_TO_TICKS(50));
}
