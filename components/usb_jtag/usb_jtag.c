#include "usb_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TAG "USB_JTAG"

static char buffer[256];
static int buf_pos = 0;

void init_usb_jtag(void) {
    usb_serial_jtag_driver_config_t usb_serial_conf = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_conf));
}

bool receive_usb_jtag(void) {
    uint8_t data[BUF_SIZE];
    int len = usb_serial_jtag_read_bytes(data, BUF_SIZE - 1,
                                         100 / portTICK_PERIOD_MS);
    if (len > 0) {
        data[len] = '\0';
        for (int i = 0; i < len; i++) {
            char c = data[i];
            if (c == '\n') {
                buffer[buf_pos] = '\0';
                buf_pos = 0;
                return true;
            } else if (buf_pos < sizeof(buffer) - 1) {
                buffer[buf_pos++] = c;
            } else {
                buf_pos = 0;
                return false;
            }
        }
    }
    return false;
}

void parse_speed_cmd(int *l_pwm_speed_a, int *l_pwm_speed_b, int *r_pwm_speed_a,
                     int *r_pwm_speed_b) {
    char reply[128];
    int raw_l_pwm, raw_r_pwm;

    if (sscanf(buffer, "%d,%d", &raw_l_pwm, &raw_r_pwm) == 2) {

        if (raw_l_pwm < 0) {
            *l_pwm_speed_a = abs(raw_l_pwm);
            *l_pwm_speed_b = 0;
        } else {
            *l_pwm_speed_b = raw_l_pwm;
            *l_pwm_speed_a = 0;
        }

        if (raw_r_pwm < 0) {
            *r_pwm_speed_a = abs(raw_r_pwm);
            *r_pwm_speed_b = 0;
        } else {
            *r_pwm_speed_b = raw_r_pwm;
            *r_pwm_speed_a = 0;
        }

        int reply_len = snprintf(
            reply, sizeof(reply),
            "Speed set to l_a=%d, l_b=%d, r_a=%d, r_b=%d\n", *l_pwm_speed_a,
            *l_pwm_speed_b, *r_pwm_speed_a, *r_pwm_speed_b);
        usb_serial_jtag_write_bytes((uint8_t *)reply, reply_len,
                                    100 / portTICK_PERIOD_MS);
    } else {
        int reply_len = snprintf(reply, sizeof(reply), "Parse error\n");
        usb_serial_jtag_write_bytes((uint8_t *)reply, reply_len,
                                    100 / portTICK_PERIOD_MS);
    }
}

void send_enc_value(int64_t *l_enc_value, int64_t *r_enc_value) {
    char reply[64];

    int reply_len = snprintf(reply, sizeof(reply), "ENC L=%lld R=%lld\n",
                             *l_enc_value, *r_enc_value);
    usb_serial_jtag_write_bytes((uint8_t *)reply, reply_len,
                                100 / portTICK_PERIOD_MS);
}
