#include "usb_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TAG "USB_JTAG"
#define JTAG_TIMEOUT_MS (100 / portTICK_PERIOD_MS)

// Internal line buffer — filled incrementally by receive_usb_jtag()
static char s_line_buf[BUF_SIZE];
static int s_buf_pos = 0;

// ── Init
// ──────────────────────────────────────────────────────────────────────

void init_usb_jtag(void) {
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
}

// ── Receive
// ───────────────────────────────────────────────────────────────────

bool receive_usb_jtag(void) {
    uint8_t data[BUF_SIZE];
    int len = usb_serial_jtag_read_bytes(data, BUF_SIZE - 1, JTAG_TIMEOUT_MS);

    for (int i = 0; i < len; i++) {
        char c = (char)data[i];

        if (c == '\n') {
            s_line_buf[s_buf_pos] = '\0';
            s_buf_pos = 0;
            return true; // complete line ready in s_line_buf
        }

        if (s_buf_pos < (int)sizeof(s_line_buf) - 1) {
            s_line_buf[s_buf_pos++] = c;
        } else {
            // Buffer overflow — discard and start over
            s_buf_pos = 0;
        }
    }
    return false;
}

// ── Commands
// ──────────────────────────────────────────────────────────────────

void parse_speed_cmd(int *l_pwm_speed_a, int *l_pwm_speed_b, int *r_pwm_speed_a,
                     int *r_pwm_speed_b) {
    char reply[128];
    int raw_l, raw_r;

    if (sscanf(s_line_buf, "%d,%d", &raw_l, &raw_r) != 2) {
        int n = snprintf(reply, sizeof(reply), "ERR parse_error\n");
        usb_serial_jtag_write_bytes((uint8_t *)reply, n, JTAG_TIMEOUT_MS);
        return;
    }

    *l_pwm_speed_a = (raw_l < 0) ? abs(raw_l) : 0;
    *l_pwm_speed_b = (raw_l >= 0) ? raw_l : 0;
    *r_pwm_speed_a = (raw_r < 0) ? abs(raw_r) : 0;
    *r_pwm_speed_b = (raw_r >= 0) ? raw_r : 0;

    int n = snprintf(reply, sizeof(reply), "OK l_a=%d l_b=%d r_a=%d r_b=%d\n",
                     *l_pwm_speed_a, *l_pwm_speed_b, *r_pwm_speed_a,
                     *r_pwm_speed_b);
    usb_serial_jtag_write_bytes((uint8_t *)reply, n, JTAG_TIMEOUT_MS);
}

bool parse_pose_cmd(robot_state_t *state) {
    char reply[64];
    float x, y, theta;

    if (sscanf(s_line_buf, "POSE X=%f Y=%f Th=%f", &x, &y, &theta) != 3) {
        int n = snprintf(reply, sizeof(reply), "ERR pose_parse_error\n");
        usb_serial_jtag_write_bytes((uint8_t *)reply, n, JTAG_TIMEOUT_MS);
        return false;
    }

    state->x_pos = x;
    state->y_pos = y;
    state->theta = theta;

    int n = snprintf(reply, sizeof(reply), "OK POSE X=%.2f Y=%.2f Th=%.2f\n", x,
                     y, theta);
    usb_serial_jtag_write_bytes((uint8_t *)reply, n, JTAG_TIMEOUT_MS);
    return true;
}

const char *get_line_buf(void) { return s_line_buf; }

// ── Telemetry
// ─────────────────────────────────────────────────────────────────
void send_telemetry(const robot_state_t *state) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
                     "POS X=%.2f Y=%.2f Th=%.2f "
                     "ENC L=%lld R=%lld "
                     "SPD L=%.2f mm/s R=%.2f mm/s\n",
                     state->x_pos, state->y_pos, state->theta, state->left_enc,
                     state->right_enc, state->vl_speed, state->vr_speed);

    usb_serial_jtag_write_bytes((uint8_t *)buf, n, JTAG_TIMEOUT_MS);
}
