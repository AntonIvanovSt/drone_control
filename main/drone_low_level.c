#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "mcpwm.h"
#include "usb_jtag.h"
#include <stdint.h>

#define TAG "MAIN"

#define ENC_LEFT_A 11
#define ENC_LEFT_B 12
#define ENC_RIGHT_A 13
#define ENC_RIGHT_B 14

void app_main(void) {
    encoder_t enc_left, enc_right;
    motor_handle_t l_motor = {0};
    motor_handle_t r_motor = {0};

    encoder_init(&enc_left, ENC_LEFT_A, ENC_LEFT_B, -1, "LEFT");
    encoder_init(&enc_right, ENC_RIGHT_A, ENC_RIGHT_B, +1, "RIGHT");

    mcpwm_motor_init(&l_motor, 17, 18, 0); // left:  group 0
    mcpwm_motor_init(&r_motor, 15, 16, 1); // right: group 1

    init_usb_jtag();

    int64_t last_left = 0, last_right = 0;

    int l_pwm_speed_a = 0;
    int l_pwm_speed_b = 0;
    int r_pwm_speed_a = 0;
    int r_pwm_speed_b = 0;

    while (1) {
        int64_t left = encoder_get_count(&enc_left);
        int64_t right = encoder_get_count(&enc_right);

        if (left != last_left || right != last_right) {
            send_enc_value(&left, &right);
            last_left = left;
            last_right = right;
        }

        if (receive_usb_jtag()) {
            parse_speed_cmd(&l_pwm_speed_a, &l_pwm_speed_b, &r_pwm_speed_a,
                            &r_pwm_speed_b);
        }
        motor_set_duty(&l_motor, duty_ticks(l_pwm_speed_a),
                       duty_ticks(l_pwm_speed_b));
        motor_set_duty(&r_motor, duty_ticks(r_pwm_speed_a),
                       duty_ticks(r_pwm_speed_b));

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
