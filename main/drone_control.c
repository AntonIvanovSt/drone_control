#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "kinematics.h"
#include "mcpwm.h"
#include "robot_state.h"
#include "usb_jtag.h"
#include <stdint.h>

#define TAG "MAIN"

// ── Encoder GPIO pins
// ─────────────────────────────────────────────────────────
#define ENC_LEFT_A 11
#define ENC_LEFT_B 12
#define ENC_RIGHT_A 13
#define ENC_RIGHT_B 14

// ── Control loop period
// ───────────────────────────────────────────────────────
#define LOOP_PERIOD_MS 50

void app_main(void) {
    // ── Hardware init
    // ─────────────────────────────────────────────────────────
    encoder_t enc_left, enc_right;
    encoder_init(&enc_left, ENC_LEFT_A, ENC_LEFT_B, -1, "LEFT");
    encoder_init(&enc_right, ENC_RIGHT_A, ENC_RIGHT_B, +1, "RIGHT");

    motor_handle_t l_motor = {0};
    motor_handle_t r_motor = {0};
    mcpwm_motor_init(&l_motor, 17, 18, 0); // left:  MCPWM group 0
    mcpwm_motor_init(&r_motor, 15, 16, 1); // right: MCPWM group 1

    init_usb_jtag();

    // ── Shared robot state
    // ────────────────────────────────────────────────────
    robot_state_t state = {0};

    // ── Kinematics "last" state (owned here, passed to pure functions)
    // ────────
    int64_t last_odom_l = 0, last_odom_r = 0;   // for odometry deltas
    int64_t last_speed_l = 0, last_speed_r = 0; // for speed estimation
    int64_t last_time_l = 0, last_time_r = 0;   // timestamps (µs)

    // ── PWM outputs
    // ───────────────────────────────────────────────────────────
    int l_pwm_a = 0, l_pwm_b = 0;
    int r_pwm_a = 0, r_pwm_b = 0;

    while (1) {
        // Read encoders into shared state
        state.left_enc = encoder_get_count(&enc_left);
        state.right_enc = encoder_get_count(&enc_right);

        // Update kinematics
        update_odometry(&state, &last_odom_l, &last_odom_r);
        compute_speed(&state, &last_speed_l, &last_speed_r, &last_time_l,
                      &last_time_r);

        // Send telemetry
        send_telemetry(&state);

        // Handle incoming speed commands
        if (receive_usb_jtag()) {
            if (strncmp(get_line_buf(), "POSE", 4) == 0) {
                parse_pose_cmd(&state);
            } else {
                parse_speed_cmd(&l_pwm_a, &l_pwm_b, &r_pwm_a, &r_pwm_b);
            }
        }

        // Apply PWM
        motor_set_duty(&l_motor, duty_ticks(l_pwm_a), duty_ticks(l_pwm_b));
        motor_set_duty(&r_motor, duty_ticks(r_pwm_a), duty_ticks(r_pwm_b));

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
