#include "pid.h"
#include "esp_timer.h"
#include "robot_state.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

void reset_pid(pid_ctrl_t *pid, robot_state_t *state) {
    pid->error_l_integral = 0;
    pid->error_r_integral = 0;
    pid->prev_error_l = 0;
    pid->prev_error_r = 0;
    pid->last_pid_time = esp_timer_get_time();
    state->vl_speed = 0;
    state->vr_speed = 0;
}

void compute_pid(robot_state_t *state, pid_ctrl_t *pid, int *l_pwm_a,
                 int *l_pwm_b, int *r_pwm_a, int *r_pwm_b) {

    if (state->target_vl != pid->last_target_l) {
        reset_pid(pid, state);
        pid->last_target_l = state->target_vl;
    }
    if (state->target_vr != pid->last_target_r) {
        reset_pid(pid, state);
        pid->last_target_r = state->target_vr;
    }
    if (state->target_vl == 0 && state->target_vr == 0) {
        reset_pid(pid, state);
    }

    uint64_t now = esp_timer_get_time();
    float dt = (now - pid->last_pid_time) / 1000000.0f;

    if (dt < 0.001f)
        dt = 0.001f;
    pid->last_pid_time = now;

    float error_l = state->target_vl - state->vl_speed;
    float error_r = state->target_vr - state->vr_speed;

    pid->error_l_integral += error_l * dt;
    pid->error_r_integral += error_r * dt;

    if (pid->error_l_integral > pid->integral_limit)
        pid->error_l_integral = pid->integral_limit;
    if (pid->error_l_integral < -pid->integral_limit)
        pid->error_l_integral = -pid->integral_limit;
    if (pid->error_r_integral > pid->integral_limit)
        pid->error_r_integral = pid->integral_limit;
    if (pid->error_r_integral < -pid->integral_limit)
        pid->error_r_integral = -pid->integral_limit;

    float derivative_l = (error_l - pid->prev_error_l) / dt;
    float derivative_r = (error_r - pid->prev_error_r) / dt;

    pid->prev_error_l = error_l;
    pid->prev_error_r = error_r;

    float pid_l = pid->kp * error_l + pid->ki * pid->error_l_integral +
                  pid->kd * derivative_l;
    float pid_r = pid->kp * error_r + pid->ki * pid->error_r_integral +
                  pid->kd * derivative_r;

    float output_l = pid_l + (pid->kff * state->target_vl);
    float output_r = pid_r + (pid->kff * state->target_vr);

    // forward moving compensate
    if (state->going_straight && fabs(state->target_vl) > 1.0 &&
        fabs(state->target_vr) > 1.0) {
        int64_t left_ticks = (int64_t)(state->left_enc - state->left_base);
        int64_t right_ticks = (int64_t)(state->right_enc - state->right_base);
        int64_t diff = left_ticks - right_ticks;
        float k_enc_straight = 0.1f;
        float corr = k_enc_straight * (float)diff;
        output_l -= corr * 0.5f;
        output_r += corr * 0.5f;
    }
    // end compensate

    if (output_l > 100.0f)
        output_l = 100.0f;
    if (output_l < -100.0f)
        output_l = -100.0f;

    if (output_r > 100.0f)
        output_r = 100.0f;
    if (output_r < -100.0f)
        output_r = -100.0f;

    if (output_l > 0) {
        *l_pwm_a = 0;
        *l_pwm_b = output_l;
    } else {
        *l_pwm_a = -output_l;
        *l_pwm_b = 0;
    }

    if (output_r > 0) {
        *r_pwm_a = 0;
        *r_pwm_b = output_r;
    } else {
        *r_pwm_a = -output_r;
        *r_pwm_b = 0;
    }
}
