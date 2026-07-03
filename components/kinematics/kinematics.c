#include "kinematics.h"
#include "esp_timer.h"
#include <math.h>
#include <stdint.h>

static const float WHEEL_DIAMETER = 69.0f;
static const float WHEEL_BASE = 260.0f;
static const float ENCODER_RESOLUTION = 1320.0f;
static const float TICKS_TO_MM = (M_PI * WHEEL_DIAMETER) / ENCODER_RESOLUTION;
static const float ALPHA = 0.5f;

void set_robot_velocity(pid_ctrl_t *pid, robot_state_t *state) {
    if (fabs(state->target_linear) < 10 && fabs(state->target_angular) < 0.1) {
        state->target_vl = 0;
        state->target_vr = 0;
        reset_pid(pid, state);
        state->going_straight = false;
        return;
    }
    if (fabs(state->target_angular) < 0.0001f) {
        if (!state->going_straight) {
            state->left_base = state->left_enc;
            state->right_base = state->right_enc;
        }
        state->going_straight = true;
    } else {
        state->going_straight = false;
    }
    // compute l/r wheels speed via inverse kinematics
    state->target_vl =
        state->target_linear - (state->target_angular * WHEEL_BASE / 2.0);
    state->target_vr =
        state->target_linear + (state->target_angular * WHEEL_BASE / 2.0);
}

void update_odometry(robot_state_t *state, int64_t *last_left_enc,
                     int64_t *last_right_enc) {
    long d_left = (long)(state->left_enc - *last_left_enc);
    long d_right = (long)(state->right_enc - *last_right_enc);
    *last_left_enc = state->left_enc;
    *last_right_enc = state->right_enc;

    float dist_l = d_left * TICKS_TO_MM;
    float dist_r = d_right * TICKS_TO_MM;
    float d_s = (dist_l + dist_r) / 2.0f;
    float d_theta = (dist_r - dist_l) / WHEEL_BASE;

    state->theta += d_theta;
    state->x_pos += d_s * cosf(state->theta);
    state->y_pos += d_s * sinf(state->theta);
}

void compute_speed(robot_state_t *state, int64_t *last_speed_l_enc,
                   int64_t *last_speed_r_enc, int64_t *last_time_l,
                   int64_t *last_time_r) {
    int64_t now = esp_timer_get_time(); // µs

    // First call — just capture baseline, no speed yet
    if (*last_time_l == 0 || *last_time_r == 0) {
        *last_time_l = now;
        *last_time_r = now;
        *last_speed_l_enc = state->left_enc;
        *last_speed_r_enc = state->right_enc;
        return;
    }

    float dt_l = (float)(now - *last_time_l) * 1e-6f; // seconds
    *last_time_l = now;

    if (dt_l > 0.0f && state->left_enc != *last_speed_l_enc) {
        long delta_l = (long)(state->left_enc - *last_speed_l_enc);
        float vl = ((float)delta_l * TICKS_TO_MM) / dt_l;
        state->vl_speed = ALPHA * vl + (1.0f - ALPHA) * state->vl_speed;
        *last_speed_l_enc = state->left_enc;
        *last_time_l = now;
    } else if (dt_l > 0.1f) {
        state->vl_speed = (1.0f - ALPHA) * state->vl_speed; // wheel stopped
    }

    float dt_r = (float)(now - *last_time_r) * 1e-6f;
    *last_time_r = now;

    if (dt_r > 0.0f && state->right_enc != *last_speed_r_enc) {
        long delta_r = (long)(state->right_enc - *last_speed_r_enc);
        float vr = ((float)delta_r * TICKS_TO_MM) / dt_r;
        state->vr_speed = ALPHA * vr + (1.0f - ALPHA) * state->vr_speed;
        *last_speed_r_enc = state->right_enc;
        *last_time_r = now;
    } else if (dt_r > 0.1f) {
        state->vr_speed = (1.0f - ALPHA) * state->vr_speed; // wheel stopped
    }
}
