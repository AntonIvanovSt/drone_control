#include "kinematics.h"
#include "esp_timer.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static const float WHEEL_DIAMETER = 69.0f;
static const float WHEEL_BASE = 260.0f;
static const float ENCODER_RESOLUTION = 330.0f;
static const float TICKS_TO_MM = (M_PI * WHEEL_DIAMETER) / ENCODER_RESOLUTION;
static const float ALPHA = 0.2f;

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

    if (dt_l > 0.0f && state->left_enc != *last_speed_l_enc) {
        long delta_l = (long)(state->left_enc - *last_speed_l_enc);
        float ang_deg_s = ((float)delta_l / ENCODER_RESOLUTION) * 360.0f / dt_l;
        float vl = (ang_deg_s / 360.0f) * (M_PI * WHEEL_DIAMETER);
        state->vl_speed = ALPHA * vl + (1.0f - ALPHA) * state->vl_speed;
        *last_speed_l_enc = state->left_enc;
        *last_time_l = now;
    } else if (dt_l > 0.005f) {
        state->vl_speed = 0.0f; // wheel stopped
    }

    float dt_r = (float)(now - *last_time_r) * 1e-6f;

    if (dt_r > 0.0f && state->right_enc != *last_speed_r_enc) {
        long delta_r = (long)(state->right_enc - *last_speed_r_enc);
        float ang_deg_s = ((float)delta_r / ENCODER_RESOLUTION) * 360.0f / dt_r;
        float vr = (ang_deg_s / 360.0f) * (M_PI * WHEEL_DIAMETER);
        state->vr_speed = ALPHA * vr + (1.0f - ALPHA) * state->vr_speed;
        *last_speed_r_enc = state->right_enc;
        *last_time_r = now;
    } else if (dt_r > 0.005f) {
        state->vr_speed = 0.0f; // wheel stopped
    }
}
