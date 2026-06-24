#pragma once
#include <stdint.h>

typedef struct {
    // Odometry
    float x_pos; // mm
    float y_pos; // mm
    float theta; // radians

    // Filtered wheel speeds
    float vl_speed; // mm/s
    float vr_speed; // mm/s

    // Target wheel speeds
    float target_vl; // mm/s
    float target_vr; // mm/s

    // Raw encoders
    int64_t left_enc;
    int64_t right_enc;
} robot_state_t;
