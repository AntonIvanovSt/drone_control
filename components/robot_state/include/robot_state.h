#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    // Odometry
    float x_pos; // mm
    float y_pos; // mm
    float theta; // radians

    // Filtered wheel speeds
    float vl_speed; // mm/s filtered left wheel speed
    float vr_speed; // mm/s filtered right wheel speed

    // Target wheel speeds
    float target_vl; // mm/s target left wheel speed
    float target_vr; // mm/s target right wheel speed

    // Raw encoders
    int64_t left_enc;  // raw left encoder data
    int64_t right_enc; // raw right encoder data

    // Straight movement compensate
    int64_t left_base;  // left encoder value when straight move starts
    int64_t right_base; // right encoder value when straight move starts
    bool going_straight;

    // Target linear and angular speeds
    float target_linear;  // parsed target linear velocity
    float target_angular; // parsed target angular velocity
} robot_state_t;
