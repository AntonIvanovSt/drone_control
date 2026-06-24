#pragma once

#include "robot_state.h"
#include <stdint.h>
typedef struct {
    float kp;
    float ki;
    float kd;
    float kff;
    float integral_limit;
    float error_l_integral;
    float error_r_integral;
    float prev_error_l;
    float prev_error_r;
    uint64_t last_pid_time;
    float last_target_l;
    float last_target_r;
} pid_ctrl_t;

void compute_pid(robot_state_t *state, pid_ctrl_t *pid, int *l_pwm_a,
                 int *l_pwm_b, int *r_pwm_a, int *r_pwm_b);
