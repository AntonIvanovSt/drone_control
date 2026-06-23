#pragma once
#include "robot_state.h"
#include <stdint.h>

void update_odometry(robot_state_t *state, int64_t *last_left_enc,
                     int64_t *last_right_enc);

void compute_speed(robot_state_t *state, int64_t *last_speed_l_enc,
                   int64_t *last_speed_r_enc, int64_t *last_time_l,
                   int64_t *last_time_r);
