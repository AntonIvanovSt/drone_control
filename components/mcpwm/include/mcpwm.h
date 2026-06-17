#pragma once
#include "driver/mcpwm_types.h"

#define PWM_RESOLUTION_HZ 10000000
#define PWM_PERIOD_TICKS 1000

typedef struct {
    mcpwm_cmpr_handle_t cmp_a;
    mcpwm_cmpr_handle_t cmp_b;
} motor_handle_t;

int duty_ticks(int pwm_speed);
void mcpwm_motor_init(motor_handle_t *motor, int gpio_a, int gpio_b,
                      int group_id);
void motor_set_duty(motor_handle_t *motor, uint32_t duty_a, uint32_t duty_b);
