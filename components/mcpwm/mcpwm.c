#include "mcpwm.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/mcpwm_types.h"
#include "soc/clk_tree_defs.h"
#include <stdint.h>
#include <string.h>

static const char *TAG = "MCPWM";

int duty_ticks(int pwm_speed) { return PWM_PERIOD_TICKS * pwm_speed / 100; }

void motor_set_duty(motor_handle_t *motor, uint32_t duty_a, uint32_t duty_b) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->cmp_a, duty_a));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->cmp_b, duty_b));
}

static void create_channel(mcpwm_oper_handle_t oper, int gpio,
                           mcpwm_cmpr_handle_t *out_cmp) {
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, out_cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*out_cmp, 0));

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gen_cfg = {
        .gen_gpio_num = gpio,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                          MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *out_cmp,
                                            MCPWM_GEN_ACTION_LOW)));
}

void mcpwm_motor_init(motor_handle_t *motor, int gpio_a, int gpio_b,
                      int group_id) {
    ESP_LOGI(TAG, "Init motor a: %d, b: %d, group: %d", gpio_a, gpio_b,
             group_id);
    mcpwm_timer_handle_t timer = NULL;

    mcpwm_timer_config_t timer_cfg = {
        .group_id = group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = PWM_PERIOD_TICKS,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    mcpwm_oper_handle_t oper_a = NULL, oper_b = NULL;

    mcpwm_operator_config_t oper_cfg = {.group_id = group_id};
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper_a));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper_b));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_a, timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_b, timer));

    create_channel(oper_a, gpio_a, &motor->cmp_a);
    create_channel(oper_b, gpio_b, &motor->cmp_b);

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Motor ready (group %d)", group_id);
}
