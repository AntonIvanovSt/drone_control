#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "portmacro.h"
#include <stdint.h>
#include <stdio.h>

#define TAG "ENCODER"

#define PCNT_HIGH_LIMIT 32767
#define PCNT_LOW_LIMIT -32768

static bool IRAM_ATTR
pcnt_overflow_callback(pcnt_unit_handle_t unit,
                       const pcnt_watch_event_data_t *edata, void *user_ctx) {
    encoder_t *enc = (encoder_t *)user_ctx;
    enc->accumulator += edata->watch_point_value;
    return false;
}

void encoder_init(encoder_t *enc, int pin_a, int pin_b, int direction,
                  const char *name) {
    enc->pin_a = pin_a;
    enc->pin_b = pin_b;
    enc->direction = direction;
    enc->name = name;
    enc->accumulator = 0;

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &enc->pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(enc->pcnt_unit, &filter_config));

    pcnt_chan_config_t chan0_config = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
    };
    pcnt_channel_handle_t pcnt_chan0 = NULL;
    ESP_ERROR_CHECK(
        pcnt_new_channel(enc->pcnt_unit, &chan0_config, &pcnt_chan0));
    pcnt_channel_set_edge_action(pcnt_chan0, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan0, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_chan_config_t chan1_config = {
        .edge_gpio_num = pin_b,
        .level_gpio_num = pin_a,
    };
    pcnt_channel_handle_t pcnt_chan1 = NULL;
    ESP_ERROR_CHECK(
        pcnt_new_channel(enc->pcnt_unit, &chan1_config, &pcnt_chan1));
    pcnt_channel_set_edge_action(pcnt_chan1, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan1, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc->pcnt_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc->pcnt_unit, PCNT_LOW_LIMIT));

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_overflow_callback,
    };
    // here we pass struct to callback
    ESP_ERROR_CHECK(
        pcnt_unit_register_event_callbacks(enc->pcnt_unit, &cbs, enc));

    ESP_ERROR_CHECK(pcnt_unit_enable(enc->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(enc->pcnt_unit));

    ESP_LOGI(TAG, "Encoder '%s' initialized on pins A=%d B=%d", name, pin_a,
             pin_b);
}

int64_t encoder_get_count(encoder_t *enc) {
    int raw = 0;
    portDISABLE_INTERRUPTS();
    pcnt_unit_get_count(enc->pcnt_unit, &raw);
    int64_t count = (enc->accumulator + raw) * enc->direction;
    portENABLE_INTERRUPTS();
    return count;
}

void encoder_reset(encoder_t *enc) {
    enc->accumulator = 0;
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc->pcnt_unit));
}
