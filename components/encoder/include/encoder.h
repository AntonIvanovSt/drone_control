#pragma once
#include "driver/pulse_cnt.h"
#include <stdint.h>

typedef struct {
    pcnt_unit_handle_t pcnt_unit;
    volatile int64_t accumulator;
    int pin_a;
    int pin_b;
    int direction;
    const char *name;
} encoder_t;

void encoder_init(encoder_t *enc, int pin_a, int pin_b, int direction,
                  const char *name);
int64_t encoder_get_count(encoder_t *enc);
void encoder_reset(encoder_t *enc);
