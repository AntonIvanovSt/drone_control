#pragma once
#include "driver/usb_serial_jtag.h"
#include <stdbool.h>

#define BUF_SIZE (1024)

void init_usb_jtag(void);
bool receive_usb_jtag(void);
void send_enc_value(int64_t *l_enc_value, int64_t *r_enc_value);
void parse_speed_cmd(int *l_pwm_speed_a, int *l_pwm_speed_b, int *r_pwm_speed_a,
                     int *r_pwm_speed_b);
