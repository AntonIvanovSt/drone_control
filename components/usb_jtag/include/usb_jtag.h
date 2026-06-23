#pragma once
#include "driver/usb_serial_jtag.h"
#include "robot_state.h"
#include "stdint.h"
#include <stdbool.h>

#define BUF_SIZE 256

void init_usb_jtag(void);

/** Returns true when a complete '\n'-terminated line has been received. */
bool receive_usb_jtag(void);

/** Parses "L,R\n" speed command and writes PWM outputs. */
void parse_speed_cmd(int *l_pwm_speed_a, int *l_pwm_speed_b, int *r_pwm_speed_a,
                     int *r_pwm_speed_b);
bool parse_pose_cmd(robot_state_t *state);

const char *get_line_buf(void);
/**
 * Sends a single telemetry line:
 *   POS X=%.2f Y=%.2f Th=%.2f  ENC L=%lld R=%lld  SPD L=%.2f mm/s R=%.2f mm/s
 */
void send_telemetry(const robot_state_t *state);
