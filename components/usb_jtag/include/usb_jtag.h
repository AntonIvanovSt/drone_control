#pragma once
#include "driver/usb_serial_jtag.h"
#include "pid.h"
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

/* Parses SPD L=<f> R=<f>\n
 * updates state->target_vl, state->target_vr
 */
bool parse_speed_target_cmd(robot_state_t *state);

/* Parses POSE X=<f> Y=<f> Th=<f>\n
 * updates state->x_pos, state->y_pos, state->theta
 */
bool parse_pose_cmd(robot_state_t *state);

/* Parses PID Kp=<f> Ki=<f> Kd=<f> Kff=<f>
 * updates pid_ctrl_t struct
 */
void parse_pid_cmd(pid_ctrl_t *pid);

/*Parses L_A_SPD <f> <f>
 * updates state->target_linear state->target_angular*/
void parse_l_a_spd_cmd(robot_state_t *state);

/*Returns last full received line*/
const char *get_line_buf(void);

/**
 * Sends a single telemetry line:
 *   POS X=%.2f Y=%.2f Th=%.2f  ENC L=%lld R=%lld  SPD L=%.2f mm/s R=%.2f mm/s
 */
void send_telemetry(const robot_state_t *state);
