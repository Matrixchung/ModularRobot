#ifndef _CONTROL_H
#define _CONTROL_H

#include "main.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "sliding_filter.h"
#include "motor.h"
#include "encoder.h"
#define MOTOR_LEFT_POS_DEADZONE 2000
#define MOTOR_LEFT_NEG_DEADZONE 2000
#define MOTOR_RIGHT_POS_DEADZONE 2000
#define MOTOR_RIGHT_NEG_DEADZONE 2000
extern uint8_t control_state;
extern uint8_t pid_state;
extern uint8_t stable_flag;
extern uint8_t current_mode, yaw_set_flag, turn_state_flag;
extern uint8_t yaw_correction_mode;
extern float main_speed, turn_speed, angle_target, yaw_target, distance_target;
extern PID_TypeDef angle_pid, omega_pid, yaw_pid, distance_left_pid, distance_right_pid;
extern LPFilter_TypeDef gyro_x_filter, gyro_y_filter, gyro_z_filter;
extern SlidingFilter_TypeDef yaw_filter;
void control_init(void);
void set_distance_target(float target);
void control_loop(void);
void Key_Onboard_Callback(void);
#endif