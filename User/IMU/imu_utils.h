#ifndef _IMU_UTILS_H
#define _IMU_UTILS_H

#define _PI 3.141592653589793238462643383279f
#define _1_PI 0.318309886183871f
#define DEG_2_RAD 0.01745329251994329576923690768489f
#define RAD_2_DEG 57.295779513082320876798154814105f

#include "arm_math.h"
#include "math.h"
#include "delay.h"
#include "math_utils.h"

void MadgwickAHRSupdate_6(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw);
void MadgwickAHRSupdate_9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *pitch, float *roll, float *yaw);
float get_calibrated_yaw(float roll, float pitch, float mag_x, float mag_y, float mag_z);

#endif