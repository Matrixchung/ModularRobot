#include "imu_utils.h"
#define BETA 0.1f
#define GYRO_K (0.0010642251536551f) // (1.0f/16.4f/RAD_2_DEG) // 16.4f is based on GY953 settings
#define ACC_K  (0.00006103515625f) // (1.0f/16384.0f) // based on gy953 settings
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
void MadgwickAHRSupdate_6(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw) 
{
    static uint32_t madgwick_timer = 0;
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    float Ts = (get_micros() - madgwick_timer) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    madgwick_timer = get_micros();
    ax *= ACC_K;
    ay *= ACC_K;
    az *= ACC_K;
	gx *= GYRO_K;
	gy *= GYRO_K;
	gz *= GYRO_K;
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
		// Normalise accelerometer measurement
		recipNorm = fast_invsqrt(ax * ax + ay * ay + az * az);
		// ax *= recipNorm;
		// ay *= recipNorm;
		// az *= recipNorm;
        arm_mult_f32(&ax, &recipNorm, &ax, 1);
        arm_mult_f32(&ay, &recipNorm, &ay, 1);
        arm_mult_f32(&az, &recipNorm, &az, 1);
		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		// q0q0 = q0 * q0;
		// q1q1 = q1 * q1;
		// q2q2 = q2 * q2;
		// q3q3 = q3 * q3;
        arm_power_f32(&q0, 1, &q0q0);
        arm_power_f32(&q1, 1, &q1q1);
        arm_power_f32(&q2, 1, &q2q2);
        arm_power_f32(&q3, 1, &q3q3);
		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = fast_invsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		// s0 *= recipNorm;
		// s1 *= recipNorm;
		// s2 *= recipNorm;
		// s3 *= recipNorm;
        arm_mult_f32(&s0, &recipNorm, &s0, 1);
        arm_mult_f32(&s1, &recipNorm, &s1, 1);
        arm_mult_f32(&s2, &recipNorm, &s2, 1);
        arm_mult_f32(&s3, &recipNorm, &s3, 1);
		// Apply feedback step
		qDot1 -= BETA * s0;
		qDot2 -= BETA * s1;
		qDot3 -= BETA * s2;
		qDot4 -= BETA * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * Ts;
	q1 += qDot2 * Ts;
	q2 += qDot3 * Ts;
	q3 += qDot4 * Ts;
	// Normalise quaternion
	recipNorm = fast_invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	// q0 *= recipNorm;
	// q1 *= recipNorm;
	// q2 *= recipNorm;
	// q3 *= recipNorm;
    arm_mult_f32(&q0, &recipNorm, &q0, 1);
    arm_mult_f32(&q1, &recipNorm, &q1, 1);
    arm_mult_f32(&q2, &recipNorm, &q2, 1);
    arm_mult_f32(&q3, &recipNorm, &q3, 1);
    
	*pitch = -asinf(-2.0f * (q1*q3 - q0*q2)) * RAD_2_DEG;
	*roll = -atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * RAD_2_DEG;
	*yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)* RAD_2_DEG;
}
void MadgwickAHRSupdate_9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *pitch, float *roll, float *yaw) 
{
    static uint32_t madgwick_timer = 0;
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
	{
		MadgwickAHRSupdate_6(gx, gy, gz, ax, ay, az, pitch, roll, yaw);
		return;
	}
    float Ts = (get_micros() - madgwick_timer) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    madgwick_timer = get_micros();
    ax *= ACC_K;
    ay *= ACC_K;
    az *= ACC_K;
	gx *= GYRO_K;
	gy *= GYRO_K;
	gz *= GYRO_K;
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{
		// Normalise accelerometer measurement
		recipNorm = fast_invsqrt(ax * ax + ay * ay + az * az);
		// ax *= recipNorm;
		// ay *= recipNorm;
		// az *= recipNorm;   
        arm_mult_f32(&ax, &recipNorm, &ax, 1);
        arm_mult_f32(&ay, &recipNorm, &ay, 1);
        arm_mult_f32(&az, &recipNorm, &az, 1);
		// Normalise magnetometer measurement
		recipNorm = fast_invsqrt(mx * mx + my * my + mz * mz);
        arm_mult_f32(&mx, &recipNorm, &mx, 1);
        arm_mult_f32(&my, &recipNorm, &my, 1);
        arm_mult_f32(&mz, &recipNorm, &mz, 1);
		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;
		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		// _2bx = sqrt(hx * hx + hy * hy);
        arm_sqrt_f32(hx * hx + hy * hy, &_2bx);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = fast_invsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= BETA * s0;
		qDot2 -= BETA * s1;
		qDot3 -= BETA * s2;
		qDot4 -= BETA * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * Ts;
	q1 += qDot2 * Ts;
	q2 += qDot3 * Ts;
	q3 += qDot4 * Ts;
	// Normalise quaternion
	recipNorm = fast_invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	*pitch = asinf(-2.0f * (q1*q3 - q0*q2))* RAD_2_DEG;
	*roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * RAD_2_DEG;
	*yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)* RAD_2_DEG;
}
float get_calibrated_yaw(float roll, float pitch, float mag_x, float mag_y, float mag_z) 
{
    roll *= DEG_2_RAD;
    pitch *= DEG_2_RAD;
    float rotx[3][3];
    float roty[3][3];
    float rotMatrixDot[3][3];
    float magnetCalibVector[3];
    float directionXY= -180;

    //----------------Matrix rotx--------------------
    rotx[0][0]  = 1;
    rotx[0][1]  = 0;
    rotx[0][2]  = 0;

    rotx[1][0]  = 0;
    rotx[1][1]  =  arm_cos_f32(roll);
    rotx[1][2]  = -arm_sin_f32(roll);

    rotx[2][0]  = 0;
    rotx[2][1]  =  arm_sin_f32(roll);
    rotx[2][2]  =  arm_cos_f32(roll);

    //----------------Matrix roty--------------------
    roty[0][0]  = arm_cos_f32(pitch);
    roty[0][1]  = 0;
    roty[0][2]  = arm_sin_f32(pitch);

    roty[1][0]  = 0;
    roty[1][1]  = 1;
    roty[1][2]  = 0;

    roty[2][0]  = -arm_sin_f32(pitch);
    roty[2][1]  = 0;
    roty[2][2]  =  arm_cos_f32(pitch);

    //----------------rotx * roty--------------------
    rotMatrixDot[0][0]  = rotx[0][0]*roty[0][0] + rotx[0][1]*roty[1][0] + rotx[0][2]*roty[2][0];
    rotMatrixDot[0][1]  = rotx[0][0]*roty[0][1] + rotx[0][1]*roty[1][1] + rotx[0][2]*roty[2][1];
    rotMatrixDot[0][2]  = rotx[0][0]*roty[0][2] + rotx[0][1]*roty[1][2] + rotx[0][2]*roty[2][2];

    rotMatrixDot[1][0]  = rotx[1][0]*roty[0][0] + rotx[1][1]*roty[1][0] + rotx[1][2]*roty[2][0];
    rotMatrixDot[1][1]  = rotx[1][0]*roty[0][1] + rotx[1][1]*roty[1][1] + rotx[1][2]*roty[2][1];
    rotMatrixDot[1][2]  = rotx[1][0]*roty[0][2] + rotx[1][1]*roty[1][2] + rotx[1][2]*roty[2][2];

    rotMatrixDot[2][0]  = rotx[2][0]*roty[0][0] + rotx[2][1]*roty[1][0] + rotx[2][2]*roty[2][0];
    rotMatrixDot[2][1]  = rotx[2][0]*roty[0][1] + rotx[2][1]*roty[1][1] + rotx[2][2]*roty[2][1];
    rotMatrixDot[2][2]  = rotx[2][0]*roty[0][2] + rotx[2][1]*roty[1][2] + rotx[2][2]*roty[2][2];

    //----------------rotMatrixDot * magnetVector--------------------
    magnetCalibVector[0]    = rotMatrixDot[0][0]*mag_x + rotMatrixDot[0][1]*mag_y + rotMatrixDot[0][2]*mag_z;
    magnetCalibVector[1]    = rotMatrixDot[1][0]*mag_x + rotMatrixDot[1][1]*mag_y + rotMatrixDot[1][2]*mag_z;
    magnetCalibVector[2]    = rotMatrixDot[2][0]*mag_x + rotMatrixDot[2][1]*mag_y + rotMatrixDot[2][2]*mag_z;

    if (magnetCalibVector[0] < 0) // 4th quadrant
    {
        directionXY = atanf(-magnetCalibVector[1]/magnetCalibVector[0]) * 180 * _1_PI + 180;
    }
    else 
    {
        directionXY = atanf(-magnetCalibVector[1]/magnetCalibVector[0]) * 180 * _1_PI;
    }
    return directionXY;
}