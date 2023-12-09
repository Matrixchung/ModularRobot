#ifndef _MOTOR_H
#define _MOTOR_H

#include "pid.h"
#include "lowpass_filter.h"
#include "main.h"
#include "encoder.h"

#define MOTOR_MAX_COMPARE 4200

typedef void (*_Motor_SetCompare)(TIM_TypeDef *TIMx, uint32_t CompareValue);
typedef struct Motor_TypeDef
{
    TIM_TypeDef *TIMx;
    uint32_t channel;
    TIM_TypeDef *TIMxN;
    uint32_t channel_n;
    uint8_t direction;
    uint8_t decay;
    int16_t value;
    int16_t pos_deadzone;
    int16_t neg_deadzone;
    _Motor_SetCompare _set_compare;
    _Motor_SetCompare _set_compare_n;
    // float _raw_velocity;
    float set_velocity;
    // float distance;
    PID_TypeDef velocity_pid;
    Encoder_TypeDef encoder;
}Motor_TypeDef;

void Motor_InstanceInit(Motor_TypeDef *motor, TIM_TypeDef *TIMx, uint32_t channel, TIM_TypeDef *TIMxN, uint32_t channel_n, uint8_t direction, uint8_t decay, int16_t pos_deadzone, int16_t neg_deadzone, TIM_TypeDef *encoder_tim, uint32_t encoder_chn);
void Motor_VelocityPIDInit(Motor_TypeDef *motor, float p, float i, float d, float limit);
void Motor_SetOutput(Motor_TypeDef *motor, int16_t value);
void Motor_CalVelocityPID(Motor_TypeDef *motor);

extern Motor_TypeDef Motor_Left, Motor_Right;

#endif