#ifndef _ENCODER_H
#define _ENCODER_H

// #include "stdint.h"
#include "stm32f401xc.h"
typedef struct Encoder_TypeDef
{
    TIM_TypeDef *TIMx;
    uint32_t channel;
    uint32_t raw_value;
    uint32_t raw_value_prev;
    int32_t value;
    int32_t value_prev;
    uint8_t direction;
    int32_t full_rotations;
    uint32_t timestamp_prev;
    float distance;
    float velocity; // rpm
    float velocity_prev;
}Encoder_TypeDef;

#include "delay.h"

#define MOTOR_RATIO 30
#define PULSE_PER_ROUND 500
#define WHEEL_DIAMETER 6.6f
#define _PI 3.141592653589793238462643383279f

Encoder_TypeDef Encoder_Init(TIM_TypeDef *TIMx, uint32_t channel, uint8_t direction);
void Encoder_GetVelocity(Encoder_TypeDef *encoder);

#endif