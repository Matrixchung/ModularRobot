#include "encoder.h"
Encoder_TypeDef Encoder_Init(TIM_TypeDef *TIMx, uint32_t channel, uint8_t direction)
{
    Encoder_TypeDef encoder;
    encoder.TIMx = TIMx;
    encoder.channel = channel;
    encoder.direction = direction;
    encoder.raw_value = 32768;
    encoder.raw_value_prev = 32768;
    encoder.value = 0;
    encoder.timestamp_prev = get_micros();
    encoder.value_prev = 0;
    encoder.full_rotations = 0;
    encoder.distance = 0;
    LL_TIM_SetCounter(TIMx, 32768);
    LL_TIM_DisableIT_CC1(TIMx);
    LL_TIM_CC_EnableChannel(TIMx, channel);
    LL_TIM_ClearFlag_CC1(TIMx);
    LL_TIM_DisableIT_UPDATE(TIMx);
    LL_TIM_EnableCounter(TIMx);
    return encoder;
}

void Encoder_GetVelocity(Encoder_TypeDef *encoder)
{
    encoder->raw_value = LL_TIM_GetCounter(encoder->TIMx);
    // if(encoder->raw_value_prev >= 65531 && encoder->raw_value <= 4)
    // {
    //     encoder->direction = 0;
    //     encoder->full_rotations--;
    // }
    // else if(encoder->raw_value_prev <= 4 && encoder->raw_value_prev >= 65531)
    // {
    //     encoder->direction = 1;
    //     encoder->full_rotations++;
    // }
    uint32_t timestamp = get_micros();
    float Ts = (float)(timestamp - encoder->timestamp_prev) * 1e-6f;
    if(Ts < 0 || Ts > 0.1f) Ts = 1e-3f;
    // encoder->value = (encoder->raw_value - encoder->raw_value_prev) + encoder->full_rotations * LL_TIM_GetAutoReload(encoder->TIMx);
    // encoder->velocity = (float)(encoder->value - encoder->value_prev) / (Ts * 4 * MOTOR_RATIO * PULSE_PER_ROUND);
    encoder->velocity = 15.0f * ((float)((float)encoder->raw_value - (float)encoder->raw_value_prev) / (Ts * MOTOR_RATIO * PULSE_PER_ROUND)) * (encoder->direction ? 1 : -1);
    if(encoder->velocity - encoder->velocity_prev >= 100 || encoder->velocity_prev - encoder->velocity >= 100) encoder->velocity = encoder->velocity_prev;
    encoder->distance += (Ts * 0.5f * 0.016667f) * (encoder->velocity + encoder->velocity_prev) * (WHEEL_DIAMETER * _PI);
    // encoder->value_prev = encoder->value;
    encoder->raw_value_prev = encoder->raw_value;
    encoder->timestamp_prev = timestamp;
    encoder->velocity_prev = encoder->velocity;
}