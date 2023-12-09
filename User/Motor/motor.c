#include "motor.h"

Motor_TypeDef Motor_Left, Motor_Right;

void Motor_InstanceInit(Motor_TypeDef *motor, TIM_TypeDef *TIMx, uint32_t channel, TIM_TypeDef *TIMxN, uint32_t channel_n, uint8_t direction, uint8_t decay, int16_t pos_deadzone, int16_t neg_deadzone, TIM_TypeDef *encoder_tim, uint32_t encoder_chn)
{
    motor->TIMx = TIMx;
    motor->channel = channel;
    motor->TIMxN = TIMxN;
    motor->channel_n = channel_n;
    motor->direction = direction;
    motor->decay = decay;
    motor->value = 0;
    motor->pos_deadzone = pos_deadzone;
    motor->neg_deadzone = neg_deadzone;
    motor->_set_compare = NULL;
    motor->_set_compare_n = NULL;
    PID_TypeDef pid = PID_Init(0, 0, 0, MOTOR_MAX_COMPARE);
    motor->velocity_pid = pid;
    motor->set_velocity = 0;
    Encoder_TypeDef encoder = Encoder_Init(encoder_tim, encoder_chn, direction);
    motor->encoder = encoder;
    switch(channel)
    {
        case LL_TIM_CHANNEL_CH1:
            motor->_set_compare = LL_TIM_OC_SetCompareCH1;
            break;
        case LL_TIM_CHANNEL_CH2:
            motor->_set_compare = LL_TIM_OC_SetCompareCH2;
            break;
        case LL_TIM_CHANNEL_CH3:
            motor->_set_compare = LL_TIM_OC_SetCompareCH3;
            break;
        case LL_TIM_CHANNEL_CH4:
            motor->_set_compare = LL_TIM_OC_SetCompareCH4;
            break;
    }
    LL_TIM_CC_EnableChannel(TIMx, channel);
    LL_TIM_CC_EnableChannel(TIMxN, channel_n);
    __attribute__((unused)) uint32_t tmpsmcr = TIMx->SMCR & TIM_SMCR_SMS;
    motor->_set_compare(TIMx, 0);
    if(channel_n == LL_TIM_CHANNEL_CH1 || channel_n == LL_TIM_CHANNEL_CH2 || channel_n == LL_TIM_CHANNEL_CH3 || channel_n == LL_TIM_CHANNEL_CH4) // not complementary output channel
    {
        switch(channel_n)
        {
            case LL_TIM_CHANNEL_CH1:
                motor->_set_compare_n = LL_TIM_OC_SetCompareCH1;
                break;
            case LL_TIM_CHANNEL_CH2:
                motor->_set_compare_n = LL_TIM_OC_SetCompareCH2;
                break;
            case LL_TIM_CHANNEL_CH3:
                motor->_set_compare_n = LL_TIM_OC_SetCompareCH3;
                break;
            case LL_TIM_CHANNEL_CH4:
                motor->_set_compare_n = LL_TIM_OC_SetCompareCH4;
                break;
        }
        motor->_set_compare_n(TIMxN, 0);
        // if(TIMx->CR1 != TIMxN->CR1) // different timer
        // {
            tmpsmcr = TIMxN->SMCR & TIM_SMCR_SMS;
            LL_TIM_EnableAllOutputs(TIMxN);
            LL_TIM_EnableCounter(TIMxN);
        // }
    }
    LL_TIM_EnableAllOutputs(TIMx);
    LL_TIM_EnableCounter(TIMx);
}

void Motor_VelocityPIDInit(Motor_TypeDef *motor, float p, float i, float d, float limit)
{
    PID_TypeDef pid = PID_Init(p, i, d, limit);
    motor->velocity_pid = pid;
}
void Motor_CalVelocityPID(Motor_TypeDef *motor)
{
    Motor_SetOutput(motor, (int16_t)(PID_GetOutput(&motor->velocity_pid, (motor->set_velocity - motor->encoder.velocity))));
}
void Motor_SetOutput(Motor_TypeDef *motor, int16_t value)
{
    if(value == 0)
    {
        if(motor->decay)
        {
            if(motor->value > motor->pos_deadzone) motor->value = -motor->neg_deadzone;
            else if(motor->value <(-motor->neg_deadzone)) motor->value = motor->pos_deadzone;
        }
        else
        {
            if(motor->value > motor->pos_deadzone) motor->value = motor->pos_deadzone;
            else if(motor->value <(-motor->neg_deadzone)) motor->value = -motor->neg_deadzone;
        }
    }
    else motor->value = _constrain(value > 0 ? value + motor->pos_deadzone : value - motor->neg_deadzone, -MOTOR_MAX_COMPARE, MOTOR_MAX_COMPARE - 1);
    value = motor->direction ? motor->value : -motor->value;
    if(value > 0)
    {
        LL_TIM_CC_DisableChannel(motor->TIMxN, motor->channel_n);
        LL_TIM_CC_EnableChannel(motor->TIMx, motor->channel);
        motor->_set_compare(motor->TIMx, (uint32_t)value);
    }
    else if(value == 0)
    {
        LL_TIM_CC_EnableChannel(motor->TIMx, motor->channel);
        LL_TIM_CC_EnableChannel(motor->TIMxN, motor->channel_n);
        motor->_set_compare(motor->TIMx, 0);
        if(motor->_set_compare_n != NULL) motor->_set_compare_n(motor->TIMxN, 0);
    }
    else
    {
        LL_TIM_CC_DisableChannel(motor->TIMx, motor->channel);
        LL_TIM_CC_EnableChannel(motor->TIMxN, motor->channel_n);
        if(motor->_set_compare_n != NULL) motor->_set_compare_n(motor->TIMxN, (uint32_t)(-value));
        else motor->_set_compare(motor->TIMx, (uint32_t)(MOTOR_MAX_COMPARE+(value)));
    }
}