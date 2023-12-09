#include "control.h"
#define YAW_CORRECT_FLAG_THRESHOLD 20 // 20 * 100ms = 2s
#define STABLE_FLAG_THRESHOLD 10 // 10 * 100ms = 1s
#define BACK_FLAG_THRESHOLD 100 // 100 * 100ms = 10s
#define STRAIGHT_THRESHOLD 80 // 80 * 100ms = 8s
#define STRAIGHT_SPEED 20
// 长按 KEY: 若 current_mode == 0, 则设定目标角度, 再次单击即设定好 yaw_target, 同时 yaw_set_flag = 1;
// 若 current_mode 不为 0, 则执行复位
// 双击 KEY, 普通上板（不带返回）
// 单击 KEY, 如果 current_mode == 0, 则将 pid_state 置为 1, 根据 yaw_set_flag: flag = 0, 普通上板（带返回，记录轮上距离）
// flag = 1, 转弯上板
// 若为其他情况, 则单击复位
// pid_state = 0 && current_mode == 0, LED 灭
// current_mode == 4 (设定角度中), LED 翻转频率 100ms
// 转弯上板(3): 1: 直行, LED 按 100ms 闪烁两次, 再熄灭 300ms
//          2: 已上板: LED 按 300ms 闪烁
//          3: 转弯: LED 按 100ms 翻转
// 普通上板（带返回）(1): 开始记录距离, LED 翻转频率 300ms, stable_flag == 1: LED 常亮, 返回时：LED 翻转频率 500ms, 回到 A 点: LED 常亮
// 普通上板（不带返回）(2): LED 翻转频率 500ms, stable_flag == 1: LED 常亮
uint8_t control_state = 1;
uint8_t pid_state = 0; 
uint8_t stable_flag = 0; // 稳定判据: 角度与目标差值绝对值小于 1.5 度，角速度绝对值小于 1.5 rad/s, 持续 STABLE_FLAG_THRESHOLD 个循环（100ms 判断一次）
// 0 - 初始, 1 - 普通模式（带返回）, 2 - 普通上板（不带返回）, 3 - 转弯上板, 4 - 设定目标角度中, 再按一次后 current_mode = 0, yaw_set_flag = 1
// current_mode == 1 且 stable_flag 持续 BACK_FLAG_THRESHOLD 个循环, 进入模式 5 - 返回 A 点, 闭环行走记录的 distance (若磁力计有效, 则使能 yaw_correction_mode)
uint8_t current_mode = 0;
uint8_t straight_ticks = 0;
// 如果通过按键设定了 yaw_target, 则记录当前 roll 角度, 执行直行操作, 
// 并不断检测 roll 变化量, 当 roll 变化量超过定值(需要实测)并持续一段时间(测量轮上距离)后, 认为前轮上板, 执行转弯
// 直到超时/达到最大轮上距离后若 roll 变化量仍未达到定值, 则终止程序
uint8_t yaw_set_flag = 0;
uint8_t yaw_correction_mode = 0; // 航向校正模式, 当模式使能且 GY953 磁力计有效时执行航向闭环
// 转弯上板状态机: 0: 初始, 1: 开始记录距离, 直行, 2: 已上板, 直行指定距离, 3: 转弯(闭环), 4: 开始记录距离, 退出状态机, 返回到普通上板（带返回）
uint8_t turn_state_flag = 0;
uint8_t yaw_correct_flag = 0;
static uint8_t yaw_correct_flag_ticks = 0;
static uint8_t stable_flag_ticks = 0;
static uint8_t back_flag_ticks = 0;
static uint8_t timer_100ms = 0;
float main_speed = 0, turn_speed = 0;
float angle_target = 0.05; // !!使得滚球平衡的实测角度值 , 12.9: 前轮两个垫片, 0.15
float yaw_target = 0;
float distance_target = 0;
PID_TypeDef angle_pid, omega_pid, yaw_pid, distance_left_pid, distance_right_pid, speed_diff_pid;
LPFilter_TypeDef gyro_x_filter, gyro_y_filter, gyro_z_filter;
SlidingFilter_TypeDef yaw_filter;
void control_init(void)
{
    // motor_init();
    Motor_InstanceInit(&Motor_Left, TIM1, LL_TIM_CHANNEL_CH2, TIM1, LL_TIM_CHANNEL_CH2N, 1, 0, MOTOR_LEFT_POS_DEADZONE, MOTOR_LEFT_NEG_DEADZONE, TIM3, LL_TIM_CHANNEL_CH1);
    Motor_InstanceInit(&Motor_Right, TIM1, LL_TIM_CHANNEL_CH1, TIM1, LL_TIM_CHANNEL_CH1N, 0, 0, MOTOR_RIGHT_POS_DEADZONE, MOTOR_RIGHT_NEG_DEADZONE, TIM2, LL_TIM_CHANNEL_CH1);
    Motor_VelocityPIDInit(&Motor_Left, 250, 10, 0, MOTOR_MAX_COMPARE);
    Motor_VelocityPIDInit(&Motor_Right, 250, 10, 0, MOTOR_MAX_COMPARE);
    angle_pid = PID_Init(1.36, 0, 0, 360); // 角度环 P 或 PD 控制, 限幅为角速度
    omega_pid = PID_Init(3.15, 0, 0.0038, 320); // 角速度环 P 或 PI 或 PID 控制, 限幅为速度 // 11V: 3.3, 0.003 //12.9 : 3.15, 0.0036
    distance_left_pid = PID_Init(1.5, 0, 0, 10); // 距离闭环, 输出限幅为速度
    distance_right_pid = PID_Init(1.5, 0, 0, 10); // 距离闭环, 输出限幅为速度
    angle_pid.enable_integral = 0;
    omega_pid.enable_integral = 0;
    distance_left_pid.enable_integral = 0;
    distance_right_pid.enable_integral = 0;
    speed_diff_pid = PID_Init(8, 0, 0, 20); // 差速 PID
    yaw_pid = PID_Init(1.3, 0.3, 0, 15);
    gyro_x_filter = LPFilter_Init(0.01); // 100 Hz Bandwidth
    gyro_y_filter = LPFilter_Init(0.01);
    gyro_z_filter = LPFilter_Init(0.01);
    // left_encoder_filter = LPFilter_Init(0.01);
    // right_encoder_filter = LPFilter_Init(0.01);
    yaw_filter = SlidingFilter_Init(30);
    LL_TIM_SetCounter(TIM11, 0);
    LL_TIM_EnableIT_UPDATE(TIM11); // 1 KHz
    LL_TIM_EnableCounter(TIM11);
    LL_TIM_ClearFlag_UPDATE(TIM11);
}
void Key_Onboard_Callback(void)
{
    switch(Key_Onboard.state)
    {
        case PRESSED:
            if(current_mode == 0)
            {
                if(yaw_set_flag)
                {
                    current_mode = 3;
                    turn_state_flag = 1;
                    yaw_correct_flag = 0;
                }
                else
                {
                    straight_ticks = STRAIGHT_THRESHOLD;
                    current_mode = 1; // 普通上板（带返回）
                    Motor_Left.encoder.distance = 0;
                    Motor_Right.encoder.distance = 0; // 复位里程计
                }
                pid_state = 1;
            }
            else if(current_mode == 4 && yaw_filter._data_count == yaw_filter.window_size) // 设定目标角度, 需要 yaw 角度填充满滑动窗口
            {
                yaw_target = yaw;
                current_mode = 0;
                yaw_set_flag = 1;
            }
            else
            {
                main_speed = 0;
                turn_speed = 0;
                current_mode = 0;
                pid_state = 0;
                yaw_set_flag = 0;
                turn_state_flag = 0;
            }
            break;
        case LONG_PRESSED:
            if(current_mode == 0)
            {
                current_mode = 4;
            }
            else
            {
                current_mode = 0;
            }
            main_speed = 0;
            turn_speed = 0;
            pid_state = 0;
            yaw_set_flag = 0;
            turn_state_flag = 0;
            break;
        case DOUBLE_TAPPED:
            if(current_mode == 0 && pid_state == 0)
            {
                straight_ticks = STRAIGHT_THRESHOLD;
                current_mode = 2;
                pid_state = 1;
                Motor_Left.encoder.distance = 0;
                Motor_Right.encoder.distance = 0; // 复位里程计
            }
            else
            {
                main_speed = 0;
                turn_speed = 0;
                current_mode = 0;
                pid_state = 0;
                yaw_set_flag = 0;
                turn_state_flag = 0;
            }
            break;
        default: break;
    }
}
void set_distance_target(float target)
{
    distance_target = target;
    Motor_Left.encoder.distance = 0;
    Motor_Right.encoder.distance = 0;
    if(gy953_magnetic_state == 3) 
    {
        // yaw_target = yaw;
        // yaw_correction_mode = 1; // 带航向角校正的闭环距离模式
    }
    else turn_speed = 0;
}
void control_loop(void)
{
    Encoder_GetVelocity(&Motor_Left.encoder);
    Encoder_GetVelocity(&Motor_Right.encoder);
    // Motor_Left.encoder.velocity = LPFilter_GetOutput(&left_encoder_filter, Motor_Left.encoder.velocity);
    // Motor_Right.encoder.velocity = LPFilter_GetOutput(&right_encoder_filter, Motor_Right.encoder.velocity);
    if(control_state)
    {
        if(pid_state)
        {
            // omega_pid.enable_integral = 1;
            // angle_pid.enable_integral = 1;
            // 1, 2
            if(current_mode == 1 || current_mode == 2)
            {
                if(straight_ticks > 0)
                {
                    main_speed = STRAIGHT_SPEED;
                }
                else main_speed = PID_GetOutput(&omega_pid, (PID_GetOutput(&angle_pid, (angle_target - roll)) - gyro_x));
            }
            else // 分为转弯上板的三个步骤、以及退回 A 点
            {
                if(current_mode == 3) // 转弯状态机
                {
                    switch(turn_state_flag)
                    {
                        case 1: // 直行, 直到超时或者 roll 变化量大于指定值
                            if(!yaw_correct_flag) 
                            {
                                if(_abs(yaw_target - yaw) >= 20) yaw_pid.enable_integral = 0;
                                else yaw_pid.enable_integral = 1;
                                turn_speed = PID_GetOutput(&yaw_pid, (yaw_target - yaw));
                            }
                            else 
                            {
                                straight_ticks = STRAIGHT_THRESHOLD;
                                yaw_correct_flag = 0;
                                turn_speed = 0;
                                turn_state_flag = 0;
                                current_mode = 1; // 普通上板（带返回）
                                Motor_Left.encoder.distance = 0;
                                Motor_Right.encoder.distance = 0; // 复位里程计
                                yaw_pid._integral_prev = 0;
                            }
                            break;
                        case 2: // 已上板, 直行指定距离
                            break;
                        case 3: // 转弯(闭环)
                            break;
                        default: break;
                    }
                }
                else if(current_mode == 5) // 控制距离, 此模式下若初始化时将 distance_target 置 0, 则可以在磁力计有效时仅控制航向
                {
                    // main_speed = PID_GetOutput(&distance_pid, (distance_target - (float)(0.5f * (Motor_Left.encoder.distance + Motor_Right.encoder.distance))));
                    // if(yaw_correction_mode && gy953_magnetic_state == 3) // 磁力计有效, 启用航向闭环
                    // {
                    //     // if(_abs(yaw_target - yaw) < 2) turn_speed = PID_GetOutput(&yaw_pid, (yaw_target - yaw));
                    //     // else turn_speed = 0;
                    //     // turn_speed = 0;
                    // }
                    // else
                    // {
                    //     // turn_speed = 0;
                    // }
                    // left_speed = PID_GetOutput(&distance_left_pid, (distance_target - Motor_Left.encoder.distance));
                    // right_speed = PID_GetOutput(&distance_right_pid, (distance_target - Motor_Right.encoder.distance));
                    if(_abs(distance_target - (float)(0.5f * (Motor_Left.encoder.distance + Motor_Right.encoder.distance))) < 3)
                    {
                        Motor_Left.set_velocity = 0;
                        Motor_Right.set_velocity = 0;
                        main_speed = 0;
                        turn_speed = 0;
                        current_mode = 0;
                    }
                    else
                    {
                        turn_speed = PID_GetOutput(&speed_diff_pid, (Motor_Right.encoder.distance - Motor_Left.encoder.distance));
                        Motor_Left.set_velocity = PID_GetOutput(&distance_left_pid, (distance_target - Motor_Left.encoder.distance)) + turn_speed;
                        Motor_Right.set_velocity = PID_GetOutput(&distance_right_pid, (distance_target - Motor_Right.encoder.distance)) - turn_speed;
                    }
                    
                }
            }
            // main_speed = PID_GetOutput(&angle_pid, (angle_target - roll));
            // turn_speed = PID_GetOutput(&yaw_pid, (yaw_target - yaw));
        }
        // else
        // {
        //     // omega_pid.enable_integral = 0;
        //     // angle_pid.enable_integral = 0;
        //     // omega_pid._integral_prev = 0;
        //     // angle_pid._integral_prev = 0;
        // }
        if(current_mode != 5)
        {
            Motor_Left.set_velocity = main_speed + turn_speed;
            Motor_Right.set_velocity = main_speed - turn_speed;
        }
    }
    else
    {
        // omega_pid.enable_integral = 0;
        // angle_pid.enable_integral = 0;
        // omega_pid._integral_prev = 0;
        // angle_pid._integral_prev = 0;
        Motor_Left.set_velocity = 0;
        Motor_Right.set_velocity = 0;
    }
    Motor_CalVelocityPID(&Motor_Left);
    Motor_CalVelocityPID(&Motor_Right);
    if(timer_100ms >= 99)
    {
        if(current_mode == 3 && turn_state_flag == 1 && _abs(yaw_target - yaw) <= 3)
        {
            if(yaw_correct_flag_ticks++ >= YAW_CORRECT_FLAG_THRESHOLD)
            {
                yaw_correct_flag = 1;
            }
        }
        else
        {
            yaw_correct_flag = 0;
            yaw_correct_flag_ticks = 0;
        }
        if(straight_ticks > 0) straight_ticks--;
        if(gy953_state && gy953_accel_state && gy953_gyro_state && _abs(angle_pid._error_prev) <= 1.5 && _abs(gyro_x) <= 1.5)
        {
            if(stable_flag_ticks++ >= STABLE_FLAG_THRESHOLD)
            {
                stable_flag = 1;
                if(current_mode == 1 && back_flag_ticks++ >= BACK_FLAG_THRESHOLD)
                {
                    current_mode = 5;
                    // distance_target = -1 * getdistance
                    set_distance_target(-0.5f * (Motor_Left.encoder.distance + Motor_Right.encoder.distance)); // 回原点
                }
            }
        }
        else
        {
            stable_flag = 0;
            back_flag_ticks = 0;
            stable_flag_ticks = 0;
        }
        timer_100ms = 0;
    }
    else timer_100ms++;
    // Motor_SetOutput(&Motor_Left, (int16_t)(main_speed + turn_speed));
    // Motor_SetOutput(&Motor_Right, (int16_t)(main_speed - turn_speed));
}