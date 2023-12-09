/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t timer_1ms = 0;
uint8_t timer_10ms = 0;
uint8_t timer_5ms = 0;
uint8_t timer_100ms = 0;
uint8_t timer_300ms = 0;
uint8_t timer_500ms = 0;
uint8_t blink_count = 0;
uint16_t rxLen = 0;
char buffer[128] = {0};
char command_buffer[MAX_COMMAND_WORD_LENGTH] = {0};
float arg_buffer[MAX_FLOAT_ARG_COUNT] = {0.0f};
uint8_t arg_buffer_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_CommandHandler(UART_InstanceTypeDef *uart_handler);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  // LL Driver turns off SysTick_Handler() interrupt without SysTick_CTRL_TICKINT_Msk set.
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;
  ADC_HandlerInit(); // Analog Digital Converter
  UART_InstanceInit(&UART2_Handler, USART2, DMA1, LL_DMA_STREAM_5, DMA1, LL_DMA_STREAM_6, 128); 
  UART_InstanceInit(&UART6_Handler, USART6, DMA2, LL_DMA_STREAM_1, DMA2, LL_DMA_STREAM_6, 128);
  Key_InstanceInit(&Key_Onboard, KEY_GPIO_Port, KEY_Pin, 0);
  Key_RegisterCallback(&Key_Onboard, Key_Onboard_Callback);
  // UART_printf(&UART1_Handler, 1, "ModularRobot: Hello World!\n");
  GY953_Init();
  control_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(UART2_Handler.rx_finished_flag)
    {
      UART_CommandHandler(&UART2_Handler);
      UART2_Handler.rx_finished_flag = 0;
    }
    if(UART6_Handler.rx_finished_flag)
    {
      UART_CommandHandler(&UART6_Handler);
      UART6_Handler.rx_finished_flag = 0;
    }
    if(timer_1ms)
    {
      if(timer_5ms >= 4)
      {
        if(timer_10ms >= 1)
        {
          // LED 灯控部分
          if(timer_100ms >= 9)
          {
            if(current_mode == 4 || (current_mode == 3 && turn_state_flag == 3) || (!gy953_accel_state || !gy953_gyro_state))
            {
              LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            }
            else if(pid_state == 0 && current_mode == 0)
            {
              LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
            }
            else
            {
              if(current_mode == 3 && turn_state_flag == 1 && blink_count < 5)
              {
                LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                blink_count++;
                if(blink_count == 5) LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin); 
              }
              if(timer_300ms >= 2)
              {
                if(current_mode == 1)
                {
                  if(!stable_flag) LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                  else LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
                }
                else if(current_mode == 3)
                {
                  if(turn_state_flag == 1)
                  {
                    if(blink_count >= 5)
                    {
                      blink_count++;
                      if(blink_count > 6) blink_count = 0;
                    }
                  }
                  else if(turn_state_flag == 2) LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                }
                timer_300ms = 0;
              }
              else timer_300ms++;
              if(timer_500ms >= 4)
              {
                if(current_mode == 2)
                {
                  if(!stable_flag) LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                  else LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
                }
                else if(current_mode == 5)
                {
                  LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                }
                timer_500ms = 0;
              }
              else timer_500ms++;
            }
            timer_100ms = 0;
          }
          else timer_100ms++;
          // LED 灯控部分结束
          vofa_add_float(0, roll);
          vofa_add_float(1, pitch);
          vofa_add_float(2, yaw);
          vofa_add_float(3, gyro_x);
          vofa_add_float(4, gyro_y);
          vofa_add_float(5, gyro_z);
          vofa_add_float(6, (float)Motor_Left.encoder.velocity);
          vofa_add_float(7, (float)Motor_Right.encoder.velocity);
          vofa_add_float(8, (float)Motor_Left.encoder.distance);
          vofa_add_float(9, (float)Motor_Right.encoder.distance);
          vofa_add_float(10, (float)angle_target);
          vofa_add_float(11, (float)yaw_target);
          vofa_add_float(12, (float)distance_target);
          // vofa_add_float(8, (float)angle_pid.P);
          // vofa_add_float(9, (float)angle_pid.I);
          // vofa_add_float(10, (float)angle_pid.D);
          vofa_add_float(13, (float)current_mode);
          vofa_add_float(14, (float)yaw_set_flag);
          vofa_add_float(15, (float)yaw_correction_mode);
          vofa_add_float(16, (float)turn_state_flag);
          vofa_add_float(17, (float)stable_flag);
          vofa_add_float(18, (float)control_state);
          vofa_add_float(19, (float)pid_state);
          vofa_add_float(20, (float)gy953_magnetic_state);
          vofa_add_float(21, (float)gy953_accel_state);
          vofa_add_float(22, (float)gy953_gyro_state);
          vofa_add_float(23, ADC_GetVin());
          vofa_send_data();
          Key_InstanceUpdate(&Key_Onboard);
          timer_10ms = 0;
        }
        else timer_10ms++;
        GY953_IRQHandler();
        gyro_x = LPFilter_GetOutput(&gyro_x_filter, gyro_x);
        gyro_y = LPFilter_GetOutput(&gyro_y_filter, gyro_y);
        gyro_z = LPFilter_GetOutput(&gyro_z_filter, gyro_z);
        // yaw = SlidingFilter_GetOutput(&yaw_filter, get_calibrated_yaw((float)pitch, (float)roll, (float)mag_x_raw, (float)mag_y_raw, (float)mag_z_raw));
        yaw = SlidingFilter_GetOutput(&yaw_filter, yaw);
        timer_5ms = 0;
      }
      else timer_5ms++;
      timer_1ms = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSE_EnableCSS();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(84000000);
  LL_SetSystemCoreClock(84000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */
void UART_CommandHandler(UART_InstanceTypeDef *uart_handler)
{
  rxLen = UART_GetRxLen(uart_handler);
  if(rxLen > 0)
  {
    UART_GetRxFifo(uart_handler, buffer, rxLen);
    if(buffer[rxLen-1] == '\r' || buffer[rxLen-1] == ';' || buffer[rxLen-1] == '\n')
    {
      getCommand(buffer, rxLen, command_buffer);
      splitData_f(buffer, rxLen, arg_buffer, &arg_buffer_len, ' ');
      if(arg_buffer_len == 1)
      {
        if(strcmp(command_buffer, "control_state") == 0)    
        {
          control_state = (uint8_t)arg_buffer[0] ? 1 : 0;
          main_speed = 0;
          turn_speed = 0;
        }   
        else if(strcmp(command_buffer, "pid_state") == 0)      
        {
          pid_state = (uint8_t)arg_buffer[0] ? 1 : 0;
          main_speed = 0;
          turn_speed = 0;
        }
        else if(strcmp(command_buffer, "calibrate") == 0)
        {
          if(arg_buffer[0] == 1) GY953_Calibrate();
          else if(arg_buffer[0] == 2) GY953_CalibrateMag();
        }
        else if(strcmp(command_buffer, "angle_kp") == 0)        angle_pid.P = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "angle_ki") == 0)        angle_pid.I = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "angle_kd") == 0)        angle_pid.D = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "omega_kp") == 0)        omega_pid.P = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "omega_ki") == 0)        omega_pid.I = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "omega_kd") == 0)        omega_pid.D = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "yaw_kp") == 0)          yaw_pid.P = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "yaw_ki") == 0)          yaw_pid.I = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "yaw_kd") == 0)          yaw_pid.D = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "yaw_target") == 0)      yaw_target = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "angle_target") == 0)    angle_target = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "left_speed") == 0)      Motor_Left.value = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "right_speed") == 0)     Motor_Right.value = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "left_vel_kp") == 0)     Motor_Left.velocity_pid.P = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "left_vel_ki") == 0)     Motor_Left.velocity_pid.I = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "left_vel_kd") == 0)     Motor_Left.velocity_pid.D = (float)arg_buffer[0];
        else if(strcmp(command_buffer, "current_mode") == 0)    current_mode = (uint8_t)arg_buffer[0];
        else if(strcmp(command_buffer, "yaw_correct") == 0)     yaw_correction_mode = (uint8_t)arg_buffer[0];
        else if(strcmp(command_buffer, "distance_target") == 0)
        {
          if(current_mode == 5)
          {
            set_distance_target((float)arg_buffer[0]);
          }
        }
      }
      else if(arg_buffer_len == 2)
      {
        if(strcmp(command_buffer, "move") == 0)
        {
          main_speed = (float)arg_buffer[1];
          turn_speed = (float)arg_buffer[0];
        }
      }
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
