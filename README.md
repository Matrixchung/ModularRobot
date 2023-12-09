# ModularRobot
 Balance robot code for Modular Robot class @ SMEE,UESTC

# Project Structure
 * Hardware: STM32F401CCU6 + AT8236 Dual Motor Driver + GY953 IMU + 1:30 ratio motor with 500 lines GMR Encoder
 * GY953 is connected with SPI1
 * Core/Src/main.c: initialize peripheral and control loop, and handling LED + sending VOFA data
 * User/Control/control.c: main control loop, running in TIM11 @ 1KHz
