#ifndef _GY953_SPI_H
#define _GY953_SPI_H

#include "main.h"
#include "spi_handler.h"
#include "imu_utils.h"

#define GY953_CS(state) (state ? LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin) : LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin))
#define GY953_TIMEOUT 0xFFFF
#define GY953_SPIx SPI1

#define GY953_SPI_W 0x40
#define GY953_SPI_R 0x80
#define GY953_SPI_SEQ_R 0xC0

#define GY953_CONFIG_A     0x01
#define GY953_CONTROL_B    0x02
#define GY953_STATUS_C     0x24
#define GY953_STATUS_D     0x23
#define GY953_ACCEL_XOUT_H 0x03
#define GY953_ACCEL_XOUT_L 0x04
#define GY953_ACCEL_YOUT_H 0x05
#define GY953_ACCEL_YOUT_L 0x06
#define GY953_ACCEL_ZOUT_H 0x07
#define GY953_ACCEL_ZOUT_L 0x08
#define GY953_GYRO_XOUT_H  0x09
#define GY953_GYRO_XOUT_L  0x0A
#define GY953_GYRO_YOUT_H  0x0B
#define GY953_GYRO_YOUT_L  0x0C
#define GY953_GYRO_ZOUT_H  0x0D
#define GY953_GYRO_ZOUT_L  0x0E
#define GY953_MAG_XOUT_H   0x0F
#define GY953_MAG_XOUT_L   0x10
#define GY953_MAG_YOUT_H   0x11
#define GY953_MAG_YOUT_L   0x12
#define GY953_MAG_ZOUT_H   0x13
#define GY953_MAG_ZOUT_L   0x14
#define GY953_ROLL_H       0x15
#define GY953_ROLL_L       0x16
#define GY953_PITCH_H      0x17
#define GY953_PITCH_L      0x18
#define GY953_YAW_H        0x19
#define GY953_YAW_L        0x1A
#define GY953_ACC_SUM      0x25
#define GY953_GYRO_SUM     0x26
#define GY953_MAG_SUM      0x27
#define GY953_RPY_SUM      0x28
#define GY953_Q_SUM        0x29

extern float roll, pitch, yaw;
// extern int16_t roll_raw, pitch_raw;
extern float gyro_x, gyro_y, gyro_z;
// extern int16_t mag_x_raw, mag_y_raw, mag_z_raw;
extern uint8_t gy953_state, gy953_accel_state, gy953_gyro_state, gy953_magnetic_state;

void GY953_WriteByte(uint8_t reg, uint8_t data);
uint8_t GY953_Init(void);
void GY953_IRQHandler(void);
#define GY953_Calibrate() do{GY953_WriteByte(GY953_CONTROL_B, 0x15);}while(0)
#define GY953_CalibrateMag() do{GY953_WriteByte(GY953_CONTROL_B, 0x19);}while(0)
#define GY953_FactoryReset() do{GY953_WriteByte(GY953_CONTROL_B, 0x91);}while(0)

#endif