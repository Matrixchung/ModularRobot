#include "GY953_SPI.h"
uint8_t gy953_state = 0;
uint8_t gy953_gyro_state = 0;
uint8_t gy953_accel_state = 0;
uint8_t gy953_magnetic_state = 0;
float roll, pitch, yaw = 0;
float gyro_x, gyro_y, gyro_z = 0;
int16_t roll_raw, pitch_raw, yaw_raw = 0;
int16_t acc_x_raw, acc_y_raw, acc_z_raw = 0;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw = 0;
int16_t mag_x_raw, mag_y_raw, mag_z_raw = 0; 
static uint8_t temp[6] = {0};
static uint8_t checksum = 0;
void GY953_WriteByte(uint8_t reg, uint8_t data)
{
    GY953_CS(0);
    reg |= GY953_SPI_W;
    SPI_WriteReg8(GY953_SPIx, reg, data);
    GY953_CS(1);
}
void GY953_ReadByte(uint8_t reg, uint8_t *data)
{
    GY953_CS(0);
    reg |= GY953_SPI_R;
    SPI_ReadReg8_2(GY953_SPIx, reg, data);
    GY953_CS(1);
}

uint8_t GY953_Check(void)
{
    uint16_t timeout = 0;
    uint8_t ret = 0;
    do
    {
        GY953_ReadByte(GY953_CONFIG_A, &ret);
        timeout++;
        if(timeout >= GY953_TIMEOUT) return 0;
    }while(!ret);
    return 1;
}
uint8_t GY953_Init(void)
{
    LL_SPI_Enable(GY953_SPIx);
    delay_ms(500);
    gy953_state = 0;
    if(!GY953_Check()) return 0;
    // set data rate = 200Hz, and open all sensors
    GY953_WriteByte(GY953_CONFIG_A, 0x7D);
    gy953_state = 1;
    return gy953_state;
}

void GY953_IRQHandler(void)
{
    if(gy953_state)
    {
        uint8_t tmp_checksum = 0;
        GY953_ReadByte(GY953_ROLL_H, temp);
        GY953_ReadByte(GY953_ROLL_L, temp+1);
        GY953_ReadByte(GY953_PITCH_H, temp+2);
        GY953_ReadByte(GY953_PITCH_L, temp+3);
        GY953_ReadByte(GY953_YAW_H, temp+4);
        GY953_ReadByte(GY953_YAW_L, temp+5);
        GY953_ReadByte(GY953_RPY_SUM, &checksum);
        for(uint8_t i = 0; i < 6; i++) tmp_checksum += temp[i];
        if(tmp_checksum == checksum && tmp_checksum != 0) // RPY valid
        {
            roll_raw = (temp[0] << 8 | temp[1]);
            pitch_raw = (temp[2] << 8 | temp[3]);
            yaw_raw = (temp[4] << 8 | temp[5]);
            roll = roll_raw * 0.01f;
            pitch = pitch_raw * 0.01f;
            yaw = yaw_raw * 0.01f;
        }
        else return; // cancel current reading process
        // GY953_ReadByte(GY953_ACCEL_XOUT_H, temp);
        // GY953_ReadByte(GY953_ACCEL_XOUT_L, temp+1);
        // GY953_ReadByte(GY953_ACCEL_YOUT_H, temp+2);
        // GY953_ReadByte(GY953_ACCEL_YOUT_L, temp+3);
        // GY953_ReadByte(GY953_ACCEL_ZOUT_H, temp+4);
        // GY953_ReadByte(GY953_ACCEL_ZOUT_L, temp+5);
        // GY953_ReadByte(GY953_ACC_SUM, &checksum);
        // for(uint8_t i = 0; i < 6; i++) tmp_checksum += temp[i];
        // if(tmp_checksum == checksum && tmp_checksum != 0) // Acc valid
        // {
        //     acc_x_raw = (temp[0] << 8 | temp[1]);
        //     acc_y_raw = (temp[2] << 8 | temp[3]);
        //     acc_z_raw = (temp[4] << 8 | temp[5]);
        // }
        // else return;
        tmp_checksum = 0;
        GY953_ReadByte(GY953_GYRO_XOUT_H, temp);
        GY953_ReadByte(GY953_GYRO_XOUT_L, temp+1);
        GY953_ReadByte(GY953_GYRO_YOUT_H, temp+2);
        GY953_ReadByte(GY953_GYRO_YOUT_L, temp+3);
        GY953_ReadByte(GY953_GYRO_ZOUT_H, temp+4);
        GY953_ReadByte(GY953_GYRO_ZOUT_L, temp+5);
        GY953_ReadByte(GY953_GYRO_SUM, &checksum);
        for(uint8_t i = 0; i < 6; i++) tmp_checksum += temp[i];
        if(tmp_checksum == checksum && tmp_checksum != 0) // Gyro valid
        {
            gyro_x_raw = (temp[0] << 8 | temp[1]);
            gyro_y_raw = (temp[2] << 8 | temp[3]);
            gyro_z_raw = (temp[4] << 8 | temp[5]);
            gyro_x = gyro_x_raw * 0.0609756097560976f;
            gyro_y = gyro_y_raw * 0.0609756097560976f;
            gyro_z = gyro_z_raw * 0.0609756097560976f;
        }
        else return;
        tmp_checksum = 0;
        GY953_ReadByte(GY953_MAG_XOUT_H, temp);
        GY953_ReadByte(GY953_MAG_XOUT_L, temp+1);
        GY953_ReadByte(GY953_MAG_YOUT_H, temp+2);
        GY953_ReadByte(GY953_MAG_YOUT_L, temp+3);
        GY953_ReadByte(GY953_MAG_ZOUT_H, temp+4);
        GY953_ReadByte(GY953_MAG_ZOUT_L, temp+5);
        GY953_ReadByte(GY953_MAG_SUM, &checksum);
        for(uint8_t i = 0; i < 6; i++) tmp_checksum += temp[i];
        if(tmp_checksum == checksum && tmp_checksum != 0) // Magnetic valid
        {
            mag_x_raw = (temp[0] << 8 | temp[1]);
            mag_y_raw = (temp[2] << 8 | temp[3]);
            mag_z_raw = (temp[4] << 8 | temp[5]);
            GY953_ReadByte(GY953_STATUS_C, temp);
            gy953_magnetic_state = temp[0] & 0x03;
            gy953_gyro_state = (temp[0] & 0x0C) >> 2;
            gy953_accel_state = (temp[0] & 0x30) >> 4;
            // MadgwickAHRSupdate_9((float)gyro_x_raw, (float)gyro_y_raw, (float)gyro_z_raw, (float)acc_x_raw, (float)acc_y_raw, (float)acc_z_raw, (float)mag_x_raw, (float)mag_y_raw, (float)mag_z_raw, &pitch, &roll, &yaw);
        }
    }
}