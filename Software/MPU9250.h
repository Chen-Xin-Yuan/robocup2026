#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f4xx_hal.h"

// AD0 接地 → 正确地址
#define MPU6500_ADDRESS     (0x68 << 1)

// 寄存器
#define WHO_AM_I_MPU6500    0x75    // 正确ID = 0x70
#define PWR_MGMT_1          0x6B
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_XOUT_H        0x3B
#define GYRO_XOUT_H         0x43
#define TEMP_OUT_H          0x41

// 错误码
#define MPU6500_OK          0
#define MPU6500_ERR_INIT    1
#define MPU6500_ERR_COMM    2

uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6500_WhoAmI(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MPU6500_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t accel[3]);
HAL_StatusTypeDef MPU6500_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t gyro[3]);
float MPU6500_ReadTemperature(I2C_HandleTypeDef *hi2c);

#endif
