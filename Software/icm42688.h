#ifndef __ICM42688_H
#define __ICM42688_H

#include "main.h"
#include "kalman.h"

// ICM-42688-P 
#define ICM42688_ADDRESS         0x69

// ¼Ä´æÆ÷µØÖ· (Bank 0)
#define REG_BANK_SEL            0x76
#define WHO_AM_I                0x75
#define PWR_MGMT0               0x4E
#define GYRO_CONFIG0            0x4F
#define ACCEL_CONFIG0           0x50
#define DEVICE_CONFIG           0x11
#define INT_CONFIG              0x14
#define ACCEL_DATA_X1           0x1F
#define GYRO_DATA_X1            0x25
#define TEMP_DATA1              0x1D

// ´íÎó´úÂë
#define ICM42688_OK             0
#define ICM42688_ERR_INIT       1
#define ICM42688_ERR_COMM       2


uint8_t ICM42688_Init(I2C_HandleTypeDef *hi2c);
uint8_t ICM42688_WhoAmI(I2C_HandleTypeDef *hi2c);
void ICM42688_Reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ICM42688_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
HAL_StatusTypeDef ICM42688_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef ICM42688_ReadAccel(I2C_HandleTypeDef *hi2c, icm_param_t *icm_data);
HAL_StatusTypeDef ICM42688_ReadGyro(I2C_HandleTypeDef *hi2c, icm_param_t *icm_data);
float ICM42688_ReadTemperature(I2C_HandleTypeDef *hi2c);

#endif /* __ICM42688_H */
