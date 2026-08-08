#include "MPU9250.h"

static HAL_StatusTypeDef MPU_Write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg, val};
    return HAL_I2C_Master_Transmit(hi2c, MPU6500_ADDRESS, data, 2, 100);
}

static HAL_StatusTypeDef MPU_Read(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if(HAL_I2C_Master_Transmit(hi2c, MPU6500_ADDRESS, &reg, 1, 100) != HAL_OK)
        return HAL_ERROR;
    return HAL_I2C_Master_Receive(hi2c, MPU6500_ADDRESS, buf, len, 100);
}

uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;

    // 复位
    MPU_Write(hi2c, PWR_MGMT_1, 0x80);
    HAL_Delay(20);

    // 读ID
    MPU_Read(hi2c, WHO_AM_I_MPU6500, &id, 1);
    if(id != 0x70) return MPU6500_ERR_INIT;

    // 唤醒
    MPU_Write(hi2c, PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    // 配置量程
    MPU_Write(hi2c, GYRO_CONFIG, 0x18);   // ±2000dps
    MPU_Write(hi2c, ACCEL_CONFIG, 0x18);  // ±16g

    return MPU6500_OK;
}

uint8_t MPU6500_WhoAmI(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;
    MPU_Read(hi2c, WHO_AM_I_MPU6500, &id, 1);
    return id;
}

HAL_StatusTypeDef MPU6500_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t accel[3])
{
    uint8_t buf[6];
    if(MPU_Read(hi2c, ACCEL_XOUT_H, buf, 6) != HAL_OK) return HAL_ERROR;
    accel[0] = (buf[0]<<8) | buf[1];
    accel[1] = (buf[2]<<8) | buf[3];
    accel[2] = (buf[4]<<8) | buf[5];
    return HAL_OK;
}

HAL_StatusTypeDef MPU6500_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t gyro[3])
{
    uint8_t buf[6];
    if(MPU_Read(hi2c, GYRO_XOUT_H, buf, 6) != HAL_OK) return HAL_ERROR;
    gyro[0] = (buf[0]<<8) | buf[1];
    gyro[1] = (buf[2]<<8) | buf[3];
    gyro[2] = (buf[4]<<8) | buf[5];
    return HAL_OK;
}

float MPU6500_ReadTemperature(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2];
    int16_t raw;
    if(MPU_Read(hi2c, TEMP_OUT_H, buf, 2) != HAL_OK) return -273.15f;
    raw = (buf[0]<<8) | buf[1];
    return (raw / 333.87f) + 21.0f;
}