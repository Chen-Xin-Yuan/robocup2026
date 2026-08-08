#include "icm42688.h"
#include "main.h"
#include "kalman.h"
#include "i2c.h"
#include <math.h>


// int16_t gyro_offset[3];


// 初始化时调用一次，模块必须静止！

/**
  * @brief  ICM42688初始化
  * @param  hi2c: I2C句柄指针
  * @retval 初始化状态 (0:成功, 其他:错误代码)
  */
uint8_t ICM42688_Init(I2C_HandleTypeDef *hi2c)
{
    // 1. 重置设备
	ICM42688_WriteReg(hi2c, REG_BANK_SEL, 0x00);
    HAL_Delay(1);
    ICM42688_Reset(hi2c);
    HAL_Delay(20); // 等待20ms确保重置完成

    // 2. 验证设备ID
    if(ICM42688_WhoAmI(hi2c) != 0x47)
    {
        return ICM42688_ERR_INIT; // 设备ID不匹配
    }

    // 3. 配置电源管理
    // 陀螺仪和加速度计都进入低噪声模式
    if(ICM42688_WriteReg(hi2c, PWR_MGMT0, 0x0F) != HAL_OK)
    {
        return ICM42688_ERR_COMM;
    }
    HAL_Delay(50); // 等待50ms

    // 4. 配置陀螺仪 ±2000dps, ODR=1kHz
    if(ICM42688_WriteReg(hi2c, GYRO_CONFIG0, 0x06) != HAL_OK)
    {
        return ICM42688_ERR_COMM;
    }

    // 5. 配置加速度计 ±16g, ODR=1kHz
    if(ICM42688_WriteReg(hi2c, ACCEL_CONFIG0, 0x06) != HAL_OK)
    {
        return ICM42688_ERR_COMM;
    }

    // 6. 配置中断 (推挽输出，高电平有效)
    if(ICM42688_WriteReg(hi2c, INT_CONFIG, 0x08) != HAL_OK)
    {
        return ICM42688_ERR_COMM;
    }
    

    
    return ICM42688_OK;
}

/**
  * @brief  读取设备ID
  * @param  hi2c: I2C句柄指针
  * @retval 设备ID (正常应为0x47)
  */
uint8_t ICM42688_WhoAmI(I2C_HandleTypeDef *hi2c)
{
    uint8_t whoami = 0;
    ICM42688_ReadReg(hi2c, WHO_AM_I, &whoami, 1);
    return whoami;
}

/**
  * @brief  重置设备
  * @param  hi2c: I2C句柄指针
  */
void ICM42688_Reset(I2C_HandleTypeDef *hi2c)
{
    ICM42688_WriteReg(hi2c, DEVICE_CONFIG, 0x01); // 软件重置
}

/**
  * @brief  写入寄存器
  * @param  hi2c: I2C句柄指针
  * @param  reg: 寄存器地址
  * @param  value: 要写入的值
  * @retval HAL状态
  */
HAL_StatusTypeDef ICM42688_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(hi2c, ICM42688_ADDRESS << 1, data, 2, 100);
}

/**
  * @brief  读取寄存器
  * @param  hi2c: I2C句柄指针
  * @param  reg: 寄存器地址
  * @param  data: 存储数据的缓冲区
  * @param  len: 要读取的长度
  * @retval HAL状态
  */
HAL_StatusTypeDef ICM42688_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len)
{
    // 先发送寄存器地址
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, ICM42688_ADDRESS << 1, &reg, 1, 100);
    if(status != HAL_OK)
    {
        return status;
    }

    // 然后读取数据
    return HAL_I2C_Master_Receive(hi2c, ICM42688_ADDRESS << 1, data, len, 100);
}

/**
  * @brief  读取加速度计数据 (原始值)
  * @param  hi2c: I2C句柄指针
  * @param  accel: 存储数据的数组(x,y,z)
  * @retval HAL状态
  */
HAL_StatusTypeDef ICM42688_ReadAccel(I2C_HandleTypeDef *hi2c, icm_param_t *icm_data)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = ICM42688_ReadReg(hi2c, ACCEL_DATA_X1, data, 6);
    if(status != HAL_OK)
    {
        return status;
    }

    icm_data->acc_x = (int16_t)((data[0] << 8) | data[1]); // X轴
    icm_data->acc_y = (int16_t)((data[2] << 8) | data[3]); // Y轴
    icm_data->acc_z = (int16_t)((data[4] << 8) | data[5]); // Z轴

    return HAL_OK;
}

/**
  * @brief  读取陀螺仪数据 (原始值)
  * @param  hi2c: I2C句柄指针
  * @param  gyro: 存储数据的数组(x,y,z)
  * @retval HAL状态
  */
HAL_StatusTypeDef ICM42688_ReadGyro(I2C_HandleTypeDef *hi2c, icm_param_t *icm_data)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = ICM42688_ReadReg(hi2c, GYRO_DATA_X1, data, 6);
    if(status != HAL_OK)
    {
        return status;
    }

    icm_data->gyro_x = (int16_t)((data[0] << 8) | data[1]); // X轴
    icm_data->gyro_y = (int16_t)((data[2] << 8) | data[3]); // Y轴
    icm_data->gyro_z = (int16_t)((data[4] << 8) | data[5]); // Z轴

    return HAL_OK;
}

/**
  * @brief  读取温度数据 (摄氏度)
  * @param  hi2c: I2C句柄指针
  * @retval 温度值(℃)
  */
float ICM42688_ReadTemperature(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2];
    int16_t temp_raw;

    if(ICM42688_ReadReg(hi2c, TEMP_DATA1, data, 2) == HAL_OK)
    {
        temp_raw = (int16_t)((data[0] << 8) | data[1]);
        return (temp_raw / 132.48f) + 25.0f; // 根据数据手册公式转换
    }

    return -273.15f; // 读取失败返回绝对零度
}



