#include "stm32f4xx_hal.h"
#include "gw_color_sensor.h"
#include "delay.h"

#define SDA_PIN GPIO_PIN_5
#define SDA_PORT GPIOB
#define SCL_PIN GPIO_PIN_4
#define SCL_PORT GPIOB

/* 基本I2C操作宏 */
#define SDA_HIGH() HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET)
#define SDA_LOW()  HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET)
#define SCL_HIGH() HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET)
#define SCL_LOW()  HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET)
#define READ_SDA() HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN)

unsigned char Soft_IIC_Ping(void);
unsigned char Soft_IIC_Get_HSL(unsigned char * Result,unsigned char len);
unsigned char Soft_IIC_Get_RGB(unsigned char * Result,unsigned char len);

