#ifndef ALL_INIT_H
#define ALL_INIT_H
#include "main.h"
#include "tim.h"


/* ----- 硬件引脚定义（按实际修改）----- */
#define DEBUG_LED_PIN      GPIO_PIN_12
#define DEBUG_LED_PORT     GPIOB
#define DEBUG_LED_CLK_EN() __HAL_RCC_GPIOB_CLK_ENABLE()




#endif
