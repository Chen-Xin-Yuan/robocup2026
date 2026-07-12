#include "Servo_motor.h"

/**
  * @brief  舵机设置角度（0~180度）
  * @param  ch: 1=TIM9_CH1  2=TIM9_CH2
  * @param  angle: 角度 0~180
  * @retval 无
  */
void Servo_SetAngle(uint8_t ch, uint16_t angle)
{
    uint32_t ccr_val;

    // 限制角度在 0~180 防止舵机堵转
    if(angle > 180) angle = 180;
    if(angle < 0)   angle = 0;

    // 计算公式：500us(0度) ~ 2500us(180度)
    ccr_val = 500 + (angle * 2000 / 180);

    // 选择通道输出 PWM
    if(ch == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, ccr_val);
    }
    else if(ch == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, ccr_val);
    }
}

void Servo_test(void)
{
	Servo_SetAngle(1, 0);
    Servo_SetAngle(2, 0);
    HAL_Delay(1000);  // 停1秒

    Servo_SetAngle(1, 90);
    Servo_SetAngle(2, 90);
    HAL_Delay(1000);
}
