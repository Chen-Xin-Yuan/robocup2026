#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "main.h"
#include <stdint.h>

/* 同时初始化普通 PWM 舵机和 USART3 总线舵机。 */
void Servo_Init(void);

/* 普通 PWM 舵机角度控制，范围限制为 0~90 度。 */
void Servo_SetAngle(uint16_t angle_deg);

/* USART3 总线舵机角度控制，范围限制为 0~360 度。 */
void ServoBus_SetAngle(uint16_t angle_deg);

#endif
