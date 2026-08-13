#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "main.h"
#include <stdint.h>

//控制宏
// #define debug  
#define start_from_home
// #define start_from_task1
// #define start_from_task2

extern uint16_t Servo_angle[7];

/* 同时初始化普通 PWM 舵机和 USART3 总线舵机。 */
void Servo_Init(void);

/* 普通 PWM 舵机角度控制，范围限制为 0~90 度。 */
void Servo_SetAngle(uint16_t angle_deg);

/* USART3 总线舵机角度控制，范围限制为 0~360 度。非阻塞，可在中断中调用。 */
void ServoBus_SetAngle(uint16_t angle_deg);

/* 周期调用（建议放 TIM2 中断里），DMA 启动失败时重试发送队列。 */
void Servo_Process(void);

/* 由 main.c 的 HAL_UART_TxCpltCallback 转调：发送完成，继续发下一帧。 */
void Servo_TxCpltCallback(UART_HandleTypeDef *huart);

/* 由 main.c 的 HAL_UART_ErrorCallback 转调：TX DMA 出错时清空队列。 */
void Servo_UartErrorCallback(UART_HandleTypeDef *huart);

#endif
