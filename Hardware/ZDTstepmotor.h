#ifndef __ZDTSTEPMOTOR_H
#define __ZDTSTEPMOTOR_H

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "motor_def.h"
#include "main.h"


typedef struct
{
    uint8_t id; // 程序中的id,用于其它模块访问该电机模块
    //最大值限制
    float velocity_lim;
    float iq_lim;
    
    //实际
    float real_deg_pos; // (rad)
    float real_current; // (A)
    //目标
    float tar_velocity;
    float tar_deg_pos; 
    float tar_current;
    int16_t tar_rpm;
  
} Motor_Controller_struct;

typedef struct
{
    Motor_Controller_struct motor_controller_t; // 控制电机id
    UART_HandleTypeDef *_USART;
    int8_t  _dir;               // 正转方向
    float   _wheel_diameter;    // 轮子直径
    bool    _have_pub_permission; // 是否有发布权限
    uint8_t _cmd_buffer[20];    // 命令缓冲区

} StepMotorZDT_t;

/* 外部全局电机定义 */
extern StepMotorZDT_t Motor1, Motor2, Motor3, Motor4;

/* Motor commands rejected before enqueue because UART1 or queue capacity was invalid. */
extern volatile uint32_t ZDT_TxRejectedCommands;

void Step_ZDT_Init(StepMotorZDT_t *zdt_mot,  uint8_t id ,UART_HandleTypeDef *_USART,int8_t _dir, float _wheel_diameter,
     bool _have_pub_permission);

void set_speed_target(StepMotorZDT_t *zdt_motor, float target);
//定速行驶
void set_speed_pos_target(StepMotorZDT_t *zdt_motor, float target_speed, float target_pos);
//定速移动某个距离
float get_linear_speed(StepMotorZDT_t* zdt_motor);
// 获得边缘线速度


#endif
