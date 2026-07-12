/*
 * @Author: skybase
 * @Date: 2025-01-13 18:11:08
 * @LastEditors: skybase
 * @LastEditTime: 2025-02-28 03:45:06
 * @Description:  ᕕ(◠ڼ◠)ᕗ​
 * @FilePath: \undefinedd:\Project\Embedded_project\Stm_pro\25Rc_jump_stm32H7_RT_THREAD\BSP\Motor\motor_def.h
 */
// #ifndef MOTOR_DEF_H
// #define MOTOR_DEF_H

// #include "stdio.h"
// #include <stdint.h> 

// /* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/

// #define MOTOR_VAR_REVERSE(x) (-(x))


// /* 电机正反转标志 */
// typedef enum
// {
//     MOTOR_DIRECTION_NORMAL = 0,
//     MOTOR_DIRECTION_REVERSE = 1
// } Motor_Reverse_Flag_e;
// /// @brief 电机工作状态
// typedef enum
// {
//     MOTOR_STOP = 0,
//     MOTOR_ENABLED = 1,
// } Motor_Working_Type_e;

// /* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
// typedef struct
// {
//     Motor_Working_Type_e motor_enable_flag;  // 使能标志
//     Motor_Reverse_Flag_e motor_reverse_flag; // 是否反转

// } Motor_Control_Setting_s;

// typedef struct
// {

//     float velocity_lim;
//     float iq_lim;

// } Motor_Limit_Setting_s;

// /* 电机类型枚举 */
// typedef enum
// {
//     MOTOR_TYPE_NONE = 0,
//     MOTOR_TYPE_DC, // 一般直流电机

// } Motor_Type_e;

// #endif // !MOTOR_DEF_H
