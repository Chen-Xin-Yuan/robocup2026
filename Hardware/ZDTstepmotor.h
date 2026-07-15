#ifndef __ZDTSTEPMOTOR_H
#define __ZDTSTEPMOTOR_H

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "motor_def.h"
#include "main.h"

/* 外部全局电机定义 */
extern StepMotorZDT_t Motor1, Motor2, Motor3, Motor4;

typedef struct
{
    uint8_t id; // 程序中的id,用于其它模块访问该电机模块
    //最大值限制
    float velocity_lim;
    float iq_lim;
    
    //实际
    float real_velocity; // m/s
    float real_deg_pos; // (rad)
    float real_current; // (A)
    int16_t real_rpm; // r/min
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

    uint32_t _last_query_tick;   // 上次查询的时间戳 (ms), 用于轮询间隔控制
} StepMotorZDT_t;

void Step_ZDT_Init(StepMotorZDT_t *zdt_mot,  uint8_t id ,UART_HandleTypeDef *_USART,int8_t _dir, float _wheel_diameter,
     bool _have_pub_permission);

void set_speed_target(StepMotorZDT_t *zdt_motor, float target);
//定速行驶
void set_speed_pos_target(StepMotorZDT_t *zdt_motor, float target_speed, float target_pos);
//定速移动某个距离
float get_linear_speed(StepMotorZDT_t* zdt_motor);
// 获得边缘线速度

/*================== Emm_V5 真实反馈查询接口 ==================*/

/**
 * @brief 发送读取实际转速命令 (功能码 0x33)
 * @note  发送后电机将回传 5 字节帧, 需在 USART1_IRQHandler 中解析。
 *        4个电机共用总线, 不可同时发送, 必须轮询。
 */
void ZDT_Query_RPM(StepMotorZDT_t *zdt_motor);

/**
 * @brief 发送读取编码器命令 (功能码 0x30)
 * @note  回传 9 字节帧, 含 32 位编码器值 + 进位。
 */
void ZDT_Query_Encoder(StepMotorZDT_t *zdt_motor);

/**
 * @brief 轮询查询4个电机的实际转速
 * @note  每次只查询一个电机, 4次调用覆盖全部。
 *        建议在 10ms 周期任务中调用。
 */
void ZDT_Query_All_Motor_RPM(void);

/**
 * @brief 解析 Emm_V5 回传帧
 * @param data  接收缓冲区指针
 * @param len   本次接收到的总字节数
 * @note  支持多帧粘连解析, 根据首字节地址自动匹配 Motor1~Motor4。
 *        以 0x6B 为帧尾分割。
 */
void ZDT_Parse_Frame(uint8_t *data, uint8_t len);

#endif
