#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_PI                 3.14159265f
#define CONTROL_DEG_TO_RAD         (CONTROL_PI / 180.0f)
#define CONTROL_RAD_TO_DEG         (180.0f / CONTROL_PI)

#define CONTROL_TEST_LOOP_DELAY_MS       20U

// Angle-loop runtime state.
typedef struct
{
    float target_yaw_deg;//目标yaw角度
    float feedback_yaw_deg;//当前yaw角度
    float error_yaw_deg;//误差yaw角度
    float omega_cmd_deg_s;//角速度微调,由当前切换到目标

    float max_omega_deg_s;//最大角速度
    float arrive_error_deg;//角度误差阈值
    float arrive_omega_deg_s;//角速度误差阈值

    uint8_t enabled;//是否使能角度环
    uint8_t arrived;//是否到达目标角度

    //用位置环粗调一段距离后再切换到角度环微调,相关状态量
    uint8_t coarse_turn_pending;//是否_pending粗调
    uint8_t coarse_turn_active;//是否正在粗调
    uint8_t coarse_turn_stop_sent;//粗调前是否已经发送停止命令
    uint32_t coarse_turn_ready_tick;//允许发送粗调位置命令的时间戳
    uint32_t coarse_turn_end_tick;//粗调结束时间戳
} Control_t;

extern Control_t Control;

/*================== 需要调用的闭环控制函数 ==================*/

// Initialize the angle loop and tuned PID values.
void Control_Init(void);
// Reset control state but keep PID gains.
void Control_Reset(void);
// Enable or disable the angle loop.
void Control_Enable(uint8_t enable);

// Set an absolute target yaw in degrees.
void Control_SetAngleDeg(float target_yaw_deg);

// Update PID gains for the angle loop.
void Control_SetAnglePid(float kp, float ki, float kd);
// Set the maximum yaw-rate output in deg/s.
void Control_SetAngleOutputLimit(float max_omega_deg_s);
// Set the yaw-error threshold in degrees and yaw-rate threshold in deg/s.
void Control_SetAngleArriveThreshold(float error_deg, float omega_deg_s);

// Run one angle-loop step from the timer interrupt.
void Control_AngleUpdate(void);
// Set translation speed, hold the absolute yaw angle, and send the latest output.
void Control_AngleHoldMove(float vx, float vy, float hold_yaw_deg);
// Return nonzero when the target is reached.
uint8_t Control_AngleIsArrived(void);
// Stop the angle loop and the chassis.
void Control_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
