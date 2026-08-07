#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "pid.h"

// 测试函数：依次验证静态角度环和动态角度环（阻塞式）
void Control_test(void);

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_PI                 3.14159265f
#define CONTROL_DEG_TO_RAD         (CONTROL_PI / 180.0f)
#define CONTROL_RAD_TO_DEG         (180.0f / CONTROL_PI)

#define CONTROL_TEST_LOOP_DELAY_MS       20U

// 动态角度环参数（行驶中航向保持）
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
} Control_DynamicAngle_t;

extern Control_DynamicAngle_t Control_DynamicAngle;

/*================== 动态角度环（行驶中航向保持） ==================*/

// 初始化动态角度环（PID参数在 Control_Init 内修改）
void Control_Init(void);
// 周期调用角度环 PID 更新（建议 10~20 ms 一次）
void Control_AngleUpdate(void);
// 设置平移速度并保持绝对航向，发送最新输出（接口函数）
void Control_AngleHoldMove(float vx, float vy, float hold_yaw_deg);

/*================== 静态角度环（原地转向 yaw 速度环） ==================*/

typedef struct
{
    PID_t pid;
    float target_yaw_deg;
    float feedback_yaw_deg;
    float error_yaw_deg;
    float omega_cmd_deg_s;
    float max_omega_deg_s;
    float tolerance_deg;
    uint32_t last_tick_ms;
    uint8_t enabled;
    uint8_t arrived;
} Control_StaticAngle_t;

extern Control_StaticAngle_t Control_StaticAngle;

// 初始化静态角度环（PID参数在 Control_YawInit 内修改）
void Control_YawInit(void);
// 设置目标角度并开始闭环旋转（接口函数）
void Control_YawStart(float target_yaw_deg);
// 周期调用角度环 PID 更新（建议 10~20 ms 一次）
void Control_YawUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
