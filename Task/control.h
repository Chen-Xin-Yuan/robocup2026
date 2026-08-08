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

// 定角度位置旋转到指定角度（阻塞式：位置控制粗调 + 静态角度环微调）
uint8_t Control_StaticTurn(float target_deg, uint32_t timeout_ms);
// 动态角度环保持航向平移指定距离（阻塞式），返回运行时间(ms)
uint32_t Control_MoveHoldYaw(float vx, float vy, float hold_yaw_deg,
                              float distance_m, float speed_mps);

/*================== K210 十字对准（视觉闭环） ==================*/

/* 十字对准状态（K210 回传 x/y/yaw，两段式位置控制对准十字）
 *   ① 纠 yaw：原地位置旋转，使机器人航向对准十字（K210 yaw -> 0）
 *   ② 锁定航向平移：用 Chassis_MovePos 位置控制，把 x/y 平移到 0（对准十字中心）
 */
typedef struct
{
    /* K210 反馈 */
    float feedback_x_cm;      /* 十字中心横向偏差 (cm)，右为正 */
    float feedback_y_cm;      /* 十字中心距离偏差 (cm)，前为正 */
    float feedback_yaw_deg;   /* 十字角度偏差 (deg)，逆时针为正 */
    uint8_t data_valid;       /* 是否收到过有效 CROSS 数据 */

    /* 到位阈值（在 Control_AlignInit 内修改） */
    float tol_x_cm;           /* x 到位阈值 (cm) */
    float tol_y_cm;           /* y 到位阈值 (cm) */
    float tol_yaw_deg;        /* yaw 到位阈值 (deg) */
    uint32_t settle_ms;       /* 位置移动结束后的停稳时间 (ms) */
    uint32_t data_timeout_ms; /* K210 数据超时时间 (ms)，超时停止等待 */

    /* 两段式对准状态机: 1=纠yaw, 2=锁定航向平移, 3=完成 */
    uint8_t phase;
    uint8_t move_active;      /* 是否正在执行位置移动 */
    uint32_t move_end_tick;   /* 位置移动预计结束时刻 */

    uint32_t last_data_tick;  /* 最近一次收到 K210 数据的时间戳 */

    uint8_t enabled;
    uint8_t arrived;
} Control_CrossAlign_t;

extern Control_CrossAlign_t Control_CrossAlign;

// 初始化十字对准（阈值/停稳时间在 Control_AlignInit 内修改）
void Control_AlignInit(void);
// 周期调用十字对准（建议 10~20 ms 一次，内部会轮询 K210）
// 两段式：①位置旋转纠 yaw，②锁定航向位置平移对准十字中心
void Control_AlignUpdate(void);
// 阻塞式对准十字：对准完成后把当前 yaw 纠正为 -90°
uint8_t Control_AlignCross(uint32_t timeout_ms);

// 测试函数：手动输入 x/y/yaw（十字偏差），演示两段式位置控制：
//   ① 位置旋转纠 yaw → ② 锁定航向位置平移 (x,y)
void Control_AlignTest(float x_cm, float y_cm, float yaw_deg);

/*================== yaw 纠正（对准十字后校正 IMU 累计误差） ==================*/

// 把当前航向纠正为 target_deg（如 -90°），后续所有角度环都使用纠正后的 yaw
void Control_CorrectYawDeg(float target_deg);
// 获取纠正后的当前 yaw (deg)
float Control_GetYawDeg(void);

#ifdef __cplusplus
}
#endif

#endif
