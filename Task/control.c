#include "control.h"

#include <math.h>
#include <string.h>

#include "Chassis.h"
#include "kalman.h"

Control_DynamicAngle_t Control_DynamicAngle;
PID_t Control_DynamicAnglePID;
Control_StaticAngle_t Control_StaticAngle;

#define CONTROL_TEST_SETTLE_TIME_MS       100U

/* 测试参数（Control_test 使用） */
#define CONTROL_TEST_YAW_TARGET_DEG       90.0f
#define CONTROL_TEST_FORWARD_SPEED_MPS    0.10f
#define CONTROL_TEST_YAW_TIMEOUT_MS       5000U
#define CONTROL_TEST_DRIVE_TIME_MS        10000U

/* ==================== 内部工具函数（static，仅供本文件使用） ==================== */

// Clamp a value to [-limit, limit].
static float control_abs_limit(float value, float limit)
{
    if (limit < 0.0f) {
        limit = -limit;
    }

    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

// Normalize angle to [-180, 180] degrees.
static float control_angle_norm_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/* ==================== 动态角度环（行驶中航向保持） ==================== */

// 设置目标角度（内部函数，仅 Control_AngleHoldMove 调用）
static void Control_SetAngleDeg(float target_yaw_deg)
{
    Control_DynamicAngle.target_yaw_deg = target_yaw_deg;
    Control_DynamicAngle.arrived = 0U;
    Control_DynamicAngle.coarse_turn_pending = 1U;
    Control_DynamicAngle.coarse_turn_active = 0U;
    Control_DynamicAngle.coarse_turn_stop_sent = 0U;
    Control_DynamicAngle.coarse_turn_ready_tick = 0U;
}

// 初始化动态角度环（PID 参数直接在这里修改）
void Control_Init(void)
{
    memset(&Control_DynamicAngle, 0, sizeof(Control_DynamicAngle));

    Control_DynamicAngle.max_omega_deg_s = 40.0f;
    Control_DynamicAngle.arrive_error_deg = 0.5f;
    Control_DynamicAngle.arrive_omega_deg_s = 5.0f;
    Control_DynamicAngle.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control_DynamicAngle.target_yaw_deg = Control_DynamicAngle.feedback_yaw_deg;
    Control_DynamicAngle.arrived = 1U;
    Control_DynamicAngle.enabled = 1U;

    /* PID 参数在这里修改 */
    PID_Init(&Control_DynamicAnglePID,
             1.2f,
             0.001f,
             0.05f,
             -40.0f,
             40.0f);
    PID_SetIntegralLimits(&Control_DynamicAnglePID,
                          -20.0f,
                          20.0f);
    PID_SetDeadband(&Control_DynamicAnglePID, 0.57296f);
    PID_SetDerivativeFilter(&Control_DynamicAnglePID, 0.2f);
}

// 角度环 PID 更新，结果保存在 Control_DynamicAngle.omega_cmd_deg_s
void Control_AngleUpdate(void)
{
    float yaw_omega_deg_s;

    Control_DynamicAngle.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control_DynamicAngle.error_yaw_deg = control_angle_norm_deg(
        Control_DynamicAngle.target_yaw_deg - Control_DynamicAngle.feedback_yaw_deg);

    if (Control_DynamicAngle.enabled == 0U) {
        Control_DynamicAngle.omega_cmd_deg_s = 0.0f;
        return;//没有使能直接返回
    }

    if ((Control_DynamicAngle.coarse_turn_pending != 0U)
     || (Control_DynamicAngle.coarse_turn_active != 0U)) {
        Control_DynamicAngle.omega_cmd_deg_s = 0.0f;
        return;//现在处于位置环粗调阶段，角度环不工作,直接返回
    }

    Control_DynamicAngle.omega_cmd_deg_s = PID_UpdateError(&Control_DynamicAnglePID,
                                                           Control_DynamicAngle.error_yaw_deg,
                                                           (float)CONTROL_TEST_LOOP_DELAY_MS * 0.001f);
    Control_DynamicAngle.omega_cmd_deg_s = control_abs_limit(Control_DynamicAngle.omega_cmd_deg_s,
                                                             Control_DynamicAngle.max_omega_deg_s);//限幅

    yaw_omega_deg_s = Kalman_GetYawOmegaRad() * CONTROL_RAD_TO_DEG;
    Control_DynamicAngle.arrived = (uint8_t)((fabsf(Control_DynamicAngle.error_yaw_deg) <= Control_DynamicAngle.arrive_error_deg)
                                          && (fabsf(yaw_omega_deg_s) <= Control_DynamicAngle.arrive_omega_deg_s));

    /* Stop correcting once both arrival conditions are met. */
    if (Control_DynamicAngle.arrived != 0U) {
        PID_Reset(&Control_DynamicAnglePID);
        Control_DynamicAngle.omega_cmd_deg_s = 0.0f;
    }
}

// 设置平移速度并保持绝对航向，发送最新输出（外部接口函数）
void Control_AngleHoldMove(float vx, float vy, float hold_yaw_deg)
{
    float current_yaw_deg;//当前yaw角度
    float coarse_error_deg;//粗调误差
    float coarse_angle_deg;//粗调角度
    float coarse_duration_ms;//粗调时间
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float vx_body;
    float vy_body;

    if (fabsf(Control_DynamicAngle.target_yaw_deg - hold_yaw_deg) > 0.001f) {
        Control_SetAngleDeg(hold_yaw_deg);
    }//设置目标角度

    //如果粗调正在进行中，检查是否已经结束
    if (Control_DynamicAngle.coarse_turn_active != 0U) {
        if ((int32_t)(HAL_GetTick() - Control_DynamicAngle.coarse_turn_end_tick) < 0) {
            return;
        }
        Control_DynamicAngle.coarse_turn_active = 0U;
        PID_Reset(&Control_DynamicAnglePID);
        //不是重置参数,主要是重置积分项,否则粗调结束后角度环会有一个大的积分输出
    }
    //如果粗调还没有开始，检查是否需要进行粗调
    if (Control_DynamicAngle.coarse_turn_pending != 0U) {
        current_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
        coarse_error_deg = control_angle_norm_deg(Control_DynamicAngle.target_yaw_deg
                                                  - current_yaw_deg);

        /* Leave a small residual for the angle PID to correct. */
        if (fabsf(coarse_error_deg) > 5.0f) {
            if (Control_DynamicAngle.coarse_turn_stop_sent == 0U) {
                Chassis_stop();
                Control_DynamicAngle.coarse_turn_stop_sent = 1U;
                Control_DynamicAngle.coarse_turn_ready_tick = HAL_GetTick()
                                                            + CONTROL_TEST_LOOP_DELAY_MS;
                return;
            }

            if ((int32_t)(HAL_GetTick() - Control_DynamicAngle.coarse_turn_ready_tick) < 0) {
                return;
            }

            current_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
            coarse_error_deg = control_angle_norm_deg(Control_DynamicAngle.target_yaw_deg
                                                      - current_yaw_deg);
            if (fabsf(coarse_error_deg) <= 5.0f) {
                Control_DynamicAngle.coarse_turn_pending = 0U;
                Control_DynamicAngle.coarse_turn_stop_sent = 0U;
            } else {
                coarse_angle_deg = coarse_error_deg
                                 - ((coarse_error_deg > 0.0f) ? 2.0f : -2.0f);
                coarse_duration_ms = Chassis_MovePos(0.0f,
                                                     0.0f,
                                                     coarse_angle_deg);
                if (coarse_duration_ms < 0.0f) {
                    return;
                }

                Control_DynamicAngle.coarse_turn_pending = 0U;
                Control_DynamicAngle.coarse_turn_stop_sent = 0U;
                Control_DynamicAngle.omega_cmd_deg_s = 0.0f;
                if (coarse_duration_ms > 0.0f) {
                    Control_DynamicAngle.coarse_turn_active = 1U;
                    Control_DynamicAngle.coarse_turn_end_tick = HAL_GetTick()
                                                              + (uint32_t)(coarse_duration_ms + 0.5f)
                                                              + CONTROL_TEST_SETTLE_TIME_MS;
                    return;
                }
            }
        } else {
            Control_DynamicAngle.coarse_turn_pending = 0U;
            Control_DynamicAngle.coarse_turn_stop_sent = 0U;
        }
    }

    /* 粗调结束后立即前进，角度环在行进过程中进行小幅纠偏。 */
    yaw_rad = Kalman_GetYawRad();
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    vx_body = vx * cos_yaw + vy * sin_yaw;
    vy_body = -vx * sin_yaw + vy * cos_yaw;

    Chassis_Move(vx_body,
                 vy_body,
                 Control_DynamicAngle.omega_cmd_deg_s * CONTROL_DEG_TO_RAD);
}

/* ==================== 静态角度环（原地转向 yaw 速度环） ==================== */

// 初始化静态角度环（PID 参数直接在这里修改）
void Control_YawInit(void)
{
    memset(&Control_StaticAngle, 0, sizeof(Control_StaticAngle));

    Control_StaticAngle.max_omega_deg_s = 40.0f;
    Control_StaticAngle.tolerance_deg = 1.0f;
    Control_StaticAngle.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control_StaticAngle.target_yaw_deg = Control_StaticAngle.feedback_yaw_deg;
    Control_StaticAngle.arrived = 1U;
    Control_StaticAngle.last_tick_ms = HAL_GetTick();

    /* PID 参数在这里修改 */
    PID_Init(&Control_StaticAngle.pid,
            1.2f,
            0.001f,
            0.05f,
            -Control_StaticAngle.max_omega_deg_s,
            Control_StaticAngle.max_omega_deg_s);
    PID_SetIntegralLimits(&Control_StaticAngle.pid, -20.0f, 20.0f);
    PID_SetDeadband(&Control_StaticAngle.pid, 0.1f);
    PID_SetDerivativeFilter(&Control_StaticAngle.pid, 0.2f);
}

// 设置目标角度并开始闭环旋转（外部接口函数）
void Control_YawStart(float target_yaw_deg)
{
    Control_StaticAngle.target_yaw_deg = target_yaw_deg;
    Control_StaticAngle.error_yaw_deg = 0.0f;
    Control_StaticAngle.omega_cmd_deg_s = 0.0f;
    Control_StaticAngle.last_tick_ms = HAL_GetTick();
    Control_StaticAngle.enabled = 1U;
    Control_StaticAngle.arrived = 0U;
    PID_Reset(&Control_StaticAngle.pid);
}

// 周期调用角度环 PID 更新
void Control_YawUpdate(void)
{
    uint32_t now_ms;
    float dt_s;
    float yaw_rate_deg_s;

    if (Control_StaticAngle.enabled == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    dt_s = (float)(now_ms - Control_StaticAngle.last_tick_ms) * 0.001f;
    if ((dt_s <= 0.0f) || (dt_s > 0.1f)) {
        Control_StaticAngle.last_tick_ms = now_ms;
        return;
    }
    Control_StaticAngle.last_tick_ms = now_ms;

    Control_StaticAngle.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control_StaticAngle.error_yaw_deg = control_angle_norm_deg(
        Control_StaticAngle.target_yaw_deg - Control_StaticAngle.feedback_yaw_deg);

    if (fabsf(Control_StaticAngle.error_yaw_deg) <= Control_StaticAngle.tolerance_deg) {
        if (Control_StaticAngle.arrived == 0U) {
            Chassis_stop();
        }
        Control_StaticAngle.arrived = 1U;
        Control_StaticAngle.omega_cmd_deg_s = 0.0f;
        PID_Reset(&Control_StaticAngle.pid);
        return;
    }

    Control_StaticAngle.arrived = 0U;
    Control_StaticAngle.omega_cmd_deg_s = PID_UpdateError(
        &Control_StaticAngle.pid,
        Control_StaticAngle.error_yaw_deg,
        dt_s);
    Control_StaticAngle.omega_cmd_deg_s = control_abs_limit(
        Control_StaticAngle.omega_cmd_deg_s,
        Control_StaticAngle.max_omega_deg_s);
    yaw_rate_deg_s = Control_StaticAngle.omega_cmd_deg_s;
    Chassis_Move(0.0f,
                 0.0f,
                 yaw_rate_deg_s * CONTROL_DEG_TO_RAD);
}

/* ==================== 测试函数 ==================== */

// 依次验证两个角度环（阻塞式）：
//   1) 静态角度环：原地转到 90°，到位后停止；
//   2) 动态角度环：以 0.1 m/s 前进 3 秒，保持 0° 航向。
void Control_test(void)
{
    uint32_t start_tick;

    /* 1) 静态角度环：原地旋转到 90° */
    Control_YawInit();
    Control_YawStart(CONTROL_TEST_YAW_TARGET_DEG);

    start_tick = HAL_GetTick();
    while ((Control_StaticAngle.arrived == 0U)
        && ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)CONTROL_TEST_YAW_TIMEOUT_MS)) {
        Control_YawUpdate();
        HAL_Delay(CONTROL_TEST_LOOP_DELAY_MS);
    }
    Chassis_stop();
    HAL_Delay(500U);

    /* 2) 动态角度环：前进并保持 0° 航向 */
    Control_Init();

    start_tick = HAL_GetTick();
    while ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)CONTROL_TEST_DRIVE_TIME_MS) {
        Control_AngleUpdate();
        Control_AngleHoldMove(CONTROL_TEST_FORWARD_SPEED_MPS, 0.0f, 0.0f);
        HAL_Delay(CONTROL_TEST_LOOP_DELAY_MS);
    }
    Chassis_stop();
    HAL_Delay(500U);

    start_tick = HAL_GetTick();
    while ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)CONTROL_TEST_DRIVE_TIME_MS) {
        Control_AngleUpdate();
        Control_AngleHoldMove(0.0f, 0.1f, 60.0f);
        HAL_Delay(CONTROL_TEST_LOOP_DELAY_MS);
    }
    Chassis_stop();
}
