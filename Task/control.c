#include "control.h"

#include <math.h>
#include <string.h>

#include "Chassis.h"
#include "kalman.h"

Control_t Control;
static PID_t Control_AnglePID;
static Control_YawLoop_t Control_YawLoop;

#define CONTROL_TEST_SETTLE_TIME_MS       100U

/* ==================== 静态函数（未使用/内部保留，放在顶部） ==================== */
/* 说明：以下函数目前主程序不需要调用，全部改为 static 保留在文件顶部。
   如果以后确认用不到，可以直接整段删除。 */

/* 下方静态函数会调用到的公共接口（定义在文件底部） */
void Control_Init(void);
void Control_Enable(uint8_t enable);
void Control_SetAngleDeg(float target_yaw_deg);

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

/* ---------------- 未使用：独立的 yaw 速度环（保留备用） ---------------- */

static void Control_YawInit(void)
{
    memset(&Control_YawLoop, 0, sizeof(Control_YawLoop));

    Control_YawLoop.max_omega_deg_s = 40.0f;
    Control_YawLoop.tolerance_deg = 1.0f;
    Control_YawLoop.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control_YawLoop.target_yaw_deg = Control_YawLoop.feedback_yaw_deg;
    Control_YawLoop.arrived = 1U;
    Control_YawLoop.last_tick_ms = HAL_GetTick();

    /* These are usable startup values; tune them with Control_YawSetPID(). */
    PID_Init(&Control_YawLoop.pid,
             1.2f, 0.001f, 0.05f,
             -Control_YawLoop.max_omega_deg_s,
             Control_YawLoop.max_omega_deg_s);
    PID_SetIntegralLimits(&Control_YawLoop.pid, -20.0f, 20.0f);
    PID_SetDeadband(&Control_YawLoop.pid, 0.1f);
    PID_SetDerivativeFilter(&Control_YawLoop.pid, 0.2f);
}

static void Control_YawSetPID(float kp, float ki, float kd)
{
    PID_SetTunings(&Control_YawLoop.pid, kp, ki, kd);
    PID_Reset(&Control_YawLoop.pid);
}

static void Control_YawSetTolerance(float tolerance_deg)
{
    Control_YawLoop.tolerance_deg = fabsf(tolerance_deg);
}

static void Control_YawSetMaxOmega(float max_omega_deg_s)
{
    max_omega_deg_s = fabsf(max_omega_deg_s);
    Control_YawLoop.max_omega_deg_s = max_omega_deg_s;
    PID_SetOutputLimits(&Control_YawLoop.pid,
                        -max_omega_deg_s,
                        max_omega_deg_s);
}

static void Control_YawStart(float target_yaw_deg)
{
    Control_YawLoop.target_yaw_deg = target_yaw_deg;
    Control_YawLoop.error_yaw_deg = 0.0f;
    Control_YawLoop.omega_cmd_deg_s = 0.0f;
    Control_YawLoop.last_tick_ms = HAL_GetTick();
    Control_YawLoop.enabled = 1U;
    Control_YawLoop.arrived = 0U;
    PID_Reset(&Control_YawLoop.pid);
}

static void Control_YawUpdate(void)
{
    uint32_t now_ms;
    float dt_s;
    float yaw_rate_deg_s;

    if (Control_YawLoop.enabled == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    dt_s = (float)(now_ms - Control_YawLoop.last_tick_ms) * 0.001f;
    if ((dt_s <= 0.0f) || (dt_s > 0.1f)) {
        Control_YawLoop.last_tick_ms = now_ms;
        return;
    }
    Control_YawLoop.last_tick_ms = now_ms;

    Control_YawLoop.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control_YawLoop.error_yaw_deg = control_angle_norm_deg(
        Control_YawLoop.target_yaw_deg - Control_YawLoop.feedback_yaw_deg);

    if (fabsf(Control_YawLoop.error_yaw_deg) <= Control_YawLoop.tolerance_deg) {
        if (Control_YawLoop.arrived == 0U) {
            Chassis_stop();
        }
        Control_YawLoop.arrived = 1U;
        Control_YawLoop.omega_cmd_deg_s = 0.0f;
        PID_Reset(&Control_YawLoop.pid);
        return;
    }

    Control_YawLoop.arrived = 0U;
    Control_YawLoop.omega_cmd_deg_s = PID_UpdateError(
        &Control_YawLoop.pid,
        Control_YawLoop.error_yaw_deg,
        dt_s);
    Control_YawLoop.omega_cmd_deg_s = control_abs_limit(
        Control_YawLoop.omega_cmd_deg_s,
        Control_YawLoop.max_omega_deg_s);
    yaw_rate_deg_s = Control_YawLoop.omega_cmd_deg_s;
    Chassis_Move(0.0f,
                 0.0f,
                 yaw_rate_deg_s * CONTROL_DEG_TO_RAD);
}

static uint8_t Control_YawIsArrived(void)
{
    return Control_YawLoop.arrived;
}

static void Control_YawStop(void)
{
    Control_YawLoop.enabled = 0U;
    Control_YawLoop.omega_cmd_deg_s = 0.0f;
    Control_YawLoop.arrived = 1U;
    PID_Reset(&Control_YawLoop.pid);
    Chassis_stop();
}

/* ---------------- 未使用：纯底盘旋转（阻塞式，保留备用） ---------------- */

//纯底盘控制旋转一定角度
static void Control_AnglePositionMove(float angle_deg)
{
    float duration_ms;

    do {
        duration_ms = Chassis_MovePos(0.0f, 0.0f, angle_deg);
        if (duration_ms < 0.0f) {
            HAL_Delay(1U);
        }
    } while (duration_ms < 0.0f);

    if (duration_ms > 0.0f) {
        HAL_Delay((uint32_t)(duration_ms + 0.5f));
    }
}

/* ---------------- 未使用：角度设置的便捷封装（保留备用） ---------------- */

// Set an absolute target yaw in radians.
static void Control_SetAngleRad(float target_yaw_rad)
{
    Control_SetAngleDeg(target_yaw_rad * CONTROL_RAD_TO_DEG);
}

// Set a relative turn in degrees from current yaw.
static void Control_SetAngleRelativeDeg(float delta_yaw_deg)
{
    Control_SetAngleDeg(Kalman_GetYawRad() * CONTROL_RAD_TO_DEG + delta_yaw_deg);
}

// 基于当前yaw角旋转delta_yaw_rad角度（相对角度）
static void Control_SetAngleRelativeRad(float delta_yaw_rad)
{
    Control_SetAngleRelativeDeg(delta_yaw_rad * CONTROL_RAD_TO_DEG);
}

/* ---------------- 未使用：闭环自测（保留备用） ---------------- */

// Start the non-blocking angle-hold test.
static void Control_test(void)
{
    Control_Init();

    Control.enabled = 0U;
    HAL_Delay(1000U);

    Control_AnglePositionMove(90.0f);

    Control_Enable(1U);
}

/* ==================== 需要调用的闭环控制函数（公共接口，放在底部） ==================== */

// Initialize the angle loop with the tuned test parameters.
void Control_Init(void)
{
    memset(&Control, 0, sizeof(Control));

    Control.max_omega_deg_s = 40.0f;
    Control.arrive_error_deg = 0.5f;
    Control.arrive_omega_deg_s = 5.0f;
    Control.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control.target_yaw_deg = Control.feedback_yaw_deg;
    Control.arrived = 1U;

    PID_Init(&Control_AnglePID,
             1.2f,
             0.001f,
             0.05f,
             -40.0f,
             40.0f);
    PID_SetIntegralLimits(&Control_AnglePID,
                          -20.0f,
                          20.0f);
    PID_SetDeadband(&Control_AnglePID, 0.57296f);
    PID_SetDerivativeFilter(&Control_AnglePID, 0.2f);
}

// Clear runtime state but keep gains.
void Control_Reset(void)
{
    PID_Reset(&Control_AnglePID);

    Control.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control.target_yaw_deg = Control.feedback_yaw_deg;
    Control.error_yaw_deg = 0.0f;
    Control.omega_cmd_deg_s = 0.0f;
    Control.arrived = 1U;
    Control.coarse_turn_pending = 0U;
    Control.coarse_turn_active = 0U;
    Control.coarse_turn_stop_sent = 0U;
    Control.coarse_turn_ready_tick = 0U;
    Control.coarse_turn_end_tick = 0U;
}

// Enable or disable the angle loop.
void Control_Enable(uint8_t enable)
{
    Control.enabled = (enable != 0U) ? 1U : 0U;
    PID_Reset(&Control_AnglePID);

    if (Control.enabled == 0U) {
        Control.omega_cmd_deg_s = 0.0f;
    }
}

//绝对角度
void Control_SetAngleDeg(float target_yaw_deg)
{
    Control.target_yaw_deg = target_yaw_deg;
    Control.arrived = 0U;
    Control.coarse_turn_pending = 1U;
    Control.coarse_turn_active = 0U;
    Control.coarse_turn_stop_sent = 0U;
    Control.coarse_turn_ready_tick = 0U;
}

//设置pid参数
void Control_SetAnglePid(float kp, float ki, float kd)
{
    PID_SetTunings(&Control_AnglePID, kp, ki, kd);
}

// Set the maximum yaw rate output.
void Control_SetAngleOutputLimit(float max_omega_deg_s)
{
    if (max_omega_deg_s < 0.0f) {
        max_omega_deg_s = -max_omega_deg_s;
    }

    Control.max_omega_deg_s = max_omega_deg_s;
    PID_SetOutputLimits(&Control_AnglePID,
                        -Control.max_omega_deg_s,
                        Control.max_omega_deg_s);
}

//阈值
void Control_SetAngleArriveThreshold(float error_deg, float omega_deg_s)
{
    Control.arrive_error_deg = fabsf(error_deg);
    Control.arrive_omega_deg_s = fabsf(omega_deg_s);
}

//角度环更新，结果保存在 Control.omega_cmd_deg_s
void Control_AngleUpdate(void)
{
    float yaw_omega_deg_s;

    Control.feedback_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
    Control.error_yaw_deg = control_angle_norm_deg(Control.target_yaw_deg
                                                   - Control.feedback_yaw_deg);

    if (Control.enabled == 0U) {
        Control.omega_cmd_deg_s = 0.0f;
        return;//没有使能直接返回
    }

    if ((Control.coarse_turn_pending != 0U)
     || (Control.coarse_turn_active != 0U)) {
        Control.omega_cmd_deg_s = 0.0f;
        return;//现在处于位置环粗调阶段，角度环不工作,直接返回
    }

    Control.omega_cmd_deg_s = PID_UpdateError(&Control_AnglePID,
                                              Control.error_yaw_deg,
                                              (float)CONTROL_TEST_LOOP_DELAY_MS * 0.001f);
    Control.omega_cmd_deg_s = control_abs_limit(Control.omega_cmd_deg_s,
                                                Control.max_omega_deg_s);//限幅

    yaw_omega_deg_s = Kalman_GetYawOmegaRad() * CONTROL_RAD_TO_DEG;
    Control.arrived = (uint8_t)((fabsf(Control.error_yaw_deg) <= Control.arrive_error_deg)
                             && (fabsf(yaw_omega_deg_s) <= Control.arrive_omega_deg_s));

    /* Stop correcting once both arrival conditions are met. */
    if (Control.arrived != 0U) {
        PID_Reset(&Control_AnglePID);
        Control.omega_cmd_deg_s = 0.0f;
    }
}

// Set the motion command and send the latest angle-loop output.
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

    if (fabsf(Control.target_yaw_deg - hold_yaw_deg) > 0.001f) {
        Control_SetAngleDeg(hold_yaw_deg);
    }//设置目标角度

    //如果粗调正在进行中，检查是否已经结束
    if (Control.coarse_turn_active != 0U) {
        if ((int32_t)(HAL_GetTick() - Control.coarse_turn_end_tick) < 0) {
            return;
        }
        Control.coarse_turn_active = 0U;
        PID_Reset(&Control_AnglePID);
        //不是重置参数,主要是重置积分项,否则粗调结束后角度环会有一个大的积分输出
    }
    //如果粗调还没有开始，检查是否需要进行粗调
    if (Control.coarse_turn_pending != 0U) {
        current_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
        coarse_error_deg = control_angle_norm_deg(Control.target_yaw_deg
                                                  - current_yaw_deg);

        /* Leave a small residual for the angle PID to correct. */
        if (fabsf(coarse_error_deg) > 5.0f) {
            if (Control.coarse_turn_stop_sent == 0U) {
                Chassis_stop();
                Control.coarse_turn_stop_sent = 1U;
                Control.coarse_turn_ready_tick = HAL_GetTick()
                                               + CONTROL_TEST_LOOP_DELAY_MS;
                return;
            }

            if ((int32_t)(HAL_GetTick() - Control.coarse_turn_ready_tick) < 0) {
                return;
            }

            current_yaw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;
            coarse_error_deg = control_angle_norm_deg(Control.target_yaw_deg
                                                      - current_yaw_deg);
            if (fabsf(coarse_error_deg) <= 5.0f) {
                Control.coarse_turn_pending = 0U;
                Control.coarse_turn_stop_sent = 0U;
            } else {
                coarse_angle_deg = coarse_error_deg
                                 - ((coarse_error_deg > 0.0f) ? 2.0f : -2.0f);
                coarse_duration_ms = Chassis_MovePos(0.0f,
                                                     0.0f,
                                                     coarse_angle_deg);
                if (coarse_duration_ms < 0.0f) {
                    return;
                }

                Control.coarse_turn_pending = 0U;
                Control.coarse_turn_stop_sent = 0U;
                Control.omega_cmd_deg_s = 0.0f;
                if (coarse_duration_ms > 0.0f) {
                    Control.coarse_turn_active = 1U;
                    Control.coarse_turn_end_tick = HAL_GetTick()
                                                 + (uint32_t)(coarse_duration_ms + 0.5f)
                                                 + CONTROL_TEST_SETTLE_TIME_MS;
                    return;
                }
            }
        } else {
            Control.coarse_turn_pending = 0U;
            Control.coarse_turn_stop_sent = 0U;
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
                 Control.omega_cmd_deg_s * CONTROL_DEG_TO_RAD);
}

// Return nonzero when the target is reached.
uint8_t Control_AngleIsArrived(void)
{
    return Control.arrived;
}

// Stop the angle loop and the chassis.
void Control_Stop(void)
{
    Control_Enable(0U);
    Control_Reset();
    Chassis_stop();
}
