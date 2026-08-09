#include "control.h"

#include <math.h>
#include <string.h>

#include "Chassis.h"
#include "kalman.h"
#include "usart_sent.h"

Control_DynamicAngle_t Control_DynamicAngle;
PID_t Control_DynamicAnglePID;
Control_StaticAngle_t Control_StaticAngle;
Control_CrossAlign_t Control_CrossAlign;

/* yaw 纠正偏移量（deg）：对准十字后把当前航向纠正为 -90° 等目标值 */
static float control_yaw_offset_deg = 0.0f;

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
    Control_DynamicAngle.feedback_yaw_deg = Control_GetYawDeg();
    Control_DynamicAngle.target_yaw_deg = Control_DynamicAngle.feedback_yaw_deg;
    Control_DynamicAngle.arrived = 1U;
    Control_DynamicAngle.enabled = 1U;

    /* PID 参数在这里修改 */
    PID_Init(&Control_DynamicAnglePID,
             3.2f,
             0.005f,
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

    Control_DynamicAngle.feedback_yaw_deg = Control_GetYawDeg();
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
                                                           CONTROL_TEST_LOOP_DELAY_MS);
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
        current_yaw_deg = Control_GetYawDeg();
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

            current_yaw_deg = Control_GetYawDeg();
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
    yaw_rad = Control_GetYawDeg() * CONTROL_DEG_TO_RAD;
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
    Control_StaticAngle.tolerance_deg = 0.15f;
    Control_StaticAngle.feedback_yaw_deg = Control_GetYawDeg();
    Control_StaticAngle.target_yaw_deg = Control_StaticAngle.feedback_yaw_deg;
    Control_StaticAngle.arrived = 1U;
    Control_StaticAngle.last_tick_ms = HAL_GetTick();

    /* PID 参数在这里修改 */
    PID_Init(&Control_StaticAngle.pid,
            10.5f,
            0.15f,
            0.05f,
            -Control_StaticAngle.max_omega_deg_s,
            Control_StaticAngle.max_omega_deg_s);
    PID_SetIntegralLimits(&Control_StaticAngle.pid, -40.0f, 40.0f);
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
    uint16_t dt_ms;
    float yaw_rate_deg_s;

    if (Control_StaticAngle.enabled == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    dt_ms = (uint16_t)(now_ms - Control_StaticAngle.last_tick_ms);
    if ((dt_ms == 0U) || (dt_ms > 100U)) {
        Control_StaticAngle.last_tick_ms = now_ms;
        return;
    }
    Control_StaticAngle.last_tick_ms = now_ms;

    Control_StaticAngle.feedback_yaw_deg = Control_GetYawDeg();
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
        dt_ms);
    Control_StaticAngle.omega_cmd_deg_s = control_abs_limit(
        Control_StaticAngle.omega_cmd_deg_s,
        Control_StaticAngle.max_omega_deg_s);
    yaw_rate_deg_s = Control_StaticAngle.omega_cmd_deg_s;
    Chassis_Move(0.0f,
                 0.0f,
                 yaw_rate_deg_s * CONTROL_DEG_TO_RAD);
}

/* ==================== yaw 纠正（对准十字后校正 IMU 累计误差） ==================== */

// 获取纠正后的当前 yaw (deg)。未纠正时与 Kalman 原始值一致。
float Control_GetYawDeg(void)
{
    return control_angle_norm_deg(Kalman_GetYawRad() * CONTROL_RAD_TO_DEG
                                  + control_yaw_offset_deg);
}

// 把当前航向直接纠正为 target_deg（如 -90°）：消除 IMU 累计漂移。
void Control_CorrectYawDeg(float target_deg)
{
    float raw_deg = Kalman_GetYawRad() * CONTROL_RAD_TO_DEG;

    control_yaw_offset_deg = target_deg - raw_deg;
}

/* ==================== K210 十字对准（视觉闭环） ==================== */

void Control_AlignInit(void)
{
    memset(&Control_CrossAlign, 0, sizeof(Control_CrossAlign));

    Control_CrossAlign.tol_x_cm = 0.5f;     /* cm */
    Control_CrossAlign.tol_y_cm = 0.5f;     /* cm */
    Control_CrossAlign.tol_yaw_deg = 1.0f;  /* deg */
    Control_CrossAlign.settle_ms = 200U;    /* 位置移动结束后的停稳时间 */
    Control_CrossAlign.data_timeout_ms = 500U;

    Control_CrossAlign.phase = 1U;
    Control_CrossAlign.enabled = 1U;
    Control_CrossAlign.arrived = 0U;
}

// 周期调用：轮询 K210 → 两段式位置控制
//   ① 纠 yaw：原地位置旋转，使机器人航向对准十字（K210 yaw -> 0）
//   ② 锁定航向平移：用 Chassis_MovePos 位置控制，把 x/y 平移到 0（对准十字中心）
void Control_AlignUpdate(void)
{
    float x_cm;
    float y_cm;
    float yaw_deg;
    float duration_ms;
    uint8_t fresh;
    uint32_t now_ms;

    if (Control_CrossAlign.enabled == 0U) {
        return;
    }

    /* 轮询 K210 并读取最新十字数据（无新帧时沿用上一帧） */
    K210_Poll();
    fresh = K210_GetCrossData(&x_cm, &y_cm, &yaw_deg);
    if (fresh != 0U) {
        Control_CrossAlign.feedback_x_cm = x_cm;
        Control_CrossAlign.feedback_y_cm = y_cm;
        Control_CrossAlign.feedback_yaw_deg = yaw_deg;
        Control_CrossAlign.data_valid = 1U;
        Control_CrossAlign.last_data_tick = HAL_GetTick();
    }

    /* 还没有收到有效数据：停止等待 */
    if (Control_CrossAlign.data_valid == 0U) {
        Chassis_stop();
        return;
    }

    /* K210 数据超时：停止等待，防止用旧数据继续跑 */
    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - Control_CrossAlign.last_data_tick)
        > (int32_t)Control_CrossAlign.data_timeout_ms) {
        Chassis_stop();
        return;
    }

    /* 正在执行位置移动：等它走完（含停稳时间）再发下一段 */
    if (Control_CrossAlign.move_active != 0U) {
        if ((int32_t)(now_ms - Control_CrossAlign.move_end_tick) >= 0) {
            Control_CrossAlign.move_active = 0U;
            Chassis_stop();
        }
        return;
    }

    /* 阶段①：先纠 yaw —— 原地位置旋转 feedback_yaw_deg 度 */
    if (Control_CrossAlign.phase == 1U) {
        if (fabsf(Control_CrossAlign.feedback_yaw_deg) <= Control_CrossAlign.tol_yaw_deg) {
            Control_CrossAlign.phase = 2U;   /* yaw 已对准，进入平移 */
        } else {
            duration_ms = Chassis_MovePos(0.0f,
                                          0.0f,
                                          Control_CrossAlign.feedback_yaw_deg);
            if (duration_ms > 0.0f) {
                Control_CrossAlign.move_active = 1U;
                Control_CrossAlign.move_end_tick = now_ms
                    + (uint32_t)(duration_ms + 0.5f)
                    + Control_CrossAlign.settle_ms;
            }
        }
        return;
    }

    /* 阶段②：锁定航向位置平移 —— theta=0 保持航向，把 x/y 移到 0 */
    if (Control_CrossAlign.phase == 2U) {
        if ((fabsf(Control_CrossAlign.feedback_x_cm) <= Control_CrossAlign.tol_x_cm)
         && (fabsf(Control_CrossAlign.feedback_y_cm) <= Control_CrossAlign.tol_y_cm)) {
            Control_CrossAlign.arrived = 1U;
            Control_CrossAlign.phase = 3U;
            Chassis_stop();
            return;
        }
        duration_ms = Chassis_MovePos(Control_CrossAlign.feedback_x_cm / 100.0f,
                                      Control_CrossAlign.feedback_y_cm / 100.0f,
                                      0.0f);
        if (duration_ms > 0.0f) {
            Control_CrossAlign.move_active = 1U;
            Control_CrossAlign.move_end_tick = now_ms
                + (uint32_t)(duration_ms + 0.5f)
                + Control_CrossAlign.settle_ms;
        }
        return;
    }

    /* 阶段③：已完成 */
    Chassis_stop();
}

// 阻塞式对准十字：循环调用 Control_AlignUpdate，对准完成后纠正 yaw 为 -90°
uint8_t Control_AlignCross(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    K210_ClearCross();
    Control_AlignInit();

    while ((Control_CrossAlign.arrived == 0U)
        && ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)timeout_ms)) {
        Control_AlignUpdate();
        HAL_Delay(15U);
    }
    Chassis_stop();

    /* 对准成功后才把当前 yaw 纠正为 -90°（消除 IMU 累计漂移）；超时则不做纠正 */
    if (Control_CrossAlign.arrived != 0U) {
        Control_CorrectYawDeg(-90.0f);
    }
    return Control_CrossAlign.arrived;
}

// 测试函数：手动输入 x/y/yaw（十字偏差），演示两段式位置控制：
//   ① 位置旋转纠 yaw → ② 锁定航向位置平移 (x,y)
//   例: Control_AlignTest(10.0f, 5.0f, 8.0f);
void Control_AlignTest(float x_cm, float y_cm, float yaw_deg)
{
    float duration_ms;

    Chassis_stop();
    uart_printf("\r\nAlignTest: x=%.1fcm y=%.1fcm yaw=%.1fdeg\r\n",
                (double)x_cm, (double)y_cm, (double)yaw_deg);

    /* ① 纠 yaw：原地位置旋转 */
    if (fabsf(yaw_deg) > 0.5f) {
        uart_printf("rotate: %.1f°\r\n", (double)yaw_deg);
        duration_ms = Chassis_MovePos(0.0f, 0.0f, yaw_deg);
        if (duration_ms > 0.0f) {
            HAL_Delay((uint32_t)(duration_ms + 0.5f) + 200U);
        }
        Chassis_stop();
        HAL_Delay(200U);
    } else {
        uart_printf("yaw_OK\r\n");
    }

    /* ② 锁定航向位置平移：把 (x,y) 平移到 0 */
    if ((fabsf(x_cm) > 0.5f) || (fabsf(y_cm) > 0.5f)) {
        uart_printf("move:sx=%.3fm sy=%.3fm\r\n",
                    (double)(x_cm / 100.0f), (double)(y_cm / 100.0f));
        duration_ms = Chassis_MovePos(x_cm / 100.0f, y_cm / 100.0f, 0.0f);
        if (duration_ms > 0.0f) {
            HAL_Delay((uint32_t)(duration_ms + 0.5f) + 200U);
        }
        Chassis_stop();
    } else {
        uart_printf("move_OK\r\n");
    }

    uart_printf("AlignTest done\r\n");
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

/* ==================== 状态机辅助函数 ==================== */

/* 定角度位置旋转参数（Control_StaticTurn 使用） */
#define CONTROL_STATIC_TURN_COARSE_THRESH_DEG 5.0f    /* 粗调触发阈值 (deg) */
#define CONTROL_STATIC_TURN_SETTLE_MS         150U    /* 位置旋转结束后停稳时间 (ms) */

/**
 * @brief 定角度位置旋转到指定角度（阻塞式）
 *  ① 粗调：Chassis_MovePos 位置旋转 (target - current)，一次转到目标附近
 *  ② 微调：静态角度环闭环转到目标（到位阈值见 Control_YawInit 的 tolerance_deg）
 * @param target_deg  目标角度 (deg)
 * @param timeout_ms  超时(ms)
 * @retval 1=到位，0=超时
 */
uint8_t Control_StaticTurn(float target_deg, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    float current_yaw_deg;
    float error_deg;

    /* ① 粗调：定角度位置控制旋转 (target - current) */
    current_yaw_deg = Control_GetYawDeg();
    error_deg = control_angle_norm_deg(target_deg - current_yaw_deg);
    if (fabsf(error_deg) > CONTROL_STATIC_TURN_COARSE_THRESH_DEG) {
        (void)Chassis_MovePosBlocking(0.0f, 0.0f, error_deg);
        Chassis_stop();
        HAL_Delay(CONTROL_STATIC_TURN_SETTLE_MS);
    }

    /* ② 微调：静态角度环闭环转到目标角度 */
    Control_YawInit();
    Control_YawStart(target_deg);
    while ((Control_StaticAngle.arrived == 0U)//只有还没到达阈值才会执行
        && ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)timeout_ms)) {
        Control_YawUpdate();
        HAL_Delay(15U);//退出这个循环就结束了函数
    }
    Chassis_stop();
    return Control_StaticAngle.arrived;
}

/* ==================== 动态角度环平移辅助（阻塞式） ==================== */

/**
 * @brief 动态角度环保持航向平移指定距离（阻塞式，延时 = 距离/速度）
 * @param vx,vy        世界系平移速度方向 (m/s)，实际大小取 speed_mps
 * @param hold_yaw_deg 保持的绝对航向角 (deg)
 * @param distance_m   平移距离 (m)
 * @param speed_mps    平移速度大小 (m/s)，<=0 时用默认 0.2
 * @retval 实际运行时间 (ms)
 */
uint32_t Control_MoveHoldYaw(float vx, float vy, float hold_yaw_deg,
                             float distance_m, float speed_mps)
{
    float dir_len;
    float move_vx;
    float move_vy;
    uint32_t duration_ms;
    uint32_t end_tick;

    if (speed_mps <= 0.0f) {
        speed_mps = 0.2f;
    }

    dir_len = sqrtf(vx * vx + vy * vy);
    if (dir_len > 0.0001f) {
        move_vx = vx / dir_len;
        move_vy = vy / dir_len;
    } else {
        move_vx = 0.0f;
        move_vy = 1.0f;    /* 默认向前 */
    }

    if (distance_m < 0.0f) {
        move_vx = -move_vx;
        move_vy = -move_vy;
        distance_m = -distance_m;
    }

    duration_ms = (uint32_t)(distance_m / speed_mps * 1000.0f + 0.5f);
    end_tick = HAL_GetTick() + duration_ms;

    while ((int32_t)(HAL_GetTick() - end_tick) < 0) {
        Control_AngleUpdate();
        Control_AngleHoldMove(move_vx * speed_mps, move_vy * speed_mps, hold_yaw_deg);
        HAL_Delay(CONTROL_TEST_LOOP_DELAY_MS);
    }
    Chassis_stop();
    return duration_ms;
}
