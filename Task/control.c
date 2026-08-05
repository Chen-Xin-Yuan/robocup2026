#include "control.h"

#include <math.h>
#include <string.h>

#include "Chassis.h"
#include "kalman.h"

Control_t Control;
PID_t Control_AnglePID;

#define CONTROL_TEST_HEADING_DEG          90.0f
#define CONTROL_TEST_ROTATE_SPEED_MPS     0.25f
#define CONTROL_TEST_FORWARD_SPEED_MPS    0.20f
#define CONTROL_TEST_MOVE_TIME_MS         3000U
#define CONTROL_TEST_SETTLE_TIME_MS       100U
#define CONTROL_TEST_LOOP_DELAY_MS        10U
#define CONTROL_TEST_LOOP_DT_S            0.01f

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

// Normalize angle to [-pi, pi].
static float control_angle_norm(float angle_rad)
{
    while (angle_rad > CONTROL_PI) {
        angle_rad -= 2.0f * CONTROL_PI;
    }
    while (angle_rad < -CONTROL_PI) {
        angle_rad += 2.0f * CONTROL_PI;
    }
    return angle_rad;
}

static void control_service_chassis_for(uint32_t duration_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) < duration_ms) {
        Chassis_Process();
        HAL_Delay(1U);
    }
}

static void control_run_position_move(float sx, float sy, float theta_deg)
{
    float duration_ms;

    do {
        duration_ms = Chassis_MovePos(sx, sy, theta_deg);
        if (duration_ms < 0.0f) {
            Chassis_Process();
            HAL_Delay(1U);
        }
    } while (duration_ms < 0.0f);

    if (duration_ms > 0.0f) {
        control_service_chassis_for((uint32_t)(duration_ms + 0.5f));
    }
}

// Initialize the angle loop. Default target follows current yaw.
void Control_Init(void)
{
    memset(&Control, 0, sizeof(Control));

    Control.max_omega_rad_s = CONTROL_DEFAULT_MAX_OMEGA;
    Control.arrive_error_rad = CONTROL_DEFAULT_ARRIVE_ERR;
    Control.arrive_omega_rad_s = CONTROL_DEFAULT_ARRIVE_W;
    Control.feedback_yaw_rad = Kalman_GetYawRad();
    Control.target_yaw_rad = Control.feedback_yaw_rad;
    Control.arrived = 1U;

    PID_Init(&Control_AnglePID,
             CONTROL_DEFAULT_KP,
             CONTROL_DEFAULT_KI,
             CONTROL_DEFAULT_KD,
             -CONTROL_DEFAULT_MAX_OMEGA,
             CONTROL_DEFAULT_MAX_OMEGA);
    PID_SetIntegralLimits(&Control_AnglePID, -0.5f, 0.5f);
    PID_SetDeadband(&Control_AnglePID, CONTROL_DEFAULT_DEADBAND);
    PID_SetDerivativeFilter(&Control_AnglePID, 0.2f);
}

// Clear runtime state but keep gains.
void Control_Reset(void)
{
    PID_Reset(&Control_AnglePID);

    Control.feedback_yaw_rad = Kalman_GetYawRad();
    Control.target_yaw_rad = Control.feedback_yaw_rad;
    Control.error_yaw_rad = 0.0f;
    Control.omega_cmd_rad_s = 0.0f;
    Control.arrived = 1U;
}

// Enable or disable the angle loop.
void Control_Enable(uint8_t enable)
{
    Control.enabled = (enable != 0U) ? 1U : 0U;
    PID_Reset(&Control_AnglePID);

    if (Control.enabled == 0U) {
        Control.omega_cmd_rad_s = 0.0f;
    }
}

// Set an absolute target yaw in radians.
void Control_SetAngleRad(float target_yaw_rad)
{
    Control.target_yaw_rad = target_yaw_rad;
    Control.arrived = 0U;
}

//绝对角度
void Control_SetAngleDeg(float target_yaw_deg)
{
    Control_SetAngleRad(target_yaw_deg * CONTROL_DEG_TO_RAD);
}

// 基于当前yaw角旋转delta_yaw_rad角度（相对角度）
void Control_SetAngleRelativeRad(float delta_yaw_rad)
{
    Control_SetAngleRad(Kalman_GetYawRad() + delta_yaw_rad);
}

// Set a relative turn in degrees from current yaw.
void Control_SetAngleRelativeDeg(float delta_yaw_deg)
{
    Control_SetAngleRelativeRad(delta_yaw_deg * CONTROL_DEG_TO_RAD);
}

// Update the PID gains.
void Control_SetAnglePid(float kp, float ki, float kd)
{
    PID_SetTunings(&Control_AnglePID, kp, ki, kd);
}

// Set the maximum yaw rate output.
void Control_SetAngleOutputLimit(float max_omega_rad_s)
{
    if (max_omega_rad_s < 0.0f) {
        max_omega_rad_s = -max_omega_rad_s;
    }

    Control.max_omega_rad_s = max_omega_rad_s;
    PID_SetOutputLimits(&Control_AnglePID,
                        -Control.max_omega_rad_s,
                        Control.max_omega_rad_s);
}

//阈值
void Control_SetAngleArriveThreshold(float error_rad, float omega_rad_s)
{
    Control.arrive_error_rad = fabsf(error_rad);
    Control.arrive_omega_rad_s = fabsf(omega_rad_s);
}

//角度环更新，返回角速度输出
float Control_AngleUpdate(float dt_s)
{
    float yaw_omega_rad_s;

    Control.feedback_yaw_rad = Kalman_GetYawRad();
    Control.error_yaw_rad = control_angle_norm(Control.target_yaw_rad
                                               - Control.feedback_yaw_rad);

    if ((Control.enabled == 0U) || (dt_s <= 0.0f)) {
        Control.omega_cmd_rad_s = 0.0f;
        return 0.0f;
    }

    Control.omega_cmd_rad_s = PID_UpdateError(&Control_AnglePID,
                                              Control.error_yaw_rad,
                                              dt_s);
    Control.omega_cmd_rad_s = control_abs_limit(Control.omega_cmd_rad_s,
                                                Control.max_omega_rad_s);

    yaw_omega_rad_s = Kalman_GetYawOmegaRad();
    Control.arrived = (uint8_t)((fabsf(Control.error_yaw_rad) <= Control.arrive_error_rad)
                             && (fabsf(yaw_omega_rad_s) <= Control.arrive_omega_rad_s));

    /* Stop correcting once both arrival conditions are met. */
    if (Control.arrived != 0U) {
        PID_Reset(&Control_AnglePID);
        Control.omega_cmd_rad_s = 0.0f;
    }

    return Control.omega_cmd_rad_s;
}

// Keep the angle target and send the result to chassis.
void Control_AngleHoldMove(float vx, float vy, float dt_s)
{
    float omega_cmd = Control_AngleUpdate(dt_s);
    Chassis_Move(vx, vy, omega_cmd);
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

// Debug demo: rotate by position control, then translate while holding heading.
void Control_test(void)
{
    uint32_t start_tick;

    Chassis_Setdefaultspeed(CONTROL_TEST_ROTATE_SPEED_MPS);
    control_run_position_move(0.0f, 0.0f, CONTROL_TEST_HEADING_DEG);
    control_service_chassis_for(CONTROL_TEST_SETTLE_TIME_MS);

    Control_Init();
    Control_SetAnglePid(2.0f, 0.0f, 0.05f);
    Control_SetAngleOutputLimit(0.8f);
    Control_SetAngleArriveThreshold(0.01f, 0.08f);

    Control_Enable(1U);
    Control_SetAngleDeg(CONTROL_TEST_HEADING_DEG);

    start_tick = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_tick) < CONTROL_TEST_MOVE_TIME_MS) {
        Control_AngleHoldMove(0.0f,
                              CONTROL_TEST_FORWARD_SPEED_MPS,
                              CONTROL_TEST_LOOP_DT_S);
        Chassis_Process();
        HAL_Delay(CONTROL_TEST_LOOP_DELAY_MS);
    }

    Control_Stop();
}
