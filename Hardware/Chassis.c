#include "Chassis.h"
#include "main.h"
#include "motor_def.h"
#include "ZDTstepmotor.h"
#include "Emm_V5.h"
#include "usart.h"
#include "gray.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <float.h>

/*================== 全局变量 ==================*/

StepMotorZDT_t Motor1, Motor2, Motor3, Motor4;

#define CHASSIS_TX_FRAMES_PER_UPDATE    5U

volatile uint32_t Chassis_TxRejectedBatches = 0U;


static float rotate_radius = 0.0f;
static float wheel_default_speed = 0.2f;

/**
 * @brief 前置检查: 确认四个麦轮电机的 USART1 句柄一致，并且发送队列有足够的可用槽位。
 *
 * One update submits four motor frames plus one broadcast sync frame. The
 * current firmware has a single foreground producer, while the UART ISR only
 * removes frames, so this preflight prevents a group from being partially
 * queued. A rejected group leaves all four previous motor targets untouched.
 */
static bool chassis_tx_batch_available(void)
{
    if ((Motor1._USART != &huart1) || (Motor2._USART != &huart1) ||
        (Motor3._USART != &huart1) || (Motor4._USART != &huart1) ||
        (Emm_V5_TxGetFreeSlots() < CHASSIS_TX_FRAMES_PER_UPDATE)) {
        Chassis_TxRejectedBatches++;
        return false;
    }

    return true;
}

/*================== 麦轮运动学核心 ==================*/

/*
 *  【轮序与坐标系定义】
 *
 *        车头方向 (+y)
 *             ↑
 *             |
 *    1(左前)  |  2(右前)
 *            底盘中心
 *    3(左后)  |  4(右后)
 *             |
 *    (-x) ←——+——→ (+x)
 *
 *  麦轮类型: X型布局 (4个轮子辊子方向呈X形)
 *  辊子角度: 45° (与轮子轴线夹角)
 *
 *  坐标系:
 *    vx    : 底盘横向速度, 右侧为正 (+x方向)
 *    vy    : 底盘纵向速度, 前方为正 (+y方向)
 *    omega : 旋转角速度, 逆时针为正 (标准数学定义)
 */

/**
 * @brief 麦轮逆解算: 底盘速度 → 四轮速度
 * @param vx 底盘横向速度 (m/s), 右侧为正
 * @param vy    底盘纵向速度 (m/s), 前方为正
 * @param omega 旋转角速度 (rad/s), 逆时针为正
 * @param v1~v4 输出四轮线速度 (m/s)
 */
static void chassis_inverse_kinematics(float vx, float vy, float omega,
                                       float *v1, float *v2, float *v3, float *v4)
{
    float v_rot = omega * rotate_radius;

    *v1 = vy - vx - v_rot;
    *v2 = vy + vx + v_rot;
    *v3 = vy + vx - v_rot;
    *v4 = vy - vx + v_rot;
}

/**
 * @brief 麦轮正解算: 四轮速度 → 底盘速度
 * @param v1~v4 四轮线速度 (m/s)
 * @param vx    输出底盘横向速度 (m/s), 右侧为正
 * @param vy    输出底盘纵向速度 (m/s), 前方为正
 * @param omega 输出旋转角速度 (rad/s), 逆时针为正
 */
static void chassis_forward_kinematics(float v1, float v2, float v3, float v4,
                                       float *vx, float *vy, float *omega)
{
    const float inv = 0.25f;

    *vy    = ( v1 + v2 + v3 + v4) * inv;
    *vx    = (-v1 + v2 + v3 - v4) * inv;
    *omega = -(v1 - v2 + v3 - v4) * inv / rotate_radius;
}

/**
 * @brief 求 n 个 float 中的最大值
 */
static float chassis_max_float(int n, ...)
{
    va_list args;
    va_start(args, n);
    float max_val = -FLT_MAX;
    for (int i = 0; i < n; i++) {
        float current = (float)va_arg(args, double);
        if (current > max_val) max_val = current;
    }
    va_end(args);
    return max_val;
}

/*================== 初始化 ==================*/

void Chassis_Init(UART_HandleTypeDef *_USART)
{
    rotate_radius = (CHASSIS_WIDTH + CHASSIS_LENGTH) / 2.0f;

    Step_ZDT_Init(&Motor1, 1, _USART, 0, CHASSIS_WHEEL_DIA, false);
    Step_ZDT_Init(&Motor2, 2, _USART, 1, CHASSIS_WHEEL_DIA, false);
    Step_ZDT_Init(&Motor3, 3, _USART, 0, CHASSIS_WHEEL_DIA, false);
    Step_ZDT_Init(&Motor4, 4, _USART, 1, CHASSIS_WHEEL_DIA, true);

    Chassis_Setdefaultspeed(0.2f);
    Chassis_stop();
}


void Chassis_Process(void)
{
    Emm_V5_Process();
}

void Chassis_Setdefaultspeed(float v)
{
    wheel_default_speed = v;
}

/*================== 速度控制 (统一接口) ==================*/

void Chassis_Move(float vx, float vy, float omega)
{
    float v1, v2, v3, v4;

    if (!chassis_tx_batch_available()) return;

    chassis_inverse_kinematics(vx, vy, omega, &v1, &v2, &v3, &v4);

    set_speed_target(&Motor1, v1);
    set_speed_target(&Motor2, v2);
    set_speed_target(&Motor3, v3);
    set_speed_target(&Motor4, v4);
}

void Chassis_TrackDifferential(float forward_speed, float pid_output)
{
    float correction_speed;
    float left_speed;
    float right_speed;

    /* 灰度 PID 输出按 RPM 解释，再转换为轮缘线速度。 */
    correction_speed = pid_output
                     * (CHASSIS_PI * CHASSIS_WHEEL_DIA / 60.0f);

    /* 正修正量让左轮更快、右轮更慢，使底盘向右转。 */
    left_speed = forward_speed + correction_speed;
    right_speed = forward_speed - correction_speed;

    Chassis_Set4MotorSpeed(left_speed,
                           right_speed,
                           left_speed,
                           right_speed);
}

void Chassis_stop(void)
{
    /* A stop supersedes stale motion updates waiting behind the active frame. */
    Emm_V5_TxDiscardPending();
    if (!chassis_tx_batch_available()) return;

    Motor1.motor_controller_t.tar_velocity = 0.0f;
    Motor2.motor_controller_t.tar_velocity = 0.0f;
    Motor3.motor_controller_t.tar_velocity = 0.0f;
    Motor4.motor_controller_t.tar_velocity = 0.0f;
    Motor1.motor_controller_t.tar_rpm = 0;
    Motor2.motor_controller_t.tar_rpm = 0;
    Motor3.motor_controller_t.tar_rpm = 0;
    Motor4.motor_controller_t.tar_rpm = 0;

    Emm_V5_Stop_Now(Motor1.motor_controller_t.id, true);
    Emm_V5_Stop_Now(Motor2.motor_controller_t.id, true);
    Emm_V5_Stop_Now(Motor3.motor_controller_t.id, true);
    Emm_V5_Stop_Now(Motor4.motor_controller_t.id, true);
    Emm_V5_Synchronous_motion(0U);
}

void Chassis_Set4MotorSpeed(float v1, float v2, float v3, float v4)
{
    if (!chassis_tx_batch_available()) return;

    set_speed_target(&Motor1, v1);
    set_speed_target(&Motor2, v2);
    set_speed_target(&Motor3, v3);
    set_speed_target(&Motor4, v4);
}

/*================== 位置控制 (统一接口) ==================*/

/**
 * @brief 输入目标底盘位移 (sx, sy) + 旋转角度 (theta_deg)，输出预计执行时间 (ms)
 */
float Chassis_MovePos(float sx, float sy, float theta_deg)
{
    float theta_rad = theta_deg * CHASSIS_PI / 180.0f;
    float s_rot = theta_rad * rotate_radius;

    float s1 = sy - sx - s_rot;
    float s2 = sy + sx + s_rot;
    float s3 = sy + sx - s_rot;
    float s4 = sy - sx + s_rot;

    float t1 = fabsf(s1) / wheel_default_speed;
    float t2 = fabsf(s2) / wheel_default_speed;
    float t3 = fabsf(s3) / wheel_default_speed;
    float t4 = fabsf(s4) / wheel_default_speed;
    float t  = chassis_max_float(4, t1, t2, t3, t4);

    if (t < 0.001f) return 0.0f;
    if (!chassis_tx_batch_available()) return -1.0f;

    set_speed_pos_target(&Motor1, s1 / t, s1);
    set_speed_pos_target(&Motor2, s2 / t, s2);
    set_speed_pos_target(&Motor3, s3 / t, s3);
    set_speed_pos_target(&Motor4, s4 / t, s4);//速度位置控制

    return ((t + 0.1) * 1000.0f);
}

/*================== 状态查询 ==================*/

float Chassis_GetLinearSpeed(uint8_t motor_ID)
{
    switch (motor_ID) {
        case 1: return get_linear_speed(&Motor1);
        case 2: return get_linear_speed(&Motor2);
        case 3: return get_linear_speed(&Motor3);
        case 4: return get_linear_speed(&Motor4);
        default: return 0.0f;
    }
}

void Chassis_GetSpeed(void)
{
}

/**
 * @brief 实时正解算获取底盘坐标系速度
 * @note  直接读取当前四轮速度, 通过正解算得到底盘瞬时速度。
 *        不依赖任何全局状态, 可供惯导模块实时调用。
 */
void Chassis_GetBodySpeed(float *vx, float *vy, float *omega)
{
    float v1 = get_linear_speed(&Motor1);
    float v2 = get_linear_speed(&Motor2);
    float v3 = get_linear_speed(&Motor3);
    float v4 = get_linear_speed(&Motor4);

    float vx_body, vy_body, omega_body;
    chassis_forward_kinematics(v1, v2, v3, v4, &vx_body, &vy_body, &omega_body);

    if (vx)    *vx = vx_body;
    if (vy)    *vy = vy_body;
    if (omega) *omega = omega_body;
}

/*================== 测试 / 任务函数 ==================*/

void Chassis_Test(void)
{
#if 1
    float duration_ms;
    uint32_t wait_ms;
    uint32_t start_tick;

    Chassis_Setdefaultspeed(0.2f);
    do {
        duration_ms = Chassis_MovePos(0.0f, 0.5f, 0.0f);
        if (duration_ms < 0.0f) {
            Chassis_Process();
            HAL_Delay(1U);
        }
    } while (duration_ms < 0.0f);
    if (duration_ms > 0.0f) {
        wait_ms = (uint32_t)(duration_ms + 0.5f);
        start_tick = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start_tick) < wait_ms) {
            Chassis_Process();
            HAL_Delay(1U);
        }
    }

    do {
        duration_ms = Chassis_MovePos(0.1f, 0.0f, 0.0f);
        if (duration_ms < 0.0f) {
            Chassis_Process();
            HAL_Delay(1U);
        }
    } while (duration_ms < 0.0f);
    if (duration_ms > 0.0f) {
        wait_ms = (uint32_t)(duration_ms + 0.5f);
        start_tick = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start_tick) < wait_ms) {
            Chassis_Process();
            HAL_Delay(1U);
        }
    }

    do {
        duration_ms = Chassis_MovePos(0.0f, 0.0f, 90.0f);
        if (duration_ms < 0.0f) {
            Chassis_Process();
            HAL_Delay(1U);
        }
    } while (duration_ms < 0.0f);
    if (duration_ms > 0.0f) {
        wait_ms = (uint32_t)(duration_ms + 0.5f);
        start_tick = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start_tick) < wait_ms) {
            Chassis_Process();
            HAL_Delay(1U);
        }
    }

    // HAL_Delay(Chassis_MovePos(0.0f, 0.5f, 0.0f));
    // HAL_Delay(Chassis_MovePos(0.1f, 0.0f, 0.0f));
    // HAL_Delay(Chassis_MovePos(0.0f, 0.0f, 90.0f));
#endif
}

/*================== 状态机辅助函数 ==================*/

/**
 * @brief 开环运动一段时间后停止
 * @note  数值按场地标定
 */
void Chassis_OpenLoopMove(float vx, float vy, float omega, uint32_t duration_ms)
{
    Chassis_Move(vx, vy, omega);
    HAL_Delay(duration_ms);
    Chassis_stop();
}

/**
 * @brief 定位置移动（阻塞式）
 * @retval 实际执行时间(ms)；失败返回 0
 */
uint32_t Chassis_MovePosBlocking(float sx, float sy, float theta_deg)
{
    float duration_ms;

    do {
        duration_ms = Chassis_MovePos(sx, sy, theta_deg);
        if (duration_ms < 0.0f) {
            // Chassis_Process();
            HAL_Delay(1U);
        }
    } while (duration_ms < 0.0f);

    if (duration_ms > 0.0f) {
        HAL_Delay((uint32_t)(duration_ms + 0.5f));
        return (uint32_t)(duration_ms + 0.5f);
    }
    return 0U;
}

/**
 * @brief 向左横移，直到灰度中间两个传感器(3,4)同时检测到黑线
 * @param speed      横移速度 (m/s)
 * @param timeout_ms 超时(ms)
 */
void Chassis_StrafeLeftUntilLine(float speed, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)timeout_ms)
     {
        (void)gray_read_binary();   /* 刷新灰度 sensor_binary */
        if ((track_sys.sensor_binary[3] == 0U) ||
            (track_sys.sensor_binary[4] == 0U)) {
            break;   /* 中间两个都压到黑线 */
        }
        Chassis_Move(speed, 0.0f, 0.0f);   /* 向左 = -x */
        HAL_Delay(5U);
    }
    Chassis_stop();
}

/**
 * @brief 向右横移，直到灰度中间两个传感器(3,4)同时检测到黑线
 * @param speed      横移速度 (m/s, 正=向右)
 * @param timeout_ms 超时(ms)
 */
void Chassis_StrafeRightUntilLine(float speed, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)timeout_ms)
     {
        (void)gray_read_binary();   /* 刷新灰度 sensor_binary */
        if ((track_sys.sensor_binary[3] == 0U) ||
            (track_sys.sensor_binary[4] == 0U)) {
            break;   /* 中间两个都压到黑线 */
        }
        Chassis_Move(speed, 0.0f, 0.0f);   /* 向右 = +x */
        HAL_Delay(5U);
    }
    Chassis_stop();
}
