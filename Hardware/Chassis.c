#include "Chassis.h"
#include "main.h"
#include "motor_def.h"
#include "ZDTstepmotor.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <float.h>

/*================== 全局变量 ==================*/

StepMotorZDT_t Motor1, Motor2, Motor3, Motor4;
Car_Param Car;

static float wheel_perimeter = 0.0f;
static float rotate_radius = 0.0f;
static float wheel_default_speed = 0.2f;

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
    float v_rot = -omega * rotate_radius;
    float k = CHASSIS_SQRT2;

    *v1 = (vy - vx + v_rot) * k;
    *v2 = (vy + vx - v_rot) * k;
    *v3 = (vy + vx + v_rot) * k;
    *v4 = (vy - vx - v_rot) * k;
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
    const float inv = 1.0f / (4.0f * CHASSIS_SQRT2);

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
    wheel_perimeter = CHASSIS_PI * CHASSIS_WHEEL_DIA;
    rotate_radius = sqrtf((CHASSIS_WIDTH / 2.0f) * (CHASSIS_WIDTH / 2.0f)
                        + (CHASSIS_LENGTH / 2.0f) * (CHASSIS_LENGTH / 2.0f));

    Step_ZDT_Init(&Motor1, 1, _USART, 0, CHASSIS_WHEEL_DIA, false);
    Step_ZDT_Init(&Motor2, 2, _USART, 1, CHASSIS_WHEEL_DIA, false);
    Step_ZDT_Init(&Motor3, 3, _USART, 0, CHASSIS_WHEEL_DIA, false);
    Step_ZDT_Init(&Motor4, 4, _USART, 1, CHASSIS_WHEEL_DIA, true);

    Chassis_Setdefaultspeed(1.0f);
    Chassis_stop();
}

void Chassis_Setdefaultspeed(float v)
{
    wheel_default_speed = v;
}

/*================== 速度控制 (统一接口) ==================*/

void Chassis_Move(float vx, float vy, float omega)
{
    float v1, v2, v3, v4;
    chassis_inverse_kinematics(vx, vy, omega, &v1, &v2, &v3, &v4);

    set_speed_target(&Motor1, v1);
    set_speed_target(&Motor2, v2);
    set_speed_target(&Motor3, v3);
    set_speed_target(&Motor4, v4);
}

void Chassis_stop(void)
{
    set_speed_target(&Motor1, 0.0f);
    set_speed_target(&Motor2, 0.0f);
    set_speed_target(&Motor3, 0.0f);
    set_speed_target(&Motor4, 0.0f);
}

void Chassis_Set4MotorSpeed(float v1, float v2, float v3, float v4)
{
    set_speed_target(&Motor1, v1);
    set_speed_target(&Motor2, v2);
    set_speed_target(&Motor3, v3);
    set_speed_target(&Motor4, v4);
}

/*================== 位置控制 (统一接口) ==================*/

float Chassis_MovePos(float sx, float sy, float theta_deg)
{
    float theta_rad = theta_deg * CHASSIS_PI / 180.0f;
    float s_rot = theta_rad * rotate_radius;

    float k = CHASSIS_SQRT2;
    float s1 = (sy - sx + s_rot) * k;
    float s2 = (sy + sx - s_rot) * k;
    float s3 = (sy + sx + s_rot) * k;
    float s4 = (sy - sx - s_rot) * k;

    float t1 = fabsf(s1) / wheel_default_speed;
    float t2 = fabsf(s2) / wheel_default_speed;
    float t3 = fabsf(s3) / wheel_default_speed;
    float t4 = fabsf(s4) / wheel_default_speed;
    float t  = chassis_max_float(4, t1, t2, t3, t4);

    if (t < 0.001f) return 0.0f;

    set_speed_pos_target(&Motor1, s1 / t, s1);
    set_speed_pos_target(&Motor2, s2 / t, s2);
    set_speed_pos_target(&Motor3, s3 / t, s3);
    set_speed_pos_target(&Motor4, s4 / t, s4);

    return t;
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
    Car.M1_V = get_linear_speed(&Motor1);
    Car.M2_V = get_linear_speed(&Motor2);
    Car.M3_V = get_linear_speed(&Motor3);
    Car.M4_V = get_linear_speed(&Motor4);
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
    Chassis_Init(&huart1);
    Chassis_Setdefaultspeed(0.2f);

    float t = Chassis_MovePos(0.0f, 1.0f, 0.0f);
    HAL_Delay((uint32_t)(t * 1000.0f + 500.0f));
}

void Task(uint16_t RBG)
{
    (void)RBG;
}
