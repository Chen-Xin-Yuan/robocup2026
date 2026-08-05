#ifndef SOFTWARE_PID_H
#define SOFTWARE_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* PID 参数，可直接在调试器的 Watch 窗口中修改 */
    float kp;
    float ki;
    float kd;

    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;

    /* 微分低通滤波系数：1.0f 表示不滤波，数值越小滤波越强 */
    float derivative_filter_alpha;

    /* PID 运行状态，可用于调试器观察或绘制响应曲线 */
    float setpoint;
    float feedback;
    float error;
    float previous_error;
    float proportional;
    float integral;
    float derivative;
    float derivative_raw;
    float derivative_filtered;
    float output;

    uint8_t initialized;
} PID_t;

/**
 * @brief 初始化位置式 PID 控制器
 * @note  初始化时，积分限幅默认与输出限幅相同
 */
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

/** 清除 PID 运行状态，保留参数和限幅设置 */
void PID_Reset(PID_t *pid);

/** 修改 Kp、Ki、Kd，不清除当前运行状态 */
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);

void PID_SetOutputLimits(PID_t *pid, float output_min, float output_max);
void PID_SetIntegralLimits(PID_t *pid, float integral_min, float integral_max);
void PID_SetDeadband(PID_t *pid, float deadband);

/**
 * @brief 设置微分项的一阶低通滤波系数
 * @param alpha 有效范围为 0.0f 到 1.0f，1.0f 表示不滤波
 */
void PID_SetDerivativeFilter(PID_t *pid, float alpha);

/**
 * @brief 执行一次 PID 计算，误差 = 目标值 - 反馈值
 * @param dt_s 两次计算之间的时间，单位为秒，必须大于 0
 */
float PID_Update(PID_t *pid, float setpoint, float feedback, float dt_s);

/**
 * @brief 使用调用者预先计算好的误差执行一次 PID 计算
 * @note  适用于角度环绕误差或其他非线性误差
 */
float PID_UpdateError(PID_t *pid, float error, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
