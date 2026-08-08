#include "pid.h"

#include <math.h>
#include <stddef.h>

static float pid_clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static void pid_order_limits(float *minimum, float *maximum)
{
    if (*minimum > *maximum) {
        float temporary = *minimum;
        *minimum = *maximum;
        *maximum = temporary;
    }
}

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    if (pid == NULL) {
        return;
    }

    pid_order_limits(&output_min, &output_max);

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = output_min;
    pid->integral_max = output_max;
    pid->deadband = 0.0f;
    pid->derivative_filter_alpha = 1.0f;

    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->previous_error = 0.0f;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->derivative_raw = 0.0f;
    pid->derivative_filtered = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0U;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimits(PID_t *pid, float output_min, float output_max)
{
    if (pid == NULL) {
        return;
    }

    pid_order_limits(&output_min, &output_max);
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->output = pid_clamp(pid->output, output_min, output_max);
}

void PID_SetIntegralLimits(PID_t *pid, float integral_min, float integral_max)
{
    if (pid == NULL) {
        return;
    }

    pid_order_limits(&integral_min, &integral_max);
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->integral = pid_clamp(pid->integral, integral_min, integral_max);
}

void PID_SetDeadband(PID_t *pid, float deadband)
{
    if (pid == NULL) {
        return;
    }

    pid->deadband = fabsf(deadband);
}

void PID_SetDerivativeFilter(PID_t *pid, float alpha)
{
    if (pid == NULL) {
        return;
    }

    pid->derivative_filter_alpha = pid_clamp(alpha, 0.0f, 1.0f);
}

static float pid_update_core(PID_t *pid, float error, float dt_s)
{
    float previous_integral;
    float unsaturated_output;
    uint8_t blocks_integration;

    if ((pid == NULL) || (dt_s <= 0.0f)) {
        return (pid != NULL) ? pid->output : 0.0f;
    }

    if (fabsf(error) <= pid->deadband) {
        error = 0.0f;
    }

    pid->error = error;
    pid->proportional = pid->kp * error;

    if (pid->initialized != 0U) {
        float alpha = pid_clamp(pid->derivative_filter_alpha, 0.0f, 1.0f);
        pid->derivative_raw = (error - pid->previous_error) / dt_s;
        pid->derivative_filtered += alpha
                                  * (pid->derivative_raw - pid->derivative_filtered);
    } else {
        pid->derivative_raw = 0.0f;
        pid->derivative_filtered = 0.0f;
        pid->initialized = 1U;
    }
    pid->derivative = pid->kd * pid->derivative_filtered;

    previous_integral = pid->integral;
    pid->integral += pid->ki * error * dt_s;
    pid->integral = pid_clamp(pid->integral,
                              pid->integral_min,
                              pid->integral_max);

    unsaturated_output = pid->proportional
                       + pid->integral
                       + pid->derivative;

    /* 输出饱和时停止同方向积分，但允许积分向反方向退饱和 */
    blocks_integration = (uint8_t)(((unsaturated_output > pid->output_max) && (error > 0.0f))
                                || ((unsaturated_output < pid->output_min) && (error < 0.0f)));
    if (blocks_integration != 0U) {
        pid->integral = previous_integral;
        unsaturated_output = pid->proportional
                           + pid->integral
                           + pid->derivative;
    }

    pid->output = pid_clamp(unsaturated_output,
                            pid->output_min,
                            pid->output_max);
    pid->previous_error = error;

    return pid->output;
}

float PID_Update(PID_t *pid, float setpoint, float feedback, float dt_s)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->setpoint = setpoint;
    pid->feedback = feedback;
    return pid_update_core(pid, setpoint - feedback, dt_s);
}

float PID_UpdateError(PID_t *pid, float error, uint16_t dt_ms)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->setpoint = error;
    pid->feedback = 0.0f;
    return pid_update_core(pid, error, (float)dt_ms / 1000.0f);
}
