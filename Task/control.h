#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_PI                 3.14159265f
#define CONTROL_DEG_TO_RAD         (CONTROL_PI / 180.0f)
#define CONTROL_RAD_TO_DEG         (180.0f / CONTROL_PI)

#define CONTROL_DEFAULT_KP         3.0f
#define CONTROL_DEFAULT_KI         0.0f
#define CONTROL_DEFAULT_KD         0.08f
#define CONTROL_DEFAULT_MAX_OMEGA  2.0f
#define CONTROL_DEFAULT_DEADBAND   0.01f
#define CONTROL_DEFAULT_ARRIVE_ERR 0.02f
#define CONTROL_DEFAULT_ARRIVE_W   0.05f

// Angle-loop runtime state.
typedef struct
{
    float target_yaw_rad;
    float feedback_yaw_rad;
    float error_yaw_rad;
    float omega_cmd_rad_s;

    float max_omega_rad_s;
    float arrive_error_rad;
    float arrive_omega_rad_s;

    uint8_t enabled;
    uint8_t arrived;
} Control_t;

extern Control_t Control;
extern PID_t Control_AnglePID;

// Initialize the angle loop and default PID values.
void Control_Init(void);
// Reset control state but keep PID gains.
void Control_Reset(void);
// Enable or disable the angle loop.
void Control_Enable(uint8_t enable);

// Set an absolute target yaw in radians.
void Control_SetAngleRad(float target_yaw_rad);
// Set an absolute target yaw in degrees.
void Control_SetAngleDeg(float target_yaw_deg);
// Set a relative turn in radians from current yaw.
void Control_SetAngleRelativeRad(float delta_yaw_rad);
// Set a relative turn in degrees from current yaw.
void Control_SetAngleRelativeDeg(float delta_yaw_deg);

// Update PID gains for the angle loop.
void Control_SetAnglePid(float kp, float ki, float kd);
// Set the maximum yaw rate output.
void Control_SetAngleOutputLimit(float max_omega_rad_s);
// Set the arrival thresholds（阈值）.
void Control_SetAngleArriveThreshold(float error_rad, float omega_rad_s);

// Run one angle-loop step. dt_s is in seconds.
float Control_AngleUpdate(float dt_s);
// Keep the angle target and send the result to chassis.
void Control_AngleHoldMove(float vx, float vy, float dt_s);
// Return nonzero when the target is reached.
uint8_t Control_AngleIsArrived(void);
// Stop the angle loop and the chassis.
void Control_Stop(void);
// Run a position-turn plus heading-hold translation test.
void Control_test(void);

#ifdef __cplusplus
}
#endif

#endif
