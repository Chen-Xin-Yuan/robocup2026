/*
 * Kalman.h
 *
 *  Created on: 2023年10月1日
 *      Author: Monst
 */
#ifndef __KALMAN_H_
#define __KALMAN_H_

#include <stdbool.h>  // 标准bool类型定义（C99标准）
#include "stm32f4xx_hal.h"


typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t;


typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;//四元数


typedef struct {
    float pitch;    //¸©Ñö½Ç
    float roll;     //Æ«º½½Ç
    float yaw;      //·­¹ö½Ç
    float last_yaw;
    int8_t Dirchange;
} euler_param_t;//欧拉角


typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_offset_param_t;//专门存放陀螺仪零偏


extern gyro_offset_param_t GyroOffset;

extern euler_param_t eulerAngle;
extern float angle_Z,angle_R,angle_P;

extern bool GyroOffset_init;
void icm_init(void);

float fast_sqrt(float x);

void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);

void ICM_getValues(void);

void ICM_getEulerianAngles(void);
void Kalman_ResetEulerZero(void);

/*================== IMU融合对外接口 ==================*/
float Kalman_GetYawRad(void);       // 获取IMU偏航角 (rad, 连续无跳变)
float Kalman_GetYawOmegaRad(void);  // 获取IMU Z轴角速度 (rad/s, 零偏已补偿)

#endif /* CODE_KALMAN_H_ */
