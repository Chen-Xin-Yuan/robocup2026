#ifndef HARDWARE_GRAY_H
#define HARDWARE_GRAY_H

#include <stdint.h>

#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GRAY_SENSOR_NUM 8U
#define GRAY_PID_PERIOD_MS       40U
/*
 * 灰度循迹状态。PID 的参数及积分、微分历史统一由 PID_t 管理，
 * 此结构体只保留传感器数据和循迹层状态。
 */
typedef struct
{
    PID_t pid;
    uint8_t sensor_binary[GRAY_SENSOR_NUM]; /* bit0~bit7，0=黑线，1=白底 */
    uint8_t sensor_digital;
    uint8_t sensor_analog[GRAY_SENSOR_NUM];
    uint8_t sensor_normalized[GRAY_SENSOR_NUM];
    float line_position;                    /* 负值偏左，正值偏右 */
    float target_position;
    float pid_output;
    uint8_t track_lost;
    uint8_t lost_count;
    uint8_t initialized;
} Track_System;

extern Track_System track_sys;

/* 使用默认参数初始化循迹 PID。 */
void gray_read_binary(void);

void Gray_Init(void);

/* 中断安全：只提交一次灰度数据打印请求，不执行 I2C 操作。 */
void gray_show_digital(void);

/* 在主循环中反复调用，完成 I2C 采样、归一化等待和 USART2 DMA 输出。 */
void Gray_Process(void);

/*
 * 读取传感器并更新一次 PID，调用周期应保持为 10 ms。
 * 返回 0 表示正常循迹，返回 1 表示已经连续丢线。
 */
uint8_t Grey_PID_Update(void);

float Grey_Get_Output(void);

#ifdef __cplusplus
}
#endif

#endif

