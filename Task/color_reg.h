/**
  ******************************************************************************
  * @file    color_reg.h
  * @brief   本地颜色识别模块（感为颜色传感器，软件I2C读取 RGB/HSL）
  *
  * 功能:
  *   1) 通过软件I2C(Soft_IIC_Get_RGB / Soft_IIC_Get_HSL)读取颜色传感器
  *      的 RGB(0~255) 与 HSL(0~240) 原始值；
  *   2) 依据 HSL 色相/饱和度/亮度(辅以 RGB 兜底)识别物块颜色，
  *      颜色编号与比赛方案一致: 0=黑 1=白 2=红 3=绿 4=蓝；
  *   3) 连续多次识别到同一颜色物块后，置位 ColorReg_StopRequested
  *      并调用 Chassis_stop() 停车 —— 该标志位即"软件中断"信号，
  *      底盘控制线程(主循环或 TIM2 中断)读取后立即停车。
  *
  * 接线说明(软件I2C):
  *   SCL = PB4, SDA = PB5（见 software_iic.h，按实际修改）
  *
  * 使用说明(不修改 main.c 的前提下):
  *   1. 初始化时调用一次 ColorReg_Init();
  *   2. 主循环周期调用 ColorReg_Process()（建议 10~20ms 一次）；
  *   3. 需要与 TIM2 底盘控制中断联动时，在 HAL_TIM_PeriodElapsedCallback
  *      的 htim2 分支开头加:
  *         if (ColorReg_StopRequested) { Chassis_stop(); ... }
  *      防止中断里继续下发循迹指令覆盖停车。
  ******************************************************************************
  */
#ifndef __COLOR_REG_H__
#define __COLOR_REG_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 颜色编号（与任务方案一致） ==================== */
#define COLOR_REG_COLOR_BLACK  0U    /* 黑 */
#define COLOR_REG_COLOR_WHITE  1U    /* 白 */
#define COLOR_REG_COLOR_RED    2U    /* 红 */
#define COLOR_REG_COLOR_GREEN  3U    /* 绿 */
#define COLOR_REG_COLOR_BLUE   4U    /* 蓝 */
#define COLOR_REG_COLOR_NONE   0xFFU /* 未识别/无物块 */

/* 传感器每通道字节数（感为颜色传感器: RGB 3字节, HSL 3字节） */
#define COLOR_REG_RGB_LEN  3U
#define COLOR_REG_HSL_LEN  3U

/* ==================== 识别结果结构体 ==================== */
typedef struct
{
    uint8_t  r, g, b;       /* RGB 原始值 0~255 */
    uint8_t  h, s, l;       /* HSL 原始值 0~240（H=色相, S=饱和度, L=亮度） */
    float    hue_deg;       /* 由 H 换算的色相角 0~360°（hue = H * 1.5） */
    uint8_t  color;         /* 最近一次识别结果 COLOR_REG_COLOR_* */
    uint8_t  valid;         /* 最近一次采样是否成功(读回任一通道成功即1) */
    uint8_t  updated;       /* 本轮 Process/Detect 是否刷新了数据 */
    uint32_t last_tick;     /* 最近一次采样时间戳 (ms) */
} ColorReg_Data_t;

/* 全局识别结果（主循环/串口打印可直接读取） */
extern ColorReg_Data_t ColorReg_Data;

/* 停车请求标志（volatile，供 TIM2 中断 / 主循环读取）:
 * 识别到颜色物块并去抖确认后置 1，直至调用 ColorReg_ClearStop() 清除。 */
extern volatile uint8_t ColorReg_StopRequested;

/* 初始化: 清空状态与停车标志 */
void ColorReg_Init(void);

/* 周期调用（建议 10~20ms 一次）:
 * 采样 RGB/HSL -> 识别颜色 -> 连续多次命中同一颜色后触发停车 */
void ColorReg_Process(void);

/* 立即采样一次并识别，返回颜色编号（同时刷新 ColorReg_Data） */
uint8_t ColorReg_Detect(void);

/* 查询最近一次识别结果 */
uint8_t ColorReg_GetColor(void);

/* 串口打印最近一次 RGB/HSL 与识别结果（调试标定用） */
void ColorReg_Print(void);

/* 清除停车请求标志（复位后重新开始检测） */
void ColorReg_ClearStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __COLOR_REG_H__ */
