/**
  ******************************************************************************
  * @file    color_reg.c
  * @brief   本地颜色识别模块实现（感为颜色传感器，软件I2C读取 RGB/HSL）
  *
  * 传感器数据格式（感为颜色传感器手册）:
  *   RGB: 命令 0xD0，读 3 字节 R/G/B，各 0~255
  *   HSL: 命令 0xD1，读 3 字节 H/S/L，各 0~240（H=色相, S=饱和度, L=亮度）
  *   色相角换算: hue_deg = H * 360 / 240 = H * 1.5
  *   传感器校准后: 白 = RGB(255,255,255), 黑 = RGB(0,0,0)
  *
  * 识别策略:
  *   1) 亮度 L 过低 -> 黑；L 高且 S 低 -> 白；
  *   2) 其余按色相角判红/绿/蓝（S 过低视为灰阶，交给 RGB 兜底）；
  *   3) HSL 判不出时用 RGB 主色通道兜底（黑/白/红/绿/蓝）。
  *   4) 连续 COLOR_REG_DEBOUNCE_N 次识别到同一颜色 -> 触发停车。
  ******************************************************************************
  */
#include "color_reg.h"

#include <string.h>

#include "software_iic.h"   /* Soft_IIC_Get_RGB / Soft_IIC_Get_HSL */
#include "Chassis.h"        /* Chassis_stop */
#include "usart_sent.h"     /* uart_printf 调试打印 */

/* ==================== 识别阈值（按现场标定可调） ==================== */
/* HSL 范围 0~240: 直接用 0~240 判断 S/L；H 换算成 0~360° 用 hue_deg 判断 */
#define COLOR_REG_L_BLACK_MAX    60U    /* L <= 60  判黑 */
#define COLOR_REG_L_WHITE_MIN    170U   /* L >= 170 且 S 低判白 */
#define COLOR_REG_S_WHITE_MAX    60U    /* 白色最大饱和度 */
#define COLOR_REG_S_COLOR_MIN    50U    /* 判定彩色所需最小饱和度 */

/* 色相角阈值（0~360°），感为HSL色相环: 红≈0° 绿≈120° 蓝≈240° */
#define COLOR_REG_H_RED_MAX      25.0f  /* 红: H<25° 或 H>=335° */
#define COLOR_REG_H_RED_MIN      335.0f
#define COLOR_REG_H_GREEN_MIN    70.0f
#define COLOR_REG_H_GREEN_MAX    160.0f
#define COLOR_REG_H_BLUE_MIN     160.0f
#define COLOR_REG_H_BLUE_MAX     265.0f

/* RGB 兜底阈值（校准后: 白=255,255,255 黑=0,0,0） */
#define COLOR_REG_RGB_DARK_MAX     60U   /* RGB 三通道都 <=60 判黑 */
#define COLOR_REG_RGB_BRIGHT_MIN  200U   /* RGB 三通道都 >=200 判白 */
#define COLOR_REG_RGB_DOM_DIFF     40U   /* 主色通道比其余通道大 40 判对应颜色 */

/* 采样/去抖参数 */
#define COLOR_REG_SAMPLE_MS        20U   /* 采样周期 (ms) */
#define COLOR_REG_DEBOUNCE_N       3U    /* 连续 N 次识别到同一颜色才触发停车 */

/* ==================== 模块状态 ==================== */
ColorReg_Data_t ColorReg_Data;
volatile uint8_t ColorReg_StopRequested = 0U;

static uint8_t color_reg_debounce = 0U;        /* 当前颜色连续命中次数 */
static uint8_t color_reg_last_color = COLOR_REG_COLOR_NONE;

void ColorReg_Init(void)
{
    ColorReg_StopRequested = 0U;
    color_reg_debounce = 0U;
    color_reg_last_color = COLOR_REG_COLOR_NONE;
    memset(&ColorReg_Data, 0, sizeof(ColorReg_Data));
    ColorReg_Data.color = COLOR_REG_COLOR_NONE;
    ColorReg_Data.last_tick = HAL_GetTick();
}

/* ---------------- 内部: 颜色分类 ---------------- */

/* HSL 分类（主判据）: 返回 COLOR_REG_COLOR_*，判不出返回 NONE */
static uint8_t color_reg_classify_hsl(uint8_t h, uint8_t s, uint8_t l)
{
    float hue_deg;

    /* 先按亮度分黑白 */
    if (l <= COLOR_REG_L_BLACK_MAX) {
        return COLOR_REG_COLOR_BLACK;
    }
    if ((l >= COLOR_REG_L_WHITE_MIN) && (s <= COLOR_REG_S_WHITE_MAX)) {
        return COLOR_REG_COLOR_WHITE;
    }

    /* 饱和度太低 -> 灰阶，无法判彩色，交给 RGB 兜底 */
    if (s < COLOR_REG_S_COLOR_MIN) {
        return COLOR_REG_COLOR_NONE;
    }

    hue_deg = (float)h * 360.0f / 240.0f;   /* 0~240 -> 0~360° */

    if ((hue_deg < COLOR_REG_H_RED_MAX) || (hue_deg >= COLOR_REG_H_RED_MIN)) {
        return COLOR_REG_COLOR_RED;
    }
    if ((hue_deg >= COLOR_REG_H_GREEN_MIN) && (hue_deg < COLOR_REG_H_GREEN_MAX)) {
        return COLOR_REG_COLOR_GREEN;
    }
    if ((hue_deg >= COLOR_REG_H_BLUE_MIN) && (hue_deg < COLOR_REG_H_BLUE_MAX)) {
        return COLOR_REG_COLOR_BLUE;
    }
    return COLOR_REG_COLOR_NONE;
}

/* RGB 分类（兜底）: 返回 COLOR_REG_COLOR_*，判不出返回 NONE */
static uint8_t color_reg_classify_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if ((r <= COLOR_REG_RGB_DARK_MAX) &&
        (g <= COLOR_REG_RGB_DARK_MAX) &&
        (b <= COLOR_REG_RGB_DARK_MAX)) {
        return COLOR_REG_COLOR_BLACK;
    }
    if ((r >= COLOR_REG_RGB_BRIGHT_MIN) &&
        (g >= COLOR_REG_RGB_BRIGHT_MIN) &&
        (b >= COLOR_REG_RGB_BRIGHT_MIN)) {
        return COLOR_REG_COLOR_WHITE;
    }
    if ((r > g + COLOR_REG_RGB_DOM_DIFF) && (r > b + COLOR_REG_RGB_DOM_DIFF)) {
        return COLOR_REG_COLOR_RED;
    }
    if ((g > r + COLOR_REG_RGB_DOM_DIFF) && (g > b + COLOR_REG_RGB_DOM_DIFF)) {
        return COLOR_REG_COLOR_GREEN;
    }
    if ((b > r + COLOR_REG_RGB_DOM_DIFF) && (b > g + COLOR_REG_RGB_DOM_DIFF)) {
        return COLOR_REG_COLOR_BLUE;
    }
    return COLOR_REG_COLOR_NONE;
}

/* ---------------- 对外接口 ---------------- */

uint8_t ColorReg_Detect(void)
{
    uint8_t rgb[COLOR_REG_RGB_LEN];
    uint8_t hsl[COLOR_REG_HSL_LEN];
    uint8_t ok_rgb;
    uint8_t ok_hsl;
    uint8_t color_hsl;
    uint8_t color_rgb;

    ok_rgb = Soft_IIC_Get_RGB(rgb, COLOR_REG_RGB_LEN);
    ok_hsl = Soft_IIC_Get_HSL(hsl, COLOR_REG_HSL_LEN);

    ColorReg_Data.updated = 1U;
    ColorReg_Data.last_tick = HAL_GetTick();

    if (ok_rgb != 0U) {
        ColorReg_Data.r = rgb[0];
        ColorReg_Data.g = rgb[1];
        ColorReg_Data.b = rgb[2];
    }
    if (ok_hsl != 0U) {
        ColorReg_Data.h = hsl[0];
        ColorReg_Data.s = hsl[1];
        ColorReg_Data.l = hsl[2];
        ColorReg_Data.hue_deg = (float)hsl[0] * 360.0f / 240.0f;
    }

    ColorReg_Data.valid = (uint8_t)((ok_rgb != 0U) || (ok_hsl != 0U));
    if (ColorReg_Data.valid == 0U) {
        /* 两次读取都失败: 标记未识别 */
        ColorReg_Data.color = COLOR_REG_COLOR_NONE;
        return ColorReg_Data.color;
    }

    /* HSL 优先，判不出再用 RGB 兜底 */
    color_hsl = (ok_hsl != 0U) ? color_reg_classify_hsl(hsl[0], hsl[1], hsl[2])
                               : COLOR_REG_COLOR_NONE;
    color_rgb = (ok_rgb != 0U) ? color_reg_classify_rgb(rgb[0], rgb[1], rgb[2])
                               : COLOR_REG_COLOR_NONE;

    if (color_hsl != COLOR_REG_COLOR_NONE) {
        ColorReg_Data.color = color_hsl;
    } else {
        ColorReg_Data.color = color_rgb;
    }
    return ColorReg_Data.color;
}

void ColorReg_Process(void)
{
    uint32_t now;

    if (ColorReg_StopRequested != 0U) {
        return;   /* 已触发停车，保持停车状态直到 ClearStop */
    }

    now = HAL_GetTick();
    if ((int32_t)(now - ColorReg_Data.last_tick) < (int32_t)COLOR_REG_SAMPLE_MS) {
        return;
    }

    (void)ColorReg_Detect();

    if (ColorReg_Data.color == COLOR_REG_COLOR_NONE) {
        color_reg_debounce = 0U;
        color_reg_last_color = COLOR_REG_COLOR_NONE;
        return;
    }

    /* 去抖: 连续多次识别到同一颜色才认为"颜色物块出现" */
    if (ColorReg_Data.color == color_reg_last_color) {
        if (color_reg_debounce < COLOR_REG_DEBOUNCE_N) {
            color_reg_debounce++;
        }
    } else {
        color_reg_debounce = 1U;
        color_reg_last_color = ColorReg_Data.color;
    }

    if (color_reg_debounce >= COLOR_REG_DEBOUNCE_N) {
        /* 识别到颜色物块 -> 置"软件中断"标志并立即停车 */
        ColorReg_StopRequested = 1U;
        Chassis_stop();
        uart_printf("COLOR_REG: color=%u (H%u S%u L%u R%u G%u B%u) STOP\r\n",
                    (unsigned int)ColorReg_Data.color,
                    (unsigned int)ColorReg_Data.h,
                    (unsigned int)ColorReg_Data.s,
                    (unsigned int)ColorReg_Data.l,
                    (unsigned int)ColorReg_Data.r,
                    (unsigned int)ColorReg_Data.g,
                    (unsigned int)ColorReg_Data.b);
    }
}

uint8_t ColorReg_GetColor(void)
{
    return ColorReg_Data.color;
}

void ColorReg_Print(void)
{
    uart_printf("COLOR_REG: valid=%u color=%u RGB=%u,%u,%u HSL=%u,%u,%u hue=%.1f\r\n",
                (unsigned int)ColorReg_Data.valid,
                (unsigned int)ColorReg_Data.color,
                (unsigned int)ColorReg_Data.r,
                (unsigned int)ColorReg_Data.g,
                (unsigned int)ColorReg_Data.b,
                (unsigned int)ColorReg_Data.h,
                (unsigned int)ColorReg_Data.s,
                (unsigned int)ColorReg_Data.l,
                (double)ColorReg_Data.hue_deg);
}

void ColorReg_ClearStop(void)
{
    ColorReg_StopRequested = 0U;
    color_reg_debounce = 0U;
    color_reg_last_color = COLOR_REG_COLOR_NONE;
}
