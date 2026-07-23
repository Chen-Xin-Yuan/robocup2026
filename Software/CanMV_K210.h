/*
 * CanMV_K210.h
 *
 * CanMV K210 视觉模块通信协议
 * 通过 UART4 (PA0-TX, PA1-RX) @ 115200 接收视觉数据
 *
 * 固定长度帧格式:
 *   [0xAA] [帧类型] [数据...] [累加和] [0x6B]
 *
 * 帧类型:
 *   0x01 - 颜色识别  (13B)
 *   0x02 - 辅助巡线  (9B)
 *   0x03 - 二维码    (15B)
 *   0x04 - 圆心对齐  (10B)
 *   0x05 - 字母识别  (9B)
 */

#ifndef __CANMV_K210_H__
#define __CANMV_K210_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*================== 通信参数 ==================*/

#define CANMV_UART_BAUDRATE    115200
#define CANMV_FRAME_HEAD       0xAA    // 帧头
#define CANMV_FRAME_TAIL       0x6B    // 帧尾（与Emm_V5协议一致）
#define CANMV_RX_BUF_LEN       32      // 接收缓冲区长度

/*================== 帧类型枚举 ==================*/

typedef enum {
    CANMV_FRAME_NONE    = 0x00,    // 无数据
    CANMV_FRAME_COLOR   = 0x01,    // 颜色识别
    CANMV_FRAME_LINE    = 0x02,    // 辅助巡线
    CANMV_FRAME_QRCODE  = 0x03,    // 二维码识别
    CANMV_FRAME_CIRCLE  = 0x04,    // 圆心对齐
    CANMV_FRAME_LETTER  = 0x05,    // 字母识别
} CanMV_FrameType_t;

/*================== 数据结构体 ==================*/

/**
 * @brief 颜色识别数据
 */
typedef struct {
    uint8_t  color_id;      // 颜色ID (0=红, 1=绿, 2=蓝, 3=黄, ...)
    uint16_t center_x;      // 中心 X 坐标 (像素)
    uint16_t center_y;      // 中心 Y 坐标 (像素)
    uint16_t width;         // 宽度 (像素)
    uint16_t height;        // 高度 (像素)
} CanMV_Color_t;

/**
 * @brief 辅助巡线数据
 */
typedef struct {
    int16_t  offset;        // 偏移量 (像素, 正=偏右, 负=偏左)
    int16_t  angle;         // 线角度 (度)
    uint8_t  line_status;   // 线状态 (0=无线, 1=有线)
} CanMV_Line_t;

/**
 * @brief 二维码识别数据
 */
typedef struct {
    uint16_t tag_id;        // 标签 ID
    uint16_t center_x;      // 中心 X 坐标 (像素)
    uint16_t center_y;      // 中心 Y 坐标 (像素)
    int16_t  angle;         // 旋转角度 (度)
    uint16_t size;          // 尺寸 (像素)
} CanMV_QRCode_t;

/**
 * @brief 圆心对齐数据
 */
typedef struct {
    uint16_t center_x;      // 圆心 X 坐标 (像素)
    uint16_t center_y;      // 圆心 Y 坐标 (像素)
    uint16_t radius;        // 半径 (像素)
} CanMV_Circle_t;

/**
 * @brief 字母识别数据
 */
typedef struct {
    uint8_t  ascii;         // 字母 ASCII 码
    uint16_t center_x;      // 中心 X 坐标 (像素)
    uint16_t center_y;      // 中心 Y 坐标 (像素)
} CanMV_Letter_t;

/*================== 接收缓冲区（外部声明） ==================*/

extern __IO uint8_t canmv_rx_buf[CANMV_RX_BUF_LEN];
extern __IO uint8_t canmv_rx_count;
extern __IO bool    canmv_rx_flag;

/*================== API 函数声明 ==================*/

/**
 * @brief 初始化 UART4 (PA0/PA1) + DMA1_Stream2 + IDLE 中断
 * @note  必须在使用任何 CanMV API 之前调用一次
 */
void CanMV_Init(void);

/**
 * @brief 获取最新帧类型
 * @retval 帧类型枚举值，若无有效帧则返回 CANMV_FRAME_NONE
 */
CanMV_FrameType_t CanMV_GetFrameType(void);

/**
 * @brief 获取颜色识别数据
 * @param color 输出参数，颜色数据结构体指针
 * @retval 0=成功, 1=无颜色数据
 */
uint8_t CanMV_GetColor(CanMV_Color_t *color);

/**
 * @brief 获取辅助巡线数据
 * @param line 输出参数，巡线数据结构体指针
 * @retval 0=成功, 1=无巡线数据
 */
uint8_t CanMV_GetLine(CanMV_Line_t *line);

/**
 * @brief 获取二维码识别数据
 * @param qr 输出参数，二维码数据结构体指针
 * @retval 0=成功, 1=无二维码数据
 */
uint8_t CanMV_GetQRCode(CanMV_QRCode_t *qr);

/**
 * @brief 获取圆心对齐数据
 * @param circle 输出参数，圆心数据结构体指针
 * @retval 0=成功, 1=无圆心数据
 */
uint8_t CanMV_GetCircle(CanMV_Circle_t *circle);

/**
 * @brief 获取字母识别数据
 * @param letter 输出参数，字母数据结构体指针
 * @retval 0=成功, 1=无字母数据
 */
uint8_t CanMV_GetLetter(CanMV_Letter_t *letter);

/**
 * @brief 清空当前帧数据
 * @note  处理完一帧后调用，准备接收下一帧
 */
void CanMV_ClearData(void);

#endif /* __CANMV_K210_H__ */