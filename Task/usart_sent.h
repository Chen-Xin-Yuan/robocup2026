#ifndef __USART_SENT_H__
#define __USART_SENT_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 调试打印（USART2 DMA 发送） */
void uart_printf(const char *format, ...);

/* K210 视觉通信（USART6, 115200，ASCII 以 '\n' 结尾） */
void K210_Send(const char *msg);
void K210_Poll(void);

/* K210 十字对准数据接口（协议: "CROSS:<x>,<y>,<yaw>\n"） */
uint8_t K210_GetCrossData(float *x_cm, float *y_cm, float *yaw_deg);
void K210_ClearCross(void);

/* USART2 上位机命令（固定帧格式，带长度字节，无校验）: AA <长度> <数据...> 0A
 * 上位机->车:
 *   对十字纠正帧 AA 0C <x:4B> <y:4B> <yaw:4B> 0A   (12字节数据)
 *   对圆心纠正帧 AA 08 <x:4B> <y:4B> 0A            (8字节数据, 只回x/y, 无yaw)
 *   颜色帧       AA 01 <颜色:1B> 0A                (1字节数据, 收满5个不同颜色即结束, 重复自动丢弃)
 *                颜色: 0=黑 1=白 2=红 3=绿 4=蓝, 或 ASCII 字母 B/W/R/G/U
 *                顺序 = 槽位1~5 从左到右
 *   二维码帧     AA 04 <编号:1B> 0A                (类型码04, 1字节数据)
 *                编号: 任务1=1~16, 任务2=1~6
 *   字母帧       AA 06 <字母:1B> 0A                (类型码06, 1字节数据)
 *                字母: 01=a 02=b 03=c, 或 ASCII a/A/b/B/c/C
 *                顺序 = 任务2 奖杯/槽位 1~3 依次回传, 共3个
 *   完毕帧       AA 00 0A                          (对十字/对圆心 结束)
 *   报错帧       AA 0F 0A                          (任意时刻收到立即停车)
 * 车->上位机:
 *   AA 01 0A 请求对十字纠正（每次阻塞式调整结束后再请求一次，直到收到完毕帧）
 *   AA 02 0A 请求对圆心纠正（同上，每次阻塞式调整结束后再请求一次）
 *   AA 03 0A 请求颜色识别（收到1个颜色后转舵机到下一槽位，舵机到位后再请求一次，
 *            共5次，每次回1个颜色）
 *   AA 05 0A 请求二维码
 *   AA 06 0A 请求字母识别（收到1个字母后转舵机到下一槽位，舵机到位后再请求一次，
 *            共3次，每次回1个字母 01/02/03 表示 a/b/c；请求/回传都用 AA 06，与二维码帧04区分）
 *   - x/y/yaw 为小端 float
 *   - x: 横向偏差(cm, 右为正)  y: 距离偏差(cm, 前为正)  yaw: 角度偏差(deg, 逆时针为正)
 * 接收处理位置: USART2_IRQHandler 里直接读 DR 后调用 Usart2_OnByte()。
 */
void Usart2_RxInit(void);
/* 由 USART2_IRQHandler 调用：喂一个接收字节进帧状态机 */
void Usart2_OnByte(uint8_t ch);
/* DMA 发送完成回调：请求帧队列发完一帧自动发下一帧（由 main.c 的 HAL_UART_TxCpltCallback 转发） */
void Usart2_TxCpltCallback(UART_HandleTypeDef *huart);
/* 非阻塞取一帧纠正量，返回 1=收到有效帧并写入 x/y/yaw；0=暂无 */
uint8_t Cross_GetCorrection(float *x_cm, float *y_cm, float *yaw_deg);

/* 调整状态标志: 1=等待/正在调整(默认), 0=调整完毕(收到完毕帧) */
extern uint8_t cross_flag;
/* 向上位机请求回传 x/y/yaw（发送 AA 01 0A），上位机收到后回传纠正帧 */
void Cross_SendRequest(void);
/* 调整完毕: 把 cross_flag 置 0，并回传"ALIGN_DONE\n"信号给上位机 */
void Cross_SendDone(void);

/* ==================== 颜色识别（USART2，颜色帧 AA 01 <色> 0A） ==================== */
uint8_t Color_GetCount(void);        /* 已收到的不同颜色个数 0~5 */
uint8_t Color_GetColor(uint8_t i);   /* 第 i 个颜色编号(0=黑 1=白 2=红 3=绿 4=蓝)，越界返回 0xFF */
uint8_t Color_IsDone(void);          /* 是否已收满5个不同颜色(识别结束) */
void Color_Reset(void);              /* 清空颜色缓存，开始新一轮识别 */
uint8_t Color_TakeNew(void);         /* 是否有新收到的颜色(1=有, 读取后清除)，主循环据此转舵机 */
void Color_StartServoRotate(void);   /* 主循环发出舵机旋转命令后调用：开始等待舵机到位 */
void Color_Process(void);            /* 主循环：舵机到位(固定延时)后请求下一个槽位颜色(AA 03 0A) */
void Color_SendRequest(void);        /* 请求上位机回传 1 个颜色(AA 03 0A) */

/* ==================== 对圆心（USART2，纠正帧 AA 08 <x:4B> <y:4B> 0A，无 yaw） ==================== */
uint8_t Circle_GetCorrection(float *x_cm, float *y_cm); /* 1=收到新帧并写入 x/y；0=暂无 */
void Circle_SendRequest(void);       /* 请求上位机回传圆心 x/y(AA 02 0A) */
void Circle_SendDone(void);          /* 对圆心调整完毕: circle_flag=0，回传 CIRCLE_DONE */
extern uint8_t circle_flag;          /* 1=等待/正在对圆心, 0=完毕/未开始 */

/* ==================== 二维码（USART2，二维码帧 AA 04 <编号> 0A） ==================== */
uint8_t QR_IsReady(void);            /* 是否有新的二维码编号就绪 */
uint8_t QR_GetNumber(void);          /* 取二维码编号(任务1=1~16, 任务2=1~6，范围由调用方校验) */
void QR_Reset(void);                 /* 清空二维码缓存，开始新一轮等待 */
void QR_SendRequest(void);           /* 请求上位机回传二维码(AA 05 0A) */

#ifdef __cplusplus
}
#endif

#endif
