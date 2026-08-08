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
int K210_WaitNumber(uint32_t timeout_ms);
uint8_t K210_WaitCrossOK(uint32_t timeout_ms);

/* K210 十字对准数据接口（协议: "CROSS:<x>,<y>,<yaw>\n"） */
uint8_t K210_GetCrossData(float *x_cm, float *y_cm, float *yaw_deg);
uint8_t K210_CrossValid(void);
void K210_ClearCross(void);

/* USART2 上位机命令（固定帧格式，带长度字节，无校验）
 * 车->上位机: 请求帧 AA 01 0A  -> 上位机收到后回传纠正帧
 * 上位机->车: 纠正帧 AA 0C <x:4B> <y:4B> <yaw:4B> 0A   (共15字节)
 *             完毕帧 AA 00 0A  -> align_flag=0
 *   - x/y/yaw 为小端 float
 *   - x: 横向偏差(cm, 右为正)  y: 距离偏差(cm, 前为正)  yaw: 角度偏差(deg, 逆时针为正)
 * 接收处理位置: USART2_IRQHandler 里直接读 DR 后调用 Align_OnByte()。
 */
void Align_RxInit(void);
/* 由 USART2_IRQHandler 调用：喂一个接收字节进帧状态机 */
void Align_OnByte(uint8_t ch);
/* 非阻塞取一帧纠正量，返回 1=收到有效帧并写入 x/y/yaw；0=暂无 */
uint8_t Align_GetCorrection(float *x_cm, float *y_cm, float *yaw_deg);

/* 调整状态标志: 1=等待/正在调整(默认), 0=调整完毕(收到完毕帧) */
extern uint8_t align_flag;
/* 向上位机请求回传 x/y/yaw（发送 AA 01 0A），上位机收到后回传纠正帧 */
void Align_SendRequest(void);
/* 调整完毕: 把 align_flag 置 0，并回传"ALIGN_DONE\n"信号给上位机 */
void Align_SendDone(void);

#ifdef __cplusplus
}
#endif

#endif
