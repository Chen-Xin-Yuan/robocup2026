#include "usart_sent.h"
#include "usart.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#define TX_BUF_SIZE    256
uint8_t UART_DMA_Buf[TX_BUF_SIZE];

/* ==================== 调试打印（USART2 DMA 发送） ==================== */

void uart_printf(const char *format, ...)
{
    va_list args;
    int len;

    /* 注意：不能在中断里等待其他中断。 */
    if ((format == NULL) || (huart2.gState != HAL_UART_STATE_READY)) {
        return;
    }

    va_start(args, format);
    len = vsnprintf((char *)UART_DMA_Buf, TX_BUF_SIZE, format, args);
    va_end(args);

    if (len <= 0) {
        return;
    }
    if (len >= TX_BUF_SIZE) {
        len = TX_BUF_SIZE - 1;
    }

    (void)HAL_UART_Transmit_DMA(&huart2, UART_DMA_Buf, (uint16_t)len);
}

/* ==================== K210 视觉通信（USART6, 115200） ==================== */
/* 协议（ASCII，以 '\n' 结尾）：
 *   下位机→K210 : "SCAN1\n"  扫描任务1二维码
 *                 "SCAN2\n"  扫描任务2二维码
 *                 "ALIGN\n"  对准十字
 *                 "GO\n"     提示任务开始
 *   K210→下位机 : "N<数字>\n"   二维码数字（如 N12）
 *                 "CROSS_OK\n" 十字对准完成（简单模式）
 *                 "CROSS:<x>,<y>,<yaw>\n"  十字对准数据（连续回传）
 *                   x  : 十字中心横向偏差 (cm)，右为正
 *                   y  : 十字中心距离偏差 (cm)，前为正
 *                   yaw: 十字角度偏差 (deg)，逆时针为正
 *                   例  : CROSS:12.5,-3.2,1.5
 */
#define K210_RX_BUF_SIZE  32U

static uint8_t k210_rx_buf[K210_RX_BUF_SIZE];
static uint8_t k210_rx_len = 0U;
static volatile int16_t k210_qr_number = -1;
static volatile uint8_t k210_cross_ok = 0U;

/* 十字对准数据（由 "CROSS:x,y,yaw\n" 解析） */
typedef struct
{
    float x_cm;      /* 十字中心横向偏差 (cm)，右为正 */
    float y_cm;      /* 十字中心距离偏差 (cm)，前为正 */
    float yaw_deg;   /* 十字角度偏差 (deg)，逆时针为正 */
    uint8_t valid;   /* 是否收到过有效 CROSS 数据 */
    uint8_t fresh;   /* 新数据标志，K210_GetCrossData 读取后清零 */
} K210_CrossData_t;

static K210_CrossData_t k210_cross;

void K210_Send(const char *msg)
{
    size_t len = strlen(msg);

    if (len > 0U) {
        (void)HAL_UART_Transmit(&huart6, (uint8_t *)msg, (uint16_t)len, 100U);
    }
}

static void K210_RxByte(uint8_t byte)
{
    if ((byte == '\n') || (byte == '\r')) {
        if (k210_rx_len > 0U) {
            k210_rx_buf[k210_rx_len] = '\0';
            if ((k210_rx_buf[0] == 'N') || (k210_rx_buf[0] == 'n')) {
                k210_qr_number = (int16_t)atoi((const char *)&k210_rx_buf[1]);
            } else if (strncmp((const char *)k210_rx_buf, "CROSS_OK", 8U) == 0) {
                k210_cross_ok = 1U;
            } else if (strncmp((const char *)k210_rx_buf, "CROSS:", 6U) == 0) {
                float x_cm;
                float y_cm;
                float yaw_deg;
                if (sscanf((const char *)&k210_rx_buf[6], "%f,%f,%f",
                           &x_cm, &y_cm, &yaw_deg) == 3) {
                    k210_cross.x_cm = x_cm;
                    k210_cross.y_cm = y_cm;
                    k210_cross.yaw_deg = yaw_deg;
                    k210_cross.valid = 1U;
                    k210_cross.fresh = 1U;
                }
            }
        }
        k210_rx_len = 0U;
    } else {
        if (k210_rx_len < (K210_RX_BUF_SIZE - 1U)) {
            k210_rx_buf[k210_rx_len++] = byte;
        }
    }
}

/* 非阻塞轮询接收（在状态机/主循环里调用） */
void K210_Poll(void)
{
    uint8_t byte;

    while (HAL_UART_Receive(&huart6, &byte, 1U, 0U) == HAL_OK) {
        K210_RxByte(byte);
    }
}

/* 死等K210返回二维码数字，超时返回 -1 */
int K210_WaitNumber(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    k210_qr_number = -1;
    while ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)timeout_ms) {
        K210_Poll();
        if (k210_qr_number >= 0) {
            return (int)k210_qr_number;
        }
        HAL_Delay(10U);
    }
    return -1;
}

/* 死等K210十字对准完成，超时返回 0 */
uint8_t K210_WaitCrossOK(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    k210_cross_ok = 0U;
    while ((int32_t)(HAL_GetTick() - start_tick) < (int32_t)timeout_ms) {
        K210_Poll();
        if (k210_cross_ok != 0U) {
            return 1U;
        }
        HAL_Delay(10U);
    }
    return 0U;
}

/* ==================== K210 十字对准数据接口 ==================== */

/* 读取K210十字对准数据；有新数据返回1并清零新帧标志，数值始终写入最新值 */
uint8_t K210_GetCrossData(float *x_cm, float *y_cm, float *yaw_deg)
{
    uint8_t fresh = k210_cross.fresh;

    k210_cross.fresh = 0U;
    if (x_cm != NULL) {
        *x_cm = k210_cross.x_cm;
    }
    if (y_cm != NULL) {
        *y_cm = k210_cross.y_cm;
    }
    if (yaw_deg != NULL) {
        *yaw_deg = k210_cross.yaw_deg;
    }
    return fresh;
}

/* 是否收到过有效CROSS数据（用于判断K210十字识别是否正常工作） */
uint8_t K210_CrossValid(void)
{
    return k210_cross.valid;
}

/* 清空十字数据（进入对准流程前调用） */
void K210_ClearCross(void)
{
    memset(&k210_cross, 0, sizeof(k210_cross));
}

/* ==================== USART2 上位机命令接收（中断，固定帧格式） ==================== */
/* 只做接收，不影响 USART2 的 TX DMA 调试打印（uart_printf）。
 * 帧格式(带长度字节, 无校验):
 *   纠正帧: AA 0C <x:4B float> <y:4B float> <yaw:4B float> 0A   (共15字节)
 *   完毕帧: AA 00 0A  -> 上位机结束调整，解析后 align_flag=0
 *   - x/y/yaw 为小端 float
 *   - x: 横向偏差(cm,右正)  y: 距离偏差(cm,前正)  yaw: 角度偏差(deg,逆时针正)
 */
#define ALIGN_FRAME_HEAD       0xAAU   /* 帧头 1 字节 */
#define ALIGN_FRAME_TAIL       0x0AU   /* 帧尾 1 字节 */
#define ALIGN_FRAME_LEN        12U     /* 纠正帧数据长度: 3 个 float */
#define ALIGN_FRAME_DONE_LEN   0x00U   /* 完毕帧长度 */

static uint8_t align_frame_buf[ALIGN_FRAME_LEN];
static uint8_t align_frame_idx = 0U;
static uint8_t align_frame_state = 0U;   /* 0=找帧头 1=等长度 2=收数据 3=等帧尾 4=完毕帧帧尾 */
static volatile uint8_t align_frame_ready = 0U;

/* 调整状态标志: 1=等待/正在调整(默认), 0=调整完毕 */
uint8_t align_flag = 1U;

/* 启动 USART2 接收中断（NVIC 已在 CubeMX 配好，这里只需使能 RXNE） */
void Align_RxInit(void)
{
    align_frame_idx = 0U;
    align_frame_state = 0U;
    align_frame_ready = 0U;
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* 非阻塞取一帧纠正量，返回 1=收到有效帧并写入 x/y/yaw；0=暂无 */
uint8_t Align_GetCorrection(float *x_cm, float *y_cm, float *yaw_deg)
{
    uint32_t primask;
    float vx;
    float vy;
    float vyaw;

    if (align_frame_ready == 0U) {
        return 0U;
    }

    /* 关中断取走整帧，避免和接收中断竞争 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (align_frame_ready == 0U) {
        __set_PRIMASK(primask);
        return 0U;
    }
    align_frame_ready = 0U;
    __set_PRIMASK(primask);

    align_flag = 1U;   /* 收到命令，开始调整 */

    /* 小端 float */
    memcpy(&vx, &align_frame_buf[0], 4U);
    memcpy(&vy, &align_frame_buf[4], 4U);
    memcpy(&vyaw, &align_frame_buf[8], 4U);

    if (x_cm != NULL) {
        *x_cm = vx;
    }
    if (y_cm != NULL) {
        *y_cm = vy;
    }
    if (yaw_deg != NULL) {
        *yaw_deg = vyaw;
    }
    return 1U;
}

/* 请求帧: AA 01 0A（车->上位机：请回传 x/y/yaw 纠正量） */
static uint8_t align_req_frame[3] = {0xAAU, 0x01U, 0x0AU};

/* 向上位机请求回传 x/y/yaw（发送 AA 01 0A） */
void Align_SendRequest(void)
{
    /* 等 USART2 空闲再发，避免打断 uart_printf 的 TX DMA */
    if (huart2.gState == HAL_UART_STATE_READY) {
        (void)HAL_UART_Transmit(&huart2, align_req_frame, 3U, 10U);
    }
}

/* 调整完毕: align_flag 置 0，并回传信号给上位机 */
void Align_SendDone(void)
{
    align_flag = 0U;
    uart_printf("ALIGN_DONE\r\n");//传给上位机，不是k210
}

/* 由 USART2_IRQHandler 直接调用：每收到一个字节走一次帧状态机 */
void Align_OnByte(uint8_t ch)
{
    switch (align_frame_state) {
        case 0U:   /* 等帧头 AA */
            if (ch == ALIGN_FRAME_HEAD) {
                align_frame_state = 1U;
            }
            break;
        case 1U:   /* 等长度: 0C=纠正帧, 00=完毕帧 */
            if (ch == ALIGN_FRAME_LEN) {
                align_frame_idx = 0U;
                align_frame_state = 2U;
            } else if (ch == ALIGN_FRAME_DONE_LEN) {
                align_frame_state = 4U;   /* 完毕帧 */
            } else if (ch == ALIGN_FRAME_HEAD) {
                align_frame_state = 1U;   /* 连续 AA：当作新的帧头，重新等长度 */
            } else {
                align_frame_state = 0U;
            }
            break;
        case 2U:   /* 收满 12 字节数据 */
            if (align_frame_idx < ALIGN_FRAME_LEN) {
                align_frame_buf[align_frame_idx++] = ch;
            }
            if (align_frame_idx >= ALIGN_FRAME_LEN) {
                align_frame_state = 3U;
            }
            break;
        case 3U:   /* 等帧尾 0A */
            if (ch == ALIGN_FRAME_TAIL) {
                align_frame_ready = 1U;   /* 收到完整纠正帧 */
            }
            align_frame_state = 0U;
            break;
        case 4U:   /* 完毕帧: 等帧尾 0A -> 调整完毕 */
            if (ch == ALIGN_FRAME_TAIL) {
                align_flag = 0U;   /* 上位机结束调整 */
            }
            align_frame_state = 0U;
            break;
        default:
            align_frame_state = 0U;
            break;
    }

    /* RXNE 中断保持使能：读 DR 已自动清 RXNE，下一字节会继续触发 */
}
