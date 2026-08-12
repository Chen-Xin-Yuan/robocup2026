#include "usart_sent.h"
#include "usart.h"
#include "Chassis.h"

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
 *   K210→下位机 : "CROSS:<x>,<y>,<yaw>\n"  十字对准数据（连续回传）
 *                   x  : 十字中心横向偏差 (cm)，右为正
 *                   y  : 十字中心距离偏差 (cm)，前为正
 *                   yaw: 十字角度偏差 (deg)，逆时针为正
 *                   例  : CROSS:12.5,-3.2,1.5
 */
#define K210_RX_BUF_SIZE  32U

static uint8_t k210_rx_buf[K210_RX_BUF_SIZE];
static uint8_t k210_rx_len = 0U;

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
            if (strncmp((const char *)k210_rx_buf, "CROSS:", 6U) == 0) {
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

/* 清空十字数据（进入对准流程前调用） */
void K210_ClearCross(void)
{
    memset(&k210_cross, 0, sizeof(k210_cross));
}

/* ==================== USART2 上位机命令接收（中断，固定帧格式） ==================== */
/* 只做接收，不影响 USART2 的 TX DMA 调试打印（uart_printf）。
 * 帧格式(带长度字节, 无校验): AA <长度> <数据...> 0A
 *
 *   上位机->车(接收):
 *     对十字纠正帧: AA 0C <x:4B> <y:4B> <yaw:4B> 0A   (12字节数据, 共15字节)
 *     对圆心纠正帧: AA 08 <x:4B> <y:4B> 0A            (8字节数据, 只回传x/y, 无yaw)
 *     颜色帧:       AA 01 <颜色:1B> 0A                (1字节数据, 收满5个不同颜色即结束, 重复自动丢弃)
 *                   颜色: 0=黑 1=白 2=红 3=绿 4=蓝, 或 ASCII 字母 B/W/R/G/U(不区分大小写)
 *                   顺序 = 槽位1~5 从左到右
 *     二维码帧:     AA 04 <编号:1B> 0A                (类型码04, 1字节数据)
 *                   编号: 任务1=1~16, 任务2=1~6
 *     字母帧:       AA 06 <字母:1B> 0A                (类型码06, 1字节数据)
 *                   字母: 01=a 02=b 03=c, 或 ASCII a/A/b/B/c/C(不区分大小写)
 *                   顺序 = 任务2 奖杯/槽位 1~3 依次回传, 共3个
 *     完毕帧:       AA 00 0A                          (对十字/对圆心 结束)
 *     报错帧:       AA 0F 0A                          (任意时刻收到立即停车)
 *
 *   车->上位机(发送, 请求回传):
 *     AA 01 0A   请求对十字纠正 (x/y/yaw)。每次阻塞式调整结束后再发一次，
 *                上位机每收到一次回传一帧纠正量，直到收到完毕帧 AA 00 0A。
 *     AA 02 0A   请求对圆心纠正 (x/y)。同上，每次阻塞式调整结束后再发一次。
 *     AA 03 0A   请求颜色识别(单槽位)。车收到 1 个颜色后旋转舵机到下一槽位，
 *                舵机到位(固定延时)后再发一次；上位机每收到一次只回传 1 个颜色帧，共 5 次。
 *     AA 05 0A   请求二维码 (上位机回1个二维码帧)
 *     AA 06 0A   请求字母识别(任务2, 单槽位)。车收到 1 个字母后旋转舵机到下一槽位，
 *                舵机到位(固定延时)后再发一次；上位机每收到一次只回传 1 个字母帧，共 3 次。
 *                注: 请求/回传都用 06，与二维码帧 04 区分开。
 *
 *   - x/y/yaw 为小端 float
 *   - x: 横向偏差(cm,右正)  y: 距离偏差(cm,前正)  yaw: 角度偏差(deg,逆时针正)
 */
#define USART2_FRAME_HEAD        0xAAU   /* 帧头 1 字节 */
#define USART2_FRAME_TAIL        0x0AU   /* 帧尾 1 字节 */
#define CROSS_FRAME_LEN          12U     /* 对十字纠正帧数据长度: 3 个 float */
#define CIRCLE_FRAME_LEN         8U      /* 对圆心纠正帧数据长度: 2 个 float (x/y) */
#define COLOR_FRAME_LEN          1U      /* 颜色帧数据长度: 1 个颜色 */
#define QR_FRAME_LEN             4U      /* 二维码帧类型码(数据长度见 QR_DATA_LEN) */
#define QR_DATA_LEN              1U      /* 二维码帧数据长度: 1 个编号 */
#define USART2_FRAME_DONE_LEN    0x00U   /* 完毕帧长度 */
#define USART2_FRAME_STOP_LEN    0x0FU   /* 报错帧长度(类型码): AA 0F 0A，收到立即停车 */

#define COLOR_MAX                5U      /* 颜色识别: 收满5个不同颜色即结束 */
#define COLOR_SERVO_SETTLE_MS    400U   /* 舵机旋转命令发出后等其到位的固定时间(ms)，与 Servo_motor.h 的 SERVO_BUS_MOVE_TIME_MS 一致 */
#define LETTER_MAX               3U      /* 字母识别: 收满3个字母(a/b/c)即结束 */
#define LETTER_SERVO_SETTLE_MS   200U   /* 收到字母->转舵机后等其到位的固定延时(ms)，到点后请求下一个字母 */
#define LETTER_FRAME_LEN         6U      /* 字母回传帧类型码: AA 06 <字母:1B> 0A */
#define LETTER_DATA_LEN          1U      /* 字母回传帧数据长度: 1 个字母 */

static uint8_t usart2_frame_buf[CROSS_FRAME_LEN];   /* 最大帧数据长度 12 */
static uint8_t usart2_frame_type = 0U;  /* 当前帧类型(长度字节): 0C十字 08圆心 01颜色 04二维码 00完毕 */
static uint8_t usart2_data_len = 0U;    /* 当前帧数据字节数 */
static uint8_t usart2_frame_idx = 0U;
static uint8_t usart2_frame_state = 0U;  /* 0=找帧头 1=等长度 2=收数据 3=等帧尾 4=完毕帧帧尾 */
static volatile uint8_t cross_frame_ready = 0U;      /* 对十字帧就绪 */
static volatile uint8_t circle_frame_ready = 0U;     /* 对圆心帧就绪 */

/* 对十字缓存: 帧尾收到时立刻拷出，避免下一帧覆盖 */
static float cross_x_cm = 0.0f;
static float cross_y_cm = 0.0f;
static float cross_yaw_deg = 0.0f;

/* 对圆心缓存 */
static float circle_x_cm = 0.0f;
static float circle_y_cm = 0.0f;

/* 调整状态标志: 1=等待/正在调整(默认), 0=调整完毕(收到完毕帧) */
uint8_t cross_flag = 1U;
/* 对圆心状态标志: 1=等待/正在对圆心, 0=完毕/未开始 */
uint8_t circle_flag = 0U;

/* ---- 颜色识别缓存 ---- */
static uint8_t usart2_colors[COLOR_MAX];      /* 收到的颜色编号 0~4 */
static volatile uint8_t usart2_color_count = 0U;
static volatile uint8_t usart2_color_done = 0U;

/* 颜色识别改为逐槽位请求：舵机每转到一个位置，等固定延时后请求 1 个颜色 */
static volatile uint8_t  color_req_pending = 0U;   /* 舵机正在旋转，等待到位后发请求 */
static volatile uint32_t color_req_tick = 0U;      /* 最近一次舵机旋转命令发出时刻(ms) */
static volatile uint8_t  color_rx_fresh = 0U;      /* 新收到 1 个颜色，等待主循环转舵机 */

/* ---- 字母识别缓存 ---- */
static uint8_t usart2_letters[LETTER_MAX];     /* 收到的字母编号 0=a 1=b 2=c */
static volatile uint8_t usart2_letter_count = 0U;
static volatile uint8_t usart2_letter_done = 0U;

/* 字母识别逐槽位请求：舵机每转到一个位置，等固定延时后请求 1 个字母 */
static volatile uint8_t  letter_req_pending = 0U;   /* 舵机正在旋转，等待到位后发请求 */
static volatile uint32_t letter_req_tick = 0U;      /* 最近一次舵机旋转命令发出时刻(ms) */
static volatile uint8_t  letter_rx_fresh = 0U;      /* 新收到 1 个字母，等待主循环转舵机 */

/* ---- 二维码缓存 ---- */
static volatile uint8_t usart2_qr_number = 0U;
static volatile uint8_t usart2_qr_ready = 0U;

/* ==================== USART2 请求帧发送队列 ==================== */
/* 请求帧(AA 01/02/03/05 0A)通过 DMA 队列发送：即使 uart_printf/灰度打印正占着
 * USART2，请求帧也先入队、等发送空闲后自动发出，不会丢帧。
 * 发送完成由 HAL_UART_TxCpltCallback -> Usart2_TxCpltCallback 驱动下一帧。
 */
#define USART2_TX_Q_SIZE          8U      /* 队列槽位(每槽一帧3字节) */

typedef struct
{
    uint8_t data[3];   /* 请求帧数据(AA XX 0A) */
    uint8_t len;       /* 帧长度(固定3) */
} Usart2_TxFrame_t;

static Usart2_TxFrame_t usart2_tx_q[USART2_TX_Q_SIZE];
static volatile uint8_t usart2_tx_head = 0U;   /* 出队位置(发送完成回调里取) */
static volatile uint8_t usart2_tx_tail = 0U;   /* 入队位置(主循环放) */

/* 队里有待发帧且串口空闲 -> 启动下一帧 DMA(启动成功才出队) */
static void usart2_tx_pump(void)
{
    if (usart2_tx_head == usart2_tx_tail) {
        return;   /* 队列空 */
    }
    if (huart2.gState == HAL_UART_STATE_READY) {
        if (HAL_UART_Transmit_DMA(&huart2, usart2_tx_q[usart2_tx_head].data,
                                  usart2_tx_q[usart2_tx_head].len) == HAL_OK) {
            usart2_tx_head = (uint8_t)((usart2_tx_head + 1U) % USART2_TX_Q_SIZE);
        }
    }
}

/* 请求帧入队(主循环调用)：串口空闲则立即发，否则排队等空闲 */
static void usart2_tx_queue(const uint8_t *data, uint8_t len)
{
    uint8_t next;

    if ((data == NULL) || (len > 3U)) {
        return;
    }
    next = (uint8_t)((usart2_tx_tail + 1U) % USART2_TX_Q_SIZE);
    if (next == usart2_tx_head) {
        /* 队列满：丢弃最旧一帧，保证最新请求一定能入队 */
        usart2_tx_head = (uint8_t)((usart2_tx_head + 1U) % USART2_TX_Q_SIZE);
    }
    memcpy(usart2_tx_q[usart2_tx_tail].data, data, len);
    usart2_tx_q[usart2_tx_tail].len = len;
    usart2_tx_tail = next;
    usart2_tx_pump();
}

/* DMA 发送完成回调(由 main.c 的 HAL_UART_TxCpltCallback 转发)：自动发下一帧 */
void Usart2_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        usart2_tx_pump();
    }
}

/* 启动 USART2 接收中断（NVIC 已在 CubeMX 配好，这里只需使能 RXNE） */
void Usart2_RxInit(void)
{
    usart2_frame_idx = 0U;
    usart2_frame_state = 0U;
    cross_frame_ready = 0U;
    circle_frame_ready = 0U;
    usart2_qr_ready = 0U;
    usart2_qr_number = 0U;
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* 把颜色字节解析为颜色编号(0~4)；非法返回 0xFF */
static uint8_t usart2_color_parse(uint8_t ch)
{
    if (ch <= 4U) {
        return ch;
    }
    switch (ch) {
        case 'B': case 'b': return 0U;   /* 黑 */
        case 'W': case 'w': return 1U;   /* 白 */
        case 'R': case 'r': return 2U;   /* 红 */
        case 'G': case 'g': return 3U;   /* 绿 */
        case 'U': case 'u': return 4U;   /* 蓝 */
        default: return 0xFFU;
    }
}

/* 收一个颜色(中断内调用)：重复颜色自动丢弃，收满5个不同颜色置 done */
static void usart2_color_add(uint8_t ch)
{
    uint8_t c;
    uint8_t i;

    if (usart2_color_done != 0U) {
        return;   /* 已收满5个不同颜色，忽略后续 */
    }
    c = usart2_color_parse(ch);
    if (c >= 5U) {
        return;   /* 非法颜色，忽略 */
    }
    /* 重复颜色：自动丢弃后一个(不计入)，继续检测直到凑齐5个不同颜色 */
    for (i = 0U; i < (uint8_t)usart2_color_count; i++) {
        if (usart2_colors[i] == c) {
            return;
        }
    }
    usart2_colors[usart2_color_count] = c;
    usart2_color_count++;
    color_rx_fresh = 1U;   /* 新收到 1 个颜色：主循环据此转舵机到下一槽位 */
    if (usart2_color_count >= COLOR_MAX) {
        usart2_color_done = 1U;   /* 收满5个不同颜色，识别结束 */
    }
}

/* ==================== 字母识别缓存与解析 ==================== */

/* 把字母字节解析为字母编号(0=a 1=b 2=c)；非法返回 0xFF */
static uint8_t usart2_letter_parse(uint8_t ch)
{
    if ((ch >= 1U) && (ch <= 3U)) {
        return (uint8_t)(ch - 1U);   /* 01/02/03 -> a/b/c */
    }
    switch (ch) {
        case 'a': case 'A': return 0U;   /* a */
        case 'b': case 'B': return 1U;   /* b */
        case 'c': case 'C': return 2U;   /* c */
        default: return 0xFFU;
    }
}

/* 收一个字母(中断内调用)：重复自动丢弃，收满3个不同字母置 done */
static void usart2_letter_add(uint8_t ch)
{
    uint8_t c;
    uint8_t i;

    if (usart2_letter_done != 0U) {
        return;   /* 已收满3个字母，忽略后续 */
    }
    c = usart2_letter_parse(ch);
    if (c >= 3U) {
        return;   /* 非法字母，忽略 */
    }
    /* 重复字母：自动丢弃后一个(不计入)，继续检测直到凑齐3个不同字母 */
    for (i = 0U; i < (uint8_t)usart2_letter_count; i++) {
        if (usart2_letters[i] == c) {
            return;
        }
    }
    usart2_letters[usart2_letter_count] = c;
    usart2_letter_count++;
    letter_rx_fresh = 1U;   /* 新收到 1 个字母：主循环据此转舵机到下一槽位 */
    if (usart2_letter_count >= LETTER_MAX) {
        usart2_letter_done = 1U;   /* 收满3个字母，识别结束 */
    }
}

/* ==================== 颜色识别接口 ==================== */

/* 当前已收到颜色个数(0~5) */
uint8_t Color_GetCount(void)
{
    return (uint8_t)usart2_color_count;
}

/* 取第 i 个颜色编号(0=黑 1=白 2=红 3=绿 4=蓝)；越界返回 0xFF */
uint8_t Color_GetColor(uint8_t i)
{
    if (i >= (uint8_t)usart2_color_count) {
        return 0xFFU;
    }
    return usart2_colors[i];
}

/* 是否已收满5个颜色(识别结束) */
uint8_t Color_IsDone(void)
{
    return (uint8_t)usart2_color_done;
}

/* 清空颜色缓存，开始新一轮识别 */
void Color_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    usart2_color_count = 0U;
    usart2_color_done = 0U;
    color_req_pending = 0U;
    color_req_tick = 0U;
    color_rx_fresh = 0U;
    __set_PRIMASK(primask);
}

/* 请求上位机回传 1 个颜色（发送 AA 03 0A，走发送队列不丢帧）。
 * 首次由主循环进入 state21 时调用，之后由 Color_Process 在舵机到位后调用。 */
void Color_SendRequest(void)
{
    static const uint8_t frame[3] = {USART2_FRAME_HEAD, 0x03U, USART2_FRAME_TAIL};
    usart2_tx_queue(frame, 3U);
}

/* 是否有新收到的颜色（1=有，读取后清除）。主循环据此旋转舵机到下一槽位 */
uint8_t Color_TakeNew(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t fresh;

    __disable_irq();
    fresh = color_rx_fresh;
    color_rx_fresh = 0U;
    __set_PRIMASK(primask);
    return fresh;
}

/* 主循环发出舵机旋转命令后调用：记录时刻，开始等待舵机到位 */
void Color_StartServoRotate(void)
{
    color_req_tick = HAL_GetTick();
    color_req_pending = 1U;
}

/* 主循环里周期调用：舵机到位（固定延时 COLOR_SERVO_SETTLE_MS）后，
 * 向上位机请求下一个槽位颜色（AA 03 0A）。每收到 1 个颜色转一次舵机、
 * 舵机到位后请求一次；已收满 5 个不同颜色后不再请求。 */
void Color_Process(void)
{
    if (color_req_pending == 0U) {
        return;
    }
    if ((int32_t)(HAL_GetTick() - color_req_tick) < (int32_t)COLOR_SERVO_SETTLE_MS) {
        return;   /* 舵机还没到位 */
    }
    color_req_pending = 0U;
    if (Color_IsDone() != 0U) {
        return;   /* 已收满 5 个颜色，不再请求 */
    }
    Color_SendRequest();   /* 请求下一个槽位颜色 */
}

/* ==================== 字母识别接口 ==================== */

/* 当前已收到字母个数(0~3) */
uint8_t Letter_GetCount(void)
{
    return (uint8_t)usart2_letter_count;
}

/* 取第 i 个字母编号(0=a 1=b 2=c)；越界返回 0xFF */
uint8_t Letter_GetLetter(uint8_t i)
{
    if (i >= (uint8_t)usart2_letter_count) {
        return 0xFFU;
    }
    return usart2_letters[i];
}

/* 是否已收满3个字母(识别结束) */
uint8_t Letter_IsDone(void)
{
    return (uint8_t)usart2_letter_done;
}

/* 清空字母缓存，开始新一轮识别 */
void Letter_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    usart2_letter_count = 0U;
    usart2_letter_done = 0U;
    letter_req_pending = 0U;
    letter_req_tick = 0U;
    letter_rx_fresh = 0U;
    __set_PRIMASK(primask);
}

/* 请求上位机回传 1 个字母（发送 AA 06 0A，走发送队列不丢帧）。
 * 首次由主循环进入任务2循迹阶段时调用，之后由 Letter_Process 在舵机到位后调用。 */
void Letter_SendRequest(void)
{
    static const uint8_t frame[3] = {USART2_FRAME_HEAD, 0x06U, USART2_FRAME_TAIL};
    usart2_tx_queue(frame, 3U);
}

/* 是否有新收到的字母（1=有，读取后清除）。主循环据此旋转舵机到下一槽位 */
uint8_t Letter_TakeNew(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t fresh;

    __disable_irq();
    fresh = letter_rx_fresh;
    letter_rx_fresh = 0U;
    __set_PRIMASK(primask);
    return fresh;
}

/* 主循环发出舵机旋转命令后调用：记录时刻，开始等待舵机到位 */
void Letter_StartServoRotate(void)
{
    letter_req_tick = HAL_GetTick();
    letter_req_pending = 1U;
}

/* 主循环里周期调用：舵机到位（固定延时 LETTER_SERVO_SETTLE_MS）后，
 * 向上位机请求下一个字母（AA 06 0A）。每收到 1 个字母转一次舵机、
 * 舵机到位后请求一次；已收满 3 个字母后不再请求。 */
void Letter_Process(void)
{
    if (letter_req_pending == 0U) {
        return;
    }
    if ((int32_t)(HAL_GetTick() - letter_req_tick) < (int32_t)LETTER_SERVO_SETTLE_MS) {
        return;   /* 舵机还没到位 */
    }
    letter_req_pending = 0U;
    if (Letter_IsDone() != 0U) {
        return;   /* 已收满 3 个字母，不再请求 */
    }
    Letter_SendRequest();   /* 请求下一个字母 */
}

/* ==================== 二维码接口 ==================== */

/* 收到二维码帧时(中断内)调用 */
static void usart2_qr_add(uint8_t ch)
{
    usart2_qr_number = ch;
    usart2_qr_ready = 1U;
}

/* 是否有新的二维码编号就绪 */
uint8_t QR_IsReady(void)
{
    return (uint8_t)usart2_qr_ready;
}

/* 取二维码编号(任务1=1~16, 任务2=1~6，范围由调用方校验) */
uint8_t QR_GetNumber(void)
{
    return (uint8_t)usart2_qr_number;
}

/* 清空二维码缓存，开始新一轮等待 */
void QR_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    usart2_qr_ready = 0U;
    usart2_qr_number = 0U;
    __set_PRIMASK(primask);
}

/* 请求上位机回传二维码（发送 AA 05 0A，走发送队列不丢帧） */
void QR_SendRequest(void)
{
    static const uint8_t frame[3] = {USART2_FRAME_HEAD, 0x05U, USART2_FRAME_TAIL};
    usart2_tx_queue(frame, 3U);
}

/* ==================== 对圆心接口 ==================== */

/* 非阻塞取一帧圆心纠正量(x/y)，返回 1=收到新帧并写入；0=暂无 */
uint8_t Circle_GetCorrection(float *x_cm, float *y_cm)
{
    uint32_t primask;
    float vx;
    float vy;

    if (circle_frame_ready == 0U) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (circle_frame_ready == 0U) {
        __set_PRIMASK(primask);
        return 0U;
    }
    circle_frame_ready = 0U;
    __set_PRIMASK(primask);

    vx = circle_x_cm;
    vy = circle_y_cm;
    if (x_cm != NULL) {
        *x_cm = vx;
    }
    if (y_cm != NULL) {
        *y_cm = vy;
    }
    return 1U;
}

/* 请求上位机回传圆心 x/y（发送 AA 02 0A，走发送队列不丢帧） */
void Circle_SendRequest(void)
{
    static const uint8_t frame[3] = {USART2_FRAME_HEAD, 0x02U, USART2_FRAME_TAIL};
    usart2_tx_queue(frame, 3U);
}

/* 对圆心调整完毕: circle_flag 置 0，并回传信号给上位机 */
void Circle_SendDone(void)
{
    circle_flag = 0U;
    uart_printf("CIRCLE_DONE\r\n");
}

/* ==================== 对十字接口(原有) ==================== */

/* 非阻塞取一帧纠正量，返回 1=收到有效帧并写入 x/y/yaw；0=暂无 */
uint8_t Cross_GetCorrection(float *x_cm, float *y_cm, float *yaw_deg)
{
    uint32_t primask;
    float vx;
    float vy;
    float vyaw;

    if (cross_frame_ready == 0U) {
        return 0U;
    }

    /* 关中断取走整帧，避免和接收中断竞争 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (cross_frame_ready == 0U) {
        __set_PRIMASK(primask);
        return 0U;
    }
    cross_frame_ready = 0U;
    __set_PRIMASK(primask);

    cross_flag = 1U;   /* 收到命令，开始调整 */

    vx = cross_x_cm;
    vy = cross_y_cm;
    vyaw = cross_yaw_deg;

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
static uint8_t cross_req_frame[3] = {0xAAU, 0x01U, 0x0AU};

/* 向上位机请求回传 x/y/yaw（发送 AA 01 0A，走发送队列不丢帧） */
void Cross_SendRequest(void)
{
    usart2_tx_queue(cross_req_frame, 3U);
}

/* 调整完毕: cross_flag 置 0，并回传信号给上位机 */
void Cross_SendDone(void)
{
    cross_flag = 0U;
    uart_printf("ALIGN_DONE\r\n");//传给上位机，不是k210
}

/* 由 USART2_IRQHandler 直接调用：每收到一个字节走一次帧状态机。
 * 对十字/对圆心/颜色识别共用同一个帧状态机，按"长度字节"区分帧类型。
 */
void Usart2_OnByte(uint8_t ch)
{
    switch (usart2_frame_state) {
        case 0U:   /* 等帧头 AA */
            if (ch == USART2_FRAME_HEAD) {
                usart2_frame_state = 1U;
            }
            break;
        case 1U:   /* 等长度(类型): 0C=对十字 08=对圆心 01=颜色 04=二维码 06=字母 00=完毕 0F=报错停车 */
            usart2_frame_type = 0U;
            usart2_data_len = 0U;
            usart2_frame_idx = 0U;
            if (ch == CROSS_FRAME_LEN) {
                usart2_frame_type = CROSS_FRAME_LEN;
                usart2_data_len = CROSS_FRAME_LEN;
                usart2_frame_state = 2U;
            } else if (ch == CIRCLE_FRAME_LEN) {
                usart2_frame_type = CIRCLE_FRAME_LEN;
                usart2_data_len = CIRCLE_FRAME_LEN;
                usart2_frame_state = 2U;
            } else if (ch == COLOR_FRAME_LEN) {
                usart2_frame_type = COLOR_FRAME_LEN;
                usart2_data_len = COLOR_FRAME_LEN;
                usart2_frame_state = 2U;
            } else if (ch == QR_FRAME_LEN) {
                usart2_frame_type = QR_FRAME_LEN;
                usart2_data_len = QR_DATA_LEN;
                usart2_frame_state = 2U;
            } else if (ch == LETTER_FRAME_LEN) {
                usart2_frame_type = LETTER_FRAME_LEN;
                usart2_data_len = LETTER_DATA_LEN;
                usart2_frame_state = 2U;
            } else if (ch == USART2_FRAME_DONE_LEN) {
                usart2_frame_state = 4U;   /* 完毕帧 */
            } else if (ch == USART2_FRAME_STOP_LEN) {
                usart2_frame_state = 5U;   /* 报错帧: 等帧尾 0A -> 立即停车 */
            } else if (ch == USART2_FRAME_HEAD) {
                usart2_frame_state = 1U;   /* 连续 AA：当作新的帧头，重新等长度 */
            } else {
                usart2_frame_state = 0U;
            }
            break;
        case 2U:   /* 收数据 */
            if (usart2_frame_idx < usart2_data_len) {
                usart2_frame_buf[usart2_frame_idx++] = ch;
            }
            if (usart2_frame_idx >= usart2_data_len) {
                usart2_frame_state = 3U;
            }
            break;
        case 3U:   /* 等帧尾 0A -> 按帧类型分发 */
            if (ch == USART2_FRAME_TAIL) {
                if (usart2_frame_type == CROSS_FRAME_LEN) {
                    /* 对十字: 拷出 x/y/yaw，置就绪 */
                    memcpy(&cross_x_cm, &usart2_frame_buf[0], 4U);
                    memcpy(&cross_y_cm, &usart2_frame_buf[4], 4U);
                    memcpy(&cross_yaw_deg, &usart2_frame_buf[8], 4U);
                    cross_frame_ready = 1U;
                } else if (usart2_frame_type == CIRCLE_FRAME_LEN) {
                    /* 对圆心: 只解析 x/y，不解析 yaw */
                    memcpy(&circle_x_cm, &usart2_frame_buf[0], 4U);
                    memcpy(&circle_y_cm, &usart2_frame_buf[4], 4U);
                    circle_frame_ready = 1U;
                } else if (usart2_frame_type == COLOR_FRAME_LEN) {
                    usart2_color_add(usart2_frame_buf[0]);   /* 收一个颜色 */
                } else if (usart2_frame_type == QR_FRAME_LEN) {
                    usart2_qr_add(usart2_frame_buf[0]);      /* 收二维码编号 */
                } else if (usart2_frame_type == LETTER_FRAME_LEN) {
                    usart2_letter_add(usart2_frame_buf[0]);  /* 收一个字母 */
                }
            }
            usart2_frame_state = 0U;
            break;
        case 4U:   /* 完毕帧: 等帧尾 0A -> 对十字/对圆心结束 */
            if (ch == USART2_FRAME_TAIL) {
                cross_flag = 0U;    /* 对十字结束 */
                circle_flag = 0U;   /* 对圆心结束 */
            }
            usart2_frame_state = 0U;
            break;
        case 5U:   /* 报错帧: 等帧尾 0A -> 无论何时收到都立即停车 */
            if (ch == USART2_FRAME_TAIL) {
                Chassis_stop();   /* 上位机报错帧 AA 0F 0A */
            }
            usart2_frame_state = 0U;
            break;
        default:
            usart2_frame_state = 0U;
            break;
    }

    /* RXNE 中断保持使能：读 DR 已自动清 RXNE，下一字节会继续触发 */
}
