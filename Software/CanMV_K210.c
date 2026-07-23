/*
 * CanMV_K210.c
 *
 * CanMV K210 视觉模块串口通信实现
 * 通过 UART4 (PA0-TX, PA1-RX) @ 115200 + DMA + IDLE 中断接收
 *
 * 帧格式: [0xAA] [帧类型] [数据...] [累加和] [0x6B]
 */

#include "CanMV_K210.h"
#include "gpio.h"
#include <string.h>

/*================== 内部变量 ==================*/

/* UART4 句柄 */
static UART_HandleTypeDef huart4;
static DMA_HandleTypeDef hdma_uart4_rx;

/* 接收缓冲区（外部可访问，用于中断服务） */
__IO uint8_t canmv_rx_buf[CANMV_RX_BUF_LEN] = {0};
__IO uint8_t canmv_rx_count = 0;
__IO bool    canmv_rx_flag = false;

/* 最新帧数据存储 */
static CanMV_FrameType_t last_frame_type = CANMV_FRAME_NONE;
static union {
    CanMV_Color_t  color;
    CanMV_Line_t   line;
    CanMV_QRCode_t qrcode;
    CanMV_Circle_t circle;
    CanMV_Letter_t letter;
} last_frame_data;

/*================== 内部函数 ==================*/

/**
 * @brief 计算校验和（帧类型字节 + 所有数据字节）
 * @param type  帧类型
 * @param data  数据指针
 * @param len   数据长度
 * @retval 校验和（低 8 位）
 */
static uint8_t canmv_calc_checksum(uint8_t type, const uint8_t *data, uint8_t len)
{
    uint8_t sum = type;
    for(uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 解析一帧数据
 * @param buf  接收缓冲区指针
 * @param len  缓冲区有效数据长度
 */
static void canmv_parse_frame(uint8_t *buf, uint8_t len)
{
    /* 从缓冲区中扫描有效帧 */
    uint8_t i = 0;
    while(i < len) {
        /* 查找帧头 0xAA */
        if(buf[i] != CANMV_FRAME_HEAD) {
            i++;
            continue;
        }

        /* 检查剩余长度是否足够（至少：帧头+类型+校验+帧尾=4字节） */
        if(i + 4 > len) break;

        uint8_t type   = buf[i + 1];    // 帧类型
        uint8_t *pdata = &buf[i + 2];   // 数据起始位置
        uint8_t remain = len - i - 2;   // 剩余字节数（含校验+帧尾）

        uint8_t data_len = 0;           // 数据段长度
        uint8_t frame_total = 0;        // 整帧长度（含帧头帧尾）

        /* 根据帧类型确定数据长度 */
        switch(type) {
            case CANMV_FRAME_COLOR:  data_len = 9;  frame_total = 13; break;
            case CANMV_FRAME_LINE:   data_len = 5;  frame_total = 9;  break;
            case CANMV_FRAME_QRCODE: data_len = 10; frame_total = 15; break;
            case CANMV_FRAME_CIRCLE: data_len = 6;  frame_total = 10; break;
            case CANMV_FRAME_LETTER: data_len = 5;  frame_total = 9;  break;
            default: i++; continue;  // 未知帧类型，跳过
        }

        /* 检查剩余数据是否够一帧 */
        if(remain + 1 < frame_total - 1) break;  // 不足，退出扫描

        /* 检查帧尾是否为 0x6B */
        if(buf[i + frame_total - 1] != CANMV_FRAME_TAIL) {
            i++;
            continue;
        }

        /* 验证校验和 */
        uint8_t recv_checksum = buf[i + frame_total - 2];
        uint8_t calc_checksum = canmv_calc_checksum(type, pdata, data_len);
        if(recv_checksum != calc_checksum) {
            i++;
            continue;
        }

        /* 校验通过，解析数据 */
        switch(type) {
            case CANMV_FRAME_COLOR: {
                CanMV_Color_t *c = &last_frame_data.color;
                c->color_id = pdata[0];
                c->center_x = ((uint16_t)pdata[1] << 8) | pdata[2];
                c->center_y = ((uint16_t)pdata[3] << 8) | pdata[4];
                c->width    = ((uint16_t)pdata[5] << 8) | pdata[6];
                c->height   = ((uint16_t)pdata[7] << 8) | pdata[8];
                break;
            }
            case CANMV_FRAME_LINE: {
                CanMV_Line_t *l = &last_frame_data.line;
                l->offset      = (int16_t)((pdata[0] << 8) | pdata[1]);
                l->angle       = (int16_t)((pdata[2] << 8) | pdata[3]);
                l->line_status = pdata[4];
                break;
            }
            case CANMV_FRAME_QRCODE: {
                CanMV_QRCode_t *q = &last_frame_data.qrcode;
                q->tag_id   = ((uint16_t)pdata[0] << 8) | pdata[1];
                q->center_x = ((uint16_t)pdata[2] << 8) | pdata[3];
                q->center_y = ((uint16_t)pdata[4] << 8) | pdata[5];
                q->angle    = (int16_t)((pdata[6] << 8) | pdata[7]);
                q->size     = ((uint16_t)pdata[8] << 8) | pdata[9];
                break;
            }
            case CANMV_FRAME_CIRCLE: {
                CanMV_Circle_t *c = &last_frame_data.circle;
                c->center_x = ((uint16_t)pdata[0] << 8) | pdata[1];
                c->center_y = ((uint16_t)pdata[2] << 8) | pdata[3];
                c->radius   = ((uint16_t)pdata[4] << 8) | pdata[5];
                break;
            }
            case CANMV_FRAME_LETTER: {
                CanMV_Letter_t *l = &last_frame_data.letter;
                l->ascii    = pdata[0];
                l->center_x = ((uint16_t)pdata[1] << 8) | pdata[2];
                l->center_y = ((uint16_t)pdata[3] << 8) | pdata[4];
                break;
            }
        }

        last_frame_type = (CanMV_FrameType_t)type;

        /* 跳过已处理的帧，继续扫描后面的数据（支持多帧粘连） */
        i += frame_total;
    }
}

/*================== 中断服务函数 ==================*/

/**
 * @brief UART4 IDLE 中断处理
 * @note  在 stm32f4xx_it.c 中调用
 */
void UART4_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);

        /* 停止 DMA，计算接收字节数 */
        HAL_UART_DMAStop(&huart4);
        canmv_rx_count = CANMV_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);

        /* 设置接收完成标志 */
        canmv_rx_flag = true;

        /* 重启 DMA 循环接收 */
        HAL_UART_Receive_DMA(&huart4, canmv_rx_buf, CANMV_RX_BUF_LEN);

        /* 解析帧数据 */
        canmv_parse_frame((uint8_t*)canmv_rx_buf, canmv_rx_count);
    }

    HAL_UART_IRQHandler(&huart4);
}

/**
 * @brief DMA1 Stream2 中断处理（UART4 RX）
 * @note  在 stm32f4xx_it.c 中调用
 */
void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

/*================== 初始化 ==================*/

/**
 * @brief UART4 MSP 初始化（GPIO + DMA + NVIC）
 * @param uartHandle UART 句柄指针
 */
static void HAL_UART_MspInit_Custom(UART_HandleTypeDef *uartHandle)
{
    if(uartHandle->Instance == UART4)
    {
        /* 使能 UART4 时钟 */
        __HAL_RCC_UART4_CLK_ENABLE();

        /* 使能 GPIOA 时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* 配置 PA0 = UART4_TX, PA1 = UART4_RX (AF8) */
        GPIO_InitTypeDef gpio_init = {0};
        gpio_init.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        gpio_init.Mode      = GPIO_MODE_AF_PP;
        gpio_init.Pull      = GPIO_NOPULL;
        gpio_init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &gpio_init);

        /* 使能 DMA1 时钟 */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* 配置 DMA1_Stream2 = UART4_RX (Channel 4, 循环模式) */
        hdma_uart4_rx.Instance                 = DMA1_Stream2;
        hdma_uart4_rx.Init.Channel             = DMA_CHANNEL_4;
        hdma_uart4_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_uart4_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_uart4_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart4_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_uart4_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_uart4_rx.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_uart4_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart4_rx);

        /* 连接 DMA 到 UART4 */
        __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart4_rx);

        /* 使能 DMA1_Stream2 中断 */
        HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

        /* 使能 UART4 中断 */
        HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
}

/**
 * @brief UART4 MSP 反初始化
 * @param uartHandle UART 句柄指针
 */
static void HAL_UART_MspDeInit_Custom(UART_HandleTypeDef *uartHandle)
{
    if(uartHandle->Instance == UART4)
    {
        /* 关闭 UART4 时钟 */
        __HAL_RCC_UART4_CLK_DISABLE();

        /* 反初始化 GPIO */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

        /* 反初始化 DMA */
        HAL_DMA_DeInit(&hdma_uart4_rx);

        /* 禁用中断 */
        HAL_NVIC_DisableIRQ(UART4_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    }
}

/**
 * @brief 初始化 CanMV K210 通信模块
 * @note  配置 UART4 (PA0/PA1) @ 115200 + DMA1_Stream2 + IDLE 中断
 *
 * 调用示例:
 *   CanMV_Init();  // 在 main.c 中外设初始化完成后调用
 *
 * 在主循环或定时器中获取数据:
 *   CanMV_FrameType_t type = CanMV_GetFrameType();
 *   if(type == CANMV_FRAME_COLOR) { CanMV_GetColor(&color); ... }
 *   CanMV_ClearData();
 */
void CanMV_Init(void)
{
    /* 清空接收缓冲区 */
    memset((void*)canmv_rx_buf, 0, CANMV_RX_BUF_LEN);
    canmv_rx_count = 0;
    canmv_rx_flag = false;
    last_frame_type = CANMV_FRAME_NONE;

    /* 初始化 UART4 句柄 */
    huart4.Instance          = UART4;
    huart4.Init.BaudRate     = CANMV_UART_BAUDRATE;
    huart4.Init.WordLength   = UART_WORDLENGTH_8B;
    huart4.Init.StopBits     = UART_STOPBITS_1;
    huart4.Init.Parity       = UART_PARITY_NONE;
    huart4.Init.Mode         = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;

    /* MSP 初始化（GPIO + DMA + NVIC） */
    HAL_UART_MspInit_Custom(&huart4);

    /* 初始化 UART4 */
    if(HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }

    /* 清除可能的遗留 IDLE 标志 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);

    /* 使能 IDLE 中断 */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

    /* 启动 DMA 循环接收 */
    HAL_UART_Receive_DMA(&huart4, canmv_rx_buf, CANMV_RX_BUF_LEN);
}

/*================== API 函数实现 ==================*/

CanMV_FrameType_t CanMV_GetFrameType(void)
{
    return last_frame_type;
}

uint8_t CanMV_GetColor(CanMV_Color_t *color)
{
    if(last_frame_type != CANMV_FRAME_COLOR) return 1;
    if(color == NULL) return 1;
    *color = last_frame_data.color;
    return 0;
}

uint8_t CanMV_GetLine(CanMV_Line_t *line)
{
    if(last_frame_type != CANMV_FRAME_LINE) return 1;
    if(line == NULL) return 1;
    *line = last_frame_data.line;
    return 0;
}

uint8_t CanMV_GetQRCode(CanMV_QRCode_t *qr)
{
    if(last_frame_type != CANMV_FRAME_QRCODE) return 1;
    if(qr == NULL) return 1;
    *qr = last_frame_data.qrcode;
    return 0;
}

uint8_t CanMV_GetCircle(CanMV_Circle_t *circle)
{
    if(last_frame_type != CANMV_FRAME_CIRCLE) return 1;
    if(circle == NULL) return 1;
    *circle = last_frame_data.circle;
    return 0;
}

uint8_t CanMV_GetLetter(CanMV_Letter_t *letter)
{
    if(last_frame_type != CANMV_FRAME_LETTER) return 1;
    if(letter == NULL) return 1;
    *letter = last_frame_data.letter;
    return 0;
}

void CanMV_ClearData(void)
{
    last_frame_type = CANMV_FRAME_NONE;
}