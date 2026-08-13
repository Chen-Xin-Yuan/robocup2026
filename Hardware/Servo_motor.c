#include "Servo_motor.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define SERVO_POSITION_MIN            500U
#define SERVO_POSITION_MAX           2500U
#define SERVO_PWM_ANGLE_MAX_DEG        120U
#define SERVO_PWM_FULL_RANGE_DEG      180U
#define SERVO_BUS_ANGLE_MAX_DEG       360U
#define SERVO_BUS_MOVE_TIME_MS       1000U
#define SERVO_INIT_DELAY_MS           500U
#define SERVO_POWER_ON_DELAY_MS      300U
#define SERVO_COMMAND_MAX_LEN          24U
#define SERVO_TX_QUEUE_DEPTH           4U
//67 48 25
#define SERVO_BUS_1          25U
#define SERVO_BUS_2          SERVO_BUS_1+72
#define SERVO_BUS_3          SERVO_BUS_2+72
#define SERVO_BUS_4          SERVO_BUS_3+72
#define SERVO_BUS_5          SERVO_BUS_4+72
#define SERVO_BUS_TASK1_LOCK       SERVO_BUS_3+36
#define SERVO_BUS_TASK2_LOCK       SERVO_BUS_2+36

uint16_t Servo_angle[7]={SERVO_BUS_1,SERVO_BUS_2,SERVO_BUS_3,SERVO_BUS_4,SERVO_BUS_5,SERVO_BUS_TASK1_LOCK,SERVO_BUS_TASK2_LOCK};

/* ---------------- DMA 发送队列（非阻塞，可在中断中调用） ---------------- */

typedef struct
{
    uint8_t length;
    char data[SERVO_COMMAND_MAX_LEN];
} Servo_TxFrame_t;

static Servo_TxFrame_t s_servo_tx_queue[SERVO_TX_QUEUE_DEPTH];
static volatile uint8_t s_servo_tx_head = 0U;
static volatile uint8_t s_servo_tx_tail = 0U;
static volatile uint8_t s_servo_tx_count = 0U;
static volatile uint8_t s_servo_tx_active = 0U;

static uint32_t Servo_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Servo_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 队列非空且 DMA 空闲时启动下一帧。非阻塞，可在中断中调用。 */
static void Servo_TryStartTx(void)
{
    uint32_t primask;

    primask = Servo_EnterCritical();
    if (s_servo_tx_active || (s_servo_tx_count == 0U)) {
        Servo_ExitCritical(primask);
        return;
    }
    s_servo_tx_active = 1U;
    Servo_ExitCritical(primask);

    if (HAL_UART_Transmit_DMA(&huart3,
                              (uint8_t *)s_servo_tx_queue[s_servo_tx_head].data,
                              s_servo_tx_queue[s_servo_tx_head].length) != HAL_OK) {
        /* 启动失败：释放占用标记，交给 Servo_Process 重试。 */
        s_servo_tx_active = 0U;
    }
}

/* 入队一帧指令。非阻塞，可在中断中调用；队列满时返回 HAL_BUSY 并丢弃本帧。 */
static HAL_StatusTypeDef Servo_SendString(const char *command)
{
    size_t length;
    uint32_t primask;

    if (command == NULL) {
        return HAL_ERROR;
    }

    length = strlen(command);
    if ((length == 0U) || (length >= SERVO_COMMAND_MAX_LEN)) {
        return HAL_ERROR;
    }

    primask = Servo_EnterCritical();

    if (s_servo_tx_count >= SERVO_TX_QUEUE_DEPTH) {
        Servo_ExitCritical(primask);
        return HAL_BUSY;
    }

    memcpy(s_servo_tx_queue[s_servo_tx_tail].data, command, length + 1U);
    s_servo_tx_queue[s_servo_tx_tail].length = (uint8_t)length;
    s_servo_tx_tail = (uint8_t)((s_servo_tx_tail + 1U) % SERVO_TX_QUEUE_DEPTH);
    s_servo_tx_count++;

    Servo_ExitCritical(primask);

    Servo_TryStartTx();
    return HAL_OK;
}

/* 由 main.c 的 HAL_UART_TxCpltCallback 转调：上一帧发送完成，启动下一帧。 */
void Servo_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (huart != &huart3) {
        return;
    }

    primask = Servo_EnterCritical();

    if (s_servo_tx_active && (s_servo_tx_count > 0U)) {
        s_servo_tx_active = 0U;
        s_servo_tx_head = (uint8_t)((s_servo_tx_head + 1U) % SERVO_TX_QUEUE_DEPTH);
        s_servo_tx_count--;
    }

    Servo_ExitCritical(primask);

    Servo_TryStartTx();
}

/* 由 main.c 的 HAL_UART_ErrorCallback 转调：TX DMA 出错时清空整个队列。 */
void Servo_UartErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (huart != &huart3) {
        return;
    }

    primask = Servo_EnterCritical();

    if (s_servo_tx_active && (s_servo_tx_count > 0U) &&
        ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U) &&
        (huart->gState == HAL_UART_STATE_READY)) {
        s_servo_tx_active = 0U;
        s_servo_tx_head = s_servo_tx_tail;
        s_servo_tx_count = 0U;
    }

    Servo_ExitCritical(primask);
}

/* 周期调用（如 TIM2 中断里）：DMA 启动失败时重试队列。 */
void Servo_Process(void)
{
    if (!s_servo_tx_active && (s_servo_tx_count > 0U)) {
        Servo_TryStartTx();
    }
}

static HAL_StatusTypeDef Servo_SetPosition(uint16_t position)
{
    char command[SERVO_COMMAND_MAX_LEN];
    int length;

    length = snprintf(command,
                      sizeof(command),
                      "#000P%04uT%04u!",
                      (unsigned int)position,
                      (unsigned int)SERVO_BUS_MOVE_TIME_MS);
    if ((length <= 0) || ((size_t)length >= sizeof(command))) {
        return HAL_ERROR;
    }

    return Servo_SendString(command);
}

void Servo_SetAngle(uint16_t angle_deg)
{
    uint32_t pulse_us;

    if (angle_deg > SERVO_PWM_ANGLE_MAX_DEG) {
        angle_deg = SERVO_PWM_ANGLE_MAX_DEG;
    }

    pulse_us = SERVO_POSITION_MIN +
               ((uint32_t)angle_deg *
                (SERVO_POSITION_MAX - SERVO_POSITION_MIN) /
                SERVO_PWM_FULL_RANGE_DEG);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse_us);
}

void ServoBus_SetAngle(uint16_t angle_deg)
{
    uint16_t position;

    if (angle_deg > SERVO_BUS_ANGLE_MAX_DEG) {
        angle_deg = SERVO_BUS_ANGLE_MAX_DEG;
    }

    /* 0~360 degrees maps linearly to protocol position 0500~2500. */
    position = (uint16_t)(SERVO_POSITION_MIN +
               (((uint32_t)angle_deg *
                 (SERVO_POSITION_MAX - SERVO_POSITION_MIN) +
                 (SERVO_BUS_ANGLE_MAX_DEG / 2U)) /
                SERVO_BUS_ANGLE_MAX_DEG));

    (void)Servo_SetPosition(position);
}

void Servo_Init(void)
{
    #ifdef start_from_home
    Servo_SetAngle(0U);
    #endif

    #ifdef start_from_task1
    Servo_SetAngle(94U);
    #endif

    #ifdef start_from_task2
    Servo_SetAngle(94U);
    #endif

    (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    /* 等待舵机上电稳定后再发第一条指令，否则 ID 设置可能被忽略。 */
    HAL_Delay(SERVO_POWER_ON_DELAY_MS);

    /* Single-servo setup: force the connected servo to protocol ID 000. */
    if (Servo_SendString("#255PID000!") == HAL_OK) {
        HAL_Delay(SERVO_INIT_DELAY_MS);
    }
    ServoBus_SetAngle(SERVO_BUS_1);

    #ifdef start_from_task1
    HAL_Delay(1000);
    #endif
    #ifdef start_from_task2
    HAL_Delay(1000);
    #endif
}

void ServoBus_Test(void)
{
    ServoBus_SetAngle(SERVO_BUS_1);
    HAL_Delay(2000U);
    ServoBus_SetAngle(SERVO_BUS_2);
    HAL_Delay(2000U);
    ServoBus_SetAngle(SERVO_BUS_3);
    HAL_Delay(2000U);
    ServoBus_SetAngle(SERVO_BUS_4);
    HAL_Delay(2000U);
    ServoBus_SetAngle(SERVO_BUS_5);
    HAL_Delay(2000U);
}
