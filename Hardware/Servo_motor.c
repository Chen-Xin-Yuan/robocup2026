#include "Servo_motor.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define SERVO_POSITION_MIN            500U
#define SERVO_POSITION_MAX           2500U
#define SERVO_PWM_ANGLE_MAX_DEG        90U
#define SERVO_PWM_FULL_RANGE_DEG      180U
#define SERVO_BUS_ANGLE_MAX_DEG       360U
#define SERVO_BUS_MOVE_TIME_MS       1000U
#define SERVO_INIT_DELAY_MS           500U
#define SERVO_TX_TIMEOUT_MS           100U
#define SERVO_COMMAND_MAX_LEN          24U


#define SERVO_BUS_1          67U
#define SERVO_BUS_2          SERVO_BUS_1+72
#define SERVO_BUS_3          SERVO_BUS_2+72
#define SERVO_BUS_4          SERVO_BUS_3+72
#define SERVO_BUS_5          SERVO_BUS_4+72



static HAL_StatusTypeDef Servo_SendString(const char *command)
{
    size_t length;

    if (command == NULL) {
        return HAL_ERROR;
    }

    length = strlen(command);
    if ((length == 0U) || (length >= SERVO_COMMAND_MAX_LEN)) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(&huart3,
                             (uint8_t *)command,
                             (uint16_t)length,
                             SERVO_TX_TIMEOUT_MS);
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
    Servo_SetAngle(90U);
    (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    /* Single-servo setup: force the connected servo to protocol ID 000. */
    if (Servo_SendString("#255PID000!") == HAL_OK) {
        HAL_Delay(SERVO_INIT_DELAY_MS);
    }

    ServoBus_SetAngle(SERVO_BUS_1);
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
