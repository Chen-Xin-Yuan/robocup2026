#include "gray.h"

#include <stdio.h>
#include <string.h>

#include "hardware_iic.h"
#include "usart.h"


#define GRAY_LOST_COUNT_MAX     5U
#define GRAY_NORMALIZE_WAIT_MS   10U
#define GRAY_DEBUG_BUFFER_SIZE   192U

/* bit0 在最左侧，bit7 在最右侧；安装方向相反时可颠倒此数组。 */
static const int8_t gray_sensor_weight[GRAY_SENSOR_NUM] = {
    -4, -3, -2, -1, 1, 2, 3, 4
};

Track_System track_sys;

static uint8_t gray_debug_buffer[GRAY_DEBUG_BUFFER_SIZE];
static volatile uint8_t gray_debug_requested;
static uint8_t gray_debug_state;
static uint32_t gray_normalize_start_tick;

void gray_read_binary(void)
{
    uint8_t index;

    track_sys.sensor_digital = IIC_Get_Digtal();

    for (index = 0U; index < GRAY_SENSOR_NUM; ++index) {
        track_sys.sensor_binary[index] =
            (uint8_t)((track_sys.sensor_digital >> index) & 0x01U);
    }
}

static void gray_debug_transmit(void)
{
    int length;

    if (huart2.gState != HAL_UART_STATE_READY) {
        return;
    }

    length = snprintf((char *)gray_debug_buffer,
                      GRAY_DEBUG_BUFFER_SIZE,
                      "Digital: %u-%u-%u-%u-%u-%u-%u-%u\r\n"
                      "Analog: %u-%u-%u-%u-%u-%u-%u-%u\r\n"
                      "Normalize: %u-%u-%u-%u-%u-%u-%u-%u\r\n",
                      (unsigned int)track_sys.sensor_binary[0],
                      (unsigned int)track_sys.sensor_binary[1],
                      (unsigned int)track_sys.sensor_binary[2],
                      (unsigned int)track_sys.sensor_binary[3],
                      (unsigned int)track_sys.sensor_binary[4],
                      (unsigned int)track_sys.sensor_binary[5],
                      (unsigned int)track_sys.sensor_binary[6],
                      (unsigned int)track_sys.sensor_binary[7],
                      (unsigned int)track_sys.sensor_analog[0],
                      (unsigned int)track_sys.sensor_analog[1],
                      (unsigned int)track_sys.sensor_analog[2],
                      (unsigned int)track_sys.sensor_analog[3],
                      (unsigned int)track_sys.sensor_analog[4],
                      (unsigned int)track_sys.sensor_analog[5],
                      (unsigned int)track_sys.sensor_analog[6],
                      (unsigned int)track_sys.sensor_analog[7],
                      (unsigned int)track_sys.sensor_normalized[0],
                      (unsigned int)track_sys.sensor_normalized[1],
                      (unsigned int)track_sys.sensor_normalized[2],
                      (unsigned int)track_sys.sensor_normalized[3],
                      (unsigned int)track_sys.sensor_normalized[4],
                      (unsigned int)track_sys.sensor_normalized[5],
                      (unsigned int)track_sys.sensor_normalized[6],
                      (unsigned int)track_sys.sensor_normalized[7]);

    if (length <= 0) {
        return;
    }
    if (length >= (int)GRAY_DEBUG_BUFFER_SIZE) {
        length = (int)GRAY_DEBUG_BUFFER_SIZE - 1;
    }

    (void)HAL_UART_Transmit_DMA(&huart2,
                                gray_debug_buffer,
                                (uint16_t)length);
}

static void gray_debug_transmit_error(void)
{
    static uint8_t error_message[] = "GRAY: I2C read failed\r\n";

    if (huart2.gState == HAL_UART_STATE_READY) {
        (void)HAL_UART_Transmit_DMA(&huart2,
                                    error_message,
                                    (uint16_t)(sizeof(error_message) - 1U));
    }
}

static float gray_calculate_position(void)
{
    int32_t weight_sum = 0;
    uint8_t black_count = 0U;
    uint8_t index;

    for (index = 0U; index < GRAY_SENSOR_NUM; ++index) {
        if (track_sys.sensor_binary[index] == 0U) {
            weight_sum += gray_sensor_weight[index];
            ++black_count;
        }
    }

    if (black_count == 0U) {
        if (track_sys.lost_count < UINT8_MAX) {
            ++track_sys.lost_count;
        }
        track_sys.track_lost =
            (track_sys.lost_count >= GRAY_LOST_COUNT_MAX) ? 1U : 0U;

        /* 短暂漏检时保持上次位置，避免控制量突然跳到零。 */
        return track_sys.line_position;
    }

    track_sys.lost_count = 0U;
    track_sys.track_lost = 0U;
    return (float)weight_sum / (float)black_count;
}

void Gray_Init(void)
{
    memset(&track_sys, 0, sizeof(track_sys));

    PID_Init(&track_sys.pid,
             15.0f,
             0.0f,
             0.0f,
             -200.0f,
             200.0f);
    PID_SetIntegralLimits(&track_sys.pid,
                          -100.0f,
                          100.0f);

    track_sys.target_position = 0.0f;
    track_sys.initialized = 1U;
    gray_debug_requested = 0U;
    gray_debug_state = 0U;
    gray_normalize_start_tick = 0U;
}

void gray_show_digital(void)
{
    gray_debug_requested = 1U;
}

void Gray_Process(void)
{
    uint32_t now;
    uint8_t read_ok;

    if (track_sys.initialized == 0U) {
        Gray_Init();
    }

    now = HAL_GetTick();

    if (gray_debug_state == 0U) {
        if (gray_debug_requested == 0U) {
            return;
        }
        gray_debug_requested = 0U;

        gray_read_binary();
        read_ok = IIC_Get_Anolog(track_sys.sensor_analog,
                                 GRAY_SENSOR_NUM);
        if ((read_ok == 0U) ||
            (IIC_Anolog_Normalize(GW_GRAY_ANALOG_CH_EN_ALL) == 0U)) {
            (void)IIC_Anolog_Normalize(0x00U);
            gray_debug_transmit_error();
            return;
        }

        gray_normalize_start_tick = now;
        gray_debug_state = 1U;
        return;
    }

    if ((uint32_t)(now - gray_normalize_start_tick) <
        GRAY_NORMALIZE_WAIT_MS) {
        return;
    }

    read_ok = IIC_Get_Anolog(track_sys.sensor_normalized,
                             GRAY_SENSOR_NUM);
    (void)IIC_Anolog_Normalize(0x00U);

    gray_debug_state = 0U;

    if (read_ok != 0U) {
        gray_debug_transmit();
    } else {
        gray_debug_transmit_error();
    }
}

uint8_t Grey_PID_Update(void)
{
    if (track_sys.initialized == 0U) {
        Gray_Init();
    }

    gray_read_binary();
    track_sys.line_position = gray_calculate_position();

    if (track_sys.track_lost != 0U) {
        PID_Reset(&track_sys.pid);
        track_sys.pid_output = 0.0f;
        return 1U;
    }

    track_sys.pid_output = PID_UpdateError(
        &track_sys.pid,
        track_sys.line_position - track_sys.target_position,
        GRAY_PID_PERIOD_MS);

    return 0U;
}

float Grey_Get_Output(void)
{
    return track_sys.pid_output;
}
