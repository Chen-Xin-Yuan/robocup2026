/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#define USART2_MAX_SEND_LEN 400

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Chassis.h"
#include "icm42688.h"
#include "kalman.h"
#include "Emm_V5.h"
#include "Servo_motor.h"
#include "MPU9250.h"
#include "all_init.h"
#include "ZDTstepmotor.h"
#include "control.h"
#include "gray.h"
#include "usart_sent.h"
#include "task.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK1_TRACK_TIME_MS 20000U /* 任务1循迹时长上限 */
#define K210_WAIT_QR_MS 15000U     /* 等K210二维码结果超时 */
#define K210_WAIT_CROSS_MS 18000U  /* 等K210十字对准超时 */
#define K210_STRAFE_LINE_MS 18000U /* 向左找黑线超时 */
/* (对十字/对圆心已去掉超时，改为一直等上位机完毕帧) */

#define TASK2_SERVO_UP_ANGLE 80U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* state31 收物块舵机状态（TIM2 中断用；进入 state31 生成 plan2 时复位） */
static uint16_t t2_servo_count = 0U;
static uint8_t t2_servo_index = 0U;
static uint8_t t2_servo_first_sent = 0U;

#ifdef start_from_home
uint8_t overall_task_state = 1;

uint8_t QR_scanning_state = 11;
uint8_t task1_state = 0;
uint8_t task2_state = 0;
uint8_t qr_task1_number = 0;
uint8_t qr_task2_number = 0;

uint8_t home_flag = 0;
uint8_t gray_flag = 0;

float cmd_x, cmd_y, cmd_yaw; /* 上位机命令: 需要纠正的 x/y/yaw */

uint8_t task1_point_index = 0;
uint8_t task2_point_index = 0;

uint8_t slot_colors[5];        /* 颜色识别结果: 槽1~5(左->右)的颜色编号 0=黑 1=白 2=红 3=绿 4=蓝 */
uint8_t plan1[5];              /* 任务1抓取方案: plan1[i] = 第 i 步去几号槽抓(1~5) */
uint8_t plan2[3];              /* 任务2抓取方案: plan2[i] = 第 i 步去几号槽抓(1~3) */
char task2_abc_order[4] = {0}; /* 任务2现场奖杯摆放(右->左)，如 "CBA"；需上位机/视觉填入或人工标定 */

/* 任务2 槽位/领奖台坐标(体坐标, m)——按场地标定后填写 */

uint8_t task1_color_started = 0U; /* 循迹阶段是否已请求颜色识别 */
uint8_t task1_plan_ready = 0U;    /* plan1 是否已生成 */
uint8_t task2_plan_ready = 0U;    /* plan2 是否已生成 */
uint8_t task2_track_stop = 0U;    /* 1=任务2循迹超时，停止循迹便于排查 */
#endif

#ifdef start_from_task1
uint8_t overall_task_state = 2;

uint8_t QR_scanning_state = 11;
uint8_t task1_state = 21;
uint8_t task2_state = 0;
uint8_t qr_task1_number = 1;
uint8_t qr_task2_number = 2;

uint8_t home_flag = 0;
uint8_t gray_flag = 0;

float cmd_x, cmd_y, cmd_yaw; /* 上位机命令: 需要纠正的 x/y/yaw */

uint8_t task1_point_index = 0;
uint8_t task2_point_index = 0;

uint8_t slot_colors[5];        /* 颜色识别结果: 槽1~5(左->右)的颜色编号 0=黑 1=白 2=红 3=绿 4=蓝 */
uint8_t plan1[5];              /* 任务1抓取方案: plan1[i] = 第 i 步去几号槽抓(1~5) */
uint8_t plan2[3];              /* 任务2抓取方案: plan2[i] = 第 i 步去几号槽抓(1~3) */
char task2_abc_order[4] = {0}; /* 任务2现场奖杯摆放(右->左)，如 "CBA"；需上位机/视觉填入或人工标定 */

/* 任务2 槽位/领奖台坐标(体坐标, m)——按场地标定后填写 */

uint8_t task1_color_started = 0U; /* 循迹阶段是否已请求颜色识别 */
uint8_t task1_plan_ready = 0U;    /* plan1 是否已生成 */
uint8_t task2_plan_ready = 0U;    /* plan2 是否已生成 */
uint8_t task2_track_stop = 0U;    /* 1=任务2循迹超时，停止循迹便于排查 */
#endif

#ifdef start_from_task2
uint8_t overall_task_state = 3;

uint8_t QR_scanning_state = 11;
uint8_t task1_state = 0;
uint8_t task2_state = 31;
uint8_t qr_task1_number = 1;
uint8_t qr_task2_number = 4;

uint8_t home_flag = 0;
uint8_t gray_flag = 0;

float cmd_x, cmd_y, cmd_yaw; /* 上位机命令: 需要纠正的 x/y/yaw */

uint8_t task1_point_index = 0;
uint8_t task2_point_index = 0;

uint8_t slot_colors[5];        /* 颜色识别结果: 槽1~5(左->右)的颜色编号 0=黑 1=白 2=红 3=绿 4=蓝 */
uint8_t plan1[5];              /* 任务1抓取方案: plan1[i] = 第 i 步去几号槽抓(1~5) */
uint8_t plan2[3];              /* 任务2抓取方案: plan2[i] = 第 i 步去几号槽抓(1~3) */
char task2_abc_order[4] = {0}; /* 任务2现场奖杯摆放(右->左)，如 "CBA"；需上位机/视觉填入或人工标定 */

/* 任务2 槽位/领奖台坐标(体坐标, m)——按场地标定后填写 */

uint8_t task1_color_started = 0U; /* 循迹阶段是否已请求颜色识别 */
uint8_t task1_plan_ready = 0U;    /* plan1 是否已生成 */
uint8_t task2_plan_ready = 0U;    /* plan2 是否已生成 */
uint8_t task2_track_stop = 0U;    /* 1=任务2循迹超时，停止循迹便于排查 */
#endif

#ifdef debug_in_round
uint8_t overall_task_state = 2;

uint8_t QR_scanning_state = 11;
uint8_t task1_state = 22;
uint8_t task2_state = 0;
uint8_t qr_task1_number = 1;
uint8_t qr_task2_number = 4;

uint8_t home_flag = 0;
uint8_t gray_flag = 0;

float cmd_x, cmd_y, cmd_yaw; /* 上位机命令: 需要纠正的 x/y/yaw */

uint8_t task1_point_index = 0;
uint8_t task2_point_index = 0;

uint8_t slot_colors[5]; /* 颜色识别结果: 槽1~5(左->右)的颜色编号 0=黑 1=白 2=红 3=绿 4=蓝 */
uint8_t plan1[5] = {
    1, 5, 2, 4, 3}; /* 任务1抓取方案: plan1[i] = 第 i 步去几号槽抓(1~5) */
uint8_t plan2[3];              /* 任务2抓取方案: plan2[i] = 第 i 步去几号槽抓(1~3) */
char task2_abc_order[4] = {0}; /* 任务2现场奖杯摆放(右->左)，如 "CBA"；需上位机/视觉填入或人工标定 */

/* 任务2 槽位/领奖台坐标(体坐标, m)——按场地标定后填写 */

uint8_t task1_color_started = 1U; /* 循迹阶段是否已请求颜色识别 */
uint8_t task1_plan_ready = 1U;    /* plan1 是否已生成 */
uint8_t task2_plan_ready = 0U;    /* plan2 是否已生成 */
uint8_t task2_track_stop = 0U;    /* 1=任务2循迹超时，停止循迹便于排查 */
#endif

#ifdef debug_in_color
uint8_t overall_task_state = 2;

uint8_t QR_scanning_state = 11;
uint8_t task1_state = 21;
uint8_t task2_state = 0;
uint8_t qr_task1_number = 1;
uint8_t qr_task2_number = 4;

uint8_t home_flag = 0;
uint8_t gray_flag = 0;

float cmd_x, cmd_y, cmd_yaw; /* 上位机命令: 需要纠正的 x/y/yaw */

uint8_t task1_point_index = 0;
uint8_t task2_point_index = 0;

uint8_t slot_colors[5]; /* 颜色识别结果: 槽1~5(左->右)的颜色编号 0=黑 1=白 2=红 3=绿 4=蓝 */
uint8_t plan1[5] = {
    1, 5, 2, 4, 3}; /* 任务1抓取方案: plan1[i] = 第 i 步去几号槽抓(1~5) */
uint8_t plan2[3];              /* 任务2抓取方案: plan2[i] = 第 i 步去几号槽抓(1~3) */
char task2_abc_order[4] = {0}; /* 任务2现场奖杯摆放(右->左)，如 "CBA"；需上位机/视觉填入或人工标定 */

/* 任务2 槽位/领奖台坐标(体坐标, m)——按场地标定后填写 */

uint8_t task1_color_started = 0U; /* 循迹阶段是否已请求颜色识别 */
uint8_t task1_plan_ready = 1U;    /* plan1 是否已生成 */
uint8_t task2_plan_ready = 0U;    /* plan2 是否已生成 */
uint8_t task2_track_stop = 0U;    /* 1=任务2循迹超时，停止循迹便于排查 */
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *stream)
{
  uint8_t c = ch;
  HAL_UART_Transmit(&huart2, &c, 1, 0xFFFF);
  return ch;
}

/* 二维码请求轮询：没收到就每隔固定时间重发一次 AA 05 0A，总时长不超过 timeout_ms。
 * 收到返回 1，超时返回 0（调用方用默认值兜底）。 */
static uint8_t WaitQRReady(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while ((int32_t)(HAL_GetTick() - start) < (int32_t)timeout_ms)
  {
    QR_SendRequest();                
    HAL_Delay(100U);
    if (QR_IsReady() != 0U)
    {
      return 1U;
    }
  }

  return QR_IsReady();       
}

/* 补充缺失槽位颜色：用红绿蓝黑白(0~4)里还没出现的颜色补上缺失槽位；
 * 4缺1时，缺失槽位正好补上唯一没出现的颜色。 */
static void Task1_FillSlotColors(uint8_t colors[5])
{
  uint8_t used = 0U;  /* 位图: bit c = 颜色 c 已出现 */
  uint8_t missing[5]; /* 还没出现的颜色(待补) */
  uint8_t n_missing = 0U;
  uint8_t fill = 0U;
  uint8_t i;
  uint8_t c;

  for (i = 0U; i < 5U; i++)
  {
    c = colors[i];
    if (c < 5U)
    {
      used |= (uint8_t)(1U << c);
    }
  }
  for (c = 0U; c < 5U; c++)
  {
    if ((used & (uint8_t)(1U << c)) == 0U)
    {
      missing[n_missing++] = c;
    }
  }
  for (i = 0U; i < 5U; i++)
  {
    if (colors[i] >= 5U)
    {
      if (fill < n_missing)
      {
        colors[i] = missing[fill++];
      }
      else
      {
        colors[i] = 0U; /* 兜底：黑 */
      }
    }
  }
}

/* 生成任务1方案：QR 未识别默认 3；颜色缺失自动补齐；保证 plan1 一定生成 */
static void Task1_BuildPlan1(void)
{
  if ((qr_task1_number < 1U) || (qr_task1_number > 16U))
  {
    uart_printf("qr_task1 invalid %u, use default 3\r\n", qr_task1_number);
    qr_task1_number = 3U;
  }
  // Task1_FillSlotColors(slot_colors);
  if (Task1_QRPlanBySlotColors(qr_task1_number, slot_colors, plan1) == TASK_PLAN_OK)
  {
    task1_plan_ready = 1U;
    uart_printf("plan1 ready: slot%u->slot%u->slot%u->slot%u->slot%u\r\n",
                plan1[0], plan1[1], plan1[2], plan1[3], plan1[4]);
  }
  else
  {
    /* 兜底默认方案：按槽位顺序 1->2->3->4->5 */
    plan1[0] = 1U;
    plan1[1] = 2U;
    plan1[2] = 3U;
    plan1[3] = 4U;
    plan1[4] = 5U;
    task1_plan_ready = 1U;
    uart_printf("plan1 use default: 1->2->3->4->5\r\n");
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 串口2开机自检：阻塞发送固定字符串（串口助手 115200 应看到 "UART2 OK"）
   * 看到 -> TX 通路/接线/波特率正常，问题在后面；没看到 -> 查接线/COM口/固件。
   * 排查完删掉这段即可。 */
  {
    static const uint8_t boot_msg[] = "UART2 OK\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)boot_msg, sizeof(boot_msg) - 1U, 200U);
  }

  icm_init();

  // HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);

  // HAL_Delay(2000);
  // HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);

  Chassis_Init(&huart1);
  Gray_Init();

  Usart6_RxInit(); /* 串口6接收中断：上位机(K230)命令帧（x/y/yaw + 完毕帧 + 二维码/颜色/字母） */

  // printf("System start\r\n");
  HAL_TIM_Base_Start_IT(&htim2);
  // ServoBus_Test();
  Servo_Init();
  // Chassis_Test();

  // Control_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    switch (overall_task_state)
    {
    case 1: // 二维码识别阶段
      switch (QR_scanning_state)
      {
      case 11: // 开环移动到任务2二维码扫描位置，并扫描任务2二维码（两个二维码一起识别）
        Chassis_MovePosBlocking(-0.25f, 0.0f, 0.0f);
        HAL_Delay(500);
        Chassis_MovePosBlocking_Set_V(0.0f, -0.70f, 0.0f, 0.3f);
        HAL_Delay(500);

        /* 1) 通过串口6向上位机(K230)请求任务2二维码（编号1~6）；未识别到默认 5 */
        QR_Reset();
        qr_task2_number = (WaitQRReady(K210_WAIT_QR_MS) != 0U) ? QR_GetNumber() : 5U;
        if ((qr_task2_number < 1U) || (qr_task2_number > 6U))
        {
          uart_printf("QR2 invalid %u, use default 5\r\n", qr_task2_number);
          qr_task2_number = 5U;
        }
        uart_printf("QR2 == %u\r\n", qr_task2_number);

        QR_scanning_state = 12;
        break;

      case 12: // 开环移动到任务1二维码扫描位置，并扫描任务1二维码
        Chassis_MovePosBlocking_Set_V(0.0f, 1.3f, 0.0f, 0.3f);
        HAL_Delay(500);

        /* 1) 通过串口6向上位机(K230)请求任务1二维码（编号1~16）；未识别到默认 3 */
        QR_Reset();
        qr_task1_number = (WaitQRReady(K210_WAIT_QR_MS) != 0U) ? QR_GetNumber() : 3U;
        if ((qr_task1_number < 1U) || (qr_task1_number > 16U))
        {
          uart_printf("QR1 invalid %u, use default 3\r\n", qr_task1_number);
          qr_task1_number = 3U;
        }
        uart_printf("QR1 == %u\r\n", qr_task1_number);
        // Chassis_MovePosBlocking(0.0f, -0.1f, 0.0f);
        /* 2) 再向左移动到灰度中间两个(3,4)同时检测到黑线停止 */
        Chassis_StrafeLeftUntilLine(0.20f, K210_STRAFE_LINE_MS);
        Chassis_MovePosBlocking(0.04f, 0.0f, 0.0f);
        Control_StaticTurn(-8.0f, 5000U);
        Servo_SetAngle(SERVO_TASK1_ANGLE);
        HAL_Delay(1000);
        /* 3) 同时向K210发送消息，提示任务一开始 */
        K210_Send("GO\n");

        /* 4) 进入任务1: 颜色物块在循迹阶段(状态2/state21)识别，这里只做初始化 */
        task1_point_index = 0U;
        task1_color_started = 0U;
        task1_plan_ready = 0U;
        overall_task_state = 2;
        task1_state = 21;
        break;

      default:
        break;
      }
      break;

    case 2: // 任务1阶段
      switch (task1_state)
      {
      case 21: // 循迹捡物块阶段：循迹在 TIM2 中断里跑；主循环在这里收颜色、转舵机、生成方案
      {
        uint8_t i;
        uint8_t ci;

        /* 首次进入：清缓存，并发送首次颜色识别请求（请求第1个槽位颜色） */
        if (task1_color_started == 0U)
        {
          task1_color_started = 1U;
          Color_Reset();
          Color_SendRequest(); /* AA 03 0A */
        }

        /* 每识别到一个新颜色，就在串口2打印一次（0=黑 1=白 2=红 3=绿 4=蓝） */
        if (Color_TakeNew() != 0U)
        {
          ci = (uint8_t)(Color_GetCount() - 1U);
          uart_printf("color[%u]: %u\r\n", (unsigned)ci, Color_GetColor(ci));
        }

        /* 每 COLOR_REQ_INTERVAL_MS 请求一次颜色，直到收满5个不同颜色 */
        Color_Process();

        /* 收满5个不同颜色(重复自动丢弃) -> 生成任务1搬运方案 plan1[]
         * QR未识别默认3、颜色缺失自动补齐，plan1 一定生成，不阻塞状态机 */
        if ((task1_plan_ready == 0U) && (Color_IsDone() != 0U))
        {
          for (i = 0U; i < 5U; i++)
          {
            slot_colors[i] = Color_GetColor(i);
          }
          Task1_BuildPlan1();
        }

        /* 循迹继续由 TIM2 中断驱动；plan1 就绪且循迹计数到 250 后中断里切到 state 22 */
      }
      break;

      case 22: // 捡完物块，依次按方案 plan1 走5个槽位（每点: 固定距离移动 + 对十字/对圆心）
      {
        uint8_t slot; /* 本步要去的槽位 1~5（由方案 plan1 决定） */

#ifndef debug_in_round
        /* 保底：plan1 未生成(颜色没收齐/QR未识别)时用默认补齐方案，保证状态机推进 */
        if (task1_plan_ready == 0U)
        {
          Task1_BuildPlan1();
        }

        /* ① 移动到当前槽位：5个点定距离移动；*/
        if (task1_point_index == 5U)
        {
          Control_StaticTurn(90.0f, 5000U);

          Chassis_MovePosBlocking_Set_V(0.20f, 0.0f, 0.0f, 0.5f);
          Chassis_MovePosBlocking_Set_V(0.0f, 0.9f, 0.0f, 0.6f);
          Chassis_MovePosBlocking_Set_V(-0.1f, 0.0f, 0.0f, 0.5f);
          task1_state = 23;
          Chassis_stop();
          break;
        }
        else
        {
          slot = plan1[task1_point_index]; /* 本步要去的槽位 1~5 */
          Control_StaticTurn(90.0f, 5000U);
          HAL_Delay(100);                            // 每次启动前都旋转到固定角度
          if(task1_point_index == 0U || task1_point_index == 1U || task1_point_index ==4U)
          {
            ServoBus_SetAngle(Servo_angle[slot - 1U]); /* 舵机转到指定位置 */
          }
          else 
          {
            if (plan1[task1_point_index] - plan1[task1_point_index - 1] > 3)
            {
              ServoBus_SetAngle((uint16_t)(Servo_angle[(plan1[task1_point_index - 1] - 1)] + 108U));
              HAL_Delay(200);
            }
            else if (plan1[task1_point_index] - plan1[task1_point_index - 1] < -3)
            {
              ServoBus_SetAngle((uint16_t)(Servo_angle[(plan1[task1_point_index - 1] - 1)] - 108U));
              HAL_Delay(2000);

            }
            else if (plan1[task1_point_index] - plan1[task1_point_index - 1] > 0)
            {
              ServoBus_SetAngle((uint16_t)(Servo_angle[(plan1[task1_point_index - 1] - 1)] + 36U));
              HAL_Delay(700);
            }
            else if (plan1[task1_point_index] - plan1[task1_point_index - 1] < 0)
            {
              ServoBus_SetAngle((uint16_t)(Servo_angle[(plan1[task1_point_index - 1] - 1)] - 36U));
              HAL_Delay(700);
            }
          }
          if(task1_point_index == 1U)
          {
            Chassis_MovePosBlocking(0.20f, 0.0f, 0.0f);
          }
          Chassis_MovePosBlocking(task1_move_distance_X_m[task1_point_index],
                                  task1_move_distance_Y_m[task1_point_index],
                                  0.0f);
        }     

/* 方案保护：plan1 无效(未生成/越界)时停住排查 */
#endif
        /* ② 按 is_align[槽位] 决定本点动作 */
        if (task1_is_align[task1_point_index])
        {
          /* ---- 对十字（上位机回传 x/y/yaw） ---- */
          cross_flag = 1U;
          Cross_SendRequest(); /* AA 01 0A：请求回传 x/y/yaw */
          while (cross_flag)   /* 一直等上位机发完毕帧(AA 00 0A)才结束 */
          {
            if (Cross_GetCorrection(&cmd_x, &cmd_y, &cmd_yaw) != 0U)
            {
              uart_printf("x=%.1fcm y=%.1fcm yaw=%.1fdeg\r\n",
                          (double)cmd_x, (double)cmd_y, (double)cmd_yaw);
              Control_AlignTest(cmd_x, cmd_y, cmd_yaw);
              /* 阻塞式调整结束后，再向上位机请求下一次纠正 */
              Cross_SendRequest();
            }
            HAL_Delay(200);
          }
          Cross_SendDone();
          Control_CorrectYawDeg(-90.0f); /* 把当前 yaw 纠正为 -90° */

          /* ③ 对十字完成后的动作：后退 -> 舵机到固定位置 -> 延时 -> 后退 */
          Chassis_MovePosBlocking(0.0f, -0.22f, 0.0f); /* 后退 */
          Chassis_stop();
          ServoBus_SetAngle(Servo_angle[slot - 1U]); /* 舵机转到固定位置 */
          HAL_Delay(2000);
          Chassis_MovePosBlocking(0.0f, -0.10f, 0.0f); /* 后退 */
          ServoBus_SetAngle(Servo_angle[5]);           /* 锁住其他物块 */
          HAL_Delay(1000);
          Chassis_MovePosBlocking(-0.10f, 0.0f, 0.0f);

          Chassis_stop();
        }
        else
        {
#ifndef debug_in_round
          /* ---- 对圆心（上位机只回传 x/y，无 yaw） ---- */
          Control_StaticTurn(90.0f, 5000U);
#endif
#ifdef CIRCLE_ALIGN_SPEED_MODE
/* 速度控制模式：上位机定时下发 x/y 速度，收到完成帧立刻停车 */
#ifdef debug_in_round
          Circle_AlignBySpeed(0.1f, 0.0f);
#else
          Circle_AlignBySpeed(0.1f, 90.0f);
#endif
          /* 位置控制模式（原有逻辑）：上位机回传 x/y 位置偏差 */
#ifndef CIRCLE_ALIGN_SPEED_MODE

          circle_flag = 1U;
          Circle_SendRequest(); /* AA 02 0A：请求回传圆心 x/y */
          while (circle_flag)   /* 一直等上位机发完毕帧(AA 00 0A)才结束 */
          {
            if (Circle_GetCorrection(&cmd_x, &cmd_y) != 0U)
            {
              uart_printf("circle x=%.1fcm y=%.1fcm\r\n",
                          (double)cmd_x, (double)cmd_y);
              Control_AlignTest(cmd_x, cmd_y, 0.0f); /* 只平移 x/y，不纠 yaw */
              /* 阻塞式调整结束后，再向上位机请求下一次纠正 */
              Circle_SendRequest();
            }
            HAL_Delay(200);
          }
          Circle_SendDone(); /* 对圆心调整完毕 */
#endif
#ifndef debug_in_round
          if(task1_point_index == 2U || task1_point_index == 3U)
          {
          ServoBus_SetAngle(Servo_angle[slot - 1U]); /* 舵机转到指定位置 */
          HAL_Delay(1500);
          }

          Chassis_MovePosBlocking_Set_V(0.002f, 0.05f, 0.0f, 0.2f); /* 前进） */
          Chassis_stop();
          Chassis_MovePosBlocking(0.0f, -0.10f, 0.0f); /* 后退 */


          task1_point_index++;
#endif
        }
        break;
      }

      case 23: // 5个点全部走完 -> 进入任务2
        HAL_Delay(100);
        Servo_SetAngle(SERVO_TASK2_ANGLE);
        ServoBus_SetAngle(Servo_angle[6]);
        Control_StaticTurn(-160.0f, 5000U);
        HAL_Delay(100);
        Chassis_StrafeLeftUntilLine(-0.30f, K210_STRAFE_LINE_MS);
        Chassis_stop();

        overall_task_state = 3; /* 任务1完成 → 任务2 */
        task2_plan_ready = 0U;  /* 任务2计划复位，循迹阶段重新生成 */
        task2_track_stop = 0U;  /* 复位循迹超时停止标志 */
        task2_state = 31;
        break;

      default:
        break;
      }
      break;

    case 3: // 任务2阶段
      switch (task2_state)
      {
      case 31: // 任务2循迹阶段（循迹+收物块在 TIM2 中断里跑）
      {
        /* 任务2二维码已在初始扫描阶段提前识别，这里做校验 */
        if ((qr_task2_number < 1U) || (qr_task2_number > 6U))
        {
          uart_printf("QR2 invalid: %u\r\n", qr_task2_number);
          task2_track_stop = 1U;
          Chassis_stop();
          break;
        }

        /* 槽位固定：Servo_angle[0]=C(季军)  Servo_angle[1]=A(冠军)  Servo_angle[2]=B(亚军)
         * 放置顺序固定：亚军(B) → 冠军(A) → 季军(C)
         * plan2[i] = 第 i 步要去的槽位号(1~3) */
        if (task2_plan_ready == 0U)
        {
          plan2[0] = 3U; /* B 亚军：Servo_angle[2] = 槽3 */
          plan2[1] = 2U; /* A 冠军：Servo_angle[1] = 槽2 */
          plan2[2] = 1U; /* C 季军：Servo_angle[0] = 槽1 */
          task2_plan_ready = 1U;
          task2_point_index = 0U;
          /* 复位 state31 收物块舵机状态：进入 31 立即转第 1 个字母槽位 */
          t2_servo_count = 0U;
          t2_servo_index = 0U;
          t2_servo_first_sent = 0U;
          uart_printf("plan2 ready(B->A->C): %u->%u->%u\r\n",
                      plan2[0], plan2[1], plan2[2]);
        }

        /* 循迹+收物块由 TIM2 中断驱动；走到线尾后由中断处理 */
        break;
      }

      case 32: // 按 plan2 顺序搬运: 亚军(B) → 冠军(A) → 季军(C)
      {
        uint8_t slot; /* 本步去几号槽抓(1~3) */

        /* plan2 已在 31 循迹阶段由二维码生成；未就绪则停住排查 */
        if (task2_plan_ready == 0U)
        {
          uart_printf("plan2 not ready!\r\n");
          Chassis_stop();
          break;
        }

        /* 三块都放完 -> 回线返程（先判断，避免 plan2[3] 越界） */
        if (task2_point_index == 3U)
        {
          Servo_SetAngle(10U);
          Chassis_MovePosBlocking_Set_V(0.20f, 0.0f, 0.0f, 0.45f);
          Control_StaticTurn(20.0f, 5000U);
          Chassis_StrafeLeftUntilLine(-0.30f, K210_STRAFE_LINE_MS);
          Chassis_stop(); // 找到黑先后停下

          overall_task_state = 4;
          break;
        }

        /* 保护：本步槽位超出 1~3 则停住排查 */
        slot = plan2[task2_point_index];
        if ((slot < 1U) || (slot > 3U))
        {
          uart_printf("plan2[%u]=%u invalid!\r\n", task2_point_index, slot);
          Chassis_stop();
          break;
        }
        if (task2_point_index == 0U)
        {
          Control_StaticTurn(0.0f, 5000U);
          HAL_Delay(100);
          #ifndef TWO_IS_ROUND
          Chassis_MovePosBlocking_Set_V(0.0f, -0.05f, 0.0f, 0.2f);
          #else
          Chassis_MovePosBlocking_Set_V(0.0f, -0.11f, 0.0f, 0.2f);
          #endif
          Chassis_MovePosBlocking(task2_move_distance_X_m[task2_point_index],
                                  task2_move_distance_Y_m[task2_point_index],
                                  0.0f);
          Chassis_stop();
          
          Control_StaticTurn(0.0f, 5000U);
          #ifndef TWO_IS_ROUND
          Chassis_MovePosBlocking_Set_V(0.0f, 0.05f, 0.0f, 0.2f); /* 前进 */
          #endif
          Servo_SetAngle(88);//缓冲角度
          #ifdef TWO_IS_ROUND
          HAL_Delay(500);
          #endif

          
          ServoBus_SetAngle(Servo_angle[slot - 1U]); /* 舵机转到指定位置 */
          HAL_Delay(1000);
     
          Servo_SetAngle(TWO_ANGLE);
          HAL_Delay(500);
          
          #ifndef TWO_IS_ROUND
          Chassis_MovePosBlocking_Set_V(0.0f, -0.10f, 0.0f, 0.2f);
          #endif

          // #ifdef TWO_IS_ROUND
          // ServoBus_SetAngle(Servo_angle[slot - 1U]); /* 舵机转到指定位置 */
          // HAL_Delay(1000);
          // #endif /* 后退 */

          #ifndef TWO_IS_ROUND
          ServoBus_SetAngle(Servo_angle[6]);           /* 锁住其他物块 */
          HAL_Delay(1000);
          Chassis_stop();
          Servo_SetAngle(75);//稍高一点
          #endif
        }
        else if (task2_point_index == 1U)
        {
          Chassis_MovePosBlocking(0.35f, 0.0f, 0.0f);
          HAL_Delay(100);
          Control_StaticTurn(-90.0f, 5000U);
          HAL_Delay(100);
          Chassis_MovePosBlocking(task2_move_distance_X_m[task2_point_index],
                                  task2_move_distance_Y_m[task2_point_index],
                                  0.0f);
          Chassis_stop();
          Servo_SetAngle(83);
          Control_StaticTurn(-90.0f, 5000U);
          ServoBus_SetAngle(Servo_angle[slot - 1U]);  /* 舵机转到指定位置 */
          Chassis_MovePosBlocking_Set_V(0.0f, 0.04f, 0.0f, 0.15f); /* 前进 */
          HAL_Delay(1000);
          Servo_SetAngle(ONE_ANGLE);
          Chassis_MovePosBlocking_Set_V(0.0f, -0.10f, 0.0f, 0.2f); /* 后退 */
          ServoBus_SetAngle(Servo_angle[6]);           /* 锁住其他物块 */
          HAL_Delay(1500);
          Chassis_stop();
        }
        else
        {
          Control_StaticTurn(-90.0f, 5000U);
          Chassis_MovePosBlocking(task2_move_distance_X_m[task2_point_index],
                                  task2_move_distance_Y_m[task2_point_index],
                                  0.0f);
          Servo_SetAngle(98);
          HAL_Delay(500);          
          ServoBus_SetAngle(Servo_angle[slot - 1U]);  /* 舵机转到指定位置 */
          HAL_Delay(500);
          Chassis_MovePosBlocking_Set_V(0.0f, 0.05f, 0.0f, 0.2f);

          Chassis_stop();
        }

        if (task2_is_align[task2_point_index] == 1)
        {
          /* ---- 对十字（上位机回传 x/y/yaw） ---- */
          cross_flag = 1U;
          Cross_SendRequest(); /* AA 01 0A：请求回传 x/y/yaw */
          while (cross_flag)   /* 一直等上位机发完毕帧(AA 00 0A)才结束 */
          {
            if (Cross_GetCorrection(&cmd_x, &cmd_y, &cmd_yaw) != 0U)
            {
              uart_printf("x=%.1fcm y=%.1fcm yaw=%.1fdeg\r\n",
                          (double)cmd_x, (double)cmd_y, (double)cmd_yaw);
              Control_AlignTest(cmd_x, cmd_y, cmd_yaw);
              /* 阻塞式调整结束后，再向上位机请求下一次纠正 */
              Cross_SendRequest();
            }
            HAL_Delay(200);
          }
          Cross_SendDone();              /* 调整完毕: 回传 ALIGN_DONE 给上位机确认 */
          Control_CorrectYawDeg(-90.0f); /* 把当前 yaw 纠正为 -90° */

          /* ③ 对十字完成后的动作：后退 -> 舵机到固定位置 -> 延时 -> 后退 */
          Chassis_MovePosBlocking(0.0f, -0.22f, 0.0f); /* 后退 */
          Chassis_stop();
          ServoBus_SetAngle(Servo_angle[slot - 1U]); /* 舵机转到固定位置 */

          HAL_Delay(1500);
          Servo_SetAngle(SERVO_TASK2_ANGLE);
          HAL_Delay(500);
          Chassis_MovePosBlocking(0.0f, -0.10f, 0.0f); /* 后退 */
          ServoBus_SetAngle(Servo_angle[6]);           /* 锁住其他物块 */
          HAL_Delay(1000);
          Chassis_MovePosBlocking(-0.10f, 0.0f, 0.0f);

          Chassis_stop();
        }
        else if (task2_is_align[task2_point_index] == 2)
        {
          /* ---- 对圆心（上位机只回传 x/y，无 yaw） ---- */
          if (task2_point_index == 0U)
          {
            Control_StaticTurn(0.0f, 5000U);
          }
          else
          {
            Control_StaticTurn(-90.0f, 5000U);
          }

#ifdef CIRCLE_ALIGN_SPEED_MODE
          /* 速度控制模式：上位机定时下发 x/y 速度，收到完成帧立刻停车 */
          /* 限幅，保持与上面 Control_StaticTurn 一致的航向 */
          Circle_AlignBySpeed(0.1f,
                              (task2_point_index == 0U) ? 0.0f : -90.0f);
#else
          /* 位置控制模式（原有逻辑）：上位机回传 x/y 位置偏差 */
          circle_flag = 1U;
          Circle_SendRequest(); /* AA 02 0A：请求回传圆心 x/y */
          while (circle_flag)   /* 一直等上位机发完毕帧(AA 00 0A)才结束 */
          {
            if (Circle_GetCorrection(&cmd_x, &cmd_y) != 0U)
            {
              uart_printf("circle x=%.1fcm y=%.1fcm\r\n",
                          (double)cmd_x, (double)cmd_y);
              Control_AlignTest(cmd_x, cmd_y, 0.0f); /* 只平移 x/y，不纠 yaw */
              /* 阻塞式调整结束后，再向上位机请求下一次纠正 */
              Circle_SendRequest();
            }
            HAL_Delay(200);
          }
          Circle_SendDone(); /* 对圆心调整完毕 */
#endif

          /* ③ 对圆心完成后的动作：前进 -> 舵机到指定位置 -> 再后退5cm */
          // Servo_SetAngle(98); /* 舵机转到指定位置 */
          
          if (task2_point_index == 0U)
          {
            Servo_SetAngle(90);
            HAL_Delay(1000);
            Chassis_MovePosBlocking_Set_V(0.0f, 0.05f, 0.0f, 0.2f); /* 前进 */
            Servo_SetAngle(TWO_ANGLE);
            HAL_Delay(1000);   
          }
          else
          {
            Chassis_MovePosBlocking_Set_V(0.0f, 0.045f, 0.0f, 0.2f); /* 前进 */   
          }
          Chassis_MovePosBlocking_Set_V(0.0f, -0.10f, 0.0f, 0.2f); /* 后退 */
          Chassis_stop();
          if (task2_point_index == 0U)
          {
            Servo_SetAngle(75);
          }
          HAL_Delay(500);
 
          // ServoBus_SetAngle(Servo_angle[5]);           /* 锁住其他物块 */
          // HAL_Delay(1000);
      Chassis_stop();
        }
        else
        {
          // skip
        }
        task2_point_index++;
        break;
      }
      default:
        break;
      }
      break;

    case 4: // 返程阶段
      /* 定位置移动回 HOME（数值按场地标定） */
      while (home_flag == 0)
      {

      }; // 未检测到标志位就堵塞
      Chassis_stop();
      HAL_Delay(100);

      Chassis_MovePosBlocking(-0.17f, 0.24f, 0.0f);
      Chassis_stop();
      overall_task_state = 0;
      task1_state = 0;
      task2_state = 0; /* 大成ohyes */
      break;

    default:
      break;
    }
#endif
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim2.Instance)
  {
    Chassis_Process();
    Servo_Process();

    static uint16_t count1 = 0;

    static uint16_t count3 = 0;
    static uint16_t count4 = 0;

    count1++;

    count3++;
    count4++;

    if (count1 >= 5)
    {
      ICM_getEulerianAngles();
      count1 = 0;
    }
    if (count3 >= GRAY_PID_PERIOD_MS)
    {
      if (overall_task_state == 2 && task1_state == 21)
      {
        static uint16_t t1_servo_count = 0; /* 自动拾取计数(每100个循迹周期转一次) */
        static uint8_t t1_servo_index = 1;  /* 当前已对准的槽位 0~4 */

        gray_flag = Grey_PID_Update();

        if ((track_sys.sensor_binary[0] == 0U) &&
            (track_sys.sensor_binary[1] == 0U) &&
            (track_sys.sensor_binary[2] == 0U))
        {
          Chassis_stop();
          Kalman_SetYawDeg(-179.2f);
/* 循迹到终点：推进状态机（plan1 未生成时由主循环/state22 用默认方案补齐） */
#ifndef debug_in_color
          task1_state = 22;
#endif
        }

        else
        {
          if (gray_flag == 0U)
          {
#ifndef debug_in_color
            Chassis_TrackDifferential(0.4f, Grey_Get_Output());
#endif
            t1_servo_count++;
#ifdef start_from_task1
            if (t1_servo_index == 1U)
            {
              if (t1_servo_count >= 170U)
              {
                t1_servo_count = 0U;
                ServoBus_SetAngle(Servo_angle[t1_servo_index]);
                t1_servo_index++;
              }
            }

            else 
#endif      
            if (t1_servo_count >= 64U && t1_servo_index < 6U)
            {
              t1_servo_count = 0U;
              ServoBus_SetAngle(Servo_angle[t1_servo_index]);
              t1_servo_index++;
            }
          }
          else
          {
            Chassis_stop(); /* 丢线时停止，防止冲出 */
          }
        }
      }

      if (overall_task_state == 3 && task2_state == 31 && task2_track_stop == 0U)
      {
        gray_flag = Grey_PID_Update();

        if ((track_sys.sensor_binary[5] == 0U) &&
            (track_sys.sensor_binary[6] == 0U) &&
            (track_sys.sensor_binary[7] == 0U))
        {
          Chassis_stop();
          Kalman_SetYawDeg(-2.0f);

          if (task2_plan_ready != 0U)
          {
            task2_state = 32;
          }
        }
        else
        {
          if (gray_flag == 0U)
          {
            static float task2_track_speed = 0.2f;
            Chassis_TrackDifferential(task2_track_speed, Grey_Get_Output());
            t2_servo_count++;
            if (t2_servo_index < 3U)
            {
              /* 第 t2_servo_index 次要接的字母 = task2_scheme[qr-1][t2_servo_index]
               * 固定槽位：A->Servo_angle[1]  B->Servo_angle[2]  C->Servo_angle[0]
               * 节奏：刚进入 state31 立即转第 1 个字母槽位，之后每 240 个循迹周期转下一个 */
              uint8_t letter;
              uint8_t angle_idx;

              if (t2_servo_first_sent == 0U)
              {
                /* 刚进入 31：立即转到第 1 个字母对应的槽位 */
                letter = Task2_GetSchemeTrophy(qr_task2_number, 0U);
                if (letter == 0U)
                  angle_idx = 1U; /* A -> Servo_angle[1] */
                else if (letter == 1U)
                  angle_idx = 2U; /* B -> Servo_angle[2] */
                else
                  angle_idx = 0U; /* C -> Servo_angle[0] */
                ServoBus_SetAngle(Servo_angle[angle_idx]);
                t2_servo_first_sent = 1U;
                t2_servo_index = 1U;
                t2_servo_count = 0U;
              }
              else if (t2_servo_count >= 120U)
              {
                letter = Task2_GetSchemeTrophy(qr_task2_number, t2_servo_index);
                if (letter == 0U)
                  angle_idx = 1U; /* A -> Servo_angle[1] */
                else if (letter == 1U)
                  angle_idx = 2U; /* B -> Servo_angle[2] */
                else
                  angle_idx = 0U; /* C -> Servo_angle[0] */
                t2_servo_index++;
                ServoBus_SetAngle(Servo_angle[angle_idx]);
                t2_servo_count = 0U;
              }
            }
            else if (t2_servo_index == 3U && t2_servo_count >= 160U)
            {
              t2_servo_index++;
              ServoBus_SetAngle(Servo_angle[6]); /* 锁住 */
              task2_track_speed = 0.25f;
            }
            else if (t2_servo_index == 4U && t2_servo_count >= 100U)
            {
              t2_servo_index++;
              Servo_SetAngle(TASK2_SERVO_UP_ANGLE); /* 举高 */
              t2_servo_count = 0U;
            }
          }
          else
          {
            Chassis_stop(); /* 丢线时停止，防止冲出 */
          }
        }
      }

      if ((overall_task_state == 4U) && (home_flag == 0))
      {
        gray_flag = Grey_PID_Update();

        if ((track_sys.sensor_binary[5] == 0U) &&
            (track_sys.sensor_binary[6] == 0U) &&
            (track_sys.sensor_binary[7] == 0U))
        {
          Chassis_stop();
          home_flag = 1;
        }
        if (gray_flag == 0U)
        {
          Chassis_TrackDifferential(0.7f, Grey_Get_Output());
        }
        else
        {
          Chassis_stop(); /* 丢线时停止，防止冲出 */
        }
      }

      count3 = 0;
    }

    if (count4 >= 1000)
    {
      // gray_show_digital();
      // uart_printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\r\n",
      //   eulerAngle.yaw, eulerAngle.pitch, eulerAngle.roll);
      // uart_printf("%d, %d, %d\r\n",overall_task_state, task1_state, task2_state);

      count4 = 0;
    }
  }
}

// 任何一个串口发送完成就会调用此函数，发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  Emm_V5_TxCpltCallback(huart);
  Servo_TxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* TX DMA errors release the queued frame; USART1 has no RX path. */
  Emm_V5_UartErrorCallback(huart);
  Servo_UartErrorCallback(huart);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
