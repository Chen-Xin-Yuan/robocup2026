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
#define USART2_MAX_SEND_LEN        400
  
  
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
#define TASK1_TRACK_TIME_MS      20000U   /* 任务1循迹时长上限 */
#define K210_WAIT_QR_MS          18000U   /* 等K210二维码结果超时 */
#define K210_WAIT_CROSS_MS       18000U   /* 等K210十字对准超时 */
#define K210_STRAFE_LINE_MS      18000U    /* 向左找黑线超时 */
#define COLOR_RECOG_TIMEOUT_MS   28000U   /* 等上位机回传5个颜色超时 */
/* (对十字/对圆心已去掉超时，改为一直等上位机完毕帧) */


//控制



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef start_from_home
uint8_t overall_task_state = 1;

uint8_t QR_scanning_state = 11;
uint8_t task1_state = 0;
uint8_t task2_state = 0;
uint8_t qr_task1_number = 0;
uint8_t qr_task2_number = 0;

uint8_t home_flag = 0;
uint8_t gray_flag = 0;

float cmd_x, cmd_y, cmd_yaw;   /* 上位机命令: 需要纠正的 x/y/yaw */

uint8_t task1_point_index = 0;   
uint8_t task2_point_index = 0;   

uint8_t slot_colors[5];          /* 颜色识别结果: 槽1~5(左->右)的颜色编号 0=黑 1=白 2=红 3=绿 4=蓝 */
uint8_t plan1[5];                /* 任务1抓取方案: plan1[i] = 第 i 步去几号槽抓(1~5) */
uint8_t plan2[3];                /* 任务2抓取方案: plan2[i] = 第 i 步去几号槽抓(1~3) */
char task2_abc_order[4] = {0};   /* 任务2现场奖杯摆放(右->左)，如 "CBA"；需上位机/视觉填入或人工标定 */

/* 任务2 槽位/领奖台坐标(体坐标, m)——按场地标定后填写 */

uint8_t task1_color_started = 0U;     /* 循迹阶段是否已请求颜色识别 */
uint8_t task1_plan_ready = 0U;        /* plan1 是否已生成 */
uint8_t task2_plan_ready = 0U;        /* plan2 是否已生成 */
uint32_t task1_color_start_tick = 0U; /* 颜色识别请求发出时刻(超时用) */
uint8_t task2_track_stop = 0U;         /* 1=任务2循迹超时，停止循迹便于排查 */
uint8_t task2_letter_started = 0U;      /* 任务2循迹阶段是否已开始字母识别(收满a/b/c) */
uint8_t task2_letter_done_logged = 0U;   /* 是否已打印字母识别完成信息(避免刷屏) */
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

float cmd_x, cmd_y, cmd_yaw;   /* 上位机命令: 需要纠正的 x/y/yaw */

uint8_t task1_point_index = 0;   
uint8_t task2_point_index = 0;   

uint8_t slot_colors[5];          /* 颜色识别结果: 槽1~5(左->右)的颜色编号 0=黑 1=白 2=红 3=绿 4=蓝 */
uint8_t plan1[5];                /* 任务1抓取方案: plan1[i] = 第 i 步去几号槽抓(1~5) */
uint8_t plan2[3];                /* 任务2抓取方案: plan2[i] = 第 i 步去几号槽抓(1~3) */
char task2_abc_order[4] = {0};   /* 任务2现场奖杯摆放(右->左)，如 "CBA"；需上位机/视觉填入或人工标定 */

/* 任务2 槽位/领奖台坐标(体坐标, m)——按场地标定后填写 */

uint8_t task1_color_started = 0U;     /* 循迹阶段是否已请求颜色识别 */
uint8_t task1_plan_ready = 0U;        /* plan1 是否已生成 */
uint8_t task2_plan_ready = 0U;        /* plan2 是否已生成 */
uint32_t task1_color_start_tick = 0U; /* 颜色识别请求发出时刻(超时用) */
uint8_t task2_track_stop = 0U;         /* 1=任务2循迹超时，停止循迹便于排查 */
uint8_t task2_letter_started = 0U;      /* 任务2循迹阶段是否已开始字母识别(收满a/b/c) */
uint8_t task2_letter_done_logged = 0U;   /* 是否已打印字母识别完成信息(避免刷屏) */
#endif

#ifdef start_from_task2
uint8_t overall_task_state = 3;

uint8_t QR_scanning_state = 11;
uint8_t task1_state = 0;
uint8_t task2_state = 31;
uint8_t qr_task1_number = 1;
uint8_t qr_task2_number = 2;

uint8_t home_flag = 0;
uint8_t gray_flag = 0;

float cmd_x, cmd_y, cmd_yaw;   /* 上位机命令: 需要纠正的 x/y/yaw */

uint8_t task1_point_index = 0;   
uint8_t task2_point_index = 0;   

uint8_t slot_colors[5];          /* 颜色识别结果: 槽1~5(左->右)的颜色编号 0=黑 1=白 2=红 3=绿 4=蓝 */
uint8_t plan1[5];                /* 任务1抓取方案: plan1[i] = 第 i 步去几号槽抓(1~5) */
uint8_t plan2[3];                /* 任务2抓取方案: plan2[i] = 第 i 步去几号槽抓(1~3) */
char task2_abc_order[4] = {0};   /* 任务2现场奖杯摆放(右->左)，如 "CBA"；需上位机/视觉填入或人工标定 */

/* 任务2 槽位/领奖台坐标(体坐标, m)——按场地标定后填写 */

uint8_t task1_color_started = 0U;     /* 循迹阶段是否已请求颜色识别 */
uint8_t task1_plan_ready = 0U;        /* plan1 是否已生成 */
uint8_t task2_plan_ready = 0U;        /* plan2 是否已生成 */
uint32_t task1_color_start_tick = 0U; /* 颜色识别请求发出时刻(超时用) */
uint8_t task2_track_stop = 0U;         /* 1=任务2循迹超时，停止循迹便于排查 */
uint8_t task2_letter_started = 0U;      /* 任务2循迹阶段是否已开始字母识别(收满a/b/c) */
uint8_t task2_letter_done_logged = 0U;   /* 是否已打印字母识别完成信息(避免刷屏) */
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

/* 阻塞等待串口2二维码就绪，超时返回 0 */
static uint8_t WaitQRReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (QR_IsReady() == 0U) {
        if ((int32_t)(HAL_GetTick() - start) >= (int32_t)timeout_ms) {
            return 0U;
        }
        HAL_Delay(10U);
    }
    return 1U;
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

  Usart2_RxInit();   /* 串口2接收中断：上位机命令帧（x/y/yaw + 完毕帧） */


  // printf("System start\r\n");
  HAL_TIM_Base_Start_IT(&htim2);
  Servo_Init();
  // ServoBus_Test();
	// Chassis_Test();

  // Control_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifndef debug
    switch (overall_task_state){
      case 1://二维码识别阶段
        switch(QR_scanning_state)
        {
          case 11://开环移动到任务2二维码扫描位置，并扫描任务2二维码（两个二维码一起识别）
            Chassis_MovePosBlocking(-0.25f,0.0f,0.0f);
            HAL_Delay(1000);
            Chassis_MovePosBlocking(0.0f,-0.65f,0.0f);
            HAL_Delay(1000);

            /* 1) 通过串口2向上位机请求任务2二维码（编号1~6） */
            QR_Reset();
            QR_SendRequest();   /* AA 05 0A */
            if (WaitQRReady(K210_WAIT_QR_MS) == 0U) {
                uart_printf("QR2 timeout!\r\n");
                Chassis_stop();
                break;
            }
            qr_task2_number = QR_GetNumber();
            if ((qr_task2_number < 1U) || (qr_task2_number > 6U)) {
                uart_printf("QR2 invalid: %u\r\n", qr_task2_number);
                Chassis_stop();
                break;
            }
            QR_scanning_state = 12;
            break;

          case 12://开环移动到任务1二维码扫描位置，并扫描任务1二维码
            Chassis_MovePosBlocking(0.0f,1.3f,0.0f);
            HAL_Delay(1000);

            /* 1) 通过串口2向上位机请求任务1二维码（编号1~16） */
            QR_Reset();
            QR_SendRequest();   /* AA 05 0A */
            if (WaitQRReady(K210_WAIT_QR_MS) == 0U) {
                uart_printf("QR1 timeout!\r\n");
                Chassis_stop();                /* 超时/无效：停住便于排查 */
                break;
            }
            qr_task1_number = QR_GetNumber();
            if ((qr_task1_number < 1U) || (qr_task1_number > 16U)) {
                uart_printf("QR1 invalid: %u\r\n", qr_task1_number);
                Chassis_stop();
                break;
            }

            /* 2) 再向左移动到灰度中间两个(3,4)同时检测到黑线停止 */
            Chassis_StrafeLeftUntilLine(0.10f, K210_STRAFE_LINE_MS);
            Servo_SetAngle(96);
            HAL_Delay(1500);
            /* 3) 同时向K210发送消息，提示任务一开始 */
            K210_Send("GO\n");

            /* 4) 进入任务1: 颜色物块在循迹阶段(状态2/state21)识别，这里只做初始化 */
            task1_point_index = 0U;
            task1_color_started = 0U;
            task1_plan_ready = 0U;
            overall_task_state = 2;
            task1_state = 21;
            break;

          default:break;
        }
        break;

      case 2://任务1阶段
        switch (task1_state)
        {
          case 21://循迹捡物块阶段：循迹在 TIM2 中断里跑；主循环在这里收颜色、转舵机、生成方案
          {
            uint8_t i;

            /* 首次进入：清缓存，并发送首次颜色识别请求（请求第1个槽位颜色） */
            if (task1_color_started == 0U) {
                task1_color_started = 1U;
                task1_color_start_tick = HAL_GetTick();
                Color_Reset();
                Color_SendRequest();   /* AA 03 0A */
            }

            /* 收到 1 个颜色识别消息 -> 旋转舵机到下一槽位（槽位N+1 = Servo_angle[N]） */
            if (Color_TakeNew() != 0U) {
                if (Color_GetCount() < 5U) {   /* 收满5个颜色后不再转 */
                    ServoBus_SetAngle(Servo_angle[Color_GetCount()]);
                    Color_StartServoRotate();  /* 开始计时等待舵机到位 */
                }
            }

            /* 舵机旋转完成（固定延时）-> 发送下一次颜色识别请求 */
            Color_Process();//舵机旋转后，固定延时

            /* 收满5个不同颜色(重复自动丢弃) -> 用 二维码编号+槽位颜色 生成任务1搬运方案 plan1[] */
            if ((task1_plan_ready == 0U) && (Color_IsDone() != 0U)) {
                for (i = 0U; i < 5U; i++) {
                    slot_colors[i] = Color_GetColor(i);
                }
                if (Task1_QRPlanBySlotColors(qr_task1_number, slot_colors, plan1) != TASK_PLAN_OK) {
                    uart_printf("plan1 build failed! qr=%u\r\n", qr_task1_number);
                    Chassis_stop();
                    break;
                }
                task1_plan_ready = 1U;
                ServoBus_SetAngle(Servo_angle[5]);
                uart_printf("plan1 ready: slot%u->slot%u->slot%u->slot%u->slot%u\r\n",
                            plan1[0], plan1[1], plan1[2], plan1[3], plan1[4]);
            }

            /* 颜色识别超时保护：停住便于排查 */
            if ((task1_plan_ready == 0U)
                && ((int32_t)(HAL_GetTick() - task1_color_start_tick) >= (int32_t)COLOR_RECOG_TIMEOUT_MS)) {
                uart_printf("color recog timeout!\r\n");
                Chassis_stop();
                break;
            }
            /* 循迹继续由 TIM2 中断驱动；plan1 就绪且循迹计数到 250 后中断里切到 state 22 */
          }
          break;

          case 22://捡完物块，依次按方案 plan1 走5个槽位（每点: 固定距离移动 + 对十字/对圆心）
          {
            uint8_t slot;          /* 本步要去的槽位 1~5（由方案 plan1 决定） */

            /* ① 移动到当前槽位：5个点定距离移动；*/
            if (task1_point_index == 5U) {
                Chassis_MovePosBlocking(-0.20f, 0.0f, 0.0f);
                Chassis_MovePosBlocking(0.10f, 1.0f, 0.0f);
                task1_state = 23;
                Chassis_stop();
                break;
            } 
            else 
            {
                Control_StaticTurn(90.0f, 5000U);
                HAL_Delay(100);//每次启动前都旋转到固定角度
                Chassis_MovePosBlocking(task1_move_distance_X_m[task1_point_index],
                                        task1_move_distance_Y_m[task1_point_index],
                                        0.0f);
            }

            /* 方案保护：plan1 无效(未生成/越界)时停住排查 */
            slot = plan1[task1_point_index];
            if ((slot < 1U) || (slot > 5U)) {
                uart_printf("plan1[%u]=%u invalid!\r\n", task1_point_index, slot);
                Chassis_stop();
                break;
            }

            HAL_Delay(1000);
            Chassis_stop();

            /* ② 按 is_align[槽位] 决定本点动作 */
            if (task1_is_align[task1_point_index])
            {
                /* ---- 对十字（上位机回传 x/y/yaw） ---- */
                cross_flag = 1U;
                Cross_SendRequest();   /* AA 01 0A：请求回传 x/y/yaw */
                while (cross_flag)   /* 一直等上位机发完毕帧(AA 00 0A)才结束 */
                {
                  if (Cross_GetCorrection(&cmd_x, &cmd_y, &cmd_yaw) != 0U) {
                    uart_printf("x=%.1fcm y=%.1fcm yaw=%.1fdeg\r\n",
                                (double)cmd_x, (double)cmd_y, (double)cmd_yaw);
                    Control_AlignTest(cmd_x, cmd_y, cmd_yaw);
                    /* 阻塞式调整结束后，再向上位机请求下一次纠正 */
                    Cross_SendRequest();
                  }
                  HAL_Delay(200);
                }
                Cross_SendDone();   /* 调整完毕: 回传 ALIGN_DONE 给上位机确认 */
                Control_CorrectYawDeg(-90.0f);   /* 把当前 yaw 纠正为 -90° */

                /* ③ 对十字完成后的动作：后退 -> 舵机到固定位置 -> 延时 -> 后退 */
                Chassis_MovePosBlocking(0.0f, -0.22f, 0.0f);   /* 后退 */
                Chassis_stop();
                ServoBus_SetAngle(Servo_angle[slot - 1U]);     /* 舵机转到固定位置 */
                HAL_Delay(2000);
                Chassis_MovePosBlocking(0.0f, -0.10f, 0.0f);   /* 后退 */
                ServoBus_SetAngle(Servo_angle[5]);     /* 锁住其他物块 */
                HAL_Delay(1000);
                Chassis_MovePosBlocking(-0.10f, 0.0f, 0.0f);   
                
                Chassis_stop();
            } 
            else
            {
                /* ---- 对圆心（上位机只回传 x/y，无 yaw） ---- */
                circle_flag = 1U;
                Control_StaticTurn(90.0f, 5000U);

                Circle_SendRequest();   /* AA 02 0A：请求回传圆心 x/y */
                while (circle_flag)   /* 一直等上位机发完毕帧(AA 00 0A)才结束 */
                {
                  if (Circle_GetCorrection(&cmd_x, &cmd_y) != 0U) {
                    uart_printf("circle x=%.1fcm y=%.1fcm\r\n",
                                (double)cmd_x, (double)cmd_y);
                    Control_AlignTest(cmd_x, cmd_y, 0.0f);   /* 只平移 x/y，不纠 yaw */
                    /* 阻塞式调整结束后，再向上位机请求下一次纠正 */
                    Circle_SendRequest();
                  }
                  HAL_Delay(200);
                }
                Circle_SendDone();   /* 对圆心调整完毕 */

                /* ③ 对圆心完成后的动作：前进 -> 舵机到指定位置 -> 再后退5cm */
                Chassis_MovePosBlocking(0.0f, 0.05f, 0.0f);   /* 前进） */
                Chassis_stop();
                ServoBus_SetAngle(Servo_angle[slot - 1U]);     /* 舵机转到指定位置 */
                HAL_Delay(2000);               
                Chassis_MovePosBlocking(0.0f, -0.10f, 0.0f);   /* 后退 */
                ServoBus_SetAngle(Servo_angle[5]);     /* 锁住其他物块 */
                HAL_Delay(1000);
                if(task1_point_index == 0)
                {
                  Chassis_MovePosBlocking(0.20f, 0.0f, 0.0f);
                }
                Chassis_stop();
            }

            task1_point_index++;
            break;
          }

          case 23://5个点全部走完 -> 进入任务2
            HAL_Delay(200);
            Control_StaticTurn(-150.0f, 5000U);
            HAL_Delay(500);
            Chassis_StrafeLeftUntilLine(-0.10f, K210_STRAFE_LINE_MS);
            Chassis_stop();
            overall_task_state = 3;   /* 任务1完成 → 任务2 */
            task2_plan_ready = 0U;    /* 任务2计划复位，循迹阶段重新生成 */
            task2_track_stop = 0U;    /* 复位循迹超时停止标志 */
            task2_letter_started = 0U; /* 复位字母识别标志，进入循迹后重新开始 */
            task2_letter_done_logged = 0U; /* 复位字母完成打印标志 */
            task2_state = 31;
            break;

          default:break;
        }
        break;

      case 3://任务2阶段
        switch (task2_state)
        {
          case 31://任务2循迹阶段（循迹在 TIM2 中断里跑；plan2 就绪且循迹计数到后，中断里切到 state 32）
          {
            /* 任务2二维码已在初始扫描阶段提前识别，这里做校验 */
            if ((qr_task2_number < 1U) || (qr_task2_number > 6U)) {
                uart_printf("QR2 invalid: %u\r\n", qr_task2_number);
                task2_track_stop = 1U;
                Chassis_stop();
                break;
            }

            /* ---- 循迹阶段字母识别：逐槽位请求 + 舵机扫描（收满 a/b/c 即结束） ---- */
            /* 首次进入：清字母缓存，舵机回到槽位1，并发送第一次字母识别请求(AA 06 0A) */
            if (task2_letter_started == 0U) {
                task2_letter_started = 1U;
                Letter_Reset();
                ServoBus_SetAngle(Servo_angle[0]);   /* 舵机回槽位1（若已在则为空操作） */
                Letter_SendRequest();
            }

            /* 收到 1 个字母回传 -> 旋转舵机到下一角度（槽位N+1 = Servo_angle[N]） */
            if (Letter_TakeNew() != 0U) {
                if (Letter_GetCount() < 3U) {   /* 收满3个字母(a/b/c)后不再转 */
                    ServoBus_SetAngle(Servo_angle[Letter_GetCount()]);
                    Letter_StartServoRotate();  /* 开始计时等待舵机到位 */
                }
            }

            /* 舵机旋转完成后（固定延时200ms）-> 发送下一次字母识别请求 */
            Letter_Process();

            /* 收满3个字母(a/b/c) -> 识别结束，不再请求；循迹继续由 TIM2 中断驱动 */
            if ((Letter_IsDone() != 0U) && (task2_letter_done_logged == 0U)) {
                task2_letter_done_logged = 1U;   /* 只打印一次，避免刷屏 */
                uart_printf("letters recog done\r\n");
            }

            /* 字母识别收齐后，把 a/b/c 现场顺序填入 task2_abc_order（右->左）
             * 默认按“先收到的字母在前”填入；若小车先经过的是最左边的奖杯，
             * 把下面三行里的 [0] 和 [2] 对调即可 */
            if ((task2_plan_ready == 0U) && (task2_abc_order[0] == '\0')
                && (Letter_IsDone() != 0U)) {
                task2_abc_order[0] = (char)('A' + Letter_GetLetter(0U));
                task2_abc_order[1] = (char)('A' + Letter_GetLetter(1U));
                task2_abc_order[2] = (char)('A' + Letter_GetLetter(2U));
                task2_abc_order[3] = '\0';
                uart_printf("task2_abc_order=%s\r\n", task2_abc_order);
            }

            /* 生成 plan2：字母收齐即生成，不再要求外部预先提供顺序；
             * 生成后只检查 plan2 各值是否超出槽位范围 1~3 */
            if (task2_plan_ready == 0U) {
                if (task2_abc_order[0] == '\0') {
                    break;   /* 字母尚未收齐：继续循迹等待 */
                }
                if (Task2_QRPlan(qr_task2_number, task2_abc_order, plan2) != TASK_PLAN_OK) {
                    uart_printf("plan2 build failed! qr=%u order=%s\r\n",
                                qr_task2_number, task2_abc_order);
                    task2_track_stop = 1U;
                    Chassis_stop();
                    break;
                }
                /* 只校验 plan2 值是否超出 1~3 范围 */
                if ((plan2[0] < 1U) || (plan2[0] > 3U) ||
                    (plan2[1] < 1U) || (plan2[1] > 3U) ||
                    (plan2[2] < 1U) || (plan2[2] > 3U)) {
                    uart_printf("plan2 out of range: %u->%u->%u\r\n",
                                plan2[0], plan2[1], plan2[2]);
                    task2_track_stop = 1U;
                    Chassis_stop();
                    break;
                }
                task2_plan_ready = 1U;
                uart_printf("plan2 ready: slot%u->slot%u->slot%u\r\n",
                            plan2[0], plan2[1], plan2[2]);
            }

            /* 循迹继续由 TIM2 中断驱动；plan2 就绪且循迹计数到后中断里切到 state 32 */
            break;
          }

          case 32://按方案把 A/B/C 搬到领奖台（A→冠军、B→亚军、C→季军）
          {
            uint8_t i;
            uint8_t slot;      /* 本步去几号槽抓(1~3) */
            uint8_t trophy;    /* 本步奖杯 0=A 1=B 2=C */

            /* plan2 应在 31 循迹阶段生成（QR/奖杯顺序校验也在那里）；未就绪则停住排查 */
            if (task2_plan_ready == 0U) {
                uart_printf("plan2 not ready!\r\n");
                Chassis_stop();
                break;
            }

            /* 按方案依次搬运: 到槽位夹取 -> 放到对应领奖台 */
            uart_printf("grab slot order: %u->%u->%u\r\n",
                        plan2[0], plan2[1], plan2[2]);
            /*保护*/
            for (i = 0U; i < 3U; i++) {
                slot = plan2[i];
                if ((slot < 1U) || (slot > 3U)) 
                    uart_printf("plan2[%u]=%u invalid!\r\n", i, slot);
                    Chassis_stop();
                    break;
                }
                trophy = Task2_GetSchemeTrophy(qr_task2_number, i);   /* 本步奖杯 */
                if (trophy >= 3U) {
                    uart_printf("scheme trophy invalid!\r\n");
                    Chassis_stop();
                    break;
                }

            if (task2_point_index == 3U) {
                Chassis_MovePosBlocking(0.20f,0.0f,0.0f);
                Control_StaticTurn(20.0f, 5000U);
                Chassis_StrafeLeftUntilLine(-0.15f, K210_STRAFE_LINE_MS);
                Chassis_stop();//找到黑先后停下

                overall_task_state = 4;
                break;
            } 
            else if (task2_point_index == 0U)
            {
                Control_StaticTurn(0.0f, 5000U);
                HAL_Delay(100);
                Chassis_MovePosBlocking(0.0f,-0.05f,0.0f);
                Chassis_MovePosBlocking(task2_move_distance_X_m[task2_point_index],
                                        task2_move_distance_Y_m[task2_point_index],
                                        0.0f);
            }
            else if(task2_point_index == 1U)
            {
                Chassis_MovePosBlocking(0.35f, 0.0f, 0.0f);
                HAL_Delay(100);
                Control_StaticTurn(-90.0f, 5000U);
                HAL_Delay(100);
                Servo_SetAngle(80U);
                Chassis_MovePosBlocking(task2_move_distance_X_m[task2_point_index],
                                        task2_move_distance_Y_m[task2_point_index],
                                        0.0f);
                Chassis_stop();
            }
            else
            {
                HAL_Delay(100);
                Control_StaticTurn(-90.0f, 5000U);
                HAL_Delay(100);

                Chassis_MovePosBlocking(task2_move_distance_X_m[task2_point_index],
                                        task2_move_distance_Y_m[task2_point_index],
                                        0.0f);
                Chassis_MovePosBlocking(0.0f, 0.05f, 0.0f);                        
                Chassis_stop();
            }

            if (task2_is_align[task2_point_index])
            {
                /* ---- 对十字（上位机回传 x/y/yaw） ---- */
                cross_flag = 1U;
                Cross_SendRequest();   /* AA 01 0A：请求回传 x/y/yaw */
                while (cross_flag)   /* 一直等上位机发完毕帧(AA 00 0A)才结束 */
                {
                  if (Cross_GetCorrection(&cmd_x, &cmd_y, &cmd_yaw) != 0U) {
                    uart_printf("x=%.1fcm y=%.1fcm yaw=%.1fdeg\r\n",
                                (double)cmd_x, (double)cmd_y, (double)cmd_yaw);
                    Control_AlignTest(cmd_x, cmd_y, cmd_yaw);
                    /* 阻塞式调整结束后，再向上位机请求下一次纠正 */
                    Cross_SendRequest();
                  }
                  HAL_Delay(200);
                }
                Cross_SendDone();   /* 调整完毕: 回传 ALIGN_DONE 给上位机确认 */
                Control_CorrectYawDeg(-90.0f);   /* 把当前 yaw 纠正为 -90° */

                /* ③ 对十字完成后的动作：后退 -> 舵机到固定位置 -> 延时 -> 后退 */
                Chassis_MovePosBlocking(0.0f, -0.22f, 0.0f);   /* 后退 */
                Chassis_stop();
                ServoBus_SetAngle(Servo_angle[slot - 1U]);     /* 舵机转到固定位置 */
                HAL_Delay(2000);
                Chassis_MovePosBlocking(0.0f, -0.10f, 0.0f);   /* 后退 */
                ServoBus_SetAngle(Servo_angle[5]);     /* 锁住其他物块 */
                HAL_Delay(1000);
                Chassis_MovePosBlocking(-0.10f, 0.0f, 0.0f);   
                
                Chassis_stop();
            } 
            else
            {
                /* ---- 对圆心（上位机只回传 x/y，无 yaw） ---- */
                circle_flag = 1U;
                if(task2_point_index == 0U)
                {
                  Control_StaticTurn(0.0f, 5000U);
                }
                else
                {
                  Control_StaticTurn(-90.0f, 5000U);
                }
                Circle_SendRequest();   /* AA 02 0A：请求回传圆心 x/y */
                while (circle_flag)   /* 一直等上位机发完毕帧(AA 00 0A)才结束 */
                {
                  if (Circle_GetCorrection(&cmd_x, &cmd_y) != 0U) {
                    uart_printf("circle x=%.1fcm y=%.1fcm\r\n",
                                (double)cmd_x, (double)cmd_y);
                    Control_AlignTest(cmd_x, cmd_y, 0.0f);   /* 只平移 x/y，不纠 yaw */
                    /* 阻塞式调整结束后，再向上位机请求下一次纠正 */
                    Circle_SendRequest();
                  }
                  HAL_Delay(200);
                }
                Circle_SendDone();   /* 对圆心调整完毕 */

                /* ③ 对圆心完成后的动作：前进 -> 舵机到指定位置 -> 再后退5cm */
                Chassis_MovePosBlocking(0.0f, 0.05f, 0.0f);   /* 前进） */
                Chassis_stop();
                ServoBus_SetAngle(Servo_angle[slot - 1U]);     /* 舵机转到指定位置 */
                HAL_Delay(2000);               
                Chassis_MovePosBlocking(0.0f, -0.10f, 0.0f);   /* 后退 */
                ServoBus_SetAngle(Servo_angle[5]);     /* 锁住其他物块 */
                HAL_Delay(1000);                
                Chassis_stop();
            }
            task2_point_index++;
            break;
          }
          default:break;
        }
        break;

      case 4://返程阶段
        /* 定位置移动回 HOME（数值按场地标定） */
        while(home_flag == 0)
        {

        };//未检测到标志位就堵塞
        Chassis_stop();
        HAL_Delay(100);
        Servo_SetAngle(0U);
        Chassis_MovePosBlocking(-0.15f, 0.27f, 0.0f);
        Chassis_stop();
        overall_task_state = 0;
        task1_state = 0;   
        task2_state = 0;   /* 大成ohyes */
        break;

      default:break;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
	if(htim->Instance==htim2.Instance)
	{
    Chassis_Process();
    Servo_Process();

		static uint16_t count1=0;
		static uint16_t count2=0;
		static uint16_t count3=0;
		static uint16_t count4=0;

		count1++;
		count2++;
		count3++;
		count4++;


		if(count1>=5)
		{
      ICM_getEulerianAngles();

			count1 = 0;
		}		

    if(count2 >= CONTROL_TEST_LOOP_DELAY_MS)
    {
     
      // Control_AngleUpdate();
      // Control_AngleHoldMove(body_vx, body_vy, world_yaw);

      count2 = 0;
    }
    if(count3 >= GRAY_PID_PERIOD_MS)
    {
      if(overall_task_state == 2 && task1_state == 21)
      {
        gray_flag = Grey_PID_Update();

        if ((track_sys.sensor_binary[0] == 0U) &&
            (track_sys.sensor_binary[1] == 0U) &&
            (track_sys.sensor_binary[2] == 0U))
        {
            Chassis_stop();
            Kalman_SetYawDeg(-179.0f);
            if (task1_plan_ready != 0U) {
                task1_state = 22;
            }
        }
        else
        {
            if (gray_flag == 0U)
            {
                Chassis_TrackDifferential(0.2f, Grey_Get_Output());
            }
            else
            {
                Chassis_stop();   /* 丢线时停止，防止冲出 */
            }
        }
      }

      if(overall_task_state == 3 && task2_state == 31 && task2_track_stop == 0U)
      {
        gray_flag = Grey_PID_Update();

        if ((track_sys.sensor_binary[5] == 0U) &&
            (track_sys.sensor_binary[6] == 0U) &&
            (track_sys.sensor_binary[7] == 0U))
        {
            Chassis_stop();
            Kalman_SetYawDeg(-2.0f);

            if (task2_plan_ready != 0U) {
                task2_state = 32;
            }
        }
        else
        {
            if (gray_flag == 0U)
            {
                Chassis_TrackDifferential(0.2f, Grey_Get_Output());
            }
            else
            {
                Chassis_stop();   /* 丢线时停止，防止冲出 */
            }         
        }

      }

      if((overall_task_state == 4U) && (home_flag == 0))
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
            Chassis_TrackDifferential(0.4f, Grey_Get_Output());
        }
        else
        {
            Chassis_stop();   /* 丢线时停止，防止冲出 */
        }


      }

		count3 = 0;
    }

    if(count4 >= 1000)
    {     
      // gray_show_digital();
      // uart_printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\r\n", 
      //   eulerAngle.yaw, eulerAngle.pitch, eulerAngle.roll);
      uart_printf("%d, %d, %d\r\n",overall_task_state, task1_state, task2_state);

      count4 = 0;
    }

	}
}

//任何一个串口发送完成就会调用此函数，发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  Emm_V5_TxCpltCallback(huart);
  Servo_TxCpltCallback(huart);
  Usart2_TxCpltCallback(huart);
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

#ifdef  USE_FULL_ASSERT
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
