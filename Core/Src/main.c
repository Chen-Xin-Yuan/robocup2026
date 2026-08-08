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
#define TASK1_TRACK_TIME_MS      10000U   /* 任务1循迹时长上限 */
#define K210_WAIT_QR_MS          15000U   /* 等K210二维码结果超时 */
#define K210_WAIT_CROSS_MS       10000U   /* 等K210十字对准超时 */
#define K210_STRAFE_LINE_MS      8000U    /* 向左找黑线超时 */


//控制
// #define debug  

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t overall_task_state = 1;

uint8_t QR_scanning_state = 11;
uint8_t task1_state = 0;
uint8_t task2_state = 0;
uint8_t qr_task1_number = 0;
uint8_t qr_task2_number = 0;

float world_yaw = 0;
float body_vx = 0;
float body_vy = 0;

float cmd_x, cmd_y, cmd_yaw;   /* 上位机命令: 需要纠正的 x/y/yaw */

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
}/* USER CODE END 0 */

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

  icm_init();  

	// HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);

	// HAL_Delay(2000);
	// HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);

  Chassis_Init(&huart1);
  Gray_Init();

  Align_RxInit();   /* 串口2接收中断：上位机命令帧（x/y/yaw + 完毕帧） */


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
    /* 上位机串口命令（帧头+帧尾带 x/y/yaw 纠正量）-> 位置纠正演示 */

#ifndef debug
    switch (overall_task_state){
      case 1://二维码识别阶段
        switch(QR_scanning_state)
        {
          case 11://开环移动到指定地方（从HOME出发，接近左侧二维码/黑线区域）
            Chassis_MovePosBlocking(-0.25f,0.0f,0.0f);
            HAL_Delay(1000);
         
            Chassis_MovePosBlocking(0.0f,0.65f,0.0f);
            HAL_Delay(1000);
            QR_scanning_state = 12;
            break;

          case 12:
            /* 1) 向K210发送消息，进入死等阶段（不收到消息卡死在这里） */
            // K210_Send("SCAN1\n");
            // {
            //   int num = K210_WaitNumber(K210_WAIT_QR_MS);
            //   if ((num < 1) || (num > 16)) {   /* 任务1二维码对应数字 1~16 */
            //     Chassis_stop();                /* 超时/无效：停住便于排查 */
            //     break;
            //   }
            //   qr_task1_number = (uint8_t)num;
            // }

            /* 2) 再向左移动到灰度中间两个(3,4)同时检测到黑线停止 */
            Chassis_StrafeLeftUntilLine(0.10f, K210_STRAFE_LINE_MS);
            Servo_SetAngle(96);
            HAL_Delay(1500);
            /* 3) 同时向K210发送消息，提示任务一开始 */
            K210_Send("GO\n");
            overall_task_state = 2;
            task1_state = 21;
            /* 4) 之后 overall_task_state 转移到任务1 */
            break;

          default:break;
        }
        break;

      case 2://任务1阶段
        switch (task1_state)
        {
          case 21://循迹捡物块阶段，示例写法见中断
          {
            // uint32_t end_tick = HAL_GetTick() + TASK1_TRACK_TIME_MS;
            // while ((int32_t)(HAL_GetTick() - end_tick) < 0) {
            //   /* 与TIM2中断里一样的循迹写法 */
            //   if (Grey_PID_Update() == 0U) {
            //     Chassis_TrackDifferential(0.2f, Grey_Get_Output());
            //   } else {
            //     Chassis_stop();   /* 丢线 */
            //     break;
            //   }
            //   HAL_Delay(10U);
            // }
            // Chassis_stop();
            // task1_state = 22;
          }
          break;

          case 22://捡完物块去找点
            /* 第一步：把角度旋转到 -90 度，用静态角度环 */
            HAL_Delay(500);
            Control_StaticTurn(-90.0f, 5000U);

            /* 第二步：动态角度环 + 对应延时到指定位置（速度 0.2 m/s，数值按场地标定） */
            /* 对比用：原来的定位置移动（阻塞式） */
            // Chassis_MovePosBlocking(0.0f, -0.60f, 0.0f);
            // Chassis_MovePosBlocking(-0.58f, 0.0f, 0.0f);

            Control_Init();
            
            Control_MoveHoldYaw(-0.2f, 0.0f, -90.0f, 0.60f, 0.2f);//vx
            Control_MoveHoldYaw(0.0f, 0.2f, -90.0f, 0.58f, 0.2f);

            /* 第三步：和K210通信，视觉闭环对准十字（对准后 yaw 自动纠正为 -90°） */
            // K210_Send("ALIGN\n");
            // (void)Control_AlignCross(K210_WAIT_CROSS_MS);
            // Control_AlignTest(2.1f, 5.2f, 2.1f);
            /* 进入上位机手动调整模式：等待纠正帧，收到完毕帧(AA 00 0A)后退出 */
            align_flag = 1U;
            Align_SendRequest();   /* 向上位机请求回传 x/y/yaw */
            while (align_flag)
            {
              if (Align_GetCorrection(&cmd_x, &cmd_y, &cmd_yaw) != 0U) {
                uart_printf("x=%.1fcm y=%.1fcm yaw=%.1fdeg\r\n",
                            (double)cmd_x, (double)cmd_y, (double)cmd_yaw);
                Control_AlignTest(cmd_x, cmd_y, cmd_yaw);
              }
              HAL_Delay(100);
            }
            Align_SendDone();   /* 调整完毕: 回传 ALIGN_DONE 给上位机确认 */
            /* 收到结束: 把当前 yaw 纠正为 -90°（消除 IMU 累计漂移） */
            Control_CorrectYawDeg(-90.0f);
            
            Chassis_MovePosBlocking(0.0f, -0.20f, 0.0f);
            ServoBus_SetAngle(Servo_angle[1]);
            HAL_Delay(500);
            /* 任务1完成 → 进入任务2（可按策略改成直接返程） */
            overall_task_state = 3;
            task2_state = 31;
            break;

          default:break;
        }
        break;

      case 3://任务2阶段
        switch (task2_state)
        {
          case 31://扫描任务2二维码（HOME右侧）
            K210_Send("SCAN2\n");
            {
              int num = K210_WaitNumber(K210_WAIT_QR_MS);
              if ((num < 1) || (num > 6)) {    /* 任务2二维码对应数字 1~6 */
                Chassis_stop();
                break;
              }
              qr_task2_number = (uint8_t)num;
            }
            task2_state = 32;
            break;

          case 32://按方案把 A/B/C 搬到领奖台（A→冠军、B→亚军、C→季军）
            /* TODO: 按你的机械结构写搬运流程，这里给占位：
               1) 定位置移动到任务2物料区
               2) 依次夹取 A/B/C（舵机动作）
               3) 按 qr_task2_number 的方案移动到领奖台放置 */
            Chassis_stop();
            overall_task_state = 4;   /* 搬运完成，返程 */
            break;

          default:break;
        }
        break;

      case 4://返程阶段
        /* 定位置移动回 HOME（数值按场地标定） */
        Chassis_MovePosBlocking(0.0f, -0.40f, 0.0f);
        Chassis_stop();
        overall_task_state = 0;   /* 结束 */
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
      // ICM42688_ReadAccel(&hi2c2, icm_accel);
      // ICM42688_ReadGyro(&hi2c2, icm_gyro);
      // ICM_getValues();
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
     
      static uint16_t servo_count = 0U;
      static uint8_t servo_index = 0;
      static bool IS_zhuan = true;

      if(overall_task_state == 2 && task1_state == 21)
      {
        servo_count++;
        #ifndef debug
        if (servo_count>=55 && IS_zhuan)
        {
        #endif
          servo_count = 0U;               
          servo_index ++;
          if (servo_index == 5)
          {
            IS_zhuan = false;       
          }
          ServoBus_SetAngle(Servo_angle[servo_index]);
        }

        if (Grey_PID_Update() == 0U && IS_zhuan)
        {
            Chassis_TrackDifferential(0.2f, Grey_Get_Output());
        } 
        else 
        {
            Chassis_stop();
            task1_state = 22;
        }

      #ifndef debug
      }
      #endif
		count3 = 0;
    }

    if(count4 >= 1000)
    {     
      // gray_show_digital();
      uart_printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\r\n", 
        eulerAngle.yaw, eulerAngle.pitch, eulerAngle.roll);

      count4 = 0;
    }

	}
}

//任何一个串口发送完成就会调用此函数，发送完成回调函数
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
