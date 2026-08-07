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
#include <stdio.h>
#include <stdarg.h>




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t overall_task_state = 0;

uint16_t task1_state = 0;
uint16_t task2_state = 0;

float world_yaw = 0;
float body_vx = 0;
float body_vy = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TX_BUF_SIZE    256
uint8_t UART_DMA_Buf[TX_BUF_SIZE];

int fputc(int ch, FILE *stream)
{
    uint8_t c = ch;
    HAL_UART_Transmit(&huart2, &c, 1, 0xFFFF);
    return ch;
}

void uart_printf(const char *format, ...)
{
    va_list args;
    int len;

    /* This function runs from TIM2 IRQ: never wait for another IRQ here. */
    if ((format == NULL) || (huart2.gState != HAL_UART_STATE_READY)){
        return;    
    }

    va_start(args, format);
    len = vsnprintf((char*)UART_DMA_Buf, TX_BUF_SIZE, format, args);
    va_end(args);

    if (len <= 0) {
        return;
    }
    if (len >= TX_BUF_SIZE) {
        len = TX_BUF_SIZE - 1;
    }

    (void)HAL_UART_Transmit_DMA(&huart2, UART_DMA_Buf, (uint16_t)len);
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

  icm_init();  

	// HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);

	// HAL_Delay(2000);
	// HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);

  Chassis_Init(&huart1);
  Gray_Init();
  Servo_Init();

  // printf("System start\r\n");
  HAL_TIM_Base_Start_IT(&htim2);

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

    switch (overall_task_state){
      case 0://二维码识别阶段



        break;

      case 1://任务1阶段
        switch (task1_state)
        {
          case 11://循迹捡物块阶段
            
            break;
          
          case 12://捡完物块去找点

            break;

          default:
            break;
        }


        break;

      case 2://任务2阶段
        switch (task2_state)
        {
        case 21 :
          /* code */
          break;
        case 22 :
        
          break;

        default:

          break;
        }
        
        break;

      case 3://返程阶段

        break;

      default:

        break;

    }


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

		count1++;
		count2++;
		count3++;


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
    if(count3 >= 40)
    {
      // gray_show_digital();
      // uart_printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\r\n", 
      //   eulerAngle.yaw, eulerAngle.pitch, eulerAngle.roll);
     
      static uint16_t servo_count = 0U;
      static uint8_t servo_index = 0;
      static bool IS_zhuan = true;

      servo_count++;
      if (servo_count>=62 && IS_zhuan)
      {
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
      }

		count3 = 0;
    }
	}
}

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
