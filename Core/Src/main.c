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
#include "ZDTstepmotor.h"
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
    uint8_t status;
//	uint8_t a;
//	uint8_t b;
float vel;
float Motor_Vel;
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
    HAL_UART_Transmit(&huart6, &c, 1, 0xFFFF);
    return ch;
}

void uart_printf(const char *format, ...)
{
    va_list args;
    uint16_t len;

    while(huart6.gState != HAL_UART_STATE_READY);

    va_start(args, format);
    len = vsnprintf((char*)UART_DMA_Buf, TX_BUF_SIZE, format, args);
    va_end(args);

    HAL_UART_Transmit_DMA(&huart6, UART_DMA_Buf, len);
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  // HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
  // icm_init();

  __HAL_UART_CLEAR_IDLEFLAG(&huart1); 						
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 				
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxCmd, CMD_LEN); 



//	Chassis_Init(&huart1);

//	Chassis_Test();

  HAL_Delay(500);
  Emm_V5_Vel_Control(1, 0, 1000, 0, 0);
		while(rxFrameFlag == false);
		rxFrameFlag = false;

  HAL_Delay(500);
  
	Emm_V5_Read_Sys_Params(1, S_VEL);
		while(rxFrameFlag == false);
		rxFrameFlag = false;

	  if(rxCmd[0] == 1 && rxCmd[1] == 0x35 && rxCount == 6)
  {
    vel = (uint16_t)(
                      ((uint16_t)rxCmd[3] << 8)   |
                      ((uint16_t)rxCmd[4] << 0)
                    );


    Motor_Vel = vel;

   
    if(rxCmd[2]) { Motor_Vel = -Motor_Vel; }
  }
	// Servo_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    HAL_Delay(10);
		Emm_V5_Vel_Control(1, 0, 1000, 0, 0);
		while(rxFrameFlag == false);
		rxFrameFlag = false;

		HAL_Delay(10);
  
		Emm_V5_Read_Sys_Params(1, S_VEL);
		while(rxFrameFlag == false);
		rxFrameFlag = false;

		if(rxCmd[0] == 1 && rxCmd[1] == 0x35 && rxCount == 6)
		{
		vel = (uint16_t)(
                      ((uint16_t)rxCmd[3] << 8)   |
                      ((uint16_t)rxCmd[4] << 0)
                    );
		Motor_Vel = vel;   
    if(rxCmd[2]) { Motor_Vel = -Motor_Vel; }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 4;
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
		static uint16_t count1=0;
		static uint16_t count2=0;

		count1++;
		count2++;


		if(count1>=5)
		{
      // ICM42688_ReadAccel(&hi2c2, icm_accel);
      // ICM42688_ReadGyro(&hi2c2, icm_gyro);
      // ICM_getValues();
      // ICM_getEulerianAngles();
			count1=0;
		}		
    if(count2>=1000)
    {
//		a++;b++;
//		a%=120;b%=120;
//	uart_printf("a=%d\r\n", a);
	
//	printf("b=%d\r\n", b);


  	  // if(ICM42688_ReadAccel(&hi2c2, icm_accel) == HAL_OK)
		  // {
			//    printf("AX=%6d, AY=%6d, AZ=%6d\r\n", icm_accel[0], icm_accel[1], icm_accel[2]);      
		  // }

		  // if(ICM42688_ReadGyro(&hi2c2, icm_gyro) == HAL_OK)
		  // {
			//   printf("GX=%6d, GY=%6d, GZ=%6d\r\n", icm_gyro[0], icm_gyro[1], icm_gyro[2]);
		  // }
		printf("%.2f\r\n",Motor_Vel);
		count2=0;
    }
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		
	}
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
