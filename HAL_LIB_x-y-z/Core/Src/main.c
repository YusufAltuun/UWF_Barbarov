/* USER CODE BEGIN Header */
#define BUFFER_SIZE 100
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

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
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

typedef struct UART_Buffer_Type{
	uint32_t buffer[BUFFER_SIZE];
	uint32_t head_pointer;
	uint32_t tail_pointer;
}UART_Buffer_t;

volatile UART_Buffer_t UART_BufferRX;


int stp_say=0;
unsigned int x_gelen;
char x_cikti[3];


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART4_IRQHandler(void) {
	uint32_t isrflags = UART4->SR;
	uint32_t control_reg1 = UART4->CR1;

	/* WIFI MODÜL UART in mode Receiver */

	if (((isrflags & USART_SR_RXNE) != RESET)
			&& ((control_reg1 & USART_CR1_RXNEIE) != RESET)) {
		UART_BufferRX.buffer[UART_BufferRX.head_pointer] = UART4->DR;
		UART_BufferRX.head_pointer = UART_BufferRX.head_pointer + 1;

		if (UART_BufferRX.head_pointer >= BUFFER_SIZE) {
			UART_BufferRX.head_pointer = 0;
		}
		return;
	}
	HAL_GPIO_TogglePin(GPIOD, led_3_Pin);
	HAL_Delay(200);
	/* USER CODE END UART4_IRQn 0 */
	HAL_UART_IRQHandler(&huart4);
	/* USER CODE BEGIN UART4_IRQn 1 */

	/* USER CODE END UART4_IRQn 1 */
}




void bt_veri_al_x (void)
{
if ((char)UART_BufferRX.buffer[UART_BufferRX.tail_pointer] == 'X')
		{
		for (int i = 0 ; i<stp_say ;i++)
			x_cikti[i] = UART_BufferRX.buffer[UART_BufferRX.tail_pointer + i + 1];
		}
	x_gelen = atoi(x_cikti);
}



bool STP_kontrol()
{
	for (int s=1; s<6; s++)
	{
		if ((char)UART_BufferRX.buffer[UART_BufferRX.tail_pointer + s] == 'S' || (char)UART_BufferRX.buffer[UART_BufferRX.tail_pointer + s+1] == 'T' || (char)UART_BufferRX.buffer[UART_BufferRX.tail_pointer + s+2] == 'P')
		{
			stp_say = s;
		return true;
		}
	}
	return false;
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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

	  HAL_GPIO_WritePin(GPIOD, led_3_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(GPIOD, led_3_Pin, GPIO_PIN_RESET);
	  HAL_Delay(200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (UART_BufferRX.tail_pointer >= BUFFER_SIZE) UART_BufferRX.tail_pointer = 0;
		if(abs(UART_BufferRX.head_pointer - UART_BufferRX.tail_pointer) > 20)
		{
			UART_BufferRX.tail_pointer++;
		}

		if((char)UART_BufferRX.buffer[UART_BufferRX.tail_pointer] == 'X' & STP_kontrol()==1)
		bt_veri_al_x();

		if (x_gelen==20){
		  HAL_GPIO_WritePin(GPIOD, led_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, led_2_Pin, GPIO_PIN_RESET);
		}
		if (x_gelen==50){
		  HAL_GPIO_WritePin(GPIOD, led_2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, led_1_Pin, GPIO_PIN_RESET);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, led_1_Pin|led_2_Pin|led_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_1_Pin led_2_Pin led_3_Pin */
  GPIO_InitStruct.Pin = led_1_Pin|led_2_Pin|led_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

