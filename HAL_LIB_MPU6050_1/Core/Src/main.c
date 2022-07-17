/* USER CODE BEGIN Header */
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
	#include "stdio.h"
	#include "math.h"
	#include "string.h"
	#include "stdlib.h"
	#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mpu6050_slave_adress 0x68<<1
#define power_management_1_register 0x6B
#define gyro_config_register 0x1B
#define accelerometer_config_register 0x1C
#define smplrt_div_reg 0x19
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t data;
uint8_t buffer[2], tuffer[6], cuffer[6];
int16_t gyro_raw[3],acc_raw[3];
float gyro_cal[3];
int16_t acc_total_vector;
float angle_pitch_gyro, angle_roll_gyro;
float angle_pitch_acc, angle_roll_acc;
float angle_pitch, angle_roll;
int16_t raw_temp;
float temp;
int i, mapL, mapR;
float prev_time, prev_time1, time1, elapsed_time1, prev_time2, time2, elapsed_time2;
float xi, yi, zi, xj, yj, zj;
HAL_StatusTypeDef set_gyro;
uint8_t buffrec[5],total ;
int donusum1,donusum2; // değeri 2^x olarak değerlendiriği için saatlerdir ağlamaktatım :))))))) ^^
char gelen[5], *deneme;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t map(uint32_t sens_giris, uint32_t gir_min, uint32_t gir_max, uint32_t cik_min, uint32_t cik_max){
	return ((((sens_giris - gir_min)*(cik_max - cik_min)) / (gir_max - gir_min)) + cik_min);
}


void denge(int egim){

	if (egim < -256){
	//egim = egim *(-1);
	egim = abs(egim);
		if (egim > 2048){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1490);
		}
		else {
				mapR = map(egim, 512, 2048, 1500, 2000);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, mapR);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1490);
		}
	}
	else if (egim > 256){
		if(egim > 2048){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1490);
		}
		else {
			mapL = map(egim, 512, 2048, 1500, 2000);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, mapL);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1490);
		}
	}
	else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1490);
	}
}

void led() {
	// 0-2^16-1 değerler arası değerler alınabilr
	//alım formatı x(+/-)...y(+/-)... şekilde x sıfırıncı bit olacak y altıncı bit olacak alınan veri paketi 10 bit olacak
	HAL_UART_Receive_IT(&huart4, (uint8_t*) gelen, 5);	// şu an sadece x ekseni için yazılıyor.
	//donusum1 = atoi(gelen);
	if (gelen[4] == 'X') {
		if (gelen[3] == '+') {
			donusum2 = atoi(gelen);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1300);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1700);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1300);

		}
	} else if (gelen[3] == '-') {
		donusum2 = atoi(gelen);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1300);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1700);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1300);
	}
	else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1500);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);

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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
	HAL_Delay(1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
	HAL_Delay(1000);

  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, mpu6050_slave_adress, power_management_1_register, 1, &data, 1, HAL_MAX_DELAY);
  // power management reigster ayarı

  data = 0x08;
  HAL_I2C_Mem_Write(&hi2c1, mpu6050_slave_adress, gyro_config_register, 1, &data, 1, HAL_MAX_DELAY);
  // gyro ayarı

  data = 0x10;
  HAL_I2C_Mem_Write(&hi2c1, mpu6050_slave_adress, accelerometer_config_register, 1, &data, 1, HAL_MAX_DELAY);
  //ivme ölçer ayarı

  for(i=0;i<2000;i++){
	  prev_time2 = time2;
	  time2 = HAL_GetTick();
	  elapsed_time2 = (time2 - prev_time2)*1000;

	  cuffer[0] = 0x43;
	  HAL_I2C_Master_Transmit(&hi2c1, mpu6050_slave_adress, cuffer, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, mpu6050_slave_adress, cuffer, 6, HAL_MAX_DELAY);

	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

	  gyro_cal[0] += gyro_raw[0];
	  gyro_cal[1] += gyro_raw[1];
	  gyro_cal[2] += gyro_raw[2];

	  HAL_Delay(3);
  }

  gyro_cal[0] /= 2000;
  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;

  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  prev_time1 = time1;
	  time1 = HAL_GetTick();
	  elapsed_time1 = (time1-prev_time1)*1000;

	  tuffer[0] = 0x3B; //accelerometer
	  HAL_I2C_Master_Transmit(&hi2c1, mpu6050_slave_adress, tuffer, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, mpu6050_slave_adress, tuffer, 1, HAL_MAX_DELAY);

	  xi = acc_raw[0] = (tuffer[0]<<8 | tuffer[1]);
	  yi = acc_raw[1] = (tuffer[2]<<8 | tuffer[3]);
	  zi = acc_raw[2] = (tuffer[4]<<8 | tuffer[5]);

	  buffer[0] = 0x41; // temperature
	  HAL_I2C_Master_Transmit(&hi2c1, mpu6050_slave_adress, buffer, 1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, mpu6050_slave_adress, buffer, 2, HAL_MAX_DELAY);

	  raw_temp = (buffer[0]<<8 | buffer[1]);
	  temp = (raw_temp/340) + 36.53;

	  cuffer[0] = 0x43; //gyro
	  HAL_I2C_Master_Transmit(&hi2c1, mpu6050_slave_adress, cuffer, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, mpu6050_slave_adress, cuffer, 1, HAL_MAX_DELAY);

	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

	  xj = gyro_raw[0] -= gyro_cal[0];
	  yj = gyro_raw[1] -= gyro_cal[1];
	  zj = gyro_raw[2] -= gyro_cal[2];

	  angle_pitch_gyro += gyro_raw[0]*0.0000611; //gyro_raw/(65.5*[örnekleme süresi]0.004sn)
	  angle_roll_gyro += gyro_raw[1]*0.0000611;

	  angle_pitch_gyro += angle_roll_gyro * sin(gyro_raw[2]*0.000001066);
	  angle_roll_gyro -= angle_pitch_gyro * sin(gyro_raw[2]*0.000001066);

	  acc_total_vector = sqrt((acc_raw[0]*acc_raw[0])+(acc_raw[1]*acc_raw[1])+(acc_raw[2]*acc_raw[2]));

	  angle_pitch_acc = asin((float)acc_raw[1]/acc_total_vector)*57.296;
	  angle_roll_acc = asin((float)acc_raw[0]/acc_total_vector)*-57.296;

	  angle_pitch_acc -= 0.00;
	  angle_roll_acc -= 0.00;

	  if(set_gyro){
		  angle_pitch = angle_pitch_gyro*0.9996 + angle_pitch_acc*0.0004;
		  angle_roll = angle_roll_gyro*0.9996 + angle_roll_acc*0.0004;
	  }
	  else {
		  angle_pitch = angle_pitch_acc;
		  set_gyro = true;

	  }
	  while ((HAL_GetTick()-prev_time)*1000 < 4000);
	  prev_time = HAL_GetTick();

	  denge(xi);
	  led();
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart4.Init.BaudRate = 115200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

