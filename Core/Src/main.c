/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int count = 0;
int prevZero = 0;
int threshold1 = 0;
int threshold2 = 0;
int threshold3 = 0;
int threshold4 = 0;
int state = 0;
int counter = 0;
int whileState = 0;
char in[2];
char in2[128];
int len = 0;
int arr[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// delay in microsecond
void delay_us(int us){
	TIM1->CNT = 0;
	while (TIM1->CNT < us);
}
//get distance from the sensor
int get_distance(int sensor){
	GPIO_TypeDef* trig_port;
	uint16_t trig_pin;
	GPIO_TypeDef* echo_port;
	uint16_t echo_pin;
	if (sensor == 1){
		trig_port = trig1_GPIO_Port;
		trig_pin = trig1_Pin;
		echo_port = echo1_GPIO_Port;
		echo_pin = echo1_Pin;
	}else{
		trig_port = trig2_GPIO_Port;
		trig_pin = trig2_Pin;
		echo_port = echo2_GPIO_Port;
		echo_pin = echo2_Pin;
	}
	long duration;
	HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(echo_port, echo_pin) != 1);
	TIM3->CNT = 0;
	while (HAL_GPIO_ReadPin(echo_port, echo_pin) != 0);
	duration = TIM3->CNT;
	return duration*0.034/2;
}
// display on lcd screen
void display(){
	//cap, airconOn, airconTemp, light1,light2,light3,light4,light
	ssd1306_Fill(Black);
	char line1[100];
	char line2[100];
	sprintf(line1, "%d/%d", count, arr[0]);
	if (arr[1]){
		sprintf(line2, "aircon: %d", arr[2]);
	}else{
		sprintf(line2, "aircon: off");
	}

	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(line1, Font_11x18, White);
	ssd1306_SetCursor(0, 22);
	ssd1306_WriteString(line2, Font_7x10, White);
	ssd1306_DrawRectangle(80, 24, 85, 29, White);
	ssd1306_DrawRectangle(90, 24, 95, 29, White);
	ssd1306_DrawRectangle(100, 24, 105, 29, White);
	ssd1306_DrawRectangle(110, 24, 115, 29, White);
	ssd1306_DrawRectangle(120, 24, 125, 29, White);
	// to fill the rectangle
	for (int k=0;k<5;k++){
		if (arr[3+k]){
			for (int i = 80+(10*k);i<85+(10*k);i++){
			  for (int j=24;j<29;j++){
				  ssd1306_DrawPixel(i, j, White);
			  }
			}
		}
	}
	ssd1306_UpdateScreen();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim3);
//  HAL_TIM_Base_Start_IT(&htim4);
  char out[50];
  char text1[50];
  char text2[50];
  ssd1306_Init();
  HAL_Delay(1000);

  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  int distance1 = get_distance(1);
  int distance2 = get_distance(2);
  threshold1 = (int)((float)distance1*0.3);
  threshold2 = (int)((float)distance2*0.3);
  threshold3 = (int)((float)distance1*0.9);
  threshold4 = (int)((float)distance2*0.9);

  HAL_TIM_Base_Stop_IT(&htim4);
  TIM4->CNT = 0;

  char temp[2];
  sprintf(temp, "-1"); // request data
  HAL_UART_Transmit(&huart1, temp, strlen(temp), 100);
// for getting the first set of data
  while(1){
	if (whileState == 0){
	  if (HAL_UART_Receive(&huart1, &in, 2, 100) == HAL_OK){
		  len = atoi(in);
		  sprintf(in, "%d\n\r", len);
		  HAL_UART_Transmit(&huart2, &in, strlen(in), 100);
		  whileState = 1;
	  }
	}else if (whileState == 1){
	  if (HAL_UART_Receive(&huart1, &in2, len, 100) == HAL_OK){
		  HAL_UART_Transmit(&huart2, &in2, len, 100);
		  char temp2[2];
		  sprintf(temp2, "0");
		  HAL_UART_Transmit(&huart1, temp2, strlen(temp2), 100);
		  prevZero = 1;
		  char *pt;
		  pt = strtok(in2, ",");
		  for(int i=0; i<8;i++){
			  arr[i] = atoi(pt);
			  sprintf(out, "%d\r\n", arr[i]);
			  pt = strtok (NULL, ",");
		  }
		  if (count == 0){
			prevZero = 1;
			arr[1] = 0;
			arr[3] = 0;
			arr[4] = 0;
			arr[5] = 0;
			arr[6] = 0;
			arr[7] = 0;
		}
		  whileState = 0;
		  break;
	  }
	}

  }
  display();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  int distance1 = get_distance(1);
//	  HAL_Delay(10);
//	  int distance2 = get_distance(2);
//	  sprintf(out, "sensor1: %d sensor2: %d\r\n", distance1, distance2);
//	  HAL_UART_Transmit(&huart2, &out, strlen(out), 100);
	  if (whileState == 0){
		  calSensor();
//		  sprintf(out, "count: %d state: %d\r\n", count, state);
//		  HAL_UART_Transmit(&huart2, &out, strlen(out), 100);
		  if (HAL_UART_Receive(&huart1, &in, 2, 100) == HAL_OK){
			  len = atoi(in);
			  sprintf(in, "%d\n\r", len);
//			  HAL_UART_Transmit(&huart2, &in, strlen(in), 100);
			  whileState = 1;
		  }
	  }else if (whileState == 1){
		  if (HAL_UART_Receive(&huart1, &in2, len, 100) == HAL_OK){
//			  HAL_UART_Transmit(&huart2, &in2, len, 100);
//			  HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
			  char *pt;
			  pt = strtok(in2, ",");
			  for(int i=0; i<8;i++){
				  arr[i] = atoi(pt);
				  sprintf(out, "%d\r\n", arr[i]);
//				  HAL_UART_Transmit(&huart2, out, strlen(out), 100);
				  pt = strtok (NULL, ",");
			  }

			  display();

			  whileState = 0;
		  }
	  }


//	  HAL_Delay(25);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|trig2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(trig1_GPIO_Port, trig1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin trig2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|trig2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : echo2_Pin echo1_Pin */
  GPIO_InitStruct.Pin = echo2_Pin|echo1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : trig1_Pin */
  GPIO_InitStruct.Pin = trig1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trig1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// timer for edge cases, sometimes it stucks in 1 state
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim4){
		state = 0;
		HAL_TIM_Base_Stop_IT(&htim4);
		TIM4->CNT = 0;
	}
}
// check if a person go in or go out
void calSensor(){
	int distance1 = get_distance(1);
	HAL_Delay(10);
	int distance2 = get_distance(2);
//	char out1[50];
//	sprintf(out1, "sensor1: %d sensor2: %d\r\n", distance1, distance2);
//	HAL_UART_Transmit(&huart2, &out1, strlen(out1), 100);
	if (state == 0){
		if (distance1 < threshold1){
			state = 1;
			HAL_TIM_Base_Start_IT(&htim4);
		}else if (distance2 < threshold2){
			state = 2;
			HAL_TIM_Base_Start_IT(&htim4);
		}
	}else if (state == 1){
		if (distance2 < threshold2){
			state = 3;
		}
	}else if (state == 2){
		if (distance1 < threshold1){
			state = 4;
		}
	}else if (state == 3){
		if (distance2 > threshold4){
			state = 0;
			count++;
			// auto open air/light
			if (count == 1 && prevZero){
				arr[1] = 1;
				arr[3] = 1;
				arr[4] = 1;
				arr[5] = 1;
				arr[6] = 1;
				arr[7] = 1;
			}
			prevZero = 0;
			char out[5];
			sprintf(out, "%d", count);
			HAL_UART_Transmit(&huart1, &out, strlen(out), 100);
			 HAL_TIM_Base_Stop_IT(&htim4);
			 TIM4->CNT = 0;
			display();
			HAL_Delay(250);
		}
	}else if (state == 4){
		if (distance1 > threshold3){
			state = 0;
			count--;
			if (count < 0){
				count = 0;
			}
			// auto close air/light
			if (count == 0){
				prevZero = 1;
				arr[1] = 0;
				arr[3] = 0;
				arr[4] = 0;
				arr[5] = 0;
				arr[6] = 0;
				arr[7] = 0;
			}
			char out[5];
			sprintf(out, "%d", count);
			HAL_UART_Transmit(&huart1, &out, strlen(out), 100);
			 HAL_TIM_Base_Stop_IT(&htim4);
			 TIM4->CNT = 0;
			 display();
			HAL_Delay(250);
		}
	}

	HAL_Delay(10);
	return;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
