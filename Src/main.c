/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "ENERGY_METER.h"
#include "Disp_HAL_SPI_TX.h"
#include "st7735.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

Summary_RegTypeDef summary;

double ApparentWh = 0;
double ApparentWh2 = 0;
/*
 * Read the Voltage and Current value from your external reference meter of from your power supply 
 * 	and write into VoltageRef and CurrentRef for calibration.
 */
double VoltageRef = 0xFFFF;             //8.615;//233.6;
double CurrentRef = 0xFFFF;             //6;//8.798;

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
	
	//**************** Initialization *****************************
	
	ST7735_Init();   //TFT LCD initialization 
	M90E32AS_Init(); //Energy Meter initialization
	
	HAL_Delay(1000);
	
	ST7735_FillScreen(ST7735_BLACK);
	
	CalVI(Channel_A, VoltageRef, CurrentRef); //Calibrate Voltage and Current for Channel A
	CalVI(Channel_B, VoltageRef, CurrentRef);	//Calibrate Voltage and Current for Channel B
	
	//CalibrationVI(Channel_A, 233.6, 8.798);
	//CalibrationVI(Channel_B, 233.6, 8.798);

	
	//**************** RELAYS *****************************
	
	HAL_GPIO_WritePin(RELE_1_GPIO_Port, RELE_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RELE_2_GPIO_Port, RELE_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RELE_3_GPIO_Port, RELE_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RELE_N_GPIO_Port, RELE_N_Pin, GPIO_PIN_SET);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(200);
		
		//************************ Energy Meter ************************
		printSummary(&summary);
		
		ApparentWh += GetApparentEnergy_A();
		
		ApparentWh2 += GetTotalApparentEnergy();
		//************************ TFT LCD ************************
		
		//Voltage
		ST7735_WriteString(3, 10, "V1: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(30, 10, GetRMSVoltage_A(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		ST7735_WriteString(3, 20, "V2: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(30, 20, GetRMSVoltage_B(), Font_7x10, ST7735_WHITE, ST7735_BLACK);

		ST7735_WriteString(3, 30, "V3: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(30, 30, GetRMSVoltage_C(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		//Active Power
		ST7735_WriteString(3, 50, "W1: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(30, 50, GetActivePower_A(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		ST7735_WriteString(3, 60, "W2: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(30, 60, GetActivePower_B(), Font_7x10, ST7735_WHITE, ST7735_BLACK);

		ST7735_WriteString(3, 70, "W3: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(30, 70, GetActivePower_C(), Font_7x10, ST7735_WHITE, ST7735_BLACK);	


		//Current
		ST7735_WriteString(90, 10, "I1: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(120, 10, GetRMSCurrent_A(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		ST7735_WriteString(90, 20, "I2: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(120, 20, GetRMSCurrent_B(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		ST7735_WriteString(90, 30, "I3: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(120, 30, GetRMSCurrent_C(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		//Apparent Power
		ST7735_WriteString(80, 50, "VA1: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(110, 50, GetApparentPower_A(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		ST7735_WriteString(80, 60, "VA2: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(110, 60, GetApparentPower_B(), Font_7x10, ST7735_WHITE, ST7735_BLACK);

		ST7735_WriteString(80, 70, "VA3: ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteFloat(110, 70, GetApparentPower_C(), Font_7x10, ST7735_WHITE, ST7735_BLACK);	
		

		
		//Temperature 
		ST7735_WriteFloat(3, 120, GetTemp(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
	  ST7735_WriteString(40, 120, "°C", Font_7x10, ST7735_WHITE, ST7735_BLACK);	
		
		//Frequency 
		ST7735_WriteFloat(100, 120, GetFreq(), Font_7x10, ST7735_WHITE, ST7735_BLACK);
	  ST7735_WriteString(140, 120, "Hz", Font_7x10, ST7735_WHITE, ST7735_BLACK);	
		
		
		
		ST7735_WriteFloat(50, 100, ApparentWh, Font_11x18, ST7735_WHITE, ST7735_BLACK);
		ST7735_WriteString(100, 100, "Wh", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		//ST7735_WriteFloat(20, 67, GetRMSVoltage_B(), Font_16x26, ST7735_WHITE, ST7735_BLACK);
		//ST7735_WriteString(90, 77, "V2", Font_7x10, ST7735_WHITE, ST7735_BLACK);
		
		//ST7735_WriteFloat(20, 120, GetRMSVoltage_C(), Font_16x26, ST7735_WHITE, ST7735_BLACK);
		//ST7735_WriteString(90, 130, "V3", Font_7x10, ST7735_WHITE, ST7735_BLACK);

		
		/*
		//Voltage
		ST7735_WriteString(3, 5, "V1: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 5, GetRMSVoltage_A(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteString(3, 15, "V2: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 15, GetRMSVoltage_B(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteString(3, 25, "V3: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 25, GetRMSVoltage_C(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		
		//Current
		ST7735_WriteString(3, 50, "I1: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 50, GetRMSCurrent_A(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteString(3, 60, "I2: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 60, GetRMSCurrent_B(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteString(3, 70, "I3: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 70, GetRMSCurrent_C(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		
		//Active Power
		ST7735_WriteString(3, 95, "P1: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 95, GetActivePower_A(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteString(3, 105, "P2: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 105, GetActivePower_B(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteString(3, 115, "P3: ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(30, 115, GetActivePower_C(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		
		//Frequency 
		ST7735_WriteString(90, 5, "Hz : ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		ST7735_WriteFloat(120, 5, GetFreq(), Font_7x10, ST7735_BLACK, ST7735_WHITE);
		
		//ST7735_WriteString(70, 5, "*", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		
		//Total (Arithmetic Sum) Apparent Energy
		//ST7735_WriteString(40, 100, "Active : ", Font_7x10, ST7735_BLACK, ST7735_WHITE);
		//ST7735_WriteFloat(100, 100, summary.GetWh, Font_7x10, ST7735_BLACK, ST7735_WHITE);
		*/
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST7735_DC_Pin|ST7735_CS_Pin|ST7735_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELE_1_Pin|RELE_2_Pin|RELE_3_Pin|RELE_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7735_DC_Pin ST7735_CS_Pin ST7735_RES_Pin */
  GPIO_InitStruct.Pin = ST7735_DC_Pin|ST7735_CS_Pin|ST7735_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELE_1_Pin RELE_2_Pin RELE_3_Pin RELE_N_Pin */
  GPIO_InitStruct.Pin = RELE_1_Pin|RELE_2_Pin|RELE_3_Pin|RELE_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
