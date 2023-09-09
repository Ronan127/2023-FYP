/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "INA219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THRESHOLD_OFF 0.2
#define THRESHOLD_FULL 0.90
#define SLIDERS 0
#define UART 1
#define INA_ADR1 0x40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t UART_RX_Pos = 0;
uint8_t UART_RX_Message[100];
uint8_t UART_RX_Byte[1];
uint8_t UART_RX_Flag = 0;
int UART_RX_Motor_a;
int UART_RX_Motor_b;
volatile uint8_t TIM3_flag = 0;
uint32_t UART_Counter=0;

INA219_t ina219;

int16_t vbus, vshunt, current;
double currentmA;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
double clamp(double d, double min, double max);
void num2str(int num, uint8_t *string, int numdigits);
int str2num(uint8_t *string, int numdigits);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART_RX_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	uint32_t ADC_Value[2];
	uint8_t txbuffer[200];

	uint8_t input = SLIDERS;
	HAL_UART_Receive_IT(&huart2, UART_RX_Byte, 1);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_DMA(&hadc1, ADC_Value, 2); // start adc in DMA mode
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	for (int i = 0; i < 100; i++) {
		txbuffer[i] = (uint8_t) *" ";
	}

	while (!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS)) {

	}

	//INA219_setPowerMode(&ina219, INA219_CONFIG_MODE_ADCOFF);

	vbus = INA219_ReadBusVoltage(&ina219);
	vshunt = INA219_ReadShuntVolage(&ina219);
	current = INA219_ReadCurrent(&ina219);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		double slider1 = (ADC_Value[0] / 2047.0) - 1.0;
		double slider2 = (ADC_Value[1] / 2047.0) - 1.0;

		if (input == SLIDERS) {
			if (slider1 > THRESHOLD_OFF) {
				HAL_GPIO_WritePin(Motor_a_fwd_GPIO_Port, Motor_a_fwd_Pin, 1);
				HAL_GPIO_WritePin(Motor_a_rev_GPIO_Port, Motor_a_rev_Pin, 0);
				TIM2->CCR1 = ((slider1 - THRESHOLD_OFF)
						/ (THRESHOLD_FULL - THRESHOLD_OFF)) * 999.0;
			} else if (slider1 < -THRESHOLD_OFF) {
				HAL_GPIO_WritePin(Motor_a_fwd_GPIO_Port, Motor_a_fwd_Pin, 0);
				HAL_GPIO_WritePin(Motor_a_rev_GPIO_Port, Motor_a_rev_Pin, 1);
				TIM2->CCR1 = ((-slider1 - THRESHOLD_OFF)
						/ (THRESHOLD_FULL - THRESHOLD_OFF)) * 999.0;
			} else {
				HAL_GPIO_WritePin(Motor_a_fwd_GPIO_Port, Motor_a_fwd_Pin, 0);
				HAL_GPIO_WritePin(Motor_a_rev_GPIO_Port, Motor_a_rev_Pin, 0);
				TIM2->CCR1 = 0;
			}

			if (slider2 > THRESHOLD_OFF) {
				HAL_GPIO_WritePin(Motor_b_fwd_GPIO_Port, Motor_b_fwd_Pin, 1);
				HAL_GPIO_WritePin(Motor_b_rev_GPIO_Port, Motor_b_rev_Pin, 0);
				TIM2->CCR2 = ((slider2 - THRESHOLD_OFF)
						/ (THRESHOLD_FULL - THRESHOLD_OFF)) * 999.0;
			} else if (slider2 < -THRESHOLD_OFF) {
				HAL_GPIO_WritePin(Motor_b_fwd_GPIO_Port, Motor_b_fwd_Pin, 0);
				HAL_GPIO_WritePin(Motor_b_rev_GPIO_Port, Motor_b_rev_Pin, 1);
				TIM2->CCR2 = ((-slider2 - THRESHOLD_OFF)
						/ (THRESHOLD_FULL - THRESHOLD_OFF)) * 999.0;
			} else {
				HAL_GPIO_WritePin(Motor_b_fwd_GPIO_Port, Motor_b_fwd_Pin, 0);
				HAL_GPIO_WritePin(Motor_b_rev_GPIO_Port, Motor_b_rev_Pin, 0);
				TIM2->CCR2 = 0;
			}
		} else if (input == UART) {

			if (UART_RX_Motor_a > 0) {
				HAL_GPIO_WritePin(Motor_a_fwd_GPIO_Port, Motor_a_fwd_Pin, 1);
				HAL_GPIO_WritePin(Motor_a_rev_GPIO_Port, Motor_a_rev_Pin, 0);
				TIM2->CCR1 = UART_RX_Motor_a;
			} else if (UART_RX_Motor_a < 0) {
				HAL_GPIO_WritePin(Motor_a_fwd_GPIO_Port, Motor_a_fwd_Pin, 0);
				HAL_GPIO_WritePin(Motor_a_rev_GPIO_Port, Motor_a_rev_Pin, 1);
				TIM2->CCR1 = -UART_RX_Motor_a;
			} else {
				HAL_GPIO_WritePin(Motor_a_fwd_GPIO_Port, Motor_a_fwd_Pin, 0);
				HAL_GPIO_WritePin(Motor_a_rev_GPIO_Port, Motor_a_rev_Pin, 0);
				TIM2->CCR1 = 0;
			}

			if (UART_RX_Motor_b > 0) {
				HAL_GPIO_WritePin(Motor_b_fwd_GPIO_Port, Motor_b_fwd_Pin, 1);
				HAL_GPIO_WritePin(Motor_b_rev_GPIO_Port, Motor_b_rev_Pin, 0);
				TIM2->CCR2 = UART_RX_Motor_b;
			} else if (UART_RX_Motor_b < 0) {
				HAL_GPIO_WritePin(Motor_b_fwd_GPIO_Port, Motor_b_fwd_Pin, 0);
				HAL_GPIO_WritePin(Motor_b_rev_GPIO_Port, Motor_b_rev_Pin, 1);
				TIM2->CCR2 = -UART_RX_Motor_b;
			} else {
				HAL_GPIO_WritePin(Motor_b_fwd_GPIO_Port, Motor_b_fwd_Pin, 0);
				HAL_GPIO_WritePin(Motor_b_rev_GPIO_Port, Motor_b_rev_Pin, 0);
				TIM2->CCR2 = 0;
			}
		}

		if (TIM3_flag == 1) {
			TIM3_flag = 0;
			vbus = INA219_ReadBusVoltage(&ina219);
			vshunt = INA219_ReadShuntVolage(&ina219);
			current = INA219_ReadCurrent(&ina219);
			UART_Counter +=1;
			sprintf((char*) txbuffer, "Duty Cycles: %04d, %04d\n"
					                  "Current:     %05.1f mA, %05.1f mA\n"
									  "Vbus:        %04d, %04d\n"
									  "Vshunt:      %04d, %04d\n", (int) TIM2->CCR1, (int) TIM2->CCR2, currentmA, currentmA, vbus, vbus, vshunt, vshunt);
//			sprintf((char*) txbuffer, "%d,%d\n", (int) UART_Counter, current);
			HAL_UART_Transmit(&huart2, txbuffer, strlen((char*) txbuffer), 50);
		}
		if (UART_RX_Flag == 1) {
			UART_RX_Handler();
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 720-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000-1;
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
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Motor_a_rev_Pin|Motor_b_fwd_Pin|Motor_b_rev_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_a_fwd_GPIO_Port, Motor_a_fwd_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Motor_a_rev_Pin Motor_b_fwd_Pin Motor_b_rev_Pin */
  GPIO_InitStruct.Pin = Motor_a_rev_Pin|Motor_b_fwd_Pin|Motor_b_rev_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_a_fwd_Pin */
  GPIO_InitStruct.Pin = Motor_a_fwd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_a_fwd_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
double clamp(double d, double min, double max) {
	const double t = d < min ? min : d;
	return t > max ? max : t;
}

void num2str(int num, uint8_t *string, int numdigits) {
	int unit = 1;
	for (int i = 0; i < numdigits; i++) {
		string[numdigits - i - 1] = (num / unit) % 10 + 48;
		unit = unit * 10;
	}
}
int str2num(uint8_t *string, int numdigits) {
	int unit = 1;
	int num = 0;
	for (int i = 0; i < numdigits; i++) {
		num = num + (string[numdigits - i - 1] - 48) * unit;
		unit = unit * 10;
	}
	return num;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	UART_RX_Flag = 1;
	HAL_UART_Receive_IT(&huart2, UART_RX_Byte, 1);
}
void UART_RX_Handler(void) {
	UART_RX_Flag = 0;
	UART_RX_Message[UART_RX_Pos] = UART_RX_Byte[0];
	UART_RX_Pos += 1;
	if (UART_RX_Byte[0] == (uint8_t) *"\n") {
		if (UART_RX_Message[0] == (uint8_t) *"+") {
			UART_RX_Motor_a = str2num(&UART_RX_Message[1], 3);
		} else if (UART_RX_Message[0] == (uint8_t) *"-") {
			UART_RX_Motor_a = -str2num(&UART_RX_Message[1], 3);
		}
		if (UART_RX_Message[5] == (uint8_t) *"+") {
			UART_RX_Motor_b = str2num(&UART_RX_Message[6], 3);
		} else if (UART_RX_Message[5] == (uint8_t) *"-") {
			UART_RX_Motor_b = -str2num(&UART_RX_Message[6], 3);
		}
		if (UART_RX_Message[10] == (uint8_t) *"P") {
			TIM2->PSC = str2num(&UART_RX_Message[11], 3) - 1;
		}
		UART_RX_Pos = 0;
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
	while (1) {
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
