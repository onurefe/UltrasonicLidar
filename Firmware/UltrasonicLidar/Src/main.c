/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	float real;
	float img;
} Complex_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLING_FREQ 120000
#define CARRIER_FREQ 40000
#define START_FREQ 16
#define STOP_FREQ 4096
#define FREQ_STEPS 16
#define FREQ_BAND (STOP_FREQ - START_FREQ)
#define NUM_OF_FREQS (1 + FREQ_BAND / FREQ_STEPS)
#define GROUP_ELEMENT_COUNT 8
#define GROUP_BAND (FREQ_STEPS * GROUP_ELEMENT_COUNT)
#define GROUP_COUNT (NUM_OF_FREQS / GROUP_ELEMENT_COUNT)
#define SAMPLE_POINTS (SAMPLING_FREQ / FREQ_STEPS)
#define BUFFER_ELEMENT_COUNT (SAMPLE_POINTS * 2)
#define TRANSFER_COMPLETED_EVT ((uint16_t)(1U << 0))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
uint16_t ADCBuffer[BUFFER_ELEMENT_COUNT];
uint16_t DACTable[BUFFER_ELEMENT_COUNT];

Complex_t FreqResponse[NUM_OF_FREQS];

float Signal[NUM_OF_FREQS];

volatile uint16_t Flags;
uint32_t TickDifferenceTable;
uint32_t TickDifferenceFreqResp;
uint32_t TickDifferenceWait;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
void StartADC(void);
void StartDAC(void);
void StartMasterTimer(void);
void StopMasterTimer(void);

void CreateDACTable(uint16_t startFreq, uint16_t freqSteps, uint16_t numOfFreqs,
				    uint32_t samplingFreq, uint32_t carrierFreq, uint16_t *table,
					uint16_t *elementCount);

void CalculateFreqResponse(Complex_t *respVector, uint16_t startFreq, uint16_t freqSteps,
				  uint16_t numOfFreqs, uint32_t samplingFreq, uint16_t carrierFreq,
				  uint16_t *buffer, uint16_t elementCount);

void IFFT(Complex_t *respVector, uint16_t startFreq, uint16_t freqSteps,
		  uint16_t numOfFreqs, uint16_t resolution, float *signal);

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
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();

  /* USER CODE END 2 */
  Flags = 0;
  uint16_t element_count;

  StartADC();
  StartDAC();

  // Create DAC table for the first group.
  CreateDACTable(START_FREQ, FREQ_STEPS, GROUP_ELEMENT_COUNT,
  		  		 SAMPLING_FREQ, CARRIER_FREQ, &DACTable[0],
  				 &element_count);

  StartMasterTimer();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // For each frequency group obtain response.
	  for (uint8_t g = 0; g < GROUP_COUNT; g++)
	  {
		  uint16_t dac_start_freq;
		  uint16_t adc_start_freq;
		  uint16_t dac_buff_offset;
		  uint16_t adc_buff_offset;
		  uint32_t tick_hold;

		  dac_start_freq = (((g + 1) == GROUP_COUNT) ? 0: g+1) * GROUP_BAND + START_FREQ;
		  dac_buff_offset = SAMPLE_POINTS * ((g + 1) & 0x0001);

		  tick_hold = HAL_GetTick();

		  // Create DAC table for the next group.
		  CreateDACTable(dac_start_freq, FREQ_STEPS, GROUP_ELEMENT_COUNT,
		  				 SAMPLING_FREQ, CARRIER_FREQ, &DACTable[dac_buff_offset],
						 &element_count);

		  TickDifferenceTable = HAL_GetTick() - tick_hold;

		  tick_hold = HAL_GetTick();

		  // Wait until transfer is completed.
		  while (!(Flags & TRANSFER_COMPLETED_EVT));
		  Flags ^= TRANSFER_COMPLETED_EVT;

		  TickDifferenceWait = HAL_GetTick() - tick_hold;

		  tick_hold = HAL_GetTick();

		  adc_buff_offset = SAMPLE_POINTS * (g & 0x0001);
		  adc_start_freq = g * GROUP_BAND + START_FREQ;

		  // Calculate frequency response.
		  CalculateFreqResponse(&FreqResponse[g * GROUP_ELEMENT_COUNT],
				  	  	  	    adc_start_freq, FREQ_STEPS,
								GROUP_ELEMENT_COUNT, SAMPLING_FREQ, CARRIER_FREQ,
								&ADCBuffer[adc_buff_offset], element_count);

		  TickDifferenceFreqResp = HAL_GetTick() - tick_hold;
	  }

	  IFFT(FreqResponse, START_FREQ, FREQ_STEPS, NUM_OF_FREQS,
	  	   NUM_OF_FREQS, Signal);
  }

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

void StartADC(void)
{
	// Start ADC.
	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCBuffer,
			BUFFER_ELEMENT_COUNT) != HAL_OK)
	{
		while(1);
	}
}

void StartDAC(void)
{
	// Start DAC.
	if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
			(uint32_t *)DACTable, BUFFER_ELEMENT_COUNT, DAC_ALIGN_12B_R) != HAL_OK)
	{
		while(1);
	}
}

void StartMasterTimer(void)
{
	// Start time base.
	if(HAL_TIM_Base_Start(&htim2) != HAL_OK)
	{
		while(1);
	}
}

void StopMasterTimer(void)
{
	if (HAL_TIM_Base_Stop(&htim2) != HAL_OK)
	{
		while (1);
	}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (84000000 / SAMPLING_FREQ);
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  Flags |= TRANSFER_COMPLETED_EVT;
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  Flags |= TRANSFER_COMPLETED_EVT;
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

  /* USER CODE END Error_Handler_Debug */
}

void CalculateFreqResponse(Complex_t *respVector, uint16_t startFreq, uint16_t freqSteps,
				  uint16_t numOfFreqs, uint32_t samplingFreq, uint16_t carrierFreq,
				  uint16_t *buffer, uint16_t elementCount)
{
	const float two_pi = 2 * 3.1415926535897;
	float intermediate_signal;
	float sum_x;
	float sum_y;
	float tmp_x;
	Complex_t carrier_phasor;
	Complex_t if_phasor;
	Complex_t carrier_rotate;
	Complex_t if_rotate;

	carrier_rotate.real = cos((two_pi * carrierFreq) / samplingFreq);
	carrier_rotate.img = -sin((two_pi * carrierFreq) / samplingFreq);

	// Demodulate and find frequency response.
	for (uint16_t i = 0; i < numOfFreqs; i++)
	{
		sum_x = 0;
		sum_y = 0;

		carrier_phasor.real = 1.0;
		carrier_phasor.img = 0.0;
		if_phasor.real = 1.0;
		if_phasor.img = 0.0;

		if_rotate.real = cos((two_pi * (startFreq + freqSteps * i)) / samplingFreq);
		if_rotate.img = -sin((two_pi * (startFreq + freqSteps * i)) / samplingFreq);

		for (uint16_t j = 0; j < elementCount; j++)
		{
			intermediate_signal = buffer[j] * carrier_phasor.real;

			sum_x += intermediate_signal * if_phasor.real;
			sum_y += intermediate_signal * if_phasor.img;

			// Rotate carrier and intermediate signal.
			tmp_x = carrier_phasor.real * carrier_rotate.real - carrier_phasor.img * carrier_rotate.img;
			carrier_phasor.img = carrier_phasor.real * carrier_rotate.img + carrier_phasor.img * carrier_rotate.real;
			carrier_phasor.real = tmp_x;

			tmp_x = if_phasor.real * if_rotate.real - if_phasor.img * if_rotate.img;
			if_phasor.img  = if_phasor.real * if_rotate.img + if_phasor.img * if_rotate.real;
			if_phasor.real = tmp_x;
		}

		respVector[i].real = sum_x;
		respVector[i].img = sum_y;
	}
}

void CreateDACTable(uint16_t startFreq, uint16_t freqSteps, uint16_t numOfFreqs,
				    uint32_t samplingFreq, uint32_t carrierFreq, uint16_t *table,
					uint16_t *elementCount)
{
	uint16_t __element_count;
	uint16_t dac_value;
	Complex_t start_freq_phasor;
	Complex_t start_freq_rotate;
	Complex_t freq_step_phasor;
	Complex_t freq_step_rotate;
	Complex_t diff_freq_phasor;
	Complex_t carrier_phasor;
	Complex_t carrier_rotate;

	float tmp_x;
	float sum;
	const float two_pi = 2 * 3.1415926535897;
	const float safety_scale = 0.999;
	float scale;

	__element_count = samplingFreq / freqSteps;

	scale = safety_scale / numOfFreqs;

	carrier_phasor.real = 1.0;
	carrier_phasor.img = 0.0;
	start_freq_phasor.real = 1.0;
	start_freq_phasor.img = 0.0;
	freq_step_phasor.real = 1.0;
	freq_step_phasor.img = 0.0;

	start_freq_rotate.real = cos(two_pi * startFreq / samplingFreq);
	start_freq_rotate.img = sin(two_pi * startFreq / samplingFreq);
	freq_step_rotate.real = cos(two_pi * freqSteps / samplingFreq);
	freq_step_rotate.img = sin(two_pi * freqSteps / samplingFreq);
	carrier_rotate.real = cos(two_pi * carrierFreq / samplingFreq);
	carrier_rotate.img = sin(two_pi * carrierFreq / samplingFreq);

	for (uint16_t i = 0; i < __element_count; i++)
	{
		sum = 0.0f;

		diff_freq_phasor.real = start_freq_phasor.real;
		diff_freq_phasor.img = start_freq_phasor.img;

		// Sum different frequency components.
		for (uint16_t j = 0; j < numOfFreqs; j++)
		{
			sum += diff_freq_phasor.real;

			// Rotate to obtain for different frequencies.
			tmp_x = diff_freq_phasor.real * freq_step_phasor.real - diff_freq_phasor.img * freq_step_phasor.img;
			diff_freq_phasor.img = diff_freq_phasor.real * freq_step_phasor.img + diff_freq_phasor.img * freq_step_phasor.real;
			diff_freq_phasor.real = tmp_x;
		}

		// Modulate.
		sum *= carrier_phasor.real;

		// Calculate DAC value.
		dac_value = (1.0f + sum * scale) * (1 << 11);
		table[i] = dac_value;

		// Rotate start_freq component.
		tmp_x = start_freq_phasor.real * start_freq_rotate.real - start_freq_phasor.img * start_freq_rotate.img;
		start_freq_phasor.img = start_freq_phasor.real * start_freq_rotate.img + start_freq_phasor.img * start_freq_rotate.real;
		start_freq_phasor.real = tmp_x;

		// Rotate freq step component.
		tmp_x = freq_step_phasor.real * freq_step_rotate.real - freq_step_phasor.img * freq_step_rotate.img;
		freq_step_phasor.img = freq_step_phasor.real * freq_step_rotate.img + freq_step_phasor.img * freq_step_rotate.real;
		freq_step_phasor.real = tmp_x;

		// Rotate carrier.
		tmp_x = carrier_phasor.real * carrier_rotate.real - carrier_phasor.img * carrier_rotate.img;
		carrier_phasor.img = carrier_phasor.real * carrier_rotate.img + carrier_phasor.img * carrier_rotate.real;
		carrier_phasor.real = tmp_x;
	}

	*elementCount = __element_count;
}

void IFFT(Complex_t *respVector, uint16_t startFreq, uint16_t freqSteps,
		  uint16_t numOfFreqs, uint16_t resolution, float *signal)
{
	Complex_t start_freq_phasor;
	Complex_t start_freq_rotate;
	Complex_t freq_step_phasor;
	Complex_t freq_step_rotate;
	Complex_t diff_freq_phasor;

	float tmp_x;
	float sum_x;
	float sum_y;
	float power = 0;
	const float two_pi = 2 * 3.1415926535897;
	float buff[NUM_OF_FREQS];

	start_freq_phasor.real = 1.0;
	start_freq_phasor.img = 0.0;
	freq_step_phasor.real = 1.0;
	freq_step_phasor.img = 0.0;

	start_freq_rotate.real = cos(((two_pi * startFreq) / freqSteps) / resolution);
	start_freq_rotate.img = sin(((two_pi * startFreq) / freqSteps) / resolution);
	freq_step_rotate.real = cos(two_pi / resolution);
	freq_step_rotate.img = sin(two_pi / resolution);

	for (uint16_t i = 0; i < resolution; i++)
	{
		sum_x = 0.0f;
		sum_y = 0.0f;

		diff_freq_phasor.real = start_freq_phasor.real;
		diff_freq_phasor.img = start_freq_phasor.img;

		// Sum different frequency components.
		for (uint16_t j = 0; j < numOfFreqs; j++)
		{
			float x;
			float y;

			// Add base rotation.
			x = diff_freq_phasor.real * respVector[j].real - diff_freq_phasor.img * respVector[j].img;
			y = diff_freq_phasor.real * respVector[j].img + diff_freq_phasor.img * respVector[j].real;

			sum_x += x;
			sum_y += y;

			// Rotate to obtain for different frequencies.
			tmp_x = diff_freq_phasor.real * freq_step_phasor.real - diff_freq_phasor.img * freq_step_phasor.img;
			diff_freq_phasor.img = diff_freq_phasor.real * freq_step_phasor.img + diff_freq_phasor.img * freq_step_phasor.real;
			diff_freq_phasor.real = tmp_x;
		}

		float __equivalent_power;
		__equivalent_power = (sum_x * sum_x + sum_y * sum_y) * (i + 3.5f) * (i + 3.5f);

		// Parse signal.
		buff[i] = __equivalent_power;
		power += __equivalent_power;

		// Rotate start_freq component.
		tmp_x = start_freq_phasor.real * start_freq_rotate.real - start_freq_phasor.img * start_freq_rotate.img;
		start_freq_phasor.img = start_freq_phasor.real * start_freq_rotate.img + start_freq_phasor.img * start_freq_rotate.real;
		start_freq_phasor.real = tmp_x;

		// Rotate freq step component.
		tmp_x = freq_step_phasor.real * freq_step_rotate.real - freq_step_phasor.img * freq_step_rotate.img;
		freq_step_phasor.img = freq_step_phasor.real * freq_step_rotate.img + freq_step_phasor.img * freq_step_rotate.real;
		freq_step_phasor.real = tmp_x;
	}

	for (int i = 0; i < numOfFreqs; i++)
	{
		signal[i] = sqrt(buff[i] / power);
	}
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
