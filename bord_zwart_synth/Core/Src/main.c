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
#include "arm_math.h"
#include "sample.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define n 16
#define samples 11
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

/* USER CODE BEGIN PV */
float sampleout[samples][n];
float rekensample[samples/2][n];
float M_out_f32[6][n];
int input = 0;
int change = 0;
int beat_counter = 0;
int numbersofint = 0;
int sample_change_count[samples][8] = {0};
int sample_select = 0;
int samplecounter[samples] = {0};
float volume_samples[samples] = {1};
uint16_t audio_in[4*n];
uint16_t audio_out[4*n];
uint32_t adc_data[4];
uint8_t pot[4];
q15_t audio_r_in[n];
q15_t audio_l_in[n];
q15_t audio_r_out[n];
q15_t audio_l_out[n];
q15_t M_in[n];
q15_t M_out[n];
float* samplearray[samples] = {
		kick_808,
		CH_808,
		snare_808,
		tom_808,
		kick_909,
		CH_909,
		OH_909,
		clap_909,
		tom_909,
		snare_909,
		clav_909
};
float samplelength[samples] = {
		80021,4101,10108,21576,11534,4579,20899,15130,24438,10426,2209
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2S2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adc_data, 4);
  HAL_I2S_Receive_DMA(&hi2s2, audio_in, 4*n);
  HAL_I2S_Transmit_DMA(&hi2s3, audio_out, 4*n);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  input = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	 	  if (input != change) {
	 		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	 		  change = input;
	 		  for (int j = 0; j < samples; ++j) {
	 		  for (int i = 0; i < 8; ++i) {
				sample_change_count[j][i] = 0;
	 		  }
	 		 }
	 	  }
	sample_select = pot[3]/24;
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == 1) {
		sample_change_count[sample_select][0] = 1 - sample_change_count[sample_select][0];
		HAL_Delay(250);
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 1) {
		sample_change_count[sample_select][1] = 1 - sample_change_count[sample_select][0];
		HAL_Delay(250);
	}
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 1) {
		sample_change_count[sample_select][2] = 1 - sample_change_count[sample_select][0];
		HAL_Delay(250);
	}
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1) {
		sample_change_count[sample_select][3] = 1 - sample_change_count[sample_select][0];
		HAL_Delay(250);
	}
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1) {
		sample_change_count[sample_select][4] = 1 - sample_change_count[sample_select][0];
		HAL_Delay(250);
	}
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 1) {
		sample_change_count[sample_select][5] = 1 - sample_change_count[sample_select][0];
		HAL_Delay(250);
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1) {
		sample_change_count[sample_select][6] = 1 - sample_change_count[sample_select][0];
		HAL_Delay(250);
	}
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 1) {
		sample_change_count[sample_select][7] = 1 - sample_change_count[sample_select][0];
		HAL_Delay(250);
	}
	 HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 218;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 128;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void do_processing(q15_t* audio_l_in, q15_t* audio_r_in, q15_t* audio_l_out, q15_t* audio_r_out)
{
	for (int k = 0; k < samples; ++k) {

	for (int i = 0; i < n; ++i) {
		if (samplecounter[k] > pot[2]*480) {
			samplecounter[k] = 0;
			if (k == samples -1) {
				if (beat_counter == 7) {
					beat_counter = 0;
				}
				else {
					beat_counter += 1;
				}
			}
		}

		else if (sample_change_count[k][beat_counter] == 0){
			sampleout[k][i] = 0;
			samplecounter[k] += 1;
		}
		else if (samplecounter[k] > samplelength[k]) {
			sampleout[k][i] = 0;
			samplecounter[k] += 1;
		}

		else {
			sampleout[k][i] = samplearray[k][samplecounter[k]];
			samplecounter[k] += 1;
		}
	}
	}
	volume_samples[sample_select] = pot[0]/100 + 1 ;
	for (int i = 0; i < samples; ++i) {
		arm_scale_f32(sampleout[i], volume_samples[i], sampleout[i], n);
	}
	arm_add_f32(sampleout[0], sampleout[1], rekensample[0], n);
	arm_add_f32(sampleout[2], sampleout[3], rekensample[1], n);
	arm_add_f32(sampleout[4], sampleout[5], rekensample[2], n);
	arm_add_f32(sampleout[6], sampleout[7], rekensample[3], n);
	arm_add_f32(sampleout[8], sampleout[9], rekensample[4], n);
	arm_add_f32(sampleout[10], sampleout[10], rekensample[5], n);
	arm_add_f32(rekensample[0], rekensample[1], M_out_f32[0], n);
	arm_add_f32(rekensample[2], rekensample[3], M_out_f32[1], n);
	arm_add_f32(rekensample[4], rekensample[5], M_out_f32[2], n);
	arm_add_f32(M_out_f32[0], M_out_f32[1], M_out_f32[3], n);
	arm_add_f32(M_out_f32[2], M_out_f32[2], M_out_f32[4], n);
	arm_add_f32(M_out_f32[4], M_out_f32[3], M_out_f32[5], n);
	arm_float_to_q15(M_out_f32[5], M_out, n);
	arm_scale_q15(M_out, pot[1]*4-1, 1, M_out, n);

	arm_copy_q15(M_out, audio_l_out, n);
	arm_copy_q15(M_out, audio_r_out, n);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
 /* Prevent unused argument(s) compilation warning */
	for (int i = 0; i < n; ++i) {
		audio_l_in[i] = audio_in[i*2];
		audio_r_in[i] = audio_in[i*2+1];
	}
	do_processing(audio_l_in, audio_r_in, audio_l_out, audio_r_out);
	for (int i = 0; i < n; ++i) {
		audio_out[2*i]   = audio_l_out[i];
		audio_out[2*i+1] = audio_r_out[i];
	}
	//for (int i = 0; i < n; ++i) {
	//	audio_out[i] = audio_in[i];
	//}
 /* NOTE : This function Should not be modified, when the callback is needed,
           the HAL_I2S_RxHalfCpltCallback could be implemented in the user file
  */
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
	//for (int i = n; i < 2*n; ++i) {
	//	audio_out[i] = audio_in[i];
	//}

	for (int i = 0; i < n; ++i) {
		audio_l_in[i] = audio_in[2*i+2*n];
		audio_r_in[i] = audio_in[2*i+2*n+1];
	}
	//processing audio
	//einde processing
	do_processing(audio_l_in, audio_r_in, audio_l_out, audio_r_out);
	//van array naar buffer
	for (int i = 0; i < n; ++i) {
		audio_out[2*i+2*n]   = audio_l_out[i];
		audio_out[2*i+2*n+1] = audio_r_out[i];
	}
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2S_RxCpltCallback could be implemented in the user file
   */
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

		pot[1] = adc_data[0];
		pot[0] = adc_data[1];
		pot[2] = adc_data[2];
		pot[3] = adc_data[3];

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
