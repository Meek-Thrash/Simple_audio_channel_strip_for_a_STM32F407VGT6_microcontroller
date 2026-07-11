/* USER CODE BEGIN Header */

/*
 * Real-time stereo audio processor for STM32F4 using PCM1808 and PCM5102
 *
 *
 * Features:
 * - Stereo pass-through
 * - Adjustable input gain
 * - Constant-power stereo panning
 * - Master output level
 * - Peak detection
 * - Dual-channel LED VU meter
 *
 * Audio is processed using DMA double buffering over I2S,
 * while analog controls are acquired through ADC + DMA.
 *
 * Processing chain:
 *   1. Convert I2S samples to floating point
 *   2. Apply pre-gain
 *   3. Peak detection
 *   4. Stereo panning
 *   5. Output gain
 *   6. RMS computation for VU meters
 *   7. Convert back to 24-bit samples
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>

#include "audioUtils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_FRAMES 								256
#define I2S_BUFFER_LENGTH 						NUM_FRAMES * 2
#define HALF_I2S_BUFFER 						I2S_BUFFER_LENGTH / 2
#define AUX_BUFFER_LENGTH						HALF_I2S_BUFFER / 2

#define NORMALIZED_24_BIT						((1LL << 23) - 1)

#define NUM_ADC_SENSORS							3

#define DEFAULT_PRE_GAIN						1.0f
#define DEFAULT_PAN								0.5f
#define DEFAULT_LVL_OUT							1.0f

#define CLIPPING_THRESHOLD						0.95f

#define DEBOUNCE_TIME							20

#define ALPHA_PARAMS							0.8f
#define ALPHA_RMS								0.7f
#define QUANTIZE_DEGREE							0.01f

#define PAN_TOLERANCE							0.1f
#define PAN_DEADZONE_WIDTH						0.05f

#define SLIDER_TOLERANCE						0.05f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_i2s2_ext_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
int32_t rxBuffer[NUM_FRAMES * 2];
int32_t txBuffer[NUM_FRAMES * 2];

float leftBuffer[AUX_BUFFER_LENGTH];
float rightBuffer[AUX_BUFFER_LENGTH];

volatile uint8_t halfReady = 0;
volatile uint8_t fullReady = 0;

uint16_t adc_buffer[NUM_ADC_SENSORS];

bool muted = false;
float preGain = DEFAULT_PRE_GAIN;
float pan = DEFAULT_PAN;
float lvlOut = DEFAULT_LVL_OUT;

float rmsLeft = 0.0f;
float rmsRight = 0.0f;

uint32_t lastMute = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void processBlock(int32_t *in, int32_t *out);

void update_leds(float rms, uint8_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	//Signal that the first half of the DMA buffer is ready.
	halfReady = 1;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	//Signal that the second half of the DMA buffer is ready.
	fullReady = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//ADC1
    if(hadc->Instance == ADC1)
    {
    	//ADC READING: filtered and quantized ADC readings to reduce the ADC noise
    	preGain = quantize(ALPHA_PARAMS * (adc_buffer[0]/4095.0f)+ (1.0f - ALPHA_PARAMS) * preGain, QUANTIZE_DEGREE);
    	pan = quantize(ALPHA_PARAMS * (adc_buffer[1]/4095.0f)+ (1.0f - ALPHA_PARAMS) * pan, QUANTIZE_DEGREE);
    	lvlOut = quantize(ALPHA_PARAMS * (adc_buffer[2]/4095.0f)+ (1.0f - ALPHA_PARAMS) * lvlOut, QUANTIZE_DEGREE);

    	//PAN ROUNDING: snap the control to the extremes to compensate for potentiometer tolerances
    	if (pan > 1.0f - PAN_TOLERANCE)
    	    pan = 1.0f;

    	if (pan < PAN_TOLERANCE)
    	    pan = 0.0f;

    	//DEADZONE: a deadzone around the center position of the potentiometer so that small ADC fluctuations won't affect the stereo balance
    	if (fabsf(pan - 0.5f) < PAN_DEADZONE_WIDTH)
    	{
    	    pan = 0.5f;
    	}

    	//LEVEL OUT ROUNDING: force very small output levels to zero to guarantee silence
    	if (lvlOut < SLIDER_TOLERANCE)
		{
    		lvlOut = 0.0f;
		}
    }
}

void update_leds(float rms, uint8_t channel)
{
	/*
	 * Drive the LED VU meter using logarithmic thresholds
	 * expressed in dBFS.
	 */

	if(channel == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (rms > -48) 	? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (rms > -24) 	? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (rms > -12)	? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (rms > -6) 	? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, (rms > -3)		? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	else if(channel == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (rms > -48) 	? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (rms > -24) 	? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (rms > -12) 	? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, (rms > -6) 	? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, (rms > -3) 	? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

void processBlock(int32_t *in, int32_t *out)
{

	// I2S buffer reading, converting 24 bit I2S samples to normalized floating point values, applying the pre-gain and mute/unmute
	for(size_t i = 0; i < HALF_I2S_BUFFER; i+=2)
	{
		float leftSample = (float)(swap_half_word(in[i]) >> 8) / NORMALIZED_24_BIT;
		float rightSample = (float)(swap_half_word(in[i + 1]) >> 8) / NORMALIZED_24_BIT;

		leftBuffer[i / 2] = leftSample * preGain * (muted ? 0.0f : 1.0f);;
		rightBuffer[i / 2] = rightSample * preGain * (muted ? 0.0f : 1.0f);;
	}

	//Peak detection for clipping indication; the LED turns on when the signal exceeds the clipping threshold

	/*
	 * Peak detection is intentionally performed immediately after the
	 * pre-gain stage. This allows the clip LEDs to indicate overload
	 * caused by the input gain, independently of the pan and master
	 * output controls.
	 */

	if(peakDetect(leftBuffer, AUX_BUFFER_LENGTH) > CLIPPING_THRESHOLD)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	if(peakDetect(rightBuffer, AUX_BUFFER_LENGTH) > CLIPPING_THRESHOLD)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

	//Application of constant-power stereo panning; sqrt() coefficients maintain approximately constant the perceived loudness
	float panLeft = sqrtf(1 - pan);
	float panRight = sqrtf(pan);
	applyGain(leftBuffer, AUX_BUFFER_LENGTH, panLeft);
	applyGain(rightBuffer, AUX_BUFFER_LENGTH, panRight);

	//Application of the master out level
	applyGain(leftBuffer, AUX_BUFFER_LENGTH, lvlOut);
	applyGain(rightBuffer, AUX_BUFFER_LENGTH, lvlOut);

	//Computation of an exponentially smoothed RMS level
	rmsLeft = ALPHA_RMS * rmsDetection(leftBuffer, AUX_BUFFER_LENGTH) + (1.0f -ALPHA_RMS) * rmsLeft;
	rmsRight = ALPHA_RMS * rmsDetection(rightBuffer, AUX_BUFFER_LENGTH)+ (1.0f -ALPHA_RMS) * rmsRight;

	float dbLeft = 20.0f * log10f(rmsLeft + 1e-6f);
	float dbRight = 20.0f * log10f(rmsRight + 1e-6f);
	/*
	 *  dB / linear conversion (audio amplitude domain):
	 *
	 *      dB = 20 * log10(A / A_ref)
	 *      A  = 10^(dB / 20)
	 *
	 *  where A_ref = 1.0 (full-scale normalized signal)
	 *
	 *  In this implementation:
	 *  - A is assumed to be a normalized amplitude in [0.0, 1.0]
	 *  - A_ref = 1.0, so: dB = 20 * log10(A)
	 *
	 *  The +1e-6f term avoids log10(0):
	 *      log10(0) is undefined (-infinity), which would break the VU computation
	 *      and propagate NaN/Inf through filtering stages.
	 */

	//Vu Meter
	update_leds(dbLeft, 0);
	update_leds(dbRight, 1);

	//Conversion of normalized floating point samples to 24 bit format and I2S transmit buffer writing
	for(size_t i = 0; i < HALF_I2S_BUFFER; i += 2)
	{
		out[i] = swap_half_word((int32_t)(leftBuffer[i / 2] * NORMALIZED_24_BIT) << 8);
		out[i + 1] = swap_half_word((int32_t)(rightBuffer[i / 2] * NORMALIZED_24_BIT) << 8);

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t now = HAL_GetTick();
	if(now - lastMute >= DEBOUNCE_TIME)
	{
		//Toggle mute state after debounce interval.
		muted = !muted;
		lastMute = now;
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2SEx_TransmitReceive_DMA(&hi2s2, (uint16_t*)txBuffer, (uint16_t*)rxBuffer, NUM_FRAMES  * 2);

  HAL_TIM_Base_Start(&htim2);

  HAL_StatusTypeDef st = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUM_ADC_SENSORS);

  if(st != HAL_OK)
	  Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /*
   * Audio processing runs using DMA double buffering.
   * The callbacks only set synchronization flags,
   * while all DSP processing is executed in the main loop.
   */

	if(halfReady)
	{
		processBlock(&rxBuffer[0], &txBuffer[0]);
		halfReady = 0;
	}

	if(fullReady)
	{
		processBlock(&rxBuffer[HALF_I2S_BUFFER], &txBuffer[HALF_I2S_BUFFER]);
		fullReady = 0;
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
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
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
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
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
