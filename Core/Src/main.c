/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include <string.h>
#include <stdio.h>
#include "wav_helper.h"
#include "lcd.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN (8192)
#define FFT_SIZE 512
#define FILTER_TAP_NUM 5
#define SAMPLING_RATE 18181
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// SD card/FATFS variables
FATFS fs;    // File system object
FIL fil;     // File object
FILINFO fno;
FRESULT fresult;  // Result code
char buffer[1024];
UINT br, bw;  // File read/write count
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

// .wav file variables
static wav_header_t wav_header_raw;
static wav_header_t wav_header_mod;
char wav_path_raw[26];
char wav_path_mod[26];
int write_only_mod = 0; // 0: write both the raw and modified .wav files. 1: write only the modified .wav file

// audio processing variables
uint16_t adc_buffer[ADC_BUF_LEN]; // binded memory point for the ADC DMA transfer
uint16_t robot_buffer[ADC_BUF_LEN/2];
int privacy_mode = 0;

static uint32_t max_dominant_unfiltered_frequency = 0;
static uint32_t max_dominant_filtered_frequency = 0;
static uint32_t dominant_unfiltered_frequency = 0;
static uint32_t dominant_filtered_frequency = 0;

float32_t fir_coeffs[FILTER_TAP_NUM] = {
	-0.00219403, -0.01486513,  0.9746578,  -0.01486513, -0.00219403
};

float32_t fir_state[ADC_BUF_LEN/2 + FILTER_TAP_NUM - 1];
arm_fir_instance_f32 fir_filter;

uint16_t filtered_adc_buf1[ADC_BUF_LEN/2], filtered_adc_buf2[ADC_BUF_LEN/2];
arm_rfft_fast_instance_f32 S; // RFFT instance
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void send_uart(char* string);
void bufclear(char *buffer);
void apply_high_pass_filter(uint16_t* input, uint16_t* output);
void robot_effect(uint16_t* input, uint16_t* output);
uint32_t apply_FFT(uint16_t* input);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_uart(char* string)
{
    uint8_t len = strlen(string);
    HAL_UART_Transmit(&huart2, (uint8_t*) string, len, HAL_MAX_DELAY);
}

void bufclear(char *buffer)
{
    for (int i = 0; i < 1024; i++)
    {
        buffer[i] = '\0';
    }
}

// return 1 if the buffer contains amplitudes above the threshold, 0 otherwise
int amplitude_threshold(uint16_t *buffer, uint32_t buffer_len, uint16_t threshold)
{
	for (int i = 0; i < buffer_len; i++) {
		if (buffer[i] > threshold) {
			return 1;
		}
	}
	return 0;
}

// transform "input" buffer audio data using high-pass filter into "output" buffer
void apply_high_pass_filter(uint16_t* input, uint16_t* output) {
    float32_t temp_input[ADC_BUF_LEN/2];
    float32_t temp_output[ADC_BUF_LEN/2];

    for (uint32_t i = 0; i < ADC_BUF_LEN/2; i++) {
        temp_input[i] = (float32_t)input[i];
    }

    arm_fir_f32(&fir_filter, temp_input, temp_output, ADC_BUF_LEN/2);

    for (uint32_t i = 0; i < ADC_BUF_LEN/2; i++) {
        output[i] = (uint16_t)temp_output[i];
    }
}

// apply 3 delays onto audio data from "input", result in "output"
void robot_effect(uint16_t* input, uint16_t* output) {
    for (uint32_t i = 0; i < ADC_BUF_LEN/2; i++) {
      if (i < 200) {
        output[i] = input[i];
      } else if (i < 400) {
        output[i] = (input[i] + input[i - 200]) / 2 ;
      } else {
        output[i] = (input[i] + input[i - 200] + input[i - 400]) / 3 ;
      }
    }
}

// do a fast fourier transform on "input", returns dominant frequent value
uint32_t apply_FFT(uint16_t* input) {
    float32_t temp_output[FFT_SIZE];
    float32_t current_block[FFT_SIZE];
    uint32_t max_value = 0;
    uint32_t max_index = 0;

    // Convert ADC input to float32 and copy to current_block
    for (uint32_t i = 4; i < FFT_SIZE; i++) {
        current_block[i] = (float32_t)input[i];
    }

    // Perform FFT
    arm_rfft_fast_f32(&S, current_block, temp_output, 0);

    // Calculate magnitudes and find the max value
    for (uint32_t i = 1; i < FFT_SIZE / 2; i++) { // Ignoring the DC component at index 0
        float32_t magnitude = sqrtf(temp_output[2 * i] * temp_output[2 * i] + temp_output[2 * i + 1] * temp_output[2 * i + 1]);;
        if (magnitude > max_value) {
            max_value = magnitude;
            max_index = i;
        }
    }

    // Calculate the dominant frequency in Hz
    float32_t frequency_resolution = (float32_t)SAMPLING_RATE / FFT_SIZE;
    return (uint32_t)(max_index * frequency_resolution);
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000); // Short delay to allow the SD card to settle

  HAL_TIM_Base_Start_IT(&htim3); // start 1Hz timer

  // check that LCD device is ready
  HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, 0x4E, 2, 5000);
  if (res != HAL_OK) {
	  send_uart("Failed to connect to LCD device\n");
  }

  // initialise LCD device
  lcd_init();
  lcd_clear();

  /* Mount SD card */
  fresult = f_mount(&fs, "", 1);
  if (fresult != FR_OK) {
      send_uart("ERROR!!! in mounting SD CARD...\n\n");
  }
  else
      send_uart("SD CARD mounted successfully...\n\n");

  /* Check free space */
  f_getfree("", &fre_clust, &pfs);

  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  sprintf(buffer, "SD card total size:\t%lu\n", total);
  send_uart(buffer);
  bufclear(buffer);

  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
  sprintf(buffer, "SD card free space:\t%lu\n", free_space);
  send_uart(buffer);
  bufclear(buffer);

  // create raw and process audio .wav files on the SD card
  int ret = create_new_wavfile(&wav_header_raw, wav_path_raw, 1);
  if (ret < 0) {
	  send_uart("Failed to create new .wav file\n");
  }
  ret = create_new_wavfile(&wav_header_mod, wav_path_mod, 0);
  if (ret < 0) {
	  send_uart("Failed to create new .wav file\n");
  }

  // start ADC DMA continuous conversion
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_LEN);

  // initialise FFT/high-pass instances
  arm_fir_init_f32(&fir_filter, FILTER_TAP_NUM, fir_coeffs, fir_state, ADC_BUF_LEN/2);
  arm_rfft_fast_init_f32(&S, FFT_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Main loop
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
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 156;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 38399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 14999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 12499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	// high-pass on raw audio data as a baseline
    apply_high_pass_filter(&adc_buffer[0], filtered_adc_buf1);

    // display baseline audio data if it exceeds the audio threshold (sound is detected)
    if (amplitude_threshold(filtered_adc_buf1, ADC_BUF_LEN/2, 1850) == 1) {
    	HAL_UART_Transmit(&huart2, (uint8_t*) filtered_adc_buf1, (ADC_BUF_LEN/2)*sizeof(uint16_t), 25);
	}

    // if we're in privacy mode, add robot effect and then high-pass it again
    if (privacy_mode) {
        robot_effect(filtered_adc_buf1, robot_buffer);
        apply_high_pass_filter(robot_buffer, filtered_adc_buf1); // Apply HPF again after robot effect
        dominant_filtered_frequency = apply_FFT(filtered_adc_buf1); // keep track of dominant frequency for LCD screen

	// normal mode just keep track of dominant frequency for LCD screen, no further processing needed
    } else {
        dominant_filtered_frequency = apply_FFT(filtered_adc_buf1);
    }
    // get dominant frequency of raw audio data (before even the initial high-pass) for comparison.
    dominant_unfiltered_frequency = apply_FFT(adc_buffer);

    // update current max dominant frequencies (for LCD screen, since it's only updated every second)
    if (dominant_unfiltered_frequency > max_dominant_unfiltered_frequency) {
      max_dominant_unfiltered_frequency = dominant_unfiltered_frequency;
    }
    if (dominant_filtered_frequency > max_dominant_filtered_frequency) {
      max_dominant_filtered_frequency = dominant_filtered_frequency;
    }
    // update both raw and modified .wav files on the SD card
    if (write_only_mod == 0) {
    	update_wavfile(wav_path_raw, &wav_header_raw, adc_buffer, (ADC_BUF_LEN/2)*sizeof(uint16_t));
    }
    update_wavfile(wav_path_mod, &wav_header_mod, filtered_adc_buf1, (ADC_BUF_LEN/2)*sizeof(uint16_t));
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// high-pass on raw audio data as a baseline
    apply_high_pass_filter(&adc_buffer[ADC_BUF_LEN/2], filtered_adc_buf2);

    // display baseline audio data if it exceeds the audio threshold (sound is detected)
    if (amplitude_threshold(filtered_adc_buf2, ADC_BUF_LEN/2, 1850) == 1) {
    	HAL_UART_Transmit(&huart2, (uint8_t*) filtered_adc_buf2, (ADC_BUF_LEN/2)*sizeof(uint16_t), 25);
    }

    // if we're in privacy mode, add robot effect and then high-pass it again
    if (privacy_mode) {
        robot_effect(filtered_adc_buf2, robot_buffer);
        apply_high_pass_filter(robot_buffer, filtered_adc_buf2); // Apply HPF again after robot effect
        dominant_filtered_frequency = apply_FFT(filtered_adc_buf2); // keep track of dominant frequency for LCD screen

	// normal mode just keep track of dominant frequency for LCD screen, no further processing needed
    } else {
        dominant_filtered_frequency = apply_FFT(filtered_adc_buf2);
    }
    // get dominant frequency of raw audio data (before even the initial high-pass) for comparison.
    dominant_unfiltered_frequency = apply_FFT(&adc_buffer[ADC_BUF_LEN/2]);

    // update current max dominant frequencies (for LCD screen, since it's only updated every second)
    if (dominant_unfiltered_frequency > max_dominant_unfiltered_frequency) {
      max_dominant_unfiltered_frequency = dominant_unfiltered_frequency;
    }
    if (dominant_filtered_frequency > max_dominant_filtered_frequency) {
      max_dominant_filtered_frequency = dominant_filtered_frequency;
    }
    // update both raw and modified .wav files on the SD card
    if (write_only_mod == 0) {
    	update_wavfile(wav_path_raw, &wav_header_raw, &adc_buffer[ADC_BUF_LEN/2], (ADC_BUF_LEN/2)*sizeof(uint16_t));
    }
    update_wavfile(wav_path_mod, &wav_header_mod, filtered_adc_buf2, (ADC_BUF_LEN/2)*sizeof(uint16_t));
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// if the blue push-button is pressed, toggle the privacy mode
    if (GPIO_Pin == B1_Pin) {
        privacy_mode = !privacy_mode;
        if (privacy_mode) {
            send_uart("Privacy Mode Enabled\n");
        } else {
            send_uart("Privacy Mode Disabled\n");
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// if the 1Hz timer period has ellapsed (1 second has passed):
	// update the LCD screen with the dominant frequencies of the original and modified audio signal
	// and display the current mode
	if(htim->Instance == htim3.Instance) {
		char line_buf[17];
		lcd_clear();
		HAL_Delay(10);
		lcd_set_line_cursor(0);
		sprintf(line_buf, "OG: %lu MOD: %lu", max_dominant_unfiltered_frequency, max_dominant_filtered_frequency);
		HAL_Delay(10);
		lcd_send_string(line_buf);
		HAL_Delay(10);
		if (privacy_mode) {
			sprintf(line_buf, "Privacy Mode ON");
		} else {
			sprintf(line_buf, "Privacy Mode OFF");
		}
		lcd_set_line_cursor(1);
		HAL_Delay(10);
		lcd_send_string(line_buf);

		max_dominant_filtered_frequency = 0;
		max_dominant_unfiltered_frequency = 0;
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
