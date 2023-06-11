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
#include <math.h>
#include <stdlib.h>
#include <stdbool.h> // Header-file for boolean data-type.
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// The values have to be always a power of 2 in order for FFT to work efficiently
#define BUFFER_SIZE 1024	// Always FFT_SAMPLES*2
#define FFT_SAMPLES 512	// Always BUFFER_SIZE/2
#define MAX_FREQ 20000.0f

#define FREQ_TO_MONITOR 1000.0f
#define THRESHOLD 33.0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

bool flag = true;
bool half_buffer_full = false;
bool buffer_full = false;
bool alert = false;

arm_rfft_fast_instance_f32 S;
float FFT_SampleRate;
int32_t Buff[BUFFER_SIZE] = {0};
float FFT_In[BUFFER_SIZE] = {0};
float FFT_Out[BUFFER_SIZE] = {0};
float FFT_Mag[BUFFER_SIZE / 2] = {0};
float FFT_Window[BUFFER_SIZE] = {0};
float FFT_Frq[BUFFER_SIZE / 2] = {0};
float FFT_dB[BUFFER_SIZE / 2] = {0};

uint8_t db_buffer[BUFFER_SIZE/2] = {0};
uint8_t start_datagram[] = "Start TX";
uint8_t alarm_message[50];
float alarm_value = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DFSDM1_Init(void);
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
  MX_USART2_UART_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);

  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, Buff, BUFFER_SIZE);

  // Prepare the FFT
  FFT_SampleRate = SystemCoreClock / hdfsdm1_channel0.Init.OutputClock.Divider
          / hdfsdm1_filter0.Init.FilterParam.Oversampling
          / hdfsdm1_filter0.Init.FilterParam.IntOversampling;


  // AC Coupling
  for (uint32_t i = 0; i < BUFFER_SIZE / 2; i++) {
	  *(FFT_Frq + i) = (float)i * (float)FFT_SampleRate / (float) BUFFER_SIZE;
  }


  arm_rfft_fast_init_f32(&S, BUFFER_SIZE);

  HAL_UART_Transmit(&huart2, start_datagram, sizeof(start_datagram), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(1){


  	  if(!flag){

  		alert = false;
  		alarm_value = 0.0;

  		int i = 0;

		for(i = 0; i < BUFFER_SIZE; i++){
			FFT_In[i] = (float) Buff[i];
		}

		// Calculate the FFT
		arm_rfft_fast_f32(&S, FFT_In, FFT_Out, 0);

		// calculate magnitude (only >0 part of the spectrum)
		arm_cmplx_mag_f32(FFT_Out, FFT_Mag, FFT_SAMPLES);

		// normalize data
		arm_scale_f32(FFT_Mag, 1.0f / sqrtf((float) BUFFER_SIZE), FFT_Mag, FFT_SAMPLES);

		// remove the DC offset
		for(i = 0; i < BUFFER_SIZE / 2; i++)
		{
			if (*(FFT_Frq + i) < 100)
				FFT_Mag[i] = 1.0f;
			else
				break;
		}

		// Calculate magnitude of the signal in dB's
		for(i = 0; i < BUFFER_SIZE / 2; i++ ) {
			FFT_dB [i] = 10.0 * log10(FFT_Mag[i]);
		}


		// If you want the program to monitor, uncomment this section

		// BEGIN MONITOR


		// Get the positions to monitor
		int down_limit = ((FREQ_TO_MONITOR / MAX_FREQ) * (BUFFER_SIZE/2)) - 5;
		int up_limit = down_limit + 10;

		// Check if in the desired frequency there's a peak
		for(i = down_limit; i < up_limit; i++) {
			if (FFT_dB[i] > THRESHOLD) {
				alarm_value = alarm_value < FFT_dB[i] ? FFT_dB[i] : alarm_value;
				alert = true;
			}
		}

		if (alert) {
		    sprintf(alarm_message, "Peak of %.1f dB's at %.1f Hz", alarm_value, FREQ_TO_MONITOR);

		    HAL_UART_Transmit(&huart2, alarm_message, sizeof(alarm_message), HAL_MAX_DELAY);

		    HAL_Delay(2000);
		}


		// END MONITOR


		// If you want the program to act as analysis, uncomment this section

		// BEGIN ANALYSIS

//		for(i = 0; i < BUFFER_SIZE / 2; i++){
//			db_buffer[i] = (uint8_t)round(FFT_dB[i]);
//		}
//
//
//  		//TX the start datagram first
//  		HAL_UART_Transmit(&huart2, start_datagram, sizeof(start_datagram), HAL_MAX_DELAY);
//
//  		HAL_UART_Transmit(&huart2, db_buffer, sizeof(db_buffer), HAL_MAX_DELAY);

		// END ANALYSIS

		// Allow continuous processing

  		HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, Buff, BUFFER_SIZE);

  		flag = true;
  	  }


    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 16;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 125;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 10;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = 0x03;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  huart2.Init.Mode = UART_MODE_TX;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */



// TODO: Allow DFSDM to put new data in processed_buffer[N_SAMPLES/2:N_SAMPLES - 1]
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){

}


// TODO: Allow DFSDM to put new data in processed_buffer[0:N_SAMPLES/2 - 1]
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

}


// TODO: Handle when the buffer is half full
// Called when the buffer is half it's capacity
void HAL_DFSDM_FilterHAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter){
	half_buffer_full = true;
	// Do something with half the info?
}


// Called when the buffer is completely full
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	buffer_full = true;
	HAL_DFSDM_FilterRegularStop_DMA(hdfsdm_filter);
	flag = false;

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
