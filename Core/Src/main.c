/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RevLimit 9000 // Rpm value
#define IgnitionCutTime 60000 // μs
#define TriggerCoilAngle 16
#define RPM_0    16 // This curve is linear from 1000 RPM to 4000.
#define RPM_250  16
#define RPM_500  16
#define RPM_750  16
#define RPM_1000 16
#define RPM_1250 16
#define RPM_1500 16
#define RPM_1750 16
#define RPM_2000 16
#define RPM_2250 16
#define RPM_2500 16
#define RPM_2750 16
#define RPM_3000 16
#define RPM_3250 16
#define RPM_3500 16
#define RPM_3750 16
#define RPM_4000 16 // After this point, the curve becomes flat

// Global variables
uint8_t Colors[11][3] = {
	 {255, 0, 0},   // LED 0
	 {255, 0, 0},   // LED 1
	 {223, 145, 0}, // LED 2
	 {223, 145, 0}, // LED 3
	 {186, 196, 0}, // LED 4
	 {255, 255, 0}, // LED 5
	 {167, 215, 0}, // LED 6
	 {80, 241, 0},  // LED 7
	 {80, 241, 0},  // LED 8
	 {0, 255, 0},   // LED 9
	 {0, 255, 0}    // LED 10
};
uint8_t MapIndex, AngleDifference,  TriggerNum = 99;
uint16_t Rpm = 0;
uint32_t DelayTime, PulseInterval;
bool datasentflag = false, FirstCycle = true;
uint8_t ignition_map[17] = {
  	 RPM_0,
  	 RPM_250,
  	 RPM_500,
  	 RPM_750,
  	 RPM_1000,
  	 RPM_1250,
  	 RPM_1500,
  	 RPM_1750,
  	 RPM_2000,
  	 RPM_2250,
  	 RPM_2500,
  	 RPM_2750,
  	 RPM_3000,
  	 RPM_3250,
  	 RPM_3500,
  	 RPM_3750,
  	 RPM_4000
};

uint8_t Buf[10];

/* Led array code ------------------------------------------------------------*/



// Storing the LED data
#define MAX_LED 11
uint8_t LED_Data[MAX_LED][4]; // before brightness correction
uint8_t LED_Mod[MAX_LED][4]; // after  brightness correction

void LedInit() {
	for (int i = 0; i < 11; ++i) {
		for (int j = 0; j < 4; ++j) {
			LED_Data[i][j] = 0;
			LED_Mod[i][j] = 0;
	    }
	}
}

void LedSetBrightness(const float *brightness) {
	float GammaBrightness = pow(*brightness, 2.2);

	if (GammaBrightness > 1) GammaBrightness = 1;
	if (GammaBrightness < 0) GammaBrightness = 0;

	for (int i=0; i<MAX_LED; i++) {
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++) {
			LED_Mod[i][j] = (LED_Data[i][j])*GammaBrightness;
		}
	}
}

void LedSetColor (const uint8_t LEDnum, const uint8_t Red, const uint8_t Green, const uint8_t Blue) {
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

// Convert and send the data to DMA
uint16_t pwmData[(24 * MAX_LED) + 50];

void LedSend (const float brightness) {
	LedSetBrightness(&brightness);

	uint32_t color, index = 0;

	for (int i= 0; i<MAX_LED; i++) {
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
		for (int i=23; i>=0; i--) {
			if (color&(1<<i)) {
				pwmData[index] = 71;  // 105*0.68
			}

			else pwmData[index] = 34;  // 105-71

			index++;
		}

	}

	for (int i=0; i<50; i++) {
		pwmData[index] = 0;
		index++;
	}

	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t *)pwmData, index);
	while (!datasentflag){};
	datasentflag = 0;
}

// Update Leds (Rpm calculations)
void LedUpdate() {
	#define MAX_RPM 5000 // (9000-4000)

	uint16_t num_on = round(((Rpm - 4000.0) * MAX_LED) / MAX_RPM); // RPM-4000 because MAX_RPM=9000-3000

	if (num_on < 0 || num_on > 11) {
		num_on = 0;
	}
	// Loop through the LEDs from 0 to 10
	for (int i = 0; i < 11; i++) {
		// If the LED index is less than the input, turn it on with the predefined color
		if (i < num_on) {
			LedSetColor(i, Colors[i][0], Colors[i][1], Colors[i][2]);
		}
		// Otherwise, turn it off by setting the color to 0
		else {
			LedSetColor(i, 0, 0, 0);
		}
	}
}

void LedUpdateTwoStep() {
	if (HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin) == GPIO_PIN_SET) {
		LedSetColor(10, 28, 119, 255);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_2);
	datasentflag = true;
}



/* Ignition code -------------------------------------------------------------*/



void CalculateDelayTime() {
	MapIndex = round(Rpm / 250.0f);

	if (MapIndex > 16) {
		MapIndex = 16;
	}

	AngleDifference = TriggerCoilAngle - ignition_map[MapIndex];
	DelayTime = (PulseInterval / 360.0f) * AngleDifference;
}

void CalculateDwell() {
	switch (TriggerNum) {
		case 7:
			if (Rpm > 6000) {
				HAL_GPIO_WritePin(Ignition_GPIO_Port, Ignition_Pin, GPIO_PIN_RESET); // Dwell
			}
			break;
		case 8:
			HAL_GPIO_WritePin(Ignition_GPIO_Port, Ignition_Pin, GPIO_PIN_RESET); // Dwell
			break;
	}
}

bool IsQuickShifterOn(){
	// returns true only if QuickShifter is on and Ignition pin is off, otherwise it returns false
	if (HAL_GPIO_ReadPin(QuickShifter_GPIO_Port, QuickShifter_Pin) == GPIO_PIN_SET || HAL_GPIO_ReadPin(Ignition_GPIO_Port, Ignition_Pin) == GPIO_PIN_RESET) {
		return false;
	}

	return true;
}

bool DoesRpmExeedRevLimit(){
	// returns true only if Rpm is above the Rev Limit and Ignition pin is off, otherwise it returns false
	if (Rpm < RevLimit || HAL_GPIO_ReadPin(Ignition_GPIO_Port, Ignition_Pin) == GPIO_PIN_RESET) {
		return false;
	}

	return true;
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
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Ignition_GPIO_Port, Ignition_Pin, GPIO_PIN_SET);

  LedInit();

  // LEDs startup animation
  HAL_Delay(100);
  for (int i = 0; i < 11; i++)
  {   // Loop through the LEDs from 0 to 10
	  LedSetColor(i, Colors[i][0], Colors[i][1], Colors[i][2]);
	  LedSend(0.3);
 	  HAL_Delay(100);
  }

  HAL_Delay(500);

  for (float i = 0.3; i >= 0; i-=0.005)
  {
	  LedSend(i);
 	  HAL_Delay(15);
  }

  // Start timer
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if (HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin) == GPIO_PIN_SET) {

		if (FirstCycle == false) {
			// Calculate Rpm
			Rpm = 15000000 / PulseInterval;

			if (IsQuickShifterOn() == false) {

				if (DoesRpmExeedRevLimit() == false) {

					CalculateDwell();

					if (HAL_GPIO_ReadPin(CamPosition_GPIO_Port, CamPosition_Pin) == GPIO_PIN_RESET) {

						CalculateDelayTime();

						// Ignition
						while (__HAL_TIM_GET_COUNTER(&htim2) < DelayTime);
						HAL_GPIO_WritePin(Ignition_GPIO_Port, Ignition_Pin, GPIO_PIN_SET);

						//sprintf((char *)Buf, "%u \r\n", Rpm);
						//CDC_Transmit_FS(Buf, strlen((char *)Buf));

						/*
						LedUpdate();
						LedUpdateTwoStep();
						LedSend(0.3);
						*/
					}
				} else {
					while (__HAL_TIM_GET_COUNTER(&htim2) < DelayTime + IgnitionCutTime);
				}

			} else {
				while (HAL_GPIO_ReadPin(QuickShifter_GPIO_Port, QuickShifter_Pin) == GPIO_PIN_RESET);
			}

			// Safety delay
			while (__HAL_TIM_GET_COUNTER(&htim2) < 1000);

		} else {
			//sprintf((char *)Buf, "%u \r\n", TriggerNum);
			//CDC_Transmit_FS(Buf, strlen((char *)Buf));

			if (HAL_GPIO_ReadPin(CamPosition_GPIO_Port, CamPosition_Pin) == GPIO_PIN_RESET) {
				TriggerNum = 1;
				FirstCycle = false;
			}

			while (__HAL_TIM_GET_COUNTER(&htim2) < 1000);
		}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 23;
  RCC_OscInitStruct.PLL.PLLN = 354;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 125-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ignition_GPIO_Port, Ignition_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CamPosition_Pin TwoStep_Pin */
  GPIO_InitStruct.Pin = CamPosition_Pin|TwoStep_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ignition_Pin */
  GPIO_InitStruct.Pin = Ignition_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ignition_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QuickShifter_Pin */
  GPIO_InitStruct.Pin = QuickShifter_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(QuickShifter_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Timer2 (Igntion pin) timout interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
	  HAL_GPIO_WritePin(Ignition_GPIO_Port, Ignition_Pin, GPIO_PIN_SET);
	  FirstCycle = 1;
	  TriggerNum = 99;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // Prevent unused argument(s) compilation warning
  UNUSED(GPIO_Pin);

  if (HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin) == GPIO_PIN_SET && __HAL_TIM_GET_COUNTER(&htim2) > 1000) {
	  PulseInterval = __HAL_TIM_GET_COUNTER(&htim2);
	  TIM2->CNT = 0;

	  if (TriggerNum != 99) {
		  if (TriggerNum == 8){
			  TriggerNum = 1;
		  } else {
			  TriggerNum++;
		  }
	  }
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
