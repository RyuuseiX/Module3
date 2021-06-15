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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint64_t _micros = 0;
uint64_t Time_Stamp = 0;
float Encoder_Resolution = 8192;  	//2048*4
float dt = 0.001;
float pi = 3.14159265359;			//value of pi
bool Distance_Long = true;			//Long or Short distance
									//if long use Trajectory_Generation()
									//if short use Position_Control()
bool Distance_Calculated = false;	//Calculate distance yet?

//Velocity Control
float Velocity_Encoder = 0;  		//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in pulse per second
float Velocity_Real = 0;			//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in RPM
float Velocity_Want = 10;			//Velocity calculated for control motor at the time
float Velocity_Error = 0;			//Velocity error from Velocity_Want
uint16_t Velocity_K_P = 0;			//K_P of "Velocity_Control()"
uint16_t Velocity_K_I = 0;			//K_I of "Velocity_Control()"
uint16_t Velocity_K_D = 0;			//K_D of "Velocity_Control()"
float Velocity_Error_Sum = 0;		//Sum of Velocity_Error that will be use in integrate part of "Velocity_Control()"
float Velocity_Error_Diff = 0;		//Difference of Velocity_Error and Velocity_Error_Prev that will be use in differential part of "Velocity_Control()"
float Velocity_Error_Prev = 0;		//last Velocity_Error
int16_t PWM_Out = 10000;			//PWM for motor

//Position Control
float Position_Encoder = 0;  		//Encoder's now position in CNT
float Position_Real = 0;			//Encoder's now position in degree
float Position_Want = 10;			//Position of the end point  (actually same as Point_Stop)
float Position_Error = 0;			//Position error from Position_Want
uint16_t Position_K_P = 0;			//K_P of "Position_Control()"
uint16_t Position_K_I = 0;			//K_I of "Position_Control()"
uint16_t Position_K_D = 0;			//K_D of "Position_Control()"
float Position_Error_Sum = 0;		//Sum of Position_Error that will be use in integrate part of "Position_Control()"
float Position_Error_Diff = 0;		//Difference of Position_Error and Position_Error_Prev that will be use in differential part of "Position_Control()"
float Position_Error_Prev = 0;		//last Position_Error

//Trajectory Generation
float Accel_Max = 0.5;				//Max acceleration & fixed at 0.5
float Velocity_Rad = 0;				//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in radian per second
float Velocity_Max = 10;			//Max velocity & wanted Velocity that system try to achieve & can be change by UART mode 4
float Velocity_Max_Rad = 0;			//Max velocity in radian
float Time_Blend = 0;				//Calculated time for when a != 0 and have enough distance and time to achieve Velocity_Max
float Time_All = 0;					//Calculated time for all distance
float Position_Start = 0;			//Start position
float Distance_Degree_Set = 0;		//Distance from Point_Start to Point_Stop in degree
float Distance_Radian_Set = 0;		//Distance from Point_Start to Point_Stop in radian
float Distance_Blend = 0;			//Calculated distance for when a != 0 and check that have enough distance and time to achieve Velocity_Max or not
float Position_Rad = 0;				//Encoder's now position in radian
float Position_Start = 0;			//Trajectory start position



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
float Encoder_Velocity_Update();	//Read Encoder position and calculate into velocity
void Motor_Drive_PWM();				//Supply PWM and control motor
void Velocity_Control();			//PID velocity control and calculate PWM for "Motor_Drive_PWM()"
void Position_Control();			//PID position control for short distance
void Trajectory_Generation();		//Calculate velocity base on Accel_Max for long distance
void Distance_Calculation();		//Calculate distance if it short or long

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (micros() - Time_Stamp >= 1000)//uS
	  		{

	  			Time_Stamp = micros();
	  			Position_Encoder = HTIM_ENCODER.Instance->CNT; //CNT
	  			Position_Real = (Position_Encoder*360)/Encoder_Resolution;  //degree
	  			if ((Distance_Calculated) && (Position_Real != Position_Want))
	  			{
	  				if (Distance_Long)
	  				{
	  					Trajectory_Generation();
	  					Velocity_Control();
	  				}
	  				else if (~Distance_Long)
	  				{
	  					Position_Control();
	  					Velocity_Control();
	  				}

	  				Motor_Drive_PWM();
	  			}
	  			else if ((~Distance_Calculated) && (Position_Real != Position_Want))
	  			{
	  				Distance_Calculation();
	  			}

	  			else if (Position_Real = Position_Want)
	  			{
	  				Distance_Calculated = false;
	  				Velocity_Want = 0;
	  			}


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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8191;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  htim2.Init.Period = 4294967295;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Directions_GPIO_Port, Directions_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Input_Proxreal_Pin Encoder_X_Pin */
  GPIO_InitStruct.Pin = GPIO_Input_Proxreal_Pin|Encoder_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_Input_Proxtest_Pin */
  GPIO_InitStruct.Pin = GPIO_Input_Proxtest_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_Input_Proxtest_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Directions_Pin */
  GPIO_InitStruct.Pin = Directions_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Directions_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
#define  HTIM_ENCODER htim1
#define  MAX_SUBPOSITION_OVERFLOW 4096
#define  MAX_ENCODER_PERIOD 8192

float Encoder_Velocity_Update()
{
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;

	//read data
	uint32_t EncoderNowPosition = HTIM_ENCODER.Instance->CNT;
	uint64_t EncoderNowTimestamp = micros();

	int32_t EncoderPositionDiff;
	uint64_t EncoderTimeDiff;

	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;

	//compensate overflow and underflow
	if (EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{
		EncoderPositionDiff -= MAX_ENCODER_PERIOD;
	}
	else if (-EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{
		EncoderPositionDiff += MAX_ENCODER_PERIOD;
	}

	//Update Position and time
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;

	//Calculate velocity
	//EncoderTimeDiff is in uS
	return (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;
}
void Motor_Drive_PWM()
{
	if (PWM_Out < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -PWM_Out);
	}

	else if (PWM_Out >= 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_Out);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}
void Velocity_Control()
{
	Velocity_Encoder = (Velocity_Encoder*99 + Encoder_Velocity_Update())/100.0;
	Velocity_Real = (Velocity_Encoder*60)/Encoder_Resolution;

	Velocity_Error = Velocity_Want - Velocity_Real;
	Velocity_Error_Sum = Velocity_Error_Sum + Velocity_Error;
	Velocity_Error_Diff = Velocity_Error - Velocity_Error_Prev;
	Velocity_Error_Prev = Velocity_Error;

	PWM_Out = (Velocity_K_P*Velocity_Error) + (Velocity_K_I*Velocity_Error_Sum*dt) + (Velocity_K_D*(Velocity_Error_Diff/dt));
}
void Distance_Calculation()
{
	Distance_Radian_Set = (Distance_Degree_Set*pi)/180;
	Velocity_Max_Rad = (Velocity_Max*2*pi)/60;
	Time_Blend = Velocity_Max_Rad / Accel_Max;
	Distance_Blend = 0.5*Accel_Max*(Time_Blend^2);
	Position_Start = HTIM_ENCODER.Instance->CNT;
	if ((2*Distance_Blend) >= Distance_Radian_Set)
	{
		Distance_Long = true;
	}

	else if ((2*Distance_Blend) < Distance_Radian_Set)
	{
		Distance_Long = false;
	}

	Distance_Calculated == true;
}
void Position_Control()
{
	Position_Encoder = HTIM_ENCODER.Instance->CNT;
	Position_Real = (Position_Encoder*360)/Encoder_Resolution;

	Position_Error = Position_Want - Position_Real;
	Position_Error_Sum = Position_Error_Sum + Position_Error;
	Position_Error_Diff = Position_Error - Position_Error_Prev;
	Position_Error_Prev = Position_Error;

	Velocity_Want = (Position_K_P*Position_Error) + (Position_K_I*Position_Error_Sum*dt) + (Position_K_D*(Position_Error_Diff/dt));
	if (Velocity_Want > Velocity_Max)
	{
		Velocity_Want = Velocity_Max;
	}
}
void Trajectory_Generation()
{
	//max achieve velocity is 10 RPM or ~1.05 radian per second
	//acceleration is fixed at 0.5 radian per second^2
	Position_Encoder = HTIM_ENCODER.Instance->CNT;
	Position_Rad  = (Position_Encoder*2*pi)/Encoder_Resolution;
	if (Position_Rad <= Position_Start+Distance_Blend)
	{
		Velocity_Want = Velocity_Max*((Position_Rad-Position_Start)/Distance_Blend);
	}
	else if ((Position_Rad > Position_Start+Distance_Blend) && (Position_Rad < Position_Want-Distance_Blend))
	{
		Velocity_Want = Velocty_Max;
	}
	else if (Position_Rad >= Position_Want-Distance_Blend)
	{
		Velocity_Want = (-Velocity_Max)*(((Position_Rad-(Position_Want-Distance_Blend))/Distance_Blend)-1);
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micros += 4294967295;
	}
}
uint64_t micros()
{
	return _micros + htim2.Instance->CNT;
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
