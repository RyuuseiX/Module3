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
#include "math.h"
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
uint64_t _micros = 0;				//Keep track of time
uint64_t Time_Sampling_Stamp = 0;	//Control loop time stamp
uint64_t Time_Velocity_Stamp = 0;
uint64_t Time_Trajectory_Stamp = 0;	//Time to calculate in "Trajectory_Generataion()"
uint16_t Encoder_Resolution = 8192; //2048*4
uint16_t Encoder_Overflow = 4096;	//8192/2
float pi = 3.14159265359;			//value of pi

#define LONG 1						//long distance
#define SHORT 0						//short distance
uint16_t Distance_Length = 0;		//Long or Short distance
uint16_t Distance_Calculated = 0;	//Calculate distance yet?

//Velocity Control
float Velocity_Encoder = 0;  		//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in pulse per second
float Velocity_Now_RPM = 0;			//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in RPM
float Velocity_Want_RPM = 10;		//Velocity calculated for control motor at the time
float Velocity_K_P = 2000;			//K_P of "Velocity_Control()"
float Velocity_K_I = 1;				//K_I of "Velocity_Control()"
float Velocity_K_D = 0;				//K_D of "Velocity_Control()"
float Velocity_Error = 0;			//Velocity error from Velocity_Want_RPM
float Velocity_Error_Sum = 0;		//Sum of Velocity_Error that will be use in integrate part of "Velocity_Control()"
float Velocity_Error_Diff = 0;		//Difference of Velocity_Error and Velocity_Error_Prev that will be use in differential part of "Velocity_Control()"
float Velocity_Error_Prev = 0;		//last Velocity_Error
int16_t PWM_Out = 0;				//PWM for motor

//Position Control
float Position_Encoder = 0;  		//Encoder's now position in CNT
float Position_Now_Degree = 0;		//Encoder's now position in degree
float Position_Want_Degree = 90;		//Position of the end point  (actually same as Point_Stop)
float Position_Prev_Degree = 0;		//Check that Position_Want_Degree changed or not
float Position_K_P = 228.62;		//K_P of "Position_Control()"
float Position_K_I = 0;			    //K_I of "Position_Control()"
float Position_K_D = 7.1;			//K_D of "Position_Control()"
float Position_Error = 0;			//Position error from Position_Want_Degree
float Position_Error_Sum = 0;		//Sum of Position_Error that will be use in integrate part of "Position_Control()"
float Position_Error_Diff = 0;		//Difference of Position_Error and Position_Error_Prev that will be use in differential part of "Position_Control()"
float Position_Error_Prev = 0;		//last Position_Error

//Trajectory Generation
float Accel_Max = 0.5;			    //Max acceleration & fixed at 0.5
float Velocity_Now_Rad = 0;			//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in radian per second
float Velocity_Max_RPM = 10;		//Max velocity in RPM & wanted Velocity that system try to achieve & can be change by UART mode 4
float Velocity_Max_Rad = 0;			//Max velocity in radian per second
float Velocity_Achieve_RPM = 0;	    //Velocity limit in RPM motor can achieve in short distance
float Velocity_Achieve_Rad = 0; 	//Velocity limit in radian per second motor can achieve in short distance
float Distance_Degree_Set = 0;		//Distance from Point_Start to Point_Stop in degree
float Distance_Radian_Set = 0;		//Distance from Point_Start to Point_Stop in radian
float Distance_Blend = 0;			//Calculated distance for when a != 0 and check that have enough distance and time to achieve Velocity_Max_RPM or not
float Distance_Center = 0;			//Distance when Velocity_Want = Velocity_Max_RPM
float Position_Start = 0;			//Start position
float Position_Rad = 0;				//Encoder's now position in radian
float Time_Blend = 0;				//Calculated time for when a != 0 and have enough distance and time to achieve Velocity_Max_RPM
float Time_All = 0;					//Calculated time for all distance
float Time_Center = 0;
uint64_t Time_Start = 0;			//Start time
float Time_Blend_Micro = 0;			//Time_Blend in microsecond
float Time_All_Micro = 0;			//Time_All in microsecond
float Time_Center_Micro = 0;
uint16_t Trajectory_Flag = 0;		//For keeping Position_Start & Time_Start or Finished


//debugging
float Flag4 = 0;
float P1 = 0;
float P2 = 0;
float P3 = 0;

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
  HAL_TIM_Base_Start_IT(&htim2);					//micros()
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   //Start reading encoder
  HAL_TIM_Base_Start(&htim3);						//Start TIM3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);			//Start PWM TIM3
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (micros() - Time_Velocity_Stamp >= 100)
	  {
		  Time_Velocity_Stamp = micros();
		  Velocity_Encoder = (Velocity_Encoder*9999 + Encoder_Velocity_Update())/(float)10000;
	  }
	  if (micros() - Time_Sampling_Stamp >= 1000)	  //Control loop
	  {
			Time_Sampling_Stamp = micros();
			Position_Encoder = htim1.Instance->CNT; //Read Encoder
			Position_Now_Degree = (Position_Encoder*360)/Encoder_Resolution; //Convert Encoder CNT to degree
			if (Distance_Calculated == 0) //Distance not calculated and not arrive at next station
			{
				Distance_Calculation();		//Calculate distance
			}
			else if ((Distance_Calculated == 1) && (Position_Now_Degree != Position_Want_Degree) && (Trajectory_Flag < 5)) //Distance calculated and not arrive at next station
			{
				Trajectory_Generation();	//Get Velocity_Want_RPM
				Velocity_Control();			//Get PWM_Out
				Motor_Drive_PWM();			//Drive

				if(Trajectory_Flag == 4)
				{
					Trajectory_Flag = 5;
				}


			}


			if (Trajectory_Flag == 5)		//Reach next station
			{
				if (Position_Prev_Degree != Position_Want_Degree)	//Change goal
				{
					Trajectory_Flag = 0;	//Reset flag
					Distance_Calculated = 0;//Reset distance
					Velocity_Want_RPM = 0;  //Reset Velocity_Want_RPM
					Flag4 = 0;
				}
				Velocity_Want_RPM = 0;
				Velocity_Control();			//Get PWM_Out
				Motor_Drive_PWM();			//Drive

			}
			Position_Prev_Degree = Position_Want_Degree; //Check that Position_Want_Degree change or not



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
  htim2.Init.Prescaler = 99;
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
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);

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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
float Encoder_Velocity_Update()  //Lecture code DON'T TOUCH!
{
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;

	//read data
	uint32_t EncoderNowPosition = htim1.Instance->CNT;
	uint64_t EncoderNowTimestamp = micros();

	int32_t EncoderPositionDiff;
	uint64_t EncoderTimeDiff;

	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;

	//compensate overflow and underflow
	if (EncoderPositionDiff >= Encoder_Overflow)
	{
		EncoderPositionDiff -= Encoder_Resolution;
	}
	else if (-EncoderPositionDiff >= Encoder_Overflow)
	{
		EncoderPositionDiff += Encoder_Resolution;
	}

	//Update Position and time
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;

	//Calculate velocity
	//EncoderTimeDiff is in uS
	return (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;
}

#define PWM_CHANNEL TIM_CHANNEL_4			//Set channel for PWM
#define GPIO_PIN_DIRECTION GPIO_PIN_10		//Set pin for direction
void Motor_Drive_PWM()	//Motor drive
{
	if (PWM_Out > 10000)		//If Velocity_Want_RPM exceed Velocity_Max_RPM
	{
		PWM_Out = 10000;		//Run with Velocity_Max_RPM
	}
	else if (PWM_Out < -10000)		//If Velocity_Want_RPM exceed Velocity_Max_RPM
	{
		PWM_Out = -10000;		//Run with Velocity_Max_RPM
	}

	if (PWM_Out < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, PWM_CHANNEL, -PWM_Out);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_DIRECTION, GPIO_PIN_RESET);
	}

	else if (PWM_Out >= 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, PWM_CHANNEL, PWM_Out);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_DIRECTION, GPIO_PIN_SET);
	}
}
void Velocity_Control()  //Velocity Control PID
{

	Velocity_Now_RPM = (Velocity_Encoder*60)/Encoder_Resolution;	//Convert Velocity_Encoder (Encoder's velocity at the moment) to RPM
	Velocity_Now_Rad = (Velocity_Now_RPM*2*pi)/60;

	if (Velocity_Want_RPM > Velocity_Max_RPM)		//If Velocity_Want_RPM exceed Velocity_Max_RPM
	{
		Velocity_Want_RPM = Velocity_Max_RPM;		//Run with Velocity_Max_RPM
	}
	else if (Velocity_Want_RPM < -Velocity_Max_RPM)		//If Velocity_Want_RPM exceed Velocity_Max_RPM
	{
		Velocity_Want_RPM = -Velocity_Max_RPM;		//Run with Velocity_Max_RPM
	}

	Velocity_Error = Velocity_Want_RPM - Velocity_Now_RPM;
	Velocity_Error_Sum = Velocity_Error_Sum + Velocity_Error;
	Velocity_Error_Diff = Velocity_Error - Velocity_Error_Prev;
	Velocity_Error_Prev = Velocity_Error;

	PWM_Out = (Velocity_K_P*Velocity_Error) + (Velocity_K_I*Velocity_Error_Sum) + (Velocity_K_D*(Velocity_Error_Diff));

}
void Distance_Calculation()	//Calculate that distance is short or long
{
	//acceleration is fixed at 0.5 radian per second^2
	Distance_Degree_Set = Position_Want_Degree - Position_Now_Degree;  //Get distance from  EndPoint - StartPoint in degree
	if (Distance_Degree_Set < 0)
	{
		Distance_Degree_Set += 360;
	}
	Distance_Radian_Set = (Distance_Degree_Set*pi)/180;				   //Change Distance_Degree_Set to radian

	Velocity_Max_Rad = (Velocity_Max_RPM*2*pi)/60;					   //Change max velocity to radian per second

	Time_Blend = Velocity_Max_Rad*2;								   //Time used for motor to reach Velocity_Max_Rad with a=0.5 radian per second^2
	Time_Blend_Micro = Time_Blend*1000000;							   //Change from second to microsecond

	Distance_Blend = 2*(powf(Velocity_Max_Rad, 2));					   //Distance used for motor to reach Velocity_Max_Rad with a=0.5 radian per second^2


	if ((2*Distance_Blend) < Distance_Radian_Set)					   //Distance_Radian_Set is long enough to achieve Velocity_Max_Rad
	{
		Distance_Length = LONG;
		Distance_Center = Distance_Radian_Set - (2*Distance_Blend);	   //Distance when a=0 radian per second^2
		Time_Center = Distance_Center/Velocity_Max_Rad;
		Time_Center_Micro = Time_Center*1000000;
		Time_All = (2*Time_Blend) + (Time_Center);//Time use to reach next station
		Time_All_Micro = Time_All*1000000;							   //Change from second to microsecond
	}

	else if ((2*Distance_Blend) >= Distance_Radian_Set)				   //Distance_Radian_Set is not long enough to achieve Velocity_Max_Rad
	{
		Distance_Length = SHORT;
		Time_Blend = sqrtf(Distance_Radian_Set*2);					   //Time used for motor to reach Velocity_Achieve_Rad with a=0.5 radian per second^2
		Time_Blend_Micro = Time_Blend*1000000;						   //Change from second to microsecond
		Time_All = (2*Time_Blend);
		Time_All_Micro = Time_All * 1000000;
		Velocity_Achieve_Rad = sqrtf(Distance_Radian_Set/2);		   //Top limit velocity that motor can achieve in short distance
		Velocity_Achieve_RPM = (Velocity_Achieve_Rad*60)/(2*pi);	   //Change from radian per second to RPM
	}

	Distance_Calculated = 1;
}
void Position_Control()  //Position Control PID
{
	Position_Encoder = htim1.Instance->CNT;
	Position_Now_Degree = (Position_Encoder*360)/Encoder_Resolution;  //degree

	Position_Error = Position_Want_Degree - Position_Now_Degree;
	Position_Error_Sum = Position_Error_Sum + Position_Error;
	Position_Error_Diff = Position_Error - Position_Error_Prev;
	Position_Error_Prev = Position_Error;

	Velocity_Want_RPM = (Position_K_P*Position_Error) + (Position_K_I*Position_Error_Sum) + (Position_K_D*(Position_Error_Diff));

}
void Trajectory_Generation()  //Position Control with Trajectory Generation
{

	Position_Encoder = htim1.Instance->CNT;
	Position_Rad  = (Position_Encoder*2*pi)/Encoder_Resolution;  //radian
	if (Trajectory_Flag == 0)
	{
		Time_Start = micros();
		Position_Start = Position_Rad;
		Trajectory_Flag = 1;
	}
	Time_Trajectory_Stamp = micros();

	if (Distance_Length == LONG)
	{
		if ((Time_Trajectory_Stamp-Time_Start) <= Time_Blend_Micro)
		{
			Velocity_Want_RPM = Velocity_Max_RPM*((Time_Trajectory_Stamp-Time_Start)/Time_Blend_Micro);
			Trajectory_Flag = 2;
			P1 = Position_Rad-Position_Start;
		}
		else if (((Time_Trajectory_Stamp-Time_Start) > (Time_Blend_Micro) )
				&& (Time_Trajectory_Stamp-Time_Start < Time_All_Micro-Time_Blend_Micro))
		{
			Velocity_Want_RPM = Velocity_Max_RPM;
			P2 = Position_Rad-P1-Position_Start;
		}
		else if (((Time_Trajectory_Stamp-Time_Start) >= (Time_All_Micro-Time_Blend_Micro))
				&& (Time_Trajectory_Stamp-Time_Start <= Time_All_Micro) )
		{
			Velocity_Want_RPM = (-Velocity_Max_RPM)*((((Time_Trajectory_Stamp-Time_Start)-(Time_All_Micro-Time_Blend_Micro))/Time_Blend_Micro)-1);
			Trajectory_Flag = 3;
			P3 = Position_Rad-P2-P1-Position_Start;
		}
		else if ((Time_Trajectory_Stamp-Time_Start) >= Time_All_Micro)
		{
			Velocity_Want_RPM = 0;

			Trajectory_Flag = 4;
		}
	}

	else if (Distance_Length == SHORT)
	{
		if ((Time_Trajectory_Stamp-Time_Start) <= Time_Blend_Micro)
		{
			Velocity_Want_RPM = Velocity_Achieve_RPM*((Time_Trajectory_Stamp-Time_Start)/Time_Blend_Micro);
			Trajectory_Flag = 2;
		}
		else if (((Time_Trajectory_Stamp-Time_Start) >= Time_Blend_Micro)
				&& ((Time_Trajectory_Stamp-Time_Start) < (2*Time_Blend_Micro)))
		{
			Velocity_Want_RPM = (-Velocity_Achieve_RPM)*((((Time_Trajectory_Stamp-Time_Start)-Time_Blend_Micro)/Time_Blend_Micro)-1);
			Trajectory_Flag = 3;
		}
		else if ((Time_Trajectory_Stamp-Time_Start) >= (2*Time_Blend_Micro))
		{
			Velocity_Want_RPM = 0;
			Trajectory_Flag = 4;
		}
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
