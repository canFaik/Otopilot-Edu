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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_LIS3DSH.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct motor_speed {
	volatile uint32_t motor_speed[4];
}motor_throttle;

motor_throttle loiter_mode;
motor_throttle stabilize_mode;
motor_throttle motor_pid_output;
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MEMS_CS_GPIO_Port 	GPIOE
#define MEMS_CS_Pin		 	GPIO_PIN_3
uint32_t t=0;
double roll = 0;
double pitch=0;
char tx_buf[64];

float pid_roll_value = 0;
float pid_pitch_value = 0;

volatile uint32_t ch1_rising = 0, ch2_rising = 0, ch3_rising = 0, ch4_rising = 0;
volatile uint32_t ch1_falling = 0, ch2_falling = 0, ch3_falling = 0, ch4_falling = 0;
volatile uint32_t pre_ch1 = 0, ch1 = 0, pre_ch2 = 0, ch2 = 0, pre_ch3 = 0, ch3 = 0, pre_ch4 = 0, ch4 = 0;

float kp = 1.0;
float ki =	0.1;
float kd =0.01;

float error_roll = 0, error_pitch = 0;
float last_error_roll = 0, last_error_pitch = 0;
float integral_roll = 0, integral_pitch = 0;

int motor_speeds[4] = {0, 0, 0, 0};

float desired_roll = 0, desired_pitch = 0;

float measured_roll, measured_pitch;

LIS3DSH_DataScaled  X;
LIS3DSH_DataScaledY Y;
LIS3DSH_DataScaledZ Z;

uint16_t degree_change_percentage(uint16_t In, uint16_t Inmin, uint16_t Inmax, uint16_t Outmin, uint16_t Outmax);



uint8_t drdyFlag=0;

uint16_t rec_roll, roll_right,roll_left,pitch_forward,pitch_back,yaw_right,yaw_left,rec_pitch,rec_yaw,rec_throttle;

double xAngle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void tim_start(void);

void acces_measure(void);

void loiter_drone_mode(void);

void stabilize_drone_mode(void);

void stabilize_motor_output(void);

void receiver_motor_value(void);

void pid_rol_pitch_value(float measured_roll, float measured_pitch);

void loiter_motor_output(void);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	LIS3DSH_InitTypeDef myAccConfigDef;
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  myAccConfigDef.dataRate = LIS3DSH_DATARATE_25;
  myAccConfigDef.fullScale = LIS3DSH_FULLSCALE_4;
  myAccConfigDef.antiAliasingBW = LIS3DSH_FILTER_BW_50;
  myAccConfigDef.enableAxes = LIS3DSH_XYZ_ENABLE;
  myAccConfigDef.interruptEnable = true;
  LIS3DSH_Init(&hspi1, &myAccConfigDef);

  LIS3DSH_X_calibrate(-1000.0, 980.0);
  LIS3DSH_Y_calibrate(-1020.0, 1040.0);
  LIS3DSH_Z_calibrate(-920.0, 1040.0);

  /* HAL PWM START */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,1000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,1000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,1000);

  HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  tim_start(); 											/* Timer start htim1, input capture */

	  stabilize_drone_mode();								/* Drone is stabilize mode active */

	  loiter_drone_mode();									/* Drone is loiter mode active */


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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	UNUSED(GPIO_Pin);

	drdyFlag = 1;
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	//End
};

void stabilize_drone_mode(void)
{
	// Receiver value is control.
	receiver_motor_value();

	//Motor speed, roll, pitch, yaw value.
	stabilize_mode.motor_speed[0] = 1000 + rec_throttle + roll_right - roll_left + pitch_back - pitch_forward + yaw_right - yaw_left;
	stabilize_mode.motor_speed[1] = 1000 + rec_throttle + roll_right - roll_left - pitch_back + pitch_forward + yaw_left - yaw_right;
	stabilize_mode.motor_speed[2] = 1000 + rec_throttle + roll_left - roll_right - pitch_back + pitch_forward + yaw_right - yaw_left;
	stabilize_mode.motor_speed[3] = 1000 + rec_throttle + roll_left - roll_right + pitch_back - pitch_forward + yaw_left - yaw_right;

	// Motor speed roll, pitch, yaw stopper 0 - 2000.
	if(stabilize_mode.motor_speed[0] >=2000 ) stabilize_mode.motor_speed[0] = 2000;
	if(stabilize_mode.motor_speed[1] >=2000 ) stabilize_mode.motor_speed[1] = 2000;
	if(stabilize_mode.motor_speed[2] >=2000 ) stabilize_mode.motor_speed[2] = 2000;
	if(stabilize_mode.motor_speed[3] >=2000 ) stabilize_mode.motor_speed[3] = 2000;

	if(stabilize_mode.motor_speed[0] <=1000 ) stabilize_mode.motor_speed[0] = 1000;
	if(stabilize_mode.motor_speed[1] <=1000 ) stabilize_mode.motor_speed[1] = 1000;
	if(stabilize_mode.motor_speed[2] <=1000 ) stabilize_mode.motor_speed[2] = 1000;
	if(stabilize_mode.motor_speed[3] <=1000 ) stabilize_mode.motor_speed[3] = 1000;

	// Motor value to pwm compare set.
	stabilize_motor_output();
   // End
};

void pid_rol_pitch_value(float measured_roll, float measured_pitch) {
    // Roll PID calculate

    error_roll = desired_roll - measured_roll;
    integral_roll += error_roll;

     pid_roll_value = kp * error_roll + ki * integral_roll + kd * (error_roll - last_error_roll);
     pid_roll_value /= 800;

    last_error_roll = error_roll;

    // Pitch PID calculate
    error_pitch = desired_pitch - measured_pitch;
    integral_pitch += error_pitch;

    pid_pitch_value = kp * error_pitch + ki * integral_pitch + kd * (error_pitch - last_error_pitch);
    pid_pitch_value /= 800;
    last_error_pitch = error_pitch;

};


void loiter_drone_mode(void)
{
	receiver_motor_value();									/* Receiver value is control 	*/

	acces_measure();										/* Roll and pitch angle measure */

	pid_rol_pitch_value(measured_roll, measured_pitch);		/* Roll and pitch  pid control */

	loiter_mode.motor_speed[0] = 1000 + rec_throttle + roll_right - roll_left + pitch_back - pitch_forward + yaw_right - yaw_left;
	loiter_mode.motor_speed[1] = 1000 + rec_throttle + roll_right - roll_left - pitch_back + pitch_forward + yaw_left - yaw_right;
	loiter_mode.motor_speed[2] = 1000 + rec_throttle + roll_left - roll_right - pitch_back + pitch_forward + yaw_right - yaw_left;
	loiter_mode.motor_speed[3] = 1000 + rec_throttle + roll_left - roll_right + pitch_back - pitch_forward + yaw_left - yaw_right;

	// Motor speed roll, pitch, yaw stopper 0 - 2000.
	if(loiter_mode.motor_speed[0] >=2000 ) loiter_mode.motor_speed[0] = 2000;
	if(loiter_mode.motor_speed[1] >=2000 ) loiter_mode.motor_speed[1] = 2000;
	if(loiter_mode.motor_speed[2] >=2000 ) loiter_mode.motor_speed[2] = 2000;
	if(loiter_mode.motor_speed[3] >=2000 ) loiter_mode.motor_speed[3] = 2000;

	if(loiter_mode.motor_speed[0] <=1000 ) loiter_mode.motor_speed[0] = 1000;
	if(loiter_mode.motor_speed[1] <=1000 ) loiter_mode.motor_speed[1] = 1000;
	if(loiter_mode.motor_speed[2] <=1000 ) loiter_mode.motor_speed[2] = 1000;
	if(loiter_mode.motor_speed[3] <=1000 ) loiter_mode.motor_speed[3] = 1000;


};

uint16_t degree_change_percentage(uint16_t In, uint16_t Inmin, uint16_t Inmax, uint16_t Outmin, uint16_t Outmax)
{
	return (In- Inmin) * (Outmax- Outmin) / (Inmax -Inmin) + Outmin;
	//End
};

void tim_start(void)
{
	   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	   // End
};

void stabilize_motor_output(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,stabilize_mode.motor_speed[0]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,stabilize_mode.motor_speed[1]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,stabilize_mode.motor_speed[2]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,stabilize_mode.motor_speed[3]);
	// End
};

void loiter_motor_output(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,loiter_mode.motor_speed[0]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,loiter_mode.motor_speed[1]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,loiter_mode.motor_speed[2]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,loiter_mode.motor_speed[3]);
	// End
};

void acces_measure(void)
{
	if(drdyFlag == 1)
	{

		drdyFlag = 0;
		X = LIS3DSH_GetDataScaled();
		Y = LIS3DSH_GetDataScaledY();
		Z = LIS3DSH_GetDataScaledZ();

		measured_roll = (int)((((atan2((double)(-X.x) , sqrt((double)Y.y *(double) Y.y +(double) Z.z *(double) Z.z)) * 57.3)+1.5))* -1);
	}
		measured_pitch =(int)((atan2(Y.y, Z.z) * 180 / 3.14)+1.2);
}



void receiver_motor_value(void)
{
	roll_right = degree_change_percentage(rec_roll, 500, 1000, 0, 500);
	if(roll_right >=1050) roll_right = 0;
	roll_left = degree_change_percentage(rec_roll, 0, 500, 500, 0);
	if(roll_left >=1050) roll_left = 0;
	yaw_right = degree_change_percentage(rec_yaw, 500, 1000, 0, 500);
	if(yaw_right >= 1050) yaw_right = 0;
	yaw_left = degree_change_percentage(rec_yaw, 0, 500, 500, 0);
	if(yaw_left >= 1050) yaw_left = 0;
	pitch_forward = degree_change_percentage(rec_pitch,0 , 500, 500, 0);
	if(pitch_forward >= 1050) pitch_forward = 0;
	pitch_back = degree_change_percentage(rec_pitch,500 , 1000, 0, 500);
	if(pitch_back >= 1050) pitch_back = 0;
	// End
};

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		switch(htim->Channel)
		{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			if((TIM1->CCER & TIM_CCER_CC1P)==0)
			{
				ch1_rising = TIM1->CCR1;
				TIM1->CCER |= TIM_CCER_CC1P;
			}

			else
			{
				ch1_falling = TIM1->CCR1;
				pre_ch1 = ch1_falling - ch1_rising;
				//if(pre_ch1 < 0)pre_ch1 += 0xFFFF;
				pre_ch1 = degree_change_percentage(pre_ch1, 2140, 3862, 0,1000);
				if(pre_ch1 <= 1000 && pre_ch1 >= 0)rec_roll=pre_ch1;
				TIM1->CCER &= ~TIM_CCER_CC1P;

				/*
				 * ch1_rising 65000
				 * ch1 falling 570
				 * pre_ch1 = pre_ch1 falling - pre_ch1_rising = 570 - 65000 = -64430
				 * pre_ch1 +=0xFFFF(65536) --> 1106
				 */
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			if((TIM1->CCER & TIM_CCER_CC2P)==0)
			{
				ch2_rising = TIM1->CCR2;
				TIM1->CCER |= TIM_CCER_CC2P;
			}
			else
			{
				ch2_falling = TIM1->CCR2;
				pre_ch2 = ch2_falling - ch2_rising;
				if(pre_ch2 < 0)pre_ch2 += 0xFFFF;
				pre_ch2 = degree_change_percentage(pre_ch2, 2140, 3862, 0, 1000);
				if(pre_ch2 <= 1000 && pre_ch2 >= 0)rec_pitch=pre_ch2;
				TIM1->CCER &= ~TIM_CCER_CC2P;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			if((TIM1->CCER & TIM_CCER_CC3P)==0)
			{
				ch3_rising = TIM1->CCR3;
				TIM1->CCER |= TIM_CCER_CC3P;
			}
			else
			{
				ch3_falling = TIM1->CCR3;
				pre_ch3 = ch3_falling - ch3_rising;
				if(pre_ch3 < 0)pre_ch3 += 0xFFFF;
				pre_ch3 = degree_change_percentage(pre_ch3, 2140, 3862, 0, 1000);
				if(pre_ch3 <= 1000 && pre_ch3 >= 0)rec_throttle=pre_ch3;
				TIM1->CCER &= ~TIM_CCER_CC3P;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			if((TIM1->CCER & TIM_CCER_CC4P)==0)
			{
				ch4_rising = TIM1->CCR4;
				TIM1->CCER |= TIM_CCER_CC4P;
			}
			else
			{
				ch4_falling = TIM1->CCR4;
				pre_ch4 = ch4_falling - ch4_rising;
				if(pre_ch4 < 0)pre_ch4 += 0xFFFF;
				pre_ch4 = degree_change_percentage(pre_ch4, 2140, 3862, 0, 1000);
				if(pre_ch4 <= 1000 && pre_ch4 >= 0)rec_yaw=pre_ch4;
				TIM1->CCER &= ~TIM_CCER_CC4P;
			}
			break;
		default:
			break;
		}
	}

	if(htim->Instance == TIM3) {
		if((TIM3->CCER & TIM_CCER_CC1P)==0)
					{
						ch1_rising = TIM3->CCR1;
						TIM3->CCER |= TIM_CCER_CC1P;
					}

					else
					{
						ch1_falling = TIM3->CCR1;
						pre_ch1 = ch1_falling - ch1_rising;
						//if(pre_ch1 < 0)pre_ch1 += 0xFFFF;
						pre_ch1 = degree_change_percentage(pre_ch1, 2140, 3862, 0,1000);
						if(pre_ch1 <= 1000 && pre_ch1 >= 0)rec_roll=pre_ch1;
						TIM3->CCER &= ~TIM_CCER_CC1P;

						/*
						 * ch1_rising 65000
						 * ch1 falling 570
						 * pre_ch1 = pre_ch1 falling - pre_ch1_rising = 570 - 65000 = -64430
						 * pre_ch1 +=0xFFFF(65536) --> 1106
						 */
					}

	}
};
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
