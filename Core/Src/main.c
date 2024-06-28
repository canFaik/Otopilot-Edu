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

float output_roll = 0;
float output_pitch = 0;

uint16_t motor_1_throttle = 0;
uint16_t motor_2_throttle = 0;
uint16_t motor_3_throttle = 0;
uint16_t motor_4_throttle = 0;


volatile uint32_t ch1_rising = 0, ch2_rising = 0, ch3_rising = 0, ch4_rising = 0;
volatile uint32_t ch1_falling = 0, ch2_falling = 0, ch3_falling = 0, ch4_falling = 0;
volatile uint32_t pre_ch1 = 0, ch1 = 0, pre_ch2 = 0, ch2 = 0, pre_ch3 = 0, ch3 = 0, pre_ch4 = 0, ch4 = 0;

// PID parametreleri
float kp = 1.0;
float ki =	0.1;
float kd =0.01;

// PID değişkenleri
float error_roll = 0, error_pitch = 0;
float last_error_roll = 0, last_error_pitch = 0;
float integral_roll = 0, integral_pitch = 0;

// Motor hızları
int motor_speeds[4] = {0, 0, 0, 0};

// İstenen açılar
float desired_roll = 0, desired_pitch = 0;

float measured_roll, measured_pitch;

LIS3DSH_DataScaled  X;
LIS3DSH_DataScaledY Y;
LIS3DSH_DataScaledZ Z;

uint16_t degree_change_percentage(uint16_t In, uint16_t Inmin, uint16_t Inmax, uint16_t Outmin, uint16_t Outmax);



uint8_t drdyFlag=0;


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
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void motor_control(void)
{
	motor_1_throttle =1000;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,motor_1_throttle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,motor_2_throttle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,motor_3_throttle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,motor_4_throttle);
}




void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)  // interrupt TIM1 biriminden geliyorsa gir
	{
		switch(htim->Channel) // aktif kanal hangisiyse, o kanalın case'ine git
		{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			if((TIM1->CCER & TIM_CCER_CC1P)==0) // kanalin aktif olmasi kesmenin oradan gelecegi anlamina gelmez/gpio pinini kontrol et
			{
				ch1_rising = TIM1->CCR1; // yukselen kenar degerini kaydet
				TIM1->CCER |= TIM_CCER_CC1P; // polariteyi düsen kenar olarak degistir
			}

			else
			{
				ch1_falling = TIM1->CCR1;
				pre_ch1 = ch1_falling - ch1_rising; // dusen kenar degerini kaydet ve yukselen kenar degerinden cikar
				if(pre_ch1 < 0)pre_ch1 += 0xFFFF;// eger sonuc negatifse taban tumleme yap
				if(pre_ch1 < 2010 && pre_ch1 > 990)ch1=pre_ch1;
				TIM1->CCER &= ~TIM_CCER_CC1P; // polariteyi yukselen kenar olarak ayarla

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
				if(pre_ch2 < 2010 && pre_ch2 > 990)ch2=pre_ch2;
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
				if(pre_ch3 < 2010 && pre_ch3 > 990)ch3=pre_ch3;
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
				if(pre_ch4 < 2010 && pre_ch4 > 990)ch4=pre_ch4;
				TIM1->CCER &= ~TIM_CCER_CC4P;
			}
			break;
		default:
			break;
		}
	}
}

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

void motor_limit(void)
{
    if(motor_speeds[1] <=1155 )
    	 motor_speeds[1] = 1155;

    if(motor_speeds[2] <=1155 )
    	 motor_speeds[2] = 1155;

    if(motor_speeds[3] <=1155 )
    	 motor_speeds[3] = 1155;

    if(motor_speeds[0] <=1155 )
    	 motor_speeds[0] = 1155;


    if(motor_speeds[0] >2200 )
    	 motor_speeds[0] = 2200;

    if(motor_speeds[1] >=2200 )
    	 motor_speeds[1] = 2200;

    if(motor_speeds[2] >=2200 )
    	 motor_speeds[2] = 2200;

    if(motor_speeds[3] >=2200)
    	 motor_speeds[3] = 2200;
}

void motor_compare(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,motor_speeds[0]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,motor_speeds[1]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,motor_speeds[2]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,motor_speeds[3]);
}

void pid_control(float measured_roll, float measured_pitch) {
    // Roll PID calculate

    error_roll = desired_roll - measured_roll;
    integral_roll += error_roll;

     output_roll = kp * error_roll + ki * integral_roll + kd * (error_roll - last_error_roll);

    last_error_roll = error_roll;

    // Pitch PID calculate
    error_pitch = desired_pitch - measured_pitch;
    integral_pitch += error_pitch;

    output_pitch = kp * error_pitch + ki * integral_pitch + kd * (error_pitch - last_error_pitch);
    last_error_pitch = error_pitch;




    // Motor hızlarını ayarlama
    motor_speeds[0] = 1200 + output_roll + output_pitch;  // Motor 1
    motor_speeds[1] = 1200 - output_roll - output_pitch;  // Motor 3
    motor_speeds[2] = 1200 - output_roll + output_pitch;  // Motor 2
    motor_speeds[3] = 1200 + output_roll - output_pitch;  // Motor 4

  /// motor_limit();

    motor_compare();


}


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

  /* HAL PWM STAR */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);





	motor_1_throttle =1000;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,motor_1_throttle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,motor_1_throttle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,motor_1_throttle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,motor_1_throttle);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // TIMX->DIER, CCXIE bitini aktif eder.
	   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);


	  	acces_measure();
		pid_control(measured_roll, measured_pitch);




HAL_Delay(200);

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
  htim1.Init.Prescaler = 41;
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

	drdyFlag = 1;
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}





uint16_t degree_change_percentage(uint16_t In, uint16_t Inmin, uint16_t Inmax, uint16_t Outmin, uint16_t Outmax)
{
	return (In- Inmin) * (Outmax- Outmin) / (Inmax -Inmin) + Outmin;
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
