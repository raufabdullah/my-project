/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Motor Driver Pin Definitions
// Right Motor
#define RIGHT_MOTOR_D8_PORT   GPIOB
#define RIGHT_MOTOR_D8_PIN    GPIO_PIN_2
#define RIGHT_MOTOR_D12_PORT  GPIOB
#define RIGHT_MOTOR_D12_PIN   GPIO_PIN_1
// D10 (PA6) is TIM3_CH1 for Right Motor PWM

// Left Motor
#define LEFT_MOTOR_D7_PORT    GPIOB
#define LEFT_MOTOR_D7_PIN     GPIO_PIN_10
#define LEFT_MOTOR_D6_PORT    GPIOB
#define LEFT_MOTOR_D6_PIN     GPIO_PIN_11
// D9 (PA4) is TIM3_CH2 for Left Motor PWM

// Motor Direction Definitions
#define MOTOR_FORWARD   0
#define MOTOR_BACKWARD  1
#define MOTOR_STOP      2

// PWM Configuration: 10kHz frequency, 0-1000 duty cycle range
#define PWM_PERIOD  999

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  // Start PWM on TIM3 Channel 1 (PA6 - D10 for Right Motor)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  // Start PWM on TIM3 Channel 2 (PA4 - D9 for Left Motor)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // ========== BOTH MOTORS FORWARD AT 50% SPEED (500) ==========
    // Right Motor Forward: D8=HIGH, D12=LOW, PWM=500
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // D8 (PB2) = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // D12 (PB1) = LOW
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);     // D10 (PA6) PWM = 500 (50%)
    
    // Left Motor Forward: D7=HIGH, D6=LOW, PWM=500
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // D7 (PB10) = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // D6 (PB11) = LOW
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);     // D9 (PA4) PWM = 500 (50%)
    
    HAL_Delay(3000);
    
    // ========== STOP ALL MOTORS ==========
    // Right Motor Stop: D8=HIGH, D12=HIGH, PWM=0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // D8 (PB2) = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // D12 (PB1) = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);       // D10 (PA6) PWM = 0
    
    // Left Motor Stop: D7=HIGH, D6=HIGH, PWM=0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // D7 (PB10) = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // D6 (PB11) = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);       // D9 (PA4) PWM = 0
    
    HAL_Delay(2000);
    
    // ========== BOTH MOTORS BACKWARD AT 50% SPEED (500) ==========
    // Right Motor Backward: D8=LOW, D12=HIGH, PWM=500
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // D8 (PB2) = LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // D12 (PB1) = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);     // D10 (PA6) PWM = 500 (50%)
    
    // Left Motor Backward: D7=LOW, D6=HIGH, PWM=500
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // D7 (PB10) = LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // D6 (PB11) = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);     // D9 (PA4) PWM = 500 (50%)
    
    HAL_Delay(3000);
    
    // ========== STOP ALL MOTORS ==========
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // D8 (PB2) = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // D12 (PB1) = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);       // D10 (PA6) PWM = 0
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // D7 (PB10) = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // D6 (PB11) = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);       // D9 (PA4) PWM = 0
    
    HAL_Delay(2000);
    
    // ========== ROTATE CLOCKWISE (Right Backward, Left Forward) ==========
    // Right Motor Backward
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // D8 (PB2) = LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // D12 (PB1) = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);     // D10 (PA6) PWM = 500 (50%)
    
    // Left Motor Forward
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // D7 (PB10) = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // D6 (PB11) = LOW
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);     // D9 (PA4) PWM = 500 (50%)
    
    HAL_Delay(2000);
    
    // ========== STOP ALL MOTORS ==========
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);       
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);       
    
    HAL_Delay(2000);
    
    // ========== ROTATE ANTICLOCKWISE (Right Forward, Left Backward) ==========
    // Right Motor Forward
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // D8 (PB2) = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // D12 (PB1) = LOW
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);     // D10 (PA6) PWM = 500 (50%)
    
    // Left Motor Backward
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // D7 (PB10) = LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // D6 (PB11) = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);     // D9 (PA4) PWM = 500 (50%)
    
    HAL_Delay(2000);
    
    // ========== STOP ALL MOTORS ==========
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);       
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);       
    
    HAL_Delay(3000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;  // 72MHz / 72 = 1MHz timer clock
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_PERIOD;  // 1MHz / 1000 = 10kHz PWM frequency
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Initialize motor control PWM
  * @param  None
  * @retval None
  */


/**
  * @brief  Control right motor direction and speed
  * @param  direction: MOTOR_FORWARD, MOTOR_BACKWARD, or MOTOR_STOP
  * @param  speed: PWM duty cycle (0-255), 0 = stop
  * @retval None
  * 
  * Right Motor Control Logic:
  * D8=HIGH, D12=LOW, PWM=0-255: Forward
  * D8=LOW, D12=HIGH, PWM=0-255: Backward
  * D8=HIGH, D12=HIGH or D8=LOW, D12=LOW: Stop
  */
void SetRightMotor(uint8_t direction, uint8_t speed)
{
  if (speed == 0 || direction == MOTOR_STOP)
  {
    // Stop: Set both pins HIGH or both LOW
    HAL_GPIO_WritePin(RIGHT_MOTOR_D8_PORT, RIGHT_MOTOR_D8_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RIGHT_MOTOR_D12_PORT, RIGHT_MOTOR_D12_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  }
  else if (direction == MOTOR_FORWARD)
  {
    // Forward: D8=HIGH, D12=LOW
    HAL_GPIO_WritePin(RIGHT_MOTOR_D8_PORT, RIGHT_MOTOR_D8_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RIGHT_MOTOR_D12_PORT, RIGHT_MOTOR_D12_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  }
  else if (direction == MOTOR_BACKWARD)
  {
    // Backward: D8=LOW, D12=HIGH
    HAL_GPIO_WritePin(RIGHT_MOTOR_D8_PORT, RIGHT_MOTOR_D8_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_MOTOR_D12_PORT, RIGHT_MOTOR_D12_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  }
}

/**
  * @brief  Control left motor direction and speed
  * @param  direction: MOTOR_FORWARD, MOTOR_BACKWARD, or MOTOR_STOP
  * @param  speed: PWM duty cycle (0-255), 0 = stop
  * @retval None
  * 
  * Left Motor Control Logic:
  * D7=HIGH, D6=LOW, PWM=0-255: Forward
  * D7=LOW, D6=HIGH, PWM=0-255: Backward
  * D7=HIGH, D6=HIGH or D7=LOW, D6=LOW: Stop
  */
void SetLeftMotor(uint8_t direction, uint8_t speed)
{
  if (speed == 0 || direction == MOTOR_STOP)
  {
    // Stop: Set both pins HIGH or both LOW
    HAL_GPIO_WritePin(LEFT_MOTOR_D7_PORT, LEFT_MOTOR_D7_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEFT_MOTOR_D6_PORT, LEFT_MOTOR_D6_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  }
  else if (direction == MOTOR_FORWARD)
  {
    // Forward: D7=HIGH, D6=LOW
    HAL_GPIO_WritePin(LEFT_MOTOR_D7_PORT, LEFT_MOTOR_D7_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEFT_MOTOR_D6_PORT, LEFT_MOTOR_D6_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
  }
  else if (direction == MOTOR_BACKWARD)
  {
    // Backward: D7=LOW, D6=HIGH
    HAL_GPIO_WritePin(LEFT_MOTOR_D7_PORT, LEFT_MOTOR_D7_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEFT_MOTOR_D6_PORT, LEFT_MOTOR_D6_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
  }
}

/**
  * @brief  Control both motors with same direction and speed
  * @param  direction: MOTOR_FORWARD, MOTOR_BACKWARD, or MOTOR_STOP
  * @param  speed: PWM duty cycle (0-25
  */
void SetBothMotors(uint8_t direction, uint8_t speed)
{
  SetRightMotor(direction, speed);
  SetLeftMotor(direction, speed);
}

/**
  * @brief  Stop all motors
  * @param  None
  * @retval None
  */
void StopAllMotors(void)
{
  SetRightMotor(MOTOR_STOP, 0);
  SetLeftMotor(MOTOR_STOP, 0);
}

/**
  * @brief  Rotate robot clockwise (right motor backward, left motor forward)
  * @param  speed: PWM duty cycle (0-255)
  * @retval None
  * @note   Robot rotates in place clockwise when viewed from above
  */
void RotateClockwise(uint8_t speed)
{
  SetRightMotor(MOTOR_BACKWARD, speed);  // Right motor backward
  SetLeftMotor(MOTOR_FORWARD, speed);    // Left motor forward
}

/**
  * @brief  Rotate robot anticlockwise (right motor forward, left motor backward)
  * @param  speed: PWM duty cycle (0-255)
  * @retval None
  * @note   Robot rotates in place anticlockwise when viewed from above
  */
void RotateAntiClockwise(uint8_t speed)
{
  SetRightMotor(MOTOR_FORWARD, speed);   // Right motor forward
  SetLeftMotor(MOTOR_BACKWARD, speed);   // Left motor backward
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
