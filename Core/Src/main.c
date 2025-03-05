/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define Motor Control Pins (L298N Motor Driver)
#define LEFT_IN1  GPIO_PIN_3  // Left motor forward (PA1)
#define LEFT_IN2  GPIO_PIN_4  // Left motor backward (PA2)
#define RIGHT_IN3 GPIO_PIN_2  // Right motor forward (PA3)
#define RIGHT_IN4 GPIO_PIN_1  // Right motor backward (PA4)
#define LEFT_PWM  TIM_CHANNEL_1 // PA8 (PWM for left motor)/
#define RIGHT_PWM TIM_CHANNEL_2 // PA9 (PWM for right motor)

// IR Sensor Pins
#define LEFT_SENSOR3_PIN  GPIO_PIN_5  // PA5
#define LEFT_SENSOR2_PIN  GPIO_PIN_6  // PA6
#define LEFT_SENSOR_PIN  GPIO_PIN_7  // PA7
#define RIGHT_SENSOR3_PIN GPIO_PIN_5//	pB5
#define RIGHT_SENSOR2_PIN GPIO_PIN_6 // PB6
#define RIGHT_SENSOR_PIN GPIO_PIN_7  // PB7
// PWM Speed Limits
#define BASE_SPEED  300  // Base speed (0-1000 range)
#define TURN_SPEED  250
// Reduced speed when turning
#define STOP_SPEED  0 // Stop condition // Adjust as needed (0-100% PWM)

/* TCS34725 RGB Color Sensor */
#define TCS34725_ADDR           (0x29 << 1)  // I2C address shifted for HAL
#define TCS34725_COMMAND_BIT    0x80

/* TCS34725 Registers */
#define TCS34725_ENABLE         0x00
#define TCS34725_ATIME          0x01
#define TCS34725_ID_REG         0x12
#define TCS34725_CDATAL         0x14
#define TCS34725_RDATAL         0x16
#define TCS34725_GDATAL         0x18
#define TCS34725_BDATAL         0x1A

/* TCS34725 Enable register values */
#define TCS34725_ENABLE_PON     0x01  // Power ON
#define TCS34725_ENABLE_AEN     0x02  // RGBC enable
#define TCS34725_ENABLE_AIEN    0x10  // RGBC interrupt enable
#define TCS34725_ID             0x12  // Register value

/* RGB Thresholds for red detection */
#define RED_THRESHOLD           200    // Minimum red value
#define RED_RATIO_THRESHOLD     1.5f   // Red must be this times larger than green and blue

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
I2C_HandleTypeDef hi2c2;  // I2C2 for TCS34725

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
bool tcs34725_init(void);
bool tcs34725_getRGBC(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
bool isRedLine(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void moveForward(int leftSpeed, int rightSpeed) {
    HAL_GPIO_WritePin(GPIOA, LEFT_IN1, GPIO_PIN_SET);  //
    HAL_GPIO_WritePin(GPIOA, LEFT_IN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN4, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM, leftSpeed);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_PWM, rightSpeed);
}

void moveBackward(int leftSpeed, int rightSpeed) {
    HAL_GPIO_WritePin(GPIOA, LEFT_IN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LEFT_IN2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN4, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM, leftSpeed);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_PWM, rightSpeed);
}


void turnLeft(int leftSpeed, int rightSpeed) {
    HAL_GPIO_WritePin(GPIOA, LEFT_IN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, LEFT_IN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN4, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM, leftSpeed);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_PWM, rightSpeed);
}

void turnRight(int leftSpeed, int rightSpeed) {
    HAL_GPIO_WritePin(GPIOA, LEFT_IN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, LEFT_IN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN4, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM, leftSpeed);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_PWM, rightSpeed);
}

// Function to stop the car
void stopCar() {
    HAL_GPIO_WritePin(GPIOA, LEFT_IN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LEFT_IN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RIGHT_IN4, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM, STOP_SPEED);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_PWM, STOP_SPEED);
}

bool tcs34725_init(void) {
    uint8_t id = 0;
    uint8_t data;

    // Check sensor ID
    HAL_I2C_Mem_Read(&hi2c2, TCS34725_ADDR, TCS34725_ID_REG | TCS34725_COMMAND_BIT, 1, &id, 1, 100);
    if (id != TCS34725_ID) {
        return false;  // Sensor not found
    }

    // Set integration time (ATIME register)
    data = 0xC0;  // 16ms integration time
    HAL_I2C_Mem_Write(&hi2c2, TCS34725_ADDR, TCS34725_ATIME | TCS34725_COMMAND_BIT, 1, &data, 1, 100);

    // Enable the device
    data = TCS34725_ENABLE_PON;
    HAL_I2C_Mem_Write(&hi2c2, TCS34725_ADDR, TCS34725_ENABLE | TCS34725_COMMAND_BIT, 1, &data, 1, 100);
    HAL_Delay(3);  // Wait for power on

    // Enable RGBC and Power
    data = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
    HAL_I2C_Mem_Write(&hi2c2, TCS34725_ADDR, TCS34725_ENABLE | TCS34725_COMMAND_BIT, 1, &data, 1, 100);

    return true;
}


bool tcs34725_getRGBC(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    uint8_t data[8];

    if (HAL_I2C_Mem_Read(&hi2c2, TCS34725_ADDR, TCS34725_CDATAL | TCS34725_COMMAND_BIT, 1, data, 8, 100) != HAL_OK) {
        return false;
    }

    // Convert data (low byte first, then high byte)
    *c = (data[1] << 8) | data[0];
    *r = (data[3] << 8) | data[2];
    *g = (data[5] << 8) | data[4];
    *b = (data[7] << 8) | data[6];

    return true;
}


bool isRedLine(void) {
    uint16_t r, g, b, c;

    if (!tcs34725_getRGBC(&r, &g, &b, &c)) {
        return false;  // Error reading sensor
    }

    if (r > RED_THRESHOLD &&
        (float)r/g > RED_RATIO_THRESHOLD &&
        (float)r/b > RED_RATIO_THRESHOLD) {
        return true;
    }

    return false;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // Variables for red line detection
  bool redLineDetected = false;
  uint32_t redDetectionTime = 0;
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
  MX_TIM1_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  // Initialize RGB sensor
  if (!tcs34725_init()) {
  }

  /* Start PWM channels */
  HAL_TIM_PWM_Start(&htim1, LEFT_PWM);
  HAL_TIM_PWM_Start(&htim1, RIGHT_PWM);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // Check if red line is detected
    if (isRedLine() && !redLineDetected) {
        // Red line just detected
        redLineDetected = true;
        redDetectionTime = HAL_GetTick();
        stopCar();  // Stop immediately when red line is detected
    }

    // Check if we need to resume after red line detection
    if (redLineDetected) {
        if (HAL_GetTick() - redDetectionTime >= 5000) {  // 5 seconds passed
            redLineDetected = false;  // Reset detection flag
            moveForward(BASE_SPEED, BASE_SPEED);  // Resume movement
        } else {
            // Still in the 5-second stop period
            continue;  // Skip normal line following logic
        }
    }

    // Regular line following logic (only executed when not handling red line)
    // Read IR sensors
    bool leftDetected = HAL_GPIO_ReadPin(GPIOA, LEFT_SENSOR_PIN);
    bool leftDetected2 = HAL_GPIO_ReadPin(GPIOA, LEFT_SENSOR2_PIN);
    bool leftDetected3 = HAL_GPIO_ReadPin(GPIOA, LEFT_SENSOR3_PIN);
    bool rightDetected = HAL_GPIO_ReadPin(GPIOB, RIGHT_SENSOR_PIN);
    bool rightDetected2 = HAL_GPIO_ReadPin(GPIOB, RIGHT_SENSOR2_PIN);
    bool rightDetected3 = HAL_GPIO_ReadPin(GPIOB, RIGHT_SENSOR3_PIN);

    // All sensors on the line - move forward
    if (leftDetected && leftDetected2 && leftDetected3 && rightDetected && rightDetected2 && rightDetected3) {
        moveForward(BASE_SPEED, BASE_SPEED);
    }
    // If the central sensors detect black (off the line) - move backward then forward
    else if (!leftDetected && !rightDetected && rightDetected2 && rightDetected3 && leftDetected2 && leftDetected3) {
        moveBackward(BASE_SPEED, BASE_SPEED);
        HAL_Delay(500);   // Wait for 0.5 seconds
        moveForward(BASE_SPEED, BASE_SPEED);
    }
    // Left sensor off line - turn right
    else if (!leftDetected && rightDetected && rightDetected2 && rightDetected3 && leftDetected2 && leftDetected3) {
        turnRight(TURN_SPEED * 1.5, STOP_SPEED);
    }
    // Right sensor off line - turn left
    else if (!rightDetected && leftDetected && rightDetected2 && rightDetected3 && leftDetected2 && leftDetected3) {
        turnLeft(STOP_SPEED, TURN_SPEED * 1.5);
    }
    // Left sensor off line but others on line - turn right less sharply
    else if ((!leftDetected) && leftDetected2 && leftDetected3 && rightDetected && rightDetected2 && rightDetected3) {
        turnRight(TURN_SPEED, STOP_SPEED);
    }
    // Right sensor off line but others on line - turn left less sharply
    else if ((!rightDetected) && rightDetected2 && rightDetected3 && leftDetected && leftDetected2 && leftDetected3 ) {
    	turnLeft(STOP_SPEED, TURN_SPEED);
    }
    // Multiple left sensors off line - sharp right turn
    else if ((!leftDetected || !leftDetected2 || !leftDetected3) && rightDetected && rightDetected2 && rightDetected3) {
        turnRight(TURN_SPEED * 1.3, STOP_SPEED);
    }
    // Multiple right sensors off line - sharp left turn
    else if ((!rightDetected || !rightDetected2 || !rightDetected3) && leftDetected && leftDetected2 && leftDetected3) {
        turnLeft(STOP_SPEED, TURN_SPEED * 1.3);
    }
    // Complex case - handle with specialized turns
    else if (!rightDetected && (!leftDetected2 || !leftDetected3)) {
    	turnLeft(STOP_SPEED, TURN_SPEED);
    }
    // Complex case - handle with specialized turns
    else if (!leftDetected && (!rightDetected2 || !rightDetected3)) {
        moveForward(TURN_SPEED, STOP_SPEED);
    }
    // All sensors off line - possibly at intersection or end of line
    else if (!leftDetected && !leftDetected2 && !leftDetected3 && !rightDetected && !rightDetected2 && !rightDetected3) {
        moveForward(STOP_SPEED, STOP_SPEED);
    }

  }
//
    //HAL_Delay(10);  // Small delay for stability
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : right3_Pin right2_Pin right1_Pin */
  GPIO_InitStruct.Pin = right3_Pin|right2_Pin|right1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : left3_Pin left2_Pin left1_Pin */
  GPIO_InitStruct.Pin = left3_Pin|left2_Pin|left1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
