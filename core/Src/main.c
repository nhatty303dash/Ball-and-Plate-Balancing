/* USER CODE BEGIN Header */
/**
****************************************************************************
**
* @file : main.c
* @brief : Main program body
*
* UART2 OPTIMIZATION IMPLEMENTATION:
* ==================================
* - UART2 hardware remains initialized for future compatibility
* - Software features (debug, manual input, status) are conditionally compiled
* - Performance mode: All UART2 features disabled (macros set to 0)
* - UART1 (camera) always enabled for real-time ball tracking
*
* Benefits: Reduced interrupt overhead, faster main loop, less memory usage
****************************************************************************
**
* @attention
*
* Copyright (c) 2025 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
****************************************************************************
**
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>  // For fabsf function
#include "ball_and_plate.h"
#include "ball_position.h"
#include "pid_controller.h"
#include "fuzzy_sugeno_pid.h"
#include "uart_handler.h"
#include "pca9685_servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#endif

// ============================================================================
// UART2 FUNCTIONALITY CONTROL
// ============================================================================
// Set to 1 to enable UART2 features, 0 to disable for performance
#define UART2_MANUAL_INPUT      0   // Manual coordinate input via UART2  
#define UART2_DEBUG_OUTPUT      0   // Debug output via UART2 - DISABLED to avoid bugs

// ============================================================================
// PID CONTROL SYSTEM WITH ALGORITHM SELECTION
// ============================================================================

#define USE_FUZZY_PID 1  // Switch: 1 = Fuzzy PID, 0 = Classic PID

// ==========================================================================
// SERVO CALIBRATION MODE (compile-time selectable)
// ==========================================================================

#define SERVO_CALIBRATION_MODE   0      //1=enable, 0 = disable
#define CALIB_SERVO              3  // 1,2,3
#define CALIB_PULSE_US          810


// ============================================================================
// UART2 HELPER MACROS - Conditional compilation for performance
// ============================================================================
#if UART2_DEBUG_OUTPUT  
    #define UART2_DEBUG_PRINT(msg) UartTransmitString(&uartHandler, msg)
#else
    #define UART2_DEBUG_PRINT(msg) // Do nothing - optimized out
#endif

#if UART2_MANUAL_INPUT
    #define UART2_PROCESS_INPUT() UartProcessData(&uartHandler, &ball)
#else
    #define UART2_PROCESS_INPUT() false // Return false - no manual input
#endif
// ============================================================================

// LED definitions - removed for cleaner code
//#define SERVO_UPDATE_PERIOD 20  //
#define STATUS_UPDATE_PERIOD 2000 // 2000ms update period for status messages
#define BALL_TIMEOUT_PERIOD 2500 // Moderate timeout for stability

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// Robot and control variables
RobotState robot;
BallPosition ball;
UartHandler uart1Handler;

// Control algorithm variables - only one will be used based on USE_FUZZY_PID
#if USE_FUZZY_PID
    FuzzySugenoPID fuzzyPid;
#else
    PIDController pid;
#endif

// Camera data
CameraData cameraData;

// Timing variables
uint32_t lastCameraCheckTime = 0;
uint32_t ball_lost_timestamp = 0;
uint32_t lastPidUpdateTime = 0;

// Dynamic max_theta calculation variables
float last_height = 0.0f;
bool height_changed = false;
uint32_t lastMaxThetaUpdateTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
/* System startup complete */

/* Initialize system components */
// Initialize PCA9685 for servo control
PCA9685_Init(50); // 50Hz frequency for servos

// I2C connectivity test removed for clean build

// Initialize the robot
RobotInit(&robot);

#if SERVO_CALIBRATION_MODE
// ================= Servo Calibration Mode =================
// Move only selected servo to specified pulse width and halt here
{
    uint8_t channel = 0;
    if (CALIB_SERVO == 1) channel = SERVO1_CHANNEL;
    else if (CALIB_SERVO == 2) channel = SERVO2_CHANNEL;
    else if (CALIB_SERVO == 3) channel = SERVO3_CHANNEL;
    else channel = SERVO1_CHANNEL;

    PCA9685_SetServoPWMus(channel, (uint16_t)CALIB_PULSE_US);

    // Hold position for manual measurement; blink LED if available
    while (1) {
        HAL_Delay(100);
    }
}
#endif

// Initialize controller based on selection
#if USE_FUZZY_PID
    // Initialize Fuzzy PID controller
    FuzzyPIDInit(&fuzzyPid, 2.24f, 0.7f, 0.38f, 1.5f, 0.06f, 1); //quay nhiều vòng

    // Initialize fuzzy parameters
    FuzzyPIDInitFuzzy(&fuzzyPid,
                      8.0f,     // E_max: Maximum error for normalization
                      36.0f,    // D_max: Maximum derivative error for normalization
                      0.85f,     // K_e: Error scaling factor
                      0.8f,     // K_d_in: Derivative error scaling factor
                      0.45f,    // A_PD: PD gain adjustment coefficient
                      0.5f,     // A_I: I gain adjustment coefficient
                      0.35f,     // U_mid: Mid-level fuzzy output
                      0.7f);    // U_max: Maximum fuzzy output


    FuzzyPIDSetDerivativeFilter(&fuzzyPid, 1.0f);

    // Set gain limits for adaptive control
    FuzzyPIDSetGainLimits(&fuzzyPid,
                          0.1f, 3.0f,   // Kp limits
                          0.0f, 0.8f,  // Ki limits
                          0.05f, 2.5f);  // Kd limits

    // Enable fuzzy control
    FuzzyPIDEnableFuzzy(&fuzzyPid, true);

    // Set max_theta cho Fuzzy PID
    FuzzyPIDSetMaxTheta(&fuzzyPid, 5.0f);  // max_theta = 10.0°

    // Set target
    FuzzyPIDSetTarget(&fuzzyPid, 0.0f, 0.0f);
#else
    // Initialize PID
    PIDInit(&pid, 2.24f, 0.7f, 0.38f, 1.5f, 0.06f, 1);//1.235f, 0.37f, 0.3f, 1.4f, 0.023f, 0
    //1.21f, 0.065f, 0.27f, 1.5f, 0.016f, 0) 5.0 đáp ứng nhanh
    //0.52f, 0.1f, 0.18f, 1.5f, 0.05f, 0) 3.0 đáp ứng chậm
    //0.74f, 0.008f, 0.14f, 1.45f, 0.065, 0) 2.75 hơi trượt quanh target
    // Set derivative filter coefficient
    PIDSetDerivativeFilter(&pid, 1.0f);
    // Set max_theta
    PIDSetMaxTheta(&pid, 5.0f);  // max_theta = 10.0°
    PIDSetTarget(&pid, 0.0f, 0.0f);
#endif



// Initialize ball position tracking
BallPositionInit(&ball);

// Initialize UART with DMA

// UartInit(&uartHandler, &huart2, &hdma_usart2_rx);      // Debug → UART2
// UartStartReceive(&uartHandler);

/* Initialize UART1 for camera data */
UartInit(&uart1Handler, &huart1, &hdma_usart1_rx);     // Camera → UART1
UartStartReceive(&uart1Handler);



/* Initial delay to ensure all systems are ready */
HAL_Delay(500);

/* Initialize servos to level position at current height */
PerformInitialMovements(&robot);

/* Initialize height tracking for dynamic max_theta calculation */
last_height = robot.geom.h;
height_changed = false;
lastMaxThetaUpdateTime = HAL_GetTick();

/* Main loop variables */
uint32_t currentTime;
float tilt = 0.0f, phi = 0.0f;  // Initialize PID output variables
bool plate_reset_initiated = false; // Flag to track if plate reset has been initiated

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    /* Update servo motion if in progress - MUST BE CALLED EVERY LOOP */
    UpdateServoMotion();

    /* Get current time */
    currentTime = HAL_GetTick();

    /* Check DMA buffer for camera UART only */
    UartCheckDMA(&uart1Handler);  // UART1 - Camera

    /* PRIORITY: Check for target coordinate frames from GUI immediately after DMA check */
    float target_x, target_y;
    if (UartProcessTargetFrame(&uart1Handler, &target_x, &target_y)) {
        // New target received - reset PID to prevent overshoot
#if USE_FUZZY_PID
        FuzzyPIDSetTargetWithReset(&fuzzyPid, target_x, target_y);
#else
        PIDSetTargetWithReset(&pid, target_x, target_y);
#endif
    }

    // Process trajectory frames (e.g., circle) and update dynamic setpoint
    if (UartProcessTrajectoryFrame(&uart1Handler)) {
        // New trajectory command processed (e.g., start circle)
        // No immediate action needed; UartUpdateTrajectory will drive targets
    }
    UartUpdateTrajectory(&uart1Handler);
    if (uart1Handler.trajectory.active && uart1Handler.trajectory.mode == TRAJECTORY_MODE_CIRCLE) {
#if USE_FUZZY_PID
        FuzzyPIDSetTarget(&fuzzyPid, uart1Handler.trajectory.target_x, uart1Handler.trajectory.target_y);
#else
        PIDSetTarget(&pid, uart1Handler.trajectory.target_x, uart1Handler.trajectory.target_y);
#endif
    }

    /* DYNAMIC MAX_THETA CALCULATION (like Python) */
    // Check if height has changed significantly (threshold = 0.1cm)
    if (fabsf(robot.geom.h - last_height) > 0.1f) {
        height_changed = true;
        last_height = robot.geom.h;
    }
    
    // Update max_theta every 100ms if height changed (like Python)
    if (height_changed && (currentTime - lastMaxThetaUpdateTime >= 100)) {
        // Calculate new max_theta based on current height (like Python)
        ComputeMaxThetaAtHeight(&robot, robot.geom.h);
        
        // Update PID controllers with new max_theta
#if USE_FUZZY_PID
        FuzzyPIDSetMaxTheta(&fuzzyPid, robot.geom.maxtheta);
#else
        PIDSetMaxTheta(&pid, robot.geom.maxtheta);
#endif
        
        
        height_changed = false;
        lastMaxThetaUpdateTime = currentTime;
    }

    /* Check camera timeout every 500ms */ //lấy dữ liệu mới nhất
    if (currentTime - lastCameraCheckTime >= 500) {
        UartCameraTimeout(&uart1Handler, 2000); // 2 second timeout
        lastCameraCheckTime = currentTime;
    }

    /* Try to get camera data from UART1 */
    bool cameraDataAvailable = UartGetLatestCameraData(&uart1Handler, &cameraData);

    /* Camera data processing and ball loss detection */
    if (cameraDataAvailable) {
        // Ball is detected: update its position and reset the loss timer.
        BallPositionUpdate(&ball, cameraData.x, cameraData.y);
        ball_lost_timestamp = 0;
        plate_reset_initiated = false; // Reset the plate reset flag
    } else {
        // No new camera data: check if the ball was previously detected.
        if (ball.isDetected) {
            if (ball_lost_timestamp == 0) {
                // This is the first frame without data; start the timer.
                ball_lost_timestamp = currentTime;
            } else if (currentTime - ball_lost_timestamp > 3000 && !plate_reset_initiated) {
                // More than 1 second has passed since the ball was lost.
                ball.isDetected = false; // Officially declare the ball as lost.

                // Reset controller
#if USE_FUZZY_PID
                FuzzyPIDReset(&fuzzyPid);
#else
                PIDReset(&pid);
#endif
                // Smoothly return the plate to the center.
                StartMoveTo(&robot, 0.0f, 0.0f, robot.geom.h, 1000);
                plate_reset_initiated = true; // Mark that reset has been initiated
            }
        }
    }

    /* Update control if ball detected OR within grace period (not resetting) */
    bool within_grace_period = (ball_lost_timestamp > 0 && (currentTime - ball_lost_timestamp < 2000));
    bool should_update_control = (ball.isDetected || within_grace_period) && !plate_reset_initiated;
    // Allow control updates while trajectory (circle) is active, even if ball is not detected
    if (uart1Handler.trajectory.active) {
        should_update_control = true;
    }
    
    // Additional condition: continue control if we have valid ball position data
    // This ensures PID continues even when ball is stationary (no new camera data)
    bool has_valid_position = (ball.isDetected && (ball.x != 0.0f || ball.y != 0.0f));
    
    if (should_update_control || has_valid_position) {
        // Update controller - either with fresh data or maintaining control with last known position
        lastPidUpdateTime = currentTime;
        
        // === UPDATE CONTROLLER BASED ON SELECTION ===
#if USE_FUZZY_PID
        // Use Fuzzy PID controller
        tilt = FuzzyPIDUpdateWithTime(&fuzzyPid, ball.x, ball.y);
        phi = fuzzyPid.output_phi;
#else
        //  PID controller feedback từ ballx bally
        tilt = PIDUpdateSimpleWithTime(&pid, ball.x, ball.y);
        phi = pid.output_phi;
#endif
        
        // điều khiển cân bằng
        StartMoveTo(&robot, tilt, phi, robot.geom.h, 0);
    }

    /* Ultra-minimal delay - yield to interrupts only */
    HAL_Delay(0); // No fixed delay, just yield CPU for interrupts
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
    // System error - infinite loop
    HAL_Delay(100);
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
/* Enter infinite loop */
Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
