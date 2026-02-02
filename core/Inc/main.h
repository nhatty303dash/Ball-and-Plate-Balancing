/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// ============================================================================
// UART2 OPTIMIZATION NOTES
// ============================================================================
// UART2 hardware is initialized but software features can be disabled for performance:
// 
// Control macros in main.c:
// - UART2_DEBUG_ENABLED     : Debug messages via UART2 (0=disabled)
// - UART2_MANUAL_INPUT      : Manual coordinate input via UART2 (0=disabled)  
// - UART2_STATUS_OUTPUT     : Status text output via UART2 (1=enabled for PSO logging)
// - UART2_PSO_ONBOARD       : PSO onboard optimization (1=enabled)
//
// When PSO Onboard is enabled, UART2 is used for PSO logging and commands.
// When all are disabled (0), the system runs in performance mode with:
// - No UART2 interrupt processing overhead
// - No debug string formatting 
// - Reduced memory usage
// - Faster main loop execution
//
// UART1 (camera data) is always enabled and optimized for real-time operation.
// ============================================================================

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
