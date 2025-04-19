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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LONG_DELAY 10000
#define MEDIUM_DELAY 5000
#define SHORT_DELAY 2500
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	volatile uint32_t delayValue;
	uint32_t expiryTick, lastTick;
} BLINKER_TypeDef;
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
#define Blue_B1_Pin GPIO_PIN_13
#define Blue_B1_GPIO_Port GPIOC
#define Blue_B1_EXTI_IRQn EXTI15_10_IRQn
#define Left_VCW_Pin GPIO_PIN_0
#define Left_VCW_GPIO_Port GPIOC
#define Left_VCW_EXTI_IRQn EXTI0_IRQn
#define Right_VCW_Pin GPIO_PIN_1
#define Right_VCW_GPIO_Port GPIOC
#define Right_VCW_EXTI_IRQn EXTI1_IRQn
#define BlueLED_Pin GPIO_PIN_0
#define BlueLED_GPIO_Port GPIOA
#define RedLED_Pin GPIO_PIN_1
#define RedLED_GPIO_Port GPIOA
#define Green_LD2_Pin GPIO_PIN_5
#define Green_LD2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
