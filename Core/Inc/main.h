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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define PFC_PWM_Pin GPIO_PIN_0
#define PFC_PWM_GPIO_Port GPIOA
#define Relay_Pin GPIO_PIN_4
#define Relay_GPIO_Port GPIOA
#define UL_PWM_Pin GPIO_PIN_7
#define UL_PWM_GPIO_Port GPIOA
#define VL_PWM_Pin GPIO_PIN_0
#define VL_PWM_GPIO_Port GPIOB
#define WL_PWM_Pin GPIO_PIN_1
#define WL_PWM_GPIO_Port GPIOB
#define UH_PWM_Pin GPIO_PIN_8
#define UH_PWM_GPIO_Port GPIOA
#define VH_PWM_Pin GPIO_PIN_9
#define VH_PWM_GPIO_Port GPIOA
#define WH_PWM_Pin GPIO_PIN_10
#define WH_PWM_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_5
#define CE_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_6
#define CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
