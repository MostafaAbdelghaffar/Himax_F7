/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

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
#define LD1_Green_Pin GPIO_PIN_0
#define LD1_Green_GPIO_Port GPIOB
#define MCLK_Pin GPIO_PIN_11
#define MCLK_GPIO_Port GPIOB
#define LD3_Red_Pin GPIO_PIN_14
#define LD3_Red_GPIO_Port GPIOB
#define PCLK_Pin GPIO_PIN_3
#define PCLK_GPIO_Port GPIOG
#define VSync_Pin GPIO_PIN_2
#define VSync_GPIO_Port GPIOD
#define VSync_EXTI_IRQn EXTI2_IRQn
#define D0_Pin GPIO_PIN_4
#define D0_GPIO_Port GPIOD
#define D1_Pin GPIO_PIN_5
#define D1_GPIO_Port GPIOD
#define D2_Pin GPIO_PIN_6
#define D2_GPIO_Port GPIOD
#define D3_Pin GPIO_PIN_7
#define D3_GPIO_Port GPIOD
#define LD2_Blue_Pin GPIO_PIN_7
#define LD2_Blue_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
