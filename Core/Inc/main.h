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
// receiver perintah dari serial monitor pc
void USB_CDC_RxHandler(uint8_t*, uint32_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_NOTIFIKASI_Pin GPIO_PIN_13
#define LED_NOTIFIKASI_GPIO_Port GPIOC
#define REMOTE_INPUT_Pin GPIO_PIN_6
#define REMOTE_INPUT_GPIO_Port GPIOA
#define REMOTE_INPUT_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT_ACC_DCC_Pin GPIO_PIN_7
#define LIMIT_ACC_DCC_GPIO_Port GPIOA
#define LIMIT_ACC_DCC_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT_MIN_MAX_Pin GPIO_PIN_0
#define LIMIT_MIN_MAX_GPIO_Port GPIOB
#define LIMIT_MIN_MAX_EXTI_IRQn EXTI0_IRQn
#define ENABLE_STEPPER_Pin GPIO_PIN_1
#define ENABLE_STEPPER_GPIO_Port GPIOB
#define DIR_STEPPER_Pin GPIO_PIN_10
#define DIR_STEPPER_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
