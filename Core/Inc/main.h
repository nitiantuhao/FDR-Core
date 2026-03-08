/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RL_IN1_Pin GPIO_PIN_5
#define RL_IN1_GPIO_Port GPIOE
#define RL_IN2_Pin GPIO_PIN_6
#define RL_IN2_GPIO_Port GPIOE
#define FL_ENC_A_Pin GPIO_PIN_0
#define FL_ENC_A_GPIO_Port GPIOA
#define FL_ENC_B_Pin GPIO_PIN_1
#define FL_ENC_B_GPIO_Port GPIOA
#define FL_IN1_Pin GPIO_PIN_2
#define FL_IN1_GPIO_Port GPIOA
#define FL_IN2_Pin GPIO_PIN_3
#define FL_IN2_GPIO_Port GPIOA
#define RL_ENC_A_Pin GPIO_PIN_6
#define RL_ENC_A_GPIO_Port GPIOA
#define RL_ENC_B_Pin GPIO_PIN_7
#define RL_ENC_B_GPIO_Port GPIOA
#define RR_ENC_A_Pin GPIO_PIN_9
#define RR_ENC_A_GPIO_Port GPIOE
#define RR_ENC_B_Pin GPIO_PIN_11
#define RR_ENC_B_GPIO_Port GPIOE
#define FR_ENC_A_Pin GPIO_PIN_12
#define FR_ENC_A_GPIO_Port GPIOD
#define FR_ENC_B_Pin GPIO_PIN_13
#define FR_ENC_B_GPIO_Port GPIOD
#define FR_IN1_Pin GPIO_PIN_6
#define FR_IN1_GPIO_Port GPIOC
#define FR_IN2_Pin GPIO_PIN_7
#define FR_IN2_GPIO_Port GPIOC
#define RR_IN1_Pin GPIO_PIN_8
#define RR_IN1_GPIO_Port GPIOC
#define RR_IN2_Pin GPIO_PIN_9
#define RR_IN2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
