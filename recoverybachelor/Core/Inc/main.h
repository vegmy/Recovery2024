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
#include "stm32g4xx_hal.h"

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
#define GPIO_IN_DIO2_Pin GPIO_PIN_0
#define GPIO_IN_DIO2_GPIO_Port GPIOA
#define GPIO_IN_HELI_Pin GPIO_PIN_1
#define GPIO_IN_HELI_GPIO_Port GPIOA
#define GPIO_IN_FOTO_Pin GPIO_PIN_2
#define GPIO_IN_FOTO_GPIO_Port GPIOA
#define GPIO_IN_DIO4_Pin GPIO_PIN_4
#define GPIO_IN_DIO4_GPIO_Port GPIOA
#define GPIO_IN_MAGN_Pin GPIO_PIN_7
#define GPIO_IN_MAGN_GPIO_Port GPIOA
#define GPIO_IN_DIO1_Pin GPIO_PIN_0
#define GPIO_IN_DIO1_GPIO_Port GPIOB
#define GPIO_IN_DIO3_Pin GPIO_PIN_8
#define GPIO_IN_DIO3_GPIO_Port GPIOA
#define GPIO_OUT_TRANS8_Pin GPIO_PIN_9
#define GPIO_OUT_TRANS8_GPIO_Port GPIOA
#define GPIO_OUT_TRANS6_Pin GPIO_PIN_10
#define GPIO_OUT_TRANS6_GPIO_Port GPIOA
#define GPIO_OUT_TRANS2_Pin GPIO_PIN_12
#define GPIO_OUT_TRANS2_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define GPIO_OUT_TRANS4_Pin GPIO_PIN_6
#define GPIO_OUT_TRANS4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
