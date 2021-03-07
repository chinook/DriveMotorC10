/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define nSTALL2_Pin GPIO_PIN_3
#define nSTALL2_GPIO_Port GPIOE
#define nFAULT2_Pin GPIO_PIN_4
#define nFAULT2_GPIO_Port GPIOE
#define SPI_SC2_Pin GPIO_PIN_5
#define SPI_SC2_GPIO_Port GPIOE
#define BIN2_2_Pin GPIO_PIN_0
#define BIN2_2_GPIO_Port GPIOC
#define BIN1_2_Pin GPIO_PIN_1
#define BIN1_2_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_2
#define DIR2_GPIO_Port GPIOC
#define PB_RESET_Pin GPIO_PIN_3
#define PB_RESET_GPIO_Port GPIOC
#define STEP2_Pin GPIO_PIN_2
#define STEP2_GPIO_Port GPIOA
#define nSLEEP2_Pin GPIO_PIN_3
#define nSLEEP2_GPIO_Port GPIOA
#define RESET2_Pin GPIO_PIN_4
#define RESET2_GPIO_Port GPIOA
#define nSTALL1_Pin GPIO_PIN_1
#define nSTALL1_GPIO_Port GPIOB
#define nFAULT1_Pin GPIO_PIN_2
#define nFAULT1_GPIO_Port GPIOB
#define SPI_CS1_Pin GPIO_PIN_7
#define SPI_CS1_GPIO_Port GPIOE
#define PB_1_Pin GPIO_PIN_8
#define PB_1_GPIO_Port GPIOE
#define PB_2_Pin GPIO_PIN_9
#define PB_2_GPIO_Port GPIOE
#define BIN2_1_Pin GPIO_PIN_10
#define BIN2_1_GPIO_Port GPIOE
#define BIN1_1_Pin GPIO_PIN_11
#define BIN1_1_GPIO_Port GPIOE
#define FIR1_Pin GPIO_PIN_12
#define FIR1_GPIO_Port GPIOE
#define STEP1_Pin GPIO_PIN_13
#define STEP1_GPIO_Port GPIOE
#define RESET1_Pin GPIO_PIN_14
#define RESET1_GPIO_Port GPIOE
#define nSLEEP1_Pin GPIO_PIN_15
#define nSLEEP1_GPIO_Port GPIOE
#define LED_WARNING_Pin GPIO_PIN_0
#define LED_WARNING_GPIO_Port GPIOD
#define LED_ERROR_Pin GPIO_PIN_1
#define LED_ERROR_GPIO_Port GPIOD
#define LED_CANB_Pin GPIO_PIN_2
#define LED_CANB_GPIO_Port GPIOD
#define LED_CANA_Pin GPIO_PIN_3
#define LED_CANA_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
