/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
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
#define SLP_G1_Pin GPIO_PIN_3
#define SLP_G1_GPIO_Port GPIOE
#define SLP_G2_Pin GPIO_PIN_4
#define SLP_G2_GPIO_Port GPIOE
#define SLP_G3_Pin GPIO_PIN_5
#define SLP_G3_GPIO_Port GPIOE
#define POT_M5_Pin GPIO_PIN_0
#define POT_M5_GPIO_Port GPIOC
#define POT_M6_Pin GPIO_PIN_1
#define POT_M6_GPIO_Port GPIOC
#define POT_M7_Pin GPIO_PIN_2
#define POT_M7_GPIO_Port GPIOC
#define POT_M8_Pin GPIO_PIN_3
#define POT_M8_GPIO_Port GPIOC
#define POT_M1_Pin GPIO_PIN_0
#define POT_M1_GPIO_Port GPIOA
#define POT_M2_Pin GPIO_PIN_1
#define POT_M2_GPIO_Port GPIOA
#define POT_M3_Pin GPIO_PIN_2
#define POT_M3_GPIO_Port GPIOA
#define POT_M4_Pin GPIO_PIN_3
#define POT_M4_GPIO_Port GPIOA
#define POT_M13_Pin GPIO_PIN_4
#define POT_M13_GPIO_Port GPIOA
#define POT_M12_Pin GPIO_PIN_5
#define POT_M12_GPIO_Port GPIOA
#define POT_M9_Pin GPIO_PIN_6
#define POT_M9_GPIO_Port GPIOA
#define POT_M10_Pin GPIO_PIN_7
#define POT_M10_GPIO_Port GPIOA
#define POT_M11_Pin GPIO_PIN_4
#define POT_M11_GPIO_Port GPIOC
#define CS_M__Pin GPIO_PIN_0
#define CS_M__GPIO_Port GPIOB
#define CS_M10_Pin GPIO_PIN_1
#define CS_M10_GPIO_Port GPIOB
#define CS_M11_Pin GPIO_PIN_7
#define CS_M11_GPIO_Port GPIOE
#define CS_M12_Pin GPIO_PIN_8
#define CS_M12_GPIO_Port GPIOE
#define PWM_M9_Pin GPIO_PIN_9
#define PWM_M9_GPIO_Port GPIOE
#define CS_M13_Pin GPIO_PIN_10
#define CS_M13_GPIO_Port GPIOE
#define PWM_M10_Pin GPIO_PIN_11
#define PWM_M10_GPIO_Port GPIOE
#define DIR_M9_Pin GPIO_PIN_12
#define DIR_M9_GPIO_Port GPIOE
#define PWM_M11_Pin GPIO_PIN_13
#define PWM_M11_GPIO_Port GPIOE
#define PWM_M12_Pin GPIO_PIN_14
#define PWM_M12_GPIO_Port GPIOE
#define DIR_M10_Pin GPIO_PIN_15
#define DIR_M10_GPIO_Port GPIOE
#define DIR_M11_Pin GPIO_PIN_10
#define DIR_M11_GPIO_Port GPIOB
#define DIR_M12_Pin GPIO_PIN_11
#define DIR_M12_GPIO_Port GPIOB
#define DIR_M13_Pin GPIO_PIN_13
#define DIR_M13_GPIO_Port GPIOB
#define PWM_M13_Pin GPIO_PIN_15
#define PWM_M13_GPIO_Port GPIOB
#define DIR_M5_Pin GPIO_PIN_8
#define DIR_M5_GPIO_Port GPIOD
#define DIR_M6_Pin GPIO_PIN_9
#define DIR_M6_GPIO_Port GPIOD
#define DIR_M7_Pin GPIO_PIN_10
#define DIR_M7_GPIO_Port GPIOD
#define DIR_M8_Pin GPIO_PIN_11
#define DIR_M8_GPIO_Port GPIOD
#define PWM_M5_Pin GPIO_PIN_12
#define PWM_M5_GPIO_Port GPIOD
#define PWM_M6_Pin GPIO_PIN_13
#define PWM_M6_GPIO_Port GPIOD
#define PWM_M7_Pin GPIO_PIN_14
#define PWM_M7_GPIO_Port GPIOD
#define PWM_M8_Pin GPIO_PIN_15
#define PWM_M8_GPIO_Port GPIOD
#define PWM_M1_Pin GPIO_PIN_6
#define PWM_M1_GPIO_Port GPIOC
#define PWM_M2_Pin GPIO_PIN_7
#define PWM_M2_GPIO_Port GPIOC
#define PWM_M3_Pin GPIO_PIN_8
#define PWM_M3_GPIO_Port GPIOC
#define PWM_M4_Pin GPIO_PIN_9
#define PWM_M4_GPIO_Port GPIOC
#define DIR_M1_Pin GPIO_PIN_10
#define DIR_M1_GPIO_Port GPIOC
#define DIR_M2_Pin GPIO_PIN_11
#define DIR_M2_GPIO_Port GPIOC
#define DIR_M3_Pin GPIO_PIN_12
#define DIR_M3_GPIO_Port GPIOC
#define DIR_M4_Pin GPIO_PIN_0
#define DIR_M4_GPIO_Port GPIOD
#define FLT_G1_Pin GPIO_PIN_4
#define FLT_G1_GPIO_Port GPIOB
#define FLT_G2_Pin GPIO_PIN_5
#define FLT_G2_GPIO_Port GPIOB
#define FLT_G3_Pin GPIO_PIN_6
#define FLT_G3_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
