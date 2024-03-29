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
#include "stm32h7xx_hal.h"

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
#define TOF_GPIO_1_Pin GPIO_PIN_5
#define TOF_GPIO_1_GPIO_Port GPIOE
#define TOF_GPIO_1_EXTI_IRQn EXTI9_5_IRQn
#define XSHUT_1_Pin GPIO_PIN_6
#define XSHUT_1_GPIO_Port GPIOE
#define V_LS1_Pin GPIO_PIN_4
#define V_LS1_GPIO_Port GPIOA
#define V_LS2_Pin GPIO_PIN_5
#define V_LS2_GPIO_Port GPIOA
#define V_LS3_Pin GPIO_PIN_6
#define V_LS3_GPIO_Port GPIOA
#define V_LS4_Pin GPIO_PIN_7
#define V_LS4_GPIO_Port GPIOA
#define START_SW_Pin GPIO_PIN_0
#define START_SW_GPIO_Port GPIOB
#define TOF_GPIO_3_Pin GPIO_PIN_9
#define TOF_GPIO_3_GPIO_Port GPIOE
#define TOF_GPIO_3_EXTI_IRQn EXTI9_5_IRQn
#define XSHUT_3_Pin GPIO_PIN_10
#define XSHUT_3_GPIO_Port GPIOE
#define TOF_GPIO_4_Pin GPIO_PIN_15
#define TOF_GPIO_4_GPIO_Port GPIOE
#define TOF_GPIO_4_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT_4_Pin GPIO_PIN_10
#define XSHUT_4_GPIO_Port GPIOB
#define MD_CS_2_Pin GPIO_PIN_11
#define MD_CS_2_GPIO_Port GPIOD
#define MD_DIS_2_Pin GPIO_PIN_12
#define MD_DIS_2_GPIO_Port GPIOD
#define MD_IN2_DIR_B_Pin GPIO_PIN_13
#define MD_IN2_DIR_B_GPIO_Port GPIOD
#define MD_IN1_PWM_B_Pin GPIO_PIN_14
#define MD_IN1_PWM_B_GPIO_Port GPIOD
#define MD_NDIS_Pin GPIO_PIN_15
#define MD_NDIS_GPIO_Port GPIOD
#define MD_IN1_PWM_A_Pin GPIO_PIN_6
#define MD_IN1_PWM_A_GPIO_Port GPIOC
#define MD_IN2_DIR_A_Pin GPIO_PIN_7
#define MD_IN2_DIR_A_GPIO_Port GPIOC
#define MD_DIS_1_Pin GPIO_PIN_8
#define MD_DIS_1_GPIO_Port GPIOA
#define MD_CS_1_Pin GPIO_PIN_9
#define MD_CS_1_GPIO_Port GPIOA
#define TOF_GPIO_2_Pin GPIO_PIN_11
#define TOF_GPIO_2_GPIO_Port GPIOA
#define TOF_GPIO_2_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT_2_Pin GPIO_PIN_12
#define XSHUT_2_GPIO_Port GPIOA
#define TOF_GPIO_5_Pin GPIO_PIN_10
#define TOF_GPIO_5_GPIO_Port GPIOC
#define XSHUT_5_Pin GPIO_PIN_11
#define XSHUT_5_GPIO_Port GPIOC
#define TOF_GPIO_6_Pin GPIO_PIN_12
#define TOF_GPIO_6_GPIO_Port GPIOC
#define TOF_GPIO_6_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT_6_Pin GPIO_PIN_0
#define XSHUT_6_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
