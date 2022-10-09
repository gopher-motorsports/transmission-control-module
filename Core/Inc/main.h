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
#define CLUTCH_POT_Pin GPIO_PIN_0
#define CLUTCH_POT_GPIO_Port GPIOC
#define SHIFT_POT_Pin GPIO_PIN_1
#define SHIFT_POT_GPIO_Port GPIOC
#define TRANS_Pin GPIO_PIN_0
#define TRANS_GPIO_Port GPIOA
#define NTRL_SW_Pin GPIO_PIN_4
#define NTRL_SW_GPIO_Port GPIOA
#define UPSHIFT_BTN_Pin GPIO_PIN_15
#define UPSHIFT_BTN_GPIO_Port GPIOF
#define UPSHIFT_BTN_EXTI_IRQn EXTI15_10_IRQn
#define AERO_FRONT_BTN_Pin GPIO_PIN_7
#define AERO_FRONT_BTN_GPIO_Port GPIOE
#define AERO_FRONT_BTN_EXTI_IRQn EXTI9_5_IRQn
#define DOWNSHIFT_BTN_Pin GPIO_PIN_8
#define DOWNSHIFT_BTN_GPIO_Port GPIOE
#define DOWNSHIFT_BTN_EXTI_IRQn EXTI9_5_IRQn
#define CLUTCH_FAST_BTN_Pin GPIO_PIN_9
#define CLUTCH_FAST_BTN_GPIO_Port GPIOE
#define CLUTCH_FAST_BTN_EXTI_IRQn EXTI9_5_IRQn
#define AERO_REAR_BTN_Pin GPIO_PIN_10
#define AERO_REAR_BTN_GPIO_Port GPIOE
#define AERO_REAR_BTN_EXTI_IRQn EXTI15_10_IRQn
#define CLUTCH_SLOW_BTN_Pin GPIO_PIN_11
#define CLUTCH_SLOW_BTN_GPIO_Port GPIOE
#define CLUTCH_SLOW_BTN_EXTI_IRQn EXTI15_10_IRQn
#define ECU_SPK_CUT_Pin GPIO_PIN_12
#define ECU_SPK_CUT_GPIO_Port GPIOD
#define ECU_THROTTLE_BLIP_Pin GPIO_PIN_15
#define ECU_THROTTLE_BLIP_GPIO_Port GPIOD
#define ECU_CLUTCH_POS_Pin GPIO_PIN_7
#define ECU_CLUTCH_POS_GPIO_Port GPIOC
#define LAP_TIM_9_Pin GPIO_PIN_9
#define LAP_TIM_9_GPIO_Port GPIOC
#define HEARTBEAT_Pin GPIO_PIN_10
#define HEARTBEAT_GPIO_Port GPIOC
#define DAM_LED_Pin GPIO_PIN_11
#define DAM_LED_GPIO_Port GPIOC
#define HARDFAULT_Pin GPIO_PIN_12
#define HARDFAULT_GPIO_Port GPIOC
#define DOWNSHIFT_SOL_Pin GPIO_PIN_0
#define DOWNSHIFT_SOL_GPIO_Port GPIOD
#define UPSHIFT_SOL_Pin GPIO_PIN_1
#define UPSHIFT_SOL_GPIO_Port GPIOD
#define CLUTCH_SOL_Pin GPIO_PIN_2
#define CLUTCH_SOL_GPIO_Port GPIOD
#define CLUTCH_SLOW_DROP_Pin GPIO_PIN_11
#define CLUTCH_SLOW_DROP_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
