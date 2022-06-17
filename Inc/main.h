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
#include "stm32f0xx_hal.h"

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
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define GSM_Enable_Pin GPIO_PIN_15
#define GSM_Enable_GPIO_Port GPIOC
#define Valve_Phase_Pin GPIO_PIN_0
#define Valve_Phase_GPIO_Port GPIOB
#define Wifi_CS_Pin GPIO_PIN_1
#define Wifi_CS_GPIO_Port GPIOB
#define GSM_On_Pin GPIO_PIN_2
#define GSM_On_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_12
#define Buzzer_GPIO_Port GPIOB
#define Flash_CS_Pin GPIO_PIN_8
#define Flash_CS_GPIO_Port GPIOA
#define Sensor2_Enable_Pin GPIO_PIN_11
#define Sensor2_Enable_GPIO_Port GPIOA
#define Valve_Enable_Pin GPIO_PIN_12
#define Valve_Enable_GPIO_Port GPIOA
#define AC_Voltage_Pin GPIO_PIN_13
#define AC_Voltage_GPIO_Port GPIOA
#define Sensor1_Enable_Pin GPIO_PIN_14
#define Sensor1_Enable_GPIO_Port GPIOA
#define Reset_In_Pin GPIO_PIN_15
#define Reset_In_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_3
#define LED5_GPIO_Port GPIOB
#define Relay_Open_Pin GPIO_PIN_4
#define Relay_Open_GPIO_Port GPIOB
#define Relay_Close_Pin GPIO_PIN_5
#define Relay_Close_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
