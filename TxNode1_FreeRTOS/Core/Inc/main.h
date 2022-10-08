/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define Solenoid_Pin GPIO_PIN_13
#define Solenoid_GPIO_Port GPIOC
#define SoilMoisture_Pin GPIO_PIN_1
#define SoilMoisture_GPIO_Port GPIOA
#define BatteryVol_Pin GPIO_PIN_2
#define BatteryVol_GPIO_Port GPIOA
#define soilTemp_Pin GPIO_PIN_6
#define soilTemp_Port GPIOC
#define airTemp_Pin GPIO_PIN_7
#define airTemp_Port GPIOC
#define DHT11_Pin GPIO_PIN_10
#define DHT11_Port GPIOC
#define DIO0_Pin GPIO_PIN_12
#define DIO0_GPIO_Port GPIOC
#define NSS_Pin GPIO_PIN_6
#define NSS_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_7
#define RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void selectADC1(void);
void selectADC2(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
