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
#define ERROR_DRIVE_Pin GPIO_PIN_13
#define ERROR_DRIVE_GPIO_Port GPIOC
#define CS_TMC4671_Pin GPIO_PIN_3
#define CS_TMC4671_GPIO_Port GPIOA
#define CS_TMC6100_Pin GPIO_PIN_4
#define CS_TMC6100_GPIO_Port GPIOA
#define TMC_SCK_Pin GPIO_PIN_5
#define TMC_SCK_GPIO_Port GPIOA
#define LAN9252_SCK_Pin GPIO_PIN_0
#define LAN9252_SCK_GPIO_Port GPIOB
#define TMC_STATUS_Pin GPIO_PIN_1
#define TMC_STATUS_GPIO_Port GPIOB
#define ENC_INDEX_Pin GPIO_PIN_2
#define ENC_INDEX_GPIO_Port GPIOB
#define ENC_INDEX_EXTI_IRQn EXTI2_IRQn
#define BR24L64_SCL_Pin GPIO_PIN_10
#define BR24L64_SCL_GPIO_Port GPIOB
#define ADXL345_CLK_Pin GPIO_PIN_13
#define ADXL345_CLK_GPIO_Port GPIOB
#define ADXL345_MISO_Pin GPIO_PIN_14
#define ADXL345_MISO_GPIO_Port GPIOB
#define ADXL345_MOSI_Pin GPIO_PIN_15
#define ADXL345_MOSI_GPIO_Port GPIOB
#define ADXL345_CS_Pin GPIO_PIN_9
#define ADXL345_CS_GPIO_Port GPIOA
#define LAN9252_MOSI_Pin GPIO_PIN_10
#define LAN9252_MOSI_GPIO_Port GPIOA
#define LAN9252_CS_Pin GPIO_PIN_11
#define LAN9252_CS_GPIO_Port GPIOA
#define LAN9252_MISO_Pin GPIO_PIN_12
#define LAN9252_MISO_GPIO_Port GPIOA
#define FAULT_LED_Pin GPIO_PIN_15
#define FAULT_LED_GPIO_Port GPIOA
#define TMC_MISO_Pin GPIO_PIN_4
#define TMC_MISO_GPIO_Port GPIOB
#define TMC_MOSI_Pin GPIO_PIN_5
#define TMC_MOSI_GPIO_Port GPIOB
#define PCAP_SCL_Pin GPIO_PIN_6
#define PCAP_SCL_GPIO_Port GPIOB
#define PCAP_SDA_Pin GPIO_PIN_7
#define PCAP_SDA_GPIO_Port GPIOB
#define CTRL_EN_Pin GPIO_PIN_8
#define CTRL_EN_GPIO_Port GPIOB
#define BR24L64_SDA_Pin GPIO_PIN_9
#define BR24L64_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
