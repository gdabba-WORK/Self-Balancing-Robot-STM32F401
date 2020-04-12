/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stepperMotor.h"
#include "complementary_filter.h"
#include "MPU6050.h"
#include "usart.h"
#include "i2c.h"
#include "cmsis_os.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define SM1B__Pin GPIO_PIN_0
#define SM1B__GPIO_Port GPIOC
#define SM1B_Pin GPIO_PIN_1
#define SM1B_GPIO_Port GPIOC
#define SM1EN_Pin GPIO_PIN_1
#define SM1EN_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SM1A_Pin GPIO_PIN_4
#define SM1A_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_6
#define LD4_GPIO_Port GPIOA
#define SM1A__Pin GPIO_PIN_0
#define SM1A__GPIO_Port GPIOB
#define SM2A_Pin GPIO_PIN_1
#define SM2A_GPIO_Port GPIOB
#define SM2B__Pin GPIO_PIN_13
#define SM2B__GPIO_Port GPIOB
#define SM2B_Pin GPIO_PIN_14
#define SM2B_GPIO_Port GPIOB
#define SM2A__Pin GPIO_PIN_15
#define SM2A__GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_9
#define MPU_SDA_GPIO_Port GPIOC
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOA
#define HC_TX_Pin GPIO_PIN_9
#define HC_TX_GPIO_Port GPIOA
#define HC_RX_Pin GPIO_PIN_10
#define HC_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SM2EN_Pin GPIO_PIN_5
#define SM2EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_Delay(uint32_t Delay);
uint32_t MY_GetTick();
void MY_Delay(uint32_t step_delay);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
