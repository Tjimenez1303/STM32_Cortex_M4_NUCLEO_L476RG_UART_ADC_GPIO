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
#define MEASURE_V_Pin GPIO_PIN_0
#define MEASURE_V_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_0
#define SPI_CS_GPIO_Port GPIOB
#define TTP223_Pin GPIO_PIN_1
#define TTP223_GPIO_Port GPIOB
#define TTP223_EXTI_IRQn EXTI0_1_IRQn
#define DS18B20_Pin GPIO_PIN_2
#define DS18B20_GPIO_Port GPIOB
#define BUTTON_UP_Pin GPIO_PIN_10
#define BUTTON_UP_GPIO_Port GPIOB
#define BUTTON_UP_EXTI_IRQn EXTI4_15_IRQn
#define BUTTON_LOW_Pin GPIO_PIN_11
#define BUTTON_LOW_GPIO_Port GPIOB
#define BUTTON_LOW_EXTI_IRQn EXTI4_15_IRQn
#define LED_BLUE_RGB_Pin GPIO_PIN_13
#define LED_BLUE_RGB_GPIO_Port GPIOB
#define LED_GREEN_RGB_Pin GPIO_PIN_14
#define LED_GREEN_RGB_GPIO_Port GPIOB
#define LED_RED_RGB_Pin GPIO_PIN_15
#define LED_RED_RGB_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define LED_RED_ON HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)
#define LED_RED_OFF HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
#define LED_RED_Toggle HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)

#define LED_GREEN_ON HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)
#define LED_GREEN_OFF HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)
#define LED_GREEN_Toggle HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)

#define LED_BLUE_RGB_ON HAL_GPIO_WritePin(LED_BLUE_RGB_GPIO_Port, LED_BLUE_RGB_Pin, GPIO_PIN_SET)
#define LED_BLUE_RGB_OFF HAL_GPIO_WritePin(LED_BLUE_RGB_GPIO_Port, LED_BLUE_RGB_Pin, GPIO_PIN_RESET)
#define LED_BLUE_RGB_Toggle HAL_GPIO_TogglePin(LED_BLUE_RGB_GPIO_Port, LED_BLUE_RGB_Pin)

#define LED_RED_RGB_ON HAL_GPIO_WritePin(LED_RED_RGB_GPIO_Port, LED_RED_RGB_Pin, GPIO_PIN_SET)
#define LED_RED_RGB_OFF HAL_GPIO_WritePin(LED_RED_RGB_GPIO_Port, LED_RED_RGB_Pin, GPIO_PIN_RESET)
#define LED_RED_RGB_Toggle HAL_GPIO_TogglePin(LED_RED_RGB_GPIO_Port, LED_RED_RGB_Pin)

#define LED_GREEN_RGB_ON HAL_GPIO_WritePin(LED_GREEN_RGB_GPIO_Port, LED_GREEN_RGB_Pin, GPIO_PIN_SET)
#define LED_GREEN_RGB_OFF HAL_GPIO_WritePin(LED_GREEN_RGB_GPIO_Port, LED_GREEN_RGB_Pin, GPIO_PIN_RESET)
#define LED_GREEN_RGB_Toggle HAL_GPIO_TogglePin(LED_GREEN_RGB_GPIO_Port, LED_GREEN_RGB_Pin)

#define PIN_BUTTON_UP_IT 1024
#define PIN_BUTTON_LOW_IT 2048

#define lenght_Rx 13
#define value_ms_to_s 1000 //Pasar de de ms a segundos para el HAL_Delay

#define MAX_VALUE_ADC_V 3.6
#define MAX_VALUE_ADC_BITS 4096
#define OFFSET 1.4
#define COMPARE_LIM 73


#define WREN 											0x06
#define WRDI 											0x04
#define PAGE_PROGRAM 									0x02
#define READ_STATUS_REG									0x05
#define ERASE_ALL_COMAND 								0x60
#define READ_PAGE 										0x03
#define READ_ID 										0x4b
#define DUMMY_BYTE										0


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
