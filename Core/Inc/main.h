/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

#include "stm32h5xx_ll_usart.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_dma.h"

#include "stm32h5xx_ll_exti.h"

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
#define DC_BREAK_N_Pin GPIO_PIN_0
#define DC_BREAK_N_GPIO_Port GPIOC
#define DC_BREAK_P_Pin GPIO_PIN_1
#define DC_BREAK_P_GPIO_Port GPIOC
#define DC_ACCEL_P_Pin GPIO_PIN_2
#define DC_ACCEL_P_GPIO_Port GPIOC
#define DC_ACCEL_N_Pin GPIO_PIN_3
#define DC_ACCEL_N_GPIO_Port GPIOC
#define GNSS_nRESET_Pin GPIO_PIN_2
#define GNSS_nRESET_GPIO_Port GPIOA
#define GNSS_TIMEPULSE_Pin GPIO_PIN_3
#define GNSS_TIMEPULSE_GPIO_Port GPIOA
#define GNSS_INT_Pin GPIO_PIN_4
#define GNSS_INT_GPIO_Port GPIOA
#define H_ADC_1_Pin GPIO_PIN_6
#define H_ADC_1_GPIO_Port GPIOA
#define H_ADC_2_Pin GPIO_PIN_7
#define H_ADC_2_GPIO_Port GPIOA
#define DB_LED_BLUE_Pin GPIO_PIN_0
#define DB_LED_BLUE_GPIO_Port GPIOB
#define DB_LED_RED_Pin GPIO_PIN_1
#define DB_LED_RED_GPIO_Port GPIOB
#define DB_LED_GREEN_Pin GPIO_PIN_2
#define DB_LED_GREEN_GPIO_Port GPIOB
#define IMU_FSYNC_Pin GPIO_PIN_7
#define IMU_FSYNC_GPIO_Port GPIOE
#define IMU_INT1_Pin GPIO_PIN_8
#define IMU_INT1_GPIO_Port GPIOE
#define IMU_INT2_Pin GPIO_PIN_9
#define IMU_INT2_GPIO_Port GPIOE
#define H_GPIO_3_Pin GPIO_PIN_10
#define H_GPIO_3_GPIO_Port GPIOE
#define H_GPIO_2_Pin GPIO_PIN_11
#define H_GPIO_2_GPIO_Port GPIOE
#define H_SPI2_SCK_Pin GPIO_PIN_13
#define H_SPI2_SCK_GPIO_Port GPIOB
#define H_SPI2_MISO_Pin GPIO_PIN_14
#define H_SPI2_MISO_GPIO_Port GPIOB
#define H_SPI2_MOSI_Pin GPIO_PIN_15
#define H_SPI2_MOSI_GPIO_Port GPIOB
#define PWMX_nOE_Pin GPIO_PIN_9
#define PWMX_nOE_GPIO_Port GPIOD
#define H_GPIO_1_Pin GPIO_PIN_10
#define H_GPIO_1_GPIO_Port GPIOD
#define H_GPIO_4_Pin GPIO_PIN_14
#define H_GPIO_4_GPIO_Port GPIOD
#define DB_LED_WHITE_Pin GPIO_PIN_7
#define DB_LED_WHITE_GPIO_Port GPIOC
#define H_I2C3_SDA_Pin GPIO_PIN_9
#define H_I2C3_SDA_GPIO_Port GPIOC
#define H_I2C3_SCL_Pin GPIO_PIN_8
#define H_I2C3_SCL_GPIO_Port GPIOA
#define IOX_nRESET_Pin GPIO_PIN_3
#define IOX_nRESET_GPIO_Port GPIOD
#define IOX_INTA_Pin GPIO_PIN_5
#define IOX_INTA_GPIO_Port GPIOD
#define BOOT1_Pin GPIO_PIN_7
#define BOOT1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
