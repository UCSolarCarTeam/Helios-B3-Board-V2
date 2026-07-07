/**
 ******************************************************************************
 * File Name          : main_system.hpp
 * Description        : Header file for main_system.cpp, acts as an interface between
 *  STM32CubeIDE and our application.
 ******************************************************************************
*/
#ifndef MAIN_SYSTEM_HPP_
#define MAIN_SYSTEM_HPP_

/* Includes  ----------------------------------------------------------------------------*/
#include "Mutex.hpp"
// Board specific includes

#include "stm32h563xx.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_usart.h"
#include "stm32h5xx_hal_rcc.h"
#include "stm32h5xx_ll_dma.h"
#include "stm32h5xx_hal_fdcan.h"
#include "stm32h5xx_hal_i2c.h"
#include "stm32h5xx_hal_adc.h"
#include "stm32h5xx_hal_adc_ex.h"
#include "stm32h5xx_hal_tim.h"


/* Interface Functions ------------------------------------------------------------------*/
/* These functions act as our program's 'main' and any functions inside CubeIDE's main --*/
void run_main();
void run_StartDefaultTask();

/* Global Functions ------------------------------------------------------------------*/

/* Global Variable Interfaces ------------------------------------------------------------------*/
/* All must be extern from main_system.cpp -------------------------------------------------*/

/* Globally Accessible Drivers ------------------------------------------------------------------*/

/* UART Driver Instances ------------------------------------------------------------------*/
class UARTDriver;
namespace Driver {
    extern UARTDriver uart1;
}

/* UART Driver Aliases ------------------------------------------------------------------*/
namespace UART {
	constexpr UARTDriver* Debug = &Driver::uart1;	// Debug UART
}

/* System Handles ------------------------------------------------------------------*/
extern CRC_HandleTypeDef hcrc;          // CRC - Hardware CRC System Handle
extern FDCAN_HandleTypeDef hfdcan2;     // FDCAN2 - FDCAN Peripheral Handle
extern I2C_HandleTypeDef hi2c1;			// I2C1 - IOExpander I2C Peripheral Handle
extern I2C_HandleTypeDef hi2c2;         // I2C2 - Telemetry I2C Peripheral Handle
extern ADC_HandleTypeDef hadc1;         // ADC1 - Regen ADC Peripheral Handle
extern ADC_HandleTypeDef hadc2;         // ADC2 - Accel ADC Peripheral Handle
extern TIM_HandleTypeDef htim3;

namespace SystemHandles {
    constexpr CRC_HandleTypeDef* CRC_Handle = &hcrc;
    constexpr FDCAN_HandleTypeDef* FDCAN_Handle = &hfdcan2;
    constexpr I2C_HandleTypeDef* IOX_Handle = &hi2c1;
    constexpr I2C_HandleTypeDef* I2C2_Handle = &hi2c2;
    constexpr ADC_HandleTypeDef* Regen_Handle = &hadc1;
    constexpr ADC_HandleTypeDef* Accel_Handle = &hadc2;
    constexpr TIM_HandleTypeDef* ADC_Timer_Handle = &htim3;
}

#endif /* MAIN_SYSTEM_HPP_ */
