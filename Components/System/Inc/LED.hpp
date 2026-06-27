/*
 * LED.hpp
 *
 *  Created on: Apr 18, 2026
 *      Author: Macante
 */

#ifndef SYSTEM_INC_LED_HPP_
#define SYSTEM_INC_LED_HPP_

#include "main.h"
#include "stm32h5xx_hal_gpio.h"

namespace LED
{
	namespace DB_RED
	{
		inline void On() { HAL_GPIO_WritePin(DB_LED_RED_GPIO_Port, DB_LED_RED_Pin, GPIO_PIN_SET); }
		inline void Off() { HAL_GPIO_WritePin(DB_LED_RED_GPIO_Port, DB_LED_RED_Pin, GPIO_PIN_RESET); }
		inline void Toggle() { HAL_GPIO_TogglePin(DB_LED_RED_GPIO_Port, DB_LED_RED_Pin); }

		inline bool IsOn() { return HAL_GPIO_ReadPin(DB_LED_RED_GPIO_Port, DB_LED_RED_Pin) == GPIO_PIN_SET; }
	}

	namespace DB_BLUE
	{
		inline void On() { HAL_GPIO_WritePin(DB_LED_BLUE_GPIO_Port, DB_LED_BLUE_Pin, GPIO_PIN_SET); }
		inline void Off() { HAL_GPIO_WritePin(DB_LED_BLUE_GPIO_Port, DB_LED_BLUE_Pin, GPIO_PIN_RESET); }
		inline void Toggle() { HAL_GPIO_TogglePin(DB_LED_BLUE_GPIO_Port, DB_LED_BLUE_Pin); }

		inline bool IsOn() { return HAL_GPIO_ReadPin(DB_LED_BLUE_GPIO_Port, DB_LED_BLUE_Pin) == GPIO_PIN_SET; }
	}

	namespace DB_GREEN
	{
		inline void On() { HAL_GPIO_WritePin(DB_LED_GREEN_GPIO_Port, DB_LED_GREEN_Pin, GPIO_PIN_SET); }
		inline void Off() { HAL_GPIO_WritePin(DB_LED_GREEN_GPIO_Port, DB_LED_GREEN_Pin, GPIO_PIN_RESET); }
		inline void Toggle() { HAL_GPIO_TogglePin(DB_LED_RED_GPIO_Port, DB_LED_GREEN_Pin); }

		inline bool IsOn() { return HAL_GPIO_ReadPin(DB_LED_GREEN_GPIO_Port, DB_LED_GREEN_Pin) == GPIO_PIN_SET; }
	}

	namespace DB_WHITE
	{
		inline void On() { HAL_GPIO_WritePin(DB_LED_WHITE_GPIO_Port, DB_LED_WHITE_Pin, GPIO_PIN_SET); }
		inline void Off() { HAL_GPIO_WritePin(DB_LED_WHITE_GPIO_Port, DB_LED_WHITE_Pin, GPIO_PIN_RESET); }
		inline void Toggle() { HAL_GPIO_TogglePin(DB_LED_WHITE_GPIO_Port, DB_LED_WHITE_Pin); }

		inline bool IsOn() { return HAL_GPIO_ReadPin(DB_LED_WHITE_GPIO_Port, DB_LED_WHITE_Pin) == GPIO_PIN_SET; }
	}

}


#endif /* SYSTEM_INC_LED_HPP_ */
