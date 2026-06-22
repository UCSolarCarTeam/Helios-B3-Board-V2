/**
******************************************************************************
  * @file           : SystemDefines.hpp
  * @brief          : Macros and wrappers
  ******************************************************************************
  *
  * Contains system wide macros, defines, and wrappers
  *
  ******************************************************************************
  */
#ifndef CUBE_MAIN_SYSTEM_DEFINES_H
#define CUBE_MAIN_SYSTEM_DEFINES_H

/* Environment Defines ------------------------------------------------------------------*/
//#define COMPUTER_ENVIRONMENT        // Define this if we're in Windows, Linux or Mac (not when flashing on DMB)

#ifdef COMPUTER_ENVIRONMENT
#define __CC_ARM
#endif

/* System Wide Includes ------------------------------------------------------------------*/
#include "main_system.hpp" // C++ Main File Header
#include "UARTDriver.hpp"

/* Cube++ Required Configuration ------------------------------------------------------------------*/
#include "CubeDefines.hpp"
constexpr UARTDriver* const DEFAULT_DEBUG_UART_DRIVER = UART::Debug;    // UART Handle that ASSERT messages are sent over
enum GLOBAL_COMMANDS : uint8_t
{
    COMMAND_NONE = 0,        // No command, packet can probably be ignored
    TASK_SPECIFIC_COMMAND,    // Runs a task specific command when given this object
    DATA_COMMAND,             // Data command, used to send data to a task. Target is stored in taskCommand
};

/* Cube++ Optional Code Configuration ------------------------------------------------------------------*/


/* Task Parameter Definitions ------------------------------------------------------------------*/
/* - Lower priority number means lower priority task ---------------------------------*/

// WATCHDOG TASK
constexpr uint8_t WATCHDOG_TASK_RTOS_PRIORITY = 2;            // Priority of the watchdog task
constexpr uint8_t WATCHDOG_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the watchdog task queue
constexpr uint16_t WATCHDOG_TASK_STACK_DEPTH_WORDS = 512;    // Size of the watchdog task stack

// UART TASK
constexpr uint8_t UART_TASK_RTOS_PRIORITY = 2;            // Priority of the uart task
constexpr uint8_t UART_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the uart task queue
constexpr uint16_t UART_TASK_STACK_DEPTH_WORDS = 512;    // Size of the uart task stack

// DEBUG TASK
constexpr uint8_t TASK_DEBUG_PRIORITY = 2;            // Priority of the debug task
constexpr uint8_t TASK_DEBUG_QUEUE_DEPTH_OBJS = 10;        // Size of the debug task queue
constexpr uint16_t TASK_DEBUG_STACK_DEPTH_WORDS = 512;        // Size of the debug task stack

// CAN TX TASK
constexpr uint8_t CAN_TX_TASK_RTOS_PRIORITY = 2;            // Priority of the CAN Tx task
constexpr uint8_t CAN_TX_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the CAN Tx task queue
constexpr uint16_t CAN_TX_TASK_STACK_DEPTH_WORDS = 512;    // Size of the CAN Tx task stack

// CAN RX TASK
constexpr uint8_t CAN_RX_TASK_RTOS_PRIORITY = 2;            // Priority of the CAN Rx task
constexpr uint8_t CAN_RX_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the CAN Rx task queue
constexpr uint16_t CAN_RX_TASK_STACK_DEPTH_WORDS = 512;    // Size of the CAN Rx task stack

// I2C PCA9685 TASK
constexpr uint8_t PCA9685_TASK_RTOS_PRIORITY = 2;          // Priority of the PCA9685 task
constexpr uint8_t PCA9685_TASK_QUEUE_DEPTH_OBJS = 10;      // Size of the PCA9685 task queue
constexpr uint16_t PCA9685_TASK_STACK_DEPTH_WORDS = 512;   // Size of the PCA9685 task stack

// I2C GNSSREAD TASK
constexpr uint8_t GNSSREAD_TASK_RTOS_PRIORITY = 2;          // Priority of the GNSS read task
constexpr uint8_t GNSSREAD_TASK_QUEUE_DEPTH_OBJS = 10;      // Size of the GNSS read task queue
constexpr uint16_t GNSSREAD_TASK_STACK_DEPTH_WORDS = 512;   // Size of the GNSS read task stack

#endif // CUBE_MAIN_SYSTEM_DEFINES_H
