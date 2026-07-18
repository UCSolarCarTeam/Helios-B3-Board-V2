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

// IMU READ TASK TASK
constexpr uint8_t IMU_READ_TASK_RTOS_PRIORITY = 2;       // Priority of the IMU Read Task
constexpr uint8_t IMU_READ_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the IMU Read Task queue
constexpr uint16_t IMU_READ_TASK_STACK_DEPTH_WORDS = 512;    // Size of the IMU Read Task stack

// PEDALS INPUT TASK 
constexpr uint8_t PEDALS_INPUT_TASK_RTOS_PRIORITY = 2;       // Priority of the Pedals Input Task
constexpr uint8_t PEDALS_INPUT_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the Pedals Input Task queue
constexpr uint16_t PEDALS_INPUT_TASK_STACK_DEPTH_WORDS = 512;    // Size of the Pedals Input Task stack

// PCA9685 / LIGHTS OUTPUT TASK
constexpr uint8_t PCA9685_TASK_RTOS_PRIORITY = 2;
constexpr uint8_t PCA9685_TASK_QUEUE_DEPTH_OBJS = 10;
constexpr uint16_t PCA9685_TASK_STACK_DEPTH_WORDS = 768;

// PCA9685 / LIGHTS OUTPUT TASK
constexpr uint8_t GNSSREAD_TASK_RTOS_PRIORITY = 2;
constexpr uint8_t GNSSREAD_TASK_QUEUE_DEPTH_OBJS = 10;
constexpr uint16_t GNSSREAD_TASK_STACK_DEPTH_WORDS = 768;

#endif // CUBE_MAIN_SYSTEM_DEFINES_H
