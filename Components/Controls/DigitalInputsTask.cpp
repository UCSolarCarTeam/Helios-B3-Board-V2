/**
 ******************************************************************************
 * File Name          : DigitalInputsTask.cpp
 * Description        : Task for controlling the PCA9685 PWM driver.
 ******************************************************************************
*/
#include "SystemDefines.hpp"
#include "DigitalInputsTask.hpp"
#include "pca9685.h"

/* Macros/Enums ------------------------------------------------------------*/

// Change this if your board uses a different I2C handle.
// Common options are hi2c1, hi2c2, etc.
extern I2C_HandleTypeDef hi2c1;

// Default PCA9685 7-bit I2C address.
// This assumes address pins A0-A5 are all connected LOW.
#define PCA9685_I2C_ADDR              0x40

// Default startup PWM frequency.
#define PCA9685_DEFAULT_PWM_FREQ_HZ   200.0f

// Delay between task loop checks.
#define PCA9685_TASK_DELAY_MS         100

/**
 * @brief Constructor for PCA9685Task
 */
PCA9685Task::PCA9685Task() : Task(PCA9685_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the PCA9685Task
 */
void PCA9685Task::InitTask()
{
    // Make sure the task is not already initialized
    CUBE_ASSERT(rtTaskHandle == nullptr, "Cannot initialize PCA9685 task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)PCA9685Task::RunTask,
            (const char*)"PCA9685Task",
            (uint16_t)PCA9685_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)PCA9685_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    CUBE_ASSERT(rtValue == pdPASS, "PCA9685Task::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void PCA9685Task::HandleCommand(Command& cm)
{
    switch (cm.GetCommand()) {
    case TASK_SPECIFIC_COMMAND: {
        /*
         * This is where PCA9685-specific commands will go later.
         *
         * Your driver already supports these functions:
         *
         * PCA9685_SetDuty(&hpca, channel, duty_0_to_1, phase_count);
         * PCA9685_SetPWM(&hpca, channel, on_count, off_count);
         * PCA9685_SetAllPWM(&hpca, on_count, off_count);
         * PCA9685_SetPWMFreq(&hpca, frequency_hz);
         * PCA9685_Sleep(&hpca, true);
         * PCA9685_Sleep(&hpca, false);
         * PCA9685_Restart(&hpca);
         *
         * We cannot fully decode those yet until we know how your Command
         * object stores extra data like channel, duty, frequency, etc.
         */

        break;
    }

    default:
        CUBE_PRINT("PCA9685Task - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }


    cm.Reset();
}

/**
 * @brief Instance Run loop for the PCA9685 Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void PCA9685Task::Run(void* pvParams)
{
    CUBE_PRINT("PCA9685Task - Starting initialization\r\n");

    HAL_StatusTypeDef status = PCA9685_Init(&hpca, &hi2c1, PCA9685_I2C_ADDR);

    if (status != HAL_OK) {
        CUBE_PRINT("PCA9685Task - PCA9685_Init failed\r\n");
    }
    else {
        CUBE_PRINT("PCA9685Task - PCA9685_Init complete\r\n");

        // Explicitly set output mode:
        // true  = totem-pole output
        // false = not inverted
        // false = update outputs on STOP command, not ACK
        status = PCA9685_SetOutputMode(&hpca, true, false, false);

        if (status != HAL_OK) {
            CUBE_PRINT("PCA9685Task - PCA9685_SetOutputMode failed\r\n");
        }

        // Explicitly set the PWM frequency.
        status = PCA9685_SetPWMFreq(&hpca, PCA9685_DEFAULT_PWM_FREQ_HZ);

        if (status != HAL_OK) {
            CUBE_PRINT("PCA9685Task - PCA9685_SetPWMFreq failed\r\n");
        }

        // Make sure the chip is awake.
        status = PCA9685_Sleep(&hpca, false);

        if (status != HAL_OK) {
            CUBE_PRINT("PCA9685Task - PCA9685_Sleep(false) failed\r\n");
        }

        // Restart PWM logic after configuration.
        status = PCA9685_Restart(&hpca);

        if (status != HAL_OK) {
            CUBE_PRINT("PCA9685Task - PCA9685_Restart failed\r\n");
        }

    }

// ******************************************************************************************************
    while (1) {

        Command cm;

        // Ingest the command queue, up to 5 commands
        uint8_t proced = 0;
        while (qEvtQueue->Receive(cm) && (proced < 5)) {
            HandleCommand(cm);
            ++proced;
        }

        cm.Reset();

        osDelay(PCA9685_TASK_DELAY_MS);
    }
// ******************************************************************************************************
}
