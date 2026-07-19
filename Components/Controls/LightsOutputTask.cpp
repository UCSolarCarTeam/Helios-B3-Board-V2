/**
 ******************************************************************************
 * File Name          : LightsOutputTask.cpp
 * Description        : Task for controlling the vehicle lights and horn.
 ******************************************************************************
*/
#include "SystemDefines.hpp"
#include "LightsOutputTask.hpp"
#include "DigitalInputsTask.hpp"
#include "PedalsInputTask.hpp"
#include "pca9685.h"

/* Constants ---------------------------------------------------------------*/

// Valid PCA9685 channel numbers are 0 through 15.
constexpr uint8_t FRONT_LEFT_CHANNEL   = 0U;
constexpr uint8_t FRONT_RIGHT_CHANNEL  = 1U;
constexpr uint8_t MIDDLE_LEFT_CHANNEL  = 2U;
constexpr uint8_t MIDDLE_RIGHT_CHANNEL = 3U;
constexpr uint8_t REAR_LEFT_CHANNEL    = 4U;
constexpr uint8_t REAR_CENTER_CHANNEL  = 5U;
constexpr uint8_t REAR_RIGHT_CHANNEL   = 6U;
constexpr uint8_t HORN_CHANNEL         = 7U;

/*
 * Array order matches command bits 7 through 0:
 *
 * Index 0 = Bit 7 = Front-left
 * Index 1 = Bit 6 = Front-right
 * Index 2 = Bit 5 = Middle-left
 * Index 3 = Bit 4 = Middle-right
 * Index 4 = Bit 3 = Rear-left
 * Index 5 = Bit 2 = Rear-center
 * Index 6 = Bit 1 = Rear-right
 * Index 7 = Bit 0 = Horn
 */
constexpr uint8_t OUTPUT_CHANNELS[8] = {
    FRONT_LEFT_CHANNEL,
    FRONT_RIGHT_CHANNEL,
    MIDDLE_LEFT_CHANNEL,
    MIDDLE_RIGHT_CHANNEL,
    REAR_LEFT_CHANNEL,
    REAR_CENTER_CHANNEL,
    REAR_RIGHT_CHANNEL,
    HORN_CHANNEL
};

constexpr uint8_t OUTPUT_COUNT = 8U;
constexpr float OUTPUT_ON_DUTY  = 1.0f;
constexpr float OUTPUT_OFF_DUTY = 0.0f;
constexpr uint16_t OUTPUT_PHASE = 0U;

// The PCA9685 default 7-bit address is 0x40.
// STM32 HAL uses the left-shifted address, which is 0x80.
constexpr uint8_t PCA9685_I2C_ADDR = 0x80U;


static void SetLeftSignal(PCA9685_HandleTypeDef* hpca, bool blinkState) {
    PCA9685_SetDuty(hpca, 0, blinkState ? 1.0f : 0.0f, 0);
    PCA9685_SetDuty(hpca, 4, blinkState ? 1.0f : 0.0f, 0);
    PCA9685_SetDuty(hpca, 15, blinkState ? 1.0f : 0.0f, 0);
}

static void SetRightSignal(PCA9685_HandleTypeDef* hpca, bool blinkState) {
    PCA9685_SetDuty(hpca, 6, blinkState ? 1.0f : 0.0f, 0);
    PCA9685_SetDuty(hpca, 8, blinkState ? 1.0f : 0.0f, 0);
    PCA9685_SetDuty(hpca, 14, blinkState ? 1.0f : 0.0f, 0);
}

/**
 * @brief Constructor for PCA9685Task
 */
PCA9685Task::PCA9685Task()
    : Task(PCA9685_TASK_QUEUE_DEPTH_OBJS),
      hpca{},
      pcaReady(false)
{
}

/**
 * @brief Initialize the PCA9685Task
 */
void PCA9685Task::InitTask()
{
    CUBE_ASSERT(
        rtTaskHandle == nullptr,
        "Cannot initialize PCA9685 task twice");

    BaseType_t rtValue =
        xTaskCreate(
            (TaskFunction_t)PCA9685Task::RunTask,
            (const char*)"PCA9685Task",
            (uint16_t)PCA9685_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)PCA9685_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    CUBE_ASSERT(
        rtValue == pdPASS,
        "PCA9685Task::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Apply an 8-bit lights and horn command
 * @param lightCommand Bitmask containing the requested output states
 * @return HAL_OK if every output was updated successfully
 */
HAL_StatusTypeDef PCA9685Task::ApplyLightCommand(uint8_t lightCommand)
{
    if (!pcaReady) {
        return HAL_ERROR;
    }

    /*
     * Make sure every channel has been assigned before changing any output.
     * This prevents 0xFF placeholders from reaching the hardware.
     */
    for (uint8_t i = 0U; i < OUTPUT_COUNT; ++i) {
        if (OUTPUT_CHANNELS[i] > 15U) {
            CUBE_PRINT(
                "PCA9685Task - Output channel %u is unassigned\r\n",
                i);
            return HAL_ERROR;
        }
    }

    /*
     * OUTPUT_CHANNELS[0] corresponds to bit 7.
     * OUTPUT_CHANNELS[7] corresponds to bit 0.
     */
    for (uint8_t i = 0U; i < OUTPUT_COUNT; ++i) {
        uint8_t bitPosition = 7U - i;

        bool outputOn =
            (lightCommand & (1U << bitPosition)) != 0U;

        HAL_StatusTypeDef status =
            PCA9685_SetDuty(
                &hpca,
                OUTPUT_CHANNELS[i],
                outputOn ? OUTPUT_ON_DUTY : OUTPUT_OFF_DUTY,
                OUTPUT_PHASE);

        if (status != HAL_OK) {
            return status;
        }
    }

    return HAL_OK;
}

/**
 * @brief Handle a command received through the task queue
 * @param cm Command to process
 */
void PCA9685Task::HandleCommand(Command& cm)
{
    if ((cm.GetCommand() == TASK_SPECIFIC_COMMAND) &&
        (cm.GetTaskCommand() == PCA9685_SET_OUTPUT_MASK)) {

        if ((cm.GetDataPointer() == nullptr) ||
            (cm.GetDataSize() != sizeof(uint8_t))) {
            CUBE_PRINT(
                "PCA9685Task - Invalid light command payload\r\n");
        }
        else {
            uint8_t lightCommand = cm.GetDataPointer()[0];

            if (ApplyLightCommand(lightCommand) != HAL_OK) {
                CUBE_PRINT(
                    "PCA9685Task - Failed to apply command 0x%02X\r\n",
                    lightCommand);
            }
        }
    }
    else {
        CUBE_PRINT(
            "PCA9685Task - Received unsupported command\r\n");
    }

    cm.Reset();
}

/**
 * @brief Main run loop for the PCA9685 task
 * @param pvParams RTOS task parameters
 */
void PCA9685Task::Run(void* pvParams)
{
    CUBE_PRINT("PCA9685Task - Starting initialization\r\n");

    /*
     * Initialize the class member hpca.
     * Do not declare another local variable named hpca here.
     */
    HAL_StatusTypeDef status =
        PCA9685_Init(
            &hpca,
            SystemHandles::PWMX_Handle,
            PCA9685_I2C_ADDR);

    if (status != HAL_OK) {
        pcaReady = false;
        CUBE_PRINT("PCA9685Task - PCA9685 initialization failed\r\n");
    }
    else {
        pcaReady = true;
        CUBE_PRINT("PCA9685Task - PCA9685 initialization complete\r\n");
    }

    // Signal States
    bool isBraking;
    bool leftSignal;
    bool rightSignal;
    bool hazardSignal;
    bool hornSignal;

    // Blink States
    uint8_t blinkCounter = 0;
    bool blinkState = true;

    while (1) {

        // Increment counter and check for blink
        if (blinkCounter == 10) {
            blinkState = !blinkState;
            blinkCounter = 0;
        } else {
            blinkCounter++;
        }

        // Get Digital Inputs and check if regen pedal is pressed or mech brake switch is toggled
        uint8_t digitalInputs = DigitalInputsTask::Inst().GetDriverInputs();
        uint8_t lightsInputs = DigitalInputsTask::Inst().GetLightsInputs();
        isBraking = ( 
            (PedalsInputTask::Inst().GetRegenPercentage() > 5U) || 
            (digitalInputs & (1 << 3)) 
        );

        rightSignal = lightsInputs & (1 << 0);
        leftSignal = lightsInputs & (1 << 1);
        hazardSignal = lightsInputs & (1 << 2);
        hornSignal = digitalInputs & (1 << 5);
        
        // Toggle Horn
        if (hornSignal) {
            PCA9685_SetDuty(&hpca, 13, 1.0f, 0);
        } else {
            PCA9685_SetDuty(&hpca, 13, 0.0f, 0);
        }

        // Toggle Brake Lights
        if (isBraking) {
            PCA9685_SetDuty(&hpca, 5, 1.0f, 0);
        } else {
            PCA9685_SetDuty(&hpca, 5, 0.0f, 0);
        }

        // Signal and Hazard States
        if (hazardSignal) {
            SetLeftSignal(&hpca, blinkState);
            SetRightSignal(&hpca, blinkState);
        } else {
            SetLeftSignal(&hpca, leftSignal && blinkState);
            SetRightSignal(&hpca, rightSignal && blinkState);
        } 

        // Reset Blink States if no signals
        if ( !(hazardSignal || leftSignal || rightSignal) ) {
            blinkCounter = 0;
            blinkState = true;
        }

        osDelay(100);
    }
}
