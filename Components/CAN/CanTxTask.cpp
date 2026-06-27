/**
 ******************************************************************************
 * File Name          : CANTxTask.hpp
 * Description        : CAN Tx task. Transmit CAN messages it receives from a queue on CAN bus.
 ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "CanTxTask.hpp"
#include "stm32h5xx_hal_fdcan.h"

/**
 * @brief Constructor for CANTxTask
 */
CANTxTask::CANTxTask() : Task(CAN_TX_TASK_QUEUE_DEPTH_OBJS) 
{
}

/**
 * @brief Initialize CANTxTask
 */
void CANTxTask::InitTask() {
    // Ensure task is not already initialized
    CUBE_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)CANTxTask::RunTask,
            (const char*)"CANTxTask",
            (uint16_t)CAN_TX_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)CAN_TX_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    CUBE_ASSERT(rtValue == pdPASS, "CANTxTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Waits for messages in queue to transmit.
 */
void CANTxTask::Run(void* pvParams) {
    while(1) {
        // Wait for a command to be processed
        Command cm;
        qEvtQueue->ReceiveWait(cm);

        // Verify received command is a Task Specific Command
		if (cm.GetCommand() != TASK_SPECIFIC_COMMAND) {
			// Process command received from queue
			HandleCommand(cm);
		} else {
			CUBE_PRINT("CANTxTask - Received Unsupported Command {%d}\n", cm.GetCommand());
		}

        // Deallocate command
        cm.Reset();
    }
}

/**
 * @brief Process CAN message as per ID
 */
void CANTxTask::HandleCommand(Command& cm) {

	// Handle CAN message based on ID
	switch (static_cast<CANTX_COMMANDS>(cm.GetTaskCommand())) {
		case TEST_COMMAND:
			CUBE_PRINT_CAN_MESSAGE(cm.GetTaskCommand(), cm.GetDataSize(), cm.GetDataPointer());
			break;
        
        case HEARTBEAT:
            break;

        case DIGITAL_INPUTS:
            break;
            
        case ANALOG_INPUTS:
            break;

        case LIGHTS_INPUTS:
            break;

        case LIGHTS_STATUS:
            break;

        case GPS:
            break;

        case MPU_ACCEL:
            break;

        case MPU_GYRO:
            break;

        case MPU_TEMP:
            break;
		
        default:
			CUBE_PRINT("CANTxTask - Received Unsupported CAN Message {%X}\n", cm.GetTaskCommand());
			break;
	}
}

