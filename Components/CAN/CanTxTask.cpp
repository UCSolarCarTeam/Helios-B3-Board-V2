/**
 ******************************************************************************
 * File Name          : CANTxTask.hpp
 * Description        : CAN Tx task. Transmit CAN messages it receives from a queue on CAN bus.
 ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "CanTxTask.hpp"
#include "stm32h5xx_hal_fdcan.h"
#include <string.h>

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

    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    
    // Allocate data into Tx Buffer if it exists
    uint16_t dataLength = cm.GetDataSize();
    if (dataLength != 0) {
        memcpy(txData, cm.GetDataPointer(), dataLength);
    }

    // Set message header parameters
    txHeader.Identifier = 0x0;
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = dataLength;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

	// Handle CAN message based on ID
	switch (static_cast<CANTX_COMMANDS>(cm.GetTaskCommand())) {
        case HEARTBEAT:
            txHeader.Identifier = 0x700;
            break;

        case DRIVER_INPUTS:
            txHeader.Identifier = 0x710;
            break;
            
        case ANALOG_INPUTS:
            txHeader.Identifier = 0x711;
            break;

        case LIGHTS_INPUTS:
            txHeader.Identifier = 0x720;
            break;

        case LIGHTS_STATUS:
            txHeader.Identifier = 0x721;
            break;

        case GPS_TIME:
            txHeader.Identifier = 0x730;
            break;

        case GPS_FLAGS:
            txHeader.Identifier = 0x731;
            break;

        case GPS_POSITION:
            txHeader.Identifier = 0x732;
            break;

        case MPU_ACCEL:
            txHeader.Identifier = 0x733;
            break;

        case MPU_GYRO:
            txHeader.Identifier = 0x734;
            break;

        case MPU_TEMP:
            txHeader.Identifier = 0x735;
            break;
        
        case MOTOR_DRIVE_INPUT:
            txHeader.Identifier = 0x501;
            break;

        case MOTOR_POWER_INPUT:
            txHeader.Identifier = 0x502;
            break;

        case MOTOR_RESET_INPUT:
            txHeader.Identifier = 0x503;
            break;
		
        default:
			CUBE_PRINT("CANTxTask - Received Unsupported CAN Message {%X}\n", cm.GetTaskCommand());
			// CUBE_PRINT_CAN_MESSAGE(cm.GetTaskCommand(), cm.GetDataSize(), cm.GetDataPointer());
			break;
	}

    if (txHeader.Identifier != 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(SystemHandles::FDCAN_Handle, &txHeader, txData)!= HAL_OK) {
            __NOP();
        	// Error_Handler();
        }
    }

    cm.Reset();
}

