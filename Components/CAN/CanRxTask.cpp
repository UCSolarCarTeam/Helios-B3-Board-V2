/**
 ******************************************************************************
 * File Name          : CANRxTask.hpp
 * Description        : CAN Rx gatekeeper task. Receives messages from CAN Rx interrupt through queue.
 ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "CanRxTask.hpp"
#include "stm32h5xx_hal_fdcan.h"


/**
 * @brief Constructor for CANRxTask
 */
CANRxTask::CANRxTask() : Task(CAN_TX_TASK_QUEUE_DEPTH_OBJS) 
{
}

/**
 * @brief Initialize CANRxTask
 */
void CANRxTask::InitTask() {
    // Ensure task is not already initialized
    CUBE_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)CANRxTask::RunTask,
            (const char*)"CANRxTask",
            (uint16_t)CAN_RX_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)CAN_RX_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    CUBE_ASSERT(rtValue == pdPASS, "CANRxTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Waits for received CAN messages in queue to process.
 *      Message is received from FDCAN RxFifo
 */
void CANRxTask::Run(void* pvParams) {
    while(1) {
        // Wait for a command to be processed
        Command cm;
        qEvtQueue->ReceiveWait(cm);

        // Verify received command is a Task Specific Command
		if (cm.GetCommand() != TASK_SPECIFIC_COMMAND) {
			// Process command received from queue
			HandleCommand(cm);
		} else {
			CUBE_PRINT("CANRxTask - Received Unsupported Command {%d}\n", cm.GetCommand());
		}

        // Deallocate command
        cm.Reset();
    }
}

/**
 * @brief Process CAN message as per ID
 */
void CANRxTask::HandleCommand(Command& cm) {

	// Handle CAN message based on ID
	switch (cm.GetTaskCommand()) {
		case TEST_COMMAND:
			CUBE_PRINT_CAN_MESSAGE(cm.GetTaskCommand(), cm.GetDataSize(), cm.GetDataPointer());
			break;
		default:
			CUBE_PRINT("CANRxTask - Received Unsupported CAN Message {%X}\n", cm.GetTaskCommand());
			break;
	}
}

/**
 * @brief Receive callback for RXFIFO0, receives CAN message and adds a command to process in queue.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    // Rx header and buffer to store message
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (HAL_FDCAN_GetRxMessage(SystemHandles::FDCAN_Handle, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
        // Set a CANRX Command based on identifier
        uint8_t taskCommand;
        switch(header.Identifier) {
            default:
                taskCommand = TEST_COMMAND;
                break;
        }

        // Create a new command matching message identifier and allocate data
        Command msg = Command(TASK_SPECIFIC_COMMAND, taskCommand);
        msg.CopyDataToCommand(data, header.DataLength);
        CANRxTask::Inst().GetEventQueue()->SendFromISR(msg);
            
    }

}

/**
 * @brief Helper function to print CAN message
 */
void CUBE_PRINT_CAN_MESSAGE(uint32_t id, uint8_t dlc, uint8_t *data)
{
	CUBE_PRINT("New CAN Message!\n");
    CUBE_PRINT("  ID: 0x%08X\n", id);
    CUBE_PRINT("  DLC: %u\n", dlc);
    CUBE_PRINT("  Data: ");
    for (uint8_t i = 0; i < dlc; ++i)
    {
        CUBE_PRINT("%02X ", data[i]);
    }
}
