/**
 ******************************************************************************
 * File Name          : CANRxTask.hpp
 * Description        : CAN Rx gatekeeper task. Receives messages from CAN Rx interrupt through queue.
 ******************************************************************************
*/
#ifndef HELIOS_CANRXTASK_HPP_
#define HELIOS_CANRXTASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "Queue.hpp"
#include "Mutex.hpp"
#include "Command.hpp"

/* Macros/Enums ------------------------------------------------------------*/
enum CANRX_COMMANDS {
    TEST_COMMAND
};

/* Class ------------------------------------------------------------------*/
class CANRxTask : public Task
{
public:
    static CANRxTask& Inst() {
        static CANRxTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { CANRxTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

private:
    // Private Functions
    CANRxTask();        // Private constructor
    CANRxTask(const CANRxTask&);                        // Prevent copy-construction
    CANRxTask& operator=(const CANRxTask&);            // Prevent assignment
};


//Helper function to print CAN message
void CUBE_PRINT_CAN_MESSAGE(uint32_t id, uint8_t dlc, uint8_t *data);

#endif    // HELIOS_CANRXTASK_HPP_
