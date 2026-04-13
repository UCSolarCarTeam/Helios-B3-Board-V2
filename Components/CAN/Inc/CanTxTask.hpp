/**
 ******************************************************************************
 * File Name          : CANTxTask.hpp
 * Description        : CAN Tx queue task. Receives messages from different tasks through queue and transmits to CAN bus.
 ******************************************************************************
*/
#ifndef HELIOS_CANTXTASK_HPP_
#define HELIOS_CANTXTASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "Queue.hpp"
#include "Mutex.hpp"

/* Macros/Enums ------------------------------------------------------------*/


/* Class ------------------------------------------------------------------*/
class CANTxTask : public Task
{
public:
    static CANTxTask& Inst() {
        static CANTxTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { CANTxTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

private:
    // Private Functions
    CANTxTask();        // Private constructor
    CANTxTask(const CANTxTask&);                        // Prevent copy-construction
    CANTxTask& operator=(const CANTxTask&);            // Prevent assignment
};

#endif    // HELIOS_CANTxTASK_HPP_
