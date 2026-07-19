/**
 ******************************************************************************
 * File Name          : DigitalInputsTask.hpp
 * Description        : Polls the IOExpander and performs the appropriate actions.
 ******************************************************************************
*/
#ifndef HELIOS_DIGITAL_INPUTS_TASK_HPP_
#define HELIOS_DIGITAL_INPUTS_TASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"
#include "IOExpander.h"

/* Macros/Enums ------------------------------------------------------------*/


class DigitalInputsTask : public Task
{
public:
    static DigitalInputsTask& Inst() {
        static DigitalInputsTask inst;
        return inst;
    }

    void InitTask();

    // Getter for states
    uint8_t GetFnrState() const { return fnrState; }
    uint8_t GetDriverInputs() const { return driverInputs; }
    uint8_t GetLightsInputs() const { return lightsInputs; }

protected:
    static void RunTask(void* pvParams) { DigitalInputsTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code
    void HandleCommand(Command& cm);

private:
    // Private Functions
    DigitalInputsTask();        // Private constructor
    DigitalInputsTask(const DigitalInputsTask&);                        // Prevent copy-construction
    DigitalInputsTask& operator=(const DigitalInputsTask&);            // Prevent assignment

    uint8_t driverInputs;
    uint8_t lightsInputs;
    uint8_t fnrState;
};

#endif    // HELIOS_Digital_INPUTS_TASK_HPP_
