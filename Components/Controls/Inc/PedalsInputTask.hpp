/**
 ******************************************************************************
 * File Name          : PedalsInputTask.hpp
 * Description        : Samples the positive and negative channels for acceleration and regen braking pedals.
 ******************************************************************************
*/
#ifndef HELIOS_PEDALS_INPUT_TASK_HPP_
#define HELIOS_PEDALS_INPUT_TASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"

/* Macros/Enums ------------------------------------------------------------*/


class PedalsInputTask : public Task
{
public:
    static PedalsInputTask& Inst() {
        static PedalsInputTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { PedalsInputTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

private:
    // Private Functions
    PedalsInputTask();        // Private constructor
    PedalsInputTask(const PedalsInputTask&);                        // Prevent copy-construction
    PedalsInputTask& operator=(const PedalsInputTask&);            // Prevent assignment
};

#endif    // HELIOS_PEDALS_INPUT_TASK_HPP_
