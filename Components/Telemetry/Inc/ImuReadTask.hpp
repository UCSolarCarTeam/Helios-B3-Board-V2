/**
 ******************************************************************************
 * File Name          : ImuReadTask.hpp
 * Description        : Accesses IAM-20680HT to poll accelerometer, gyroscope, and temperature.
 ******************************************************************************
*/
#ifndef HELIOS_IMUREADTASKTASK_HPP_
#define HELIOS_IMUREADTASKTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"

/* Macros/Enums ------------------------------------------------------------*/



class ImuReadTask : public Task
{
public:
    static ImuReadTask& Inst() {
        static ImuReadTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { ImuReadTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

private:
    // Private Functions
    ImuReadTask();        // Private constructor
    ImuReadTask(const ImuReadTask&);                        // Prevent copy-construction
    ImuReadTask& operator=(const ImuReadTask&);            // Prevent assignment
};

#endif    // HELIOS_ImuReadTaskTASK_HPP_
