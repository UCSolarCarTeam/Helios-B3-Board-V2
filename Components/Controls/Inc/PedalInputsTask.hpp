/**
 ******************************************************************************
 * File Name          : PedalInputsTask.hpp
 * Description        : Samples the positive and negative channels for acceleration and regen braking pedals.
 ******************************************************************************
*/
#ifndef HELIOS_PEDAL_INPUTS_TASK_HPP_
#define HELIOS_PEDAL_INPUTS_TASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"

extern Mutex pedalInputsMutex;

/* Macros/Enums ------------------------------------------------------------*/
typedef struct PedalInputs {
    uint8_t accelPercentage;
    uint8_t regenPercentage;
} PedalInputs;

/* Class ------------------------------------------------------------------*/
class PedalInputsTask : public Task
{
public:
    static PedalInputsTask& Inst() {
        static PedalInputsTask inst;
        return inst;
    }

    void InitTask();
    bool GetPedalPercentages( PedalInputs& out );

protected:
    static void RunTask(void* pvParams) { PedalInputsTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code
    void HandleCommand(Command& cm);
    
private:
    // Private Functions
    PedalInputsTask();        // Private constructor
    PedalInputsTask(const PedalInputsTask&);                        // Prevent copy-construction
    PedalInputsTask& operator=(const PedalInputsTask&);            // Prevent assignment

    // ADC DMA Buffers
    uint16_t accelDmaBuffer[2];
    uint16_t regenDmaBuffer[2]; 

    // Raw ADC Values
    uint16_t v_accelReading_P;
    uint16_t v_accelReading_N;
    uint16_t v_regenReading_P; 
    uint16_t v_regenReading_N;

    // Pedal Percentage Values
    PedalInputs pedalInputs;
};

#endif    // HELIOS_PEDAL_INPUTS_TASK_HPP_
