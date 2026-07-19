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
    uint8_t GetAccelPercentage() const { return accelPercentage; }
    uint8_t GetRegenPercentage() const { return regenPercentage; }

protected:
    static void RunTask(void* pvParams) { PedalsInputTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code
    void HandleCommand(Command& cm);

    // Pedal ADC Handlers
    ADC_HandleTypeDef *haccel_ = nullptr;
    ADC_HandleTypeDef *hregen_ = nullptr;
    
    // ADC DMA Buffers
    uint16_t accelDmaBuffer[2];
    uint16_t regenDmaBuffer[2]; 

    // Raw ADC Values
    uint16_t v_accelReading_P;
    uint16_t v_accelReading_N;
    uint16_t v_regenReading_P; 
    uint16_t v_regenReading_N;

    // Pedal Percentage Values
    uint8_t accelPercentage;
    uint8_t regenPercentage;

private:
    // Private Functions
    PedalsInputTask();        // Private constructor
    PedalsInputTask(const PedalsInputTask&);                        // Prevent copy-construction
    PedalsInputTask& operator=(const PedalsInputTask&);            // Prevent assignment
};

#endif    // HELIOS_PEDALS_INPUT_TASK_HPP_
