/**
 ******************************************************************************
 * File Name          : PedalsInputTask.hpp
 * Description        : Samples the positive and negative channels for acceleration and regen braking pedals.
 ******************************************************************************
*/
#include "PedalsInputTask.hpp"
#include <algorithm>

#define PEDAL_MIN 1600U
#define PEDAL_MAX 3800U

// Map ADC values and scale to percentage
static uint8_t MapAdcToPercent(uint16_t adc, uint16_t min, uint16_t max)
{
    // Handle increasing sensor
    if (min < max)
    {
        adc = std::clamp(adc, min, max);
        return static_cast<uint8_t>(
            ((uint32_t)(adc - min) * 100U) / (max - min));
    }

    // Handle decreasing sensor
    adc = std::clamp(adc, max, min);
    return static_cast<uint8_t>(
        ((uint32_t)(min - adc) * 100U) / (min - max));
}

/**
 * @brief Constructor for PedalsInputTask
 */
PedalsInputTask::PedalsInputTask() : Task(PEDALS_INPUT_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the PedalsInputTask
 */
void PedalsInputTask::InitTask()
{
    // Make sure the task is not already initialized
    CUBE_ASSERT(rtTaskHandle == nullptr, "Cannot initialize PedalsInput task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)PedalsInputTask::RunTask,
            (const char*)"PedalsInputTask",
            (uint16_t)PEDALS_INPUT_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)PEDALS_INPUT_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    CUBE_ASSERT(rtValue == pdPASS, "PedalsInputTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the PedalsInput Task. Reads ADC DMA buffers and processes into pedal percentage.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void PedalsInputTask::Run(void * pvParams)
{
    // HAL Status Handlers for Debugging
    HAL_StatusTypeDef statusA;
    HAL_StatusTypeDef statusR;

    // Calibrate ADCs
    statusR = HAL_ADCEx_Calibration_Start(SystemHandles::Regen_Handle, ADC_SINGLE_ENDED);
    statusA = HAL_ADCEx_Calibration_Start(SystemHandles::Accel_Handle, ADC_SINGLE_ENDED);

    //Start ADCs
    statusR = HAL_ADC_Start_DMA(SystemHandles::Regen_Handle, (uint32_t*)regenDmaBuffer, 2);
    statusA = HAL_ADC_Start_DMA(SystemHandles::Accel_Handle, (uint32_t*)accelDmaBuffer, 2);

    // Start timer for DMA conversion
    HAL_StatusTypeDef status = HAL_TIM_Base_Start(SystemHandles::ADC_Timer_Handle);

    while (1) {
        // Read Pedal Values from DMA
        v_regenReading_P = regenDmaBuffer[0];
        v_regenReading_N = regenDmaBuffer[1];
        v_accelReading_P = accelDmaBuffer[0];
        v_accelReading_N = accelDmaBuffer[1];
        // CUBE_PRINT("%u %u | %u %u\r\n", v_accelReading_P, v_accelReading_N, v_regenReading_P, v_regenReading_N);

        accelPercentage = MapAdcToPercent(v_accelReading_P, PEDAL_MIN, PEDAL_MAX);
        regenPercentage = MapAdcToPercent(v_regenReading_P, PEDAL_MIN, PEDAL_MAX);
        // CUBE_PRINT("ACCEL: %u%%  REGEN: %u%%\r\n", accelPercentage, regenPercentage);

        osDelay(200);
    }
}
