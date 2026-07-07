/**
 ******************************************************************************
 * File Name          : DigitalInputTask.hpp
 * Description        : Polls the IOExpander and performs the appropriate actions.
 ******************************************************************************
*/
#include "DigitalInputsTask.hpp"

/**
 * @brief Constructor for DigitalInputTask
 */
DigitalInputsTask::DigitalInputsTask() : Task(DIGITAL_INPUTS_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the DigitalInputTask
 */
void DigitalInputsTask::InitTask()
{
    // Make sure the task is not already initialized
    CUBE_ASSERT(rtTaskHandle == nullptr, "Cannot initialize DigitalInputs task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)DigitalInputsTask::RunTask,
            (const char*)"DigitalInputsTask",
            (uint16_t)DIGITAL_INPUTS_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)DIGITAL_INPUTS_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    CUBE_ASSERT(rtValue == pdPASS, "DigitalInputsTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the DigitalInputs Task. Polls IO expander and performs appropriate actions.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void DigitalInputsTask::Run(void * pvParams)
{
	// Create IOExpander Handler

	// Initialize IO Expander


	while (1) {



		osDelay(1000);

	}
}
