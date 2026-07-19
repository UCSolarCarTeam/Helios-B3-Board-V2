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
    MCP23017_HandleTypeDef hiox = {
        .hi2c = SystemHandles::IOX_Handle,
        .addr = 0x42,    // A0 = 1 | A1 = A2 = 0
		.last_write = {0x00, 0x00},
		.pending_write = {0x00, 0x00},
		.last_read = {0x00, 0x00}
    };

	// Initialize IO Expander
    IOE_Init(&hiox);

	while (1) {

		// Test GPIO Inputs
#if 0
        uint16_t lastread = getLastRead(&hiox);
        uint8_t gpa = (lastread >> 8) & 0xFF;
        uint8_t gpb = lastread & 0xFF;
        CUBE_PRINT("GPA0: %d\r\n", (gpa >> 0) & 1);
        CUBE_PRINT("GPA1: %d\r\n", (gpa >> 1) & 1);
        CUBE_PRINT("GPA2: %d\r\n", (gpa >> 2) & 1);
        CUBE_PRINT("GPA3: %d\r\n", (gpa >> 3) & 1);
        CUBE_PRINT("GPA4: %d\r\n", (gpa >> 4) & 1);
        CUBE_PRINT("GPA5: %d\r\n", (gpa >> 5) & 1);
        CUBE_PRINT("GPA6: %d\r\n", (gpa >> 6) & 1);
        CUBE_PRINT("GPA7: %d\r\n", (gpa >> 7) & 1);

        CUBE_PRINT("GPB0: %d\r\n", (gpb >> 0) & 1);
        CUBE_PRINT("GPB1: %d\r\n", (gpb >> 1) & 1);
        CUBE_PRINT("GPB2: %d\r\n", (gpb >> 2) & 1);
        CUBE_PRINT("GPB3: %d\r\n", (gpb >> 3) & 1);
        CUBE_PRINT("GPB4: %d\r\n", (gpb >> 4) & 1);
        CUBE_PRINT("GPB5: %d\r\n", (gpb >> 5) & 1);
        CUBE_PRINT("GPB6: %d\r\n", (gpb >> 6) & 1);
        CUBE_PRINT("GPB7: %d\r\n", (gpb >> 7) & 1);
#endif 0
        
        // DriverInputs
        uint8_t driverInputs = 0;
        uint8_t lightsInputs = 0;
        uint8_t fnrState = 0;

        // Get Last read
        // lasteread = GPA << 8 | GPB
        uint16_t lastread = getLastRead(&hiox);

        // GPA0
        // Motor Reset
        if ((lastread >> 8) & 1) {
            driverInputs |= (1 << 4);
            CUBE_PRINT("Motor Reset\r\n");
        }

        // GPA1
        // Left Signal
        if ((lastread >> 9) & 1) {
            driverInputs |= (1 << 1);
            CUBE_PRINT("Left Signal\r\n");
        }
        
        // GPA2
        // Horn Enable
        if ((lastread >> 10) & 1) {
            driverInputs |= (1 << 5);
            CUBE_PRINT("Horn Enable\r\n");
        }

        // GPA3
        // Right Signal
        if ((lastread >> 11) & 1) {
            driverInputs |= (1 << 0);
            CUBE_PRINT("Right Signal\r\n");
        }

        // GPA4
        // Dashboard Rotate
        if ((lastread >> 12) & 1) {
            driverInputs |= (1 << 6);
            CUBE_PRINT("Dashboard Rotate\r\n");
        }

        // GPA5
        // Lap Button
        if ((lastread >> 13) & 1) {
            driverInputs |= (1 << 7);
            CUBE_PRINT("Lap Button\r\n");
        }

        // GPA6
        // Forward/Neutral/Reverse Combo 0
        if ((lastread >> 14) & 1) {
            // driverInputs |= (1 << );
            fnrState |= 1;
            CUBE_PRINT("FWD 0\r\n");
        }

        // GPA7
        // Forward/Neutral/Reverse Combo 1
        if ((lastread >> 15) & 1) {
            // driverInputs |= (1 << );
            fnrState |= (1 << 1);
            CUBE_PRINT("FWD 1\r\n");
        }

        // GPB2
        // Mechanical Brake
        if ((lastread >> 2) & 1) {
            driverInputs |= (1 << 3);
            CUBE_PRINT("Mechanical Brake\r\n");
        }

        // GPB3
        // Emergency Hazard
        if ((lastread >> 3) & 1) {
            lightsInputs |= (1 << 2);
            CUBE_PRINT("Emergency Hazard\r\n");
        }

        // FNR (MAY CHANGE)
        // 00 reverse
        // 01 neutral
        // 10 forward
        // 11 not implemented
        switch (fnrState)
        {
        // REVERSE
        case 0b00:
            driverInputs |= (1 << 2);
            break;
        
        // NEUTRAL
        case 0b01:
            driverInputs |= (1 << 1);
            break;

        // FORWARD
        case 0b10:
            driverInputs |= (1 << 0);
            break;
        
        // ERROR
        case 0b11:
            break;

        default:
            break;
        }
        
		osDelay(50);

	}
}
