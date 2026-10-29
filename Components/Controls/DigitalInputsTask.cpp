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
DigitalInputsTask::DigitalInputsTask() 
    : Task(DIGITAL_INPUTS_TASK_QUEUE_DEPTH_OBJS),
    driverInputs{}, lightInputs{}
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
 * @brief Decodes the driver inputs polled from the IO expander
 * @param lastread inputs last read from the IO expander
 */
static void DecodeDriverInputs(uint16_t lastread) 
{  
    if ( !driverInputsMutex.Lock(5) ) {
        return;
    }

    driverInputs.motorReset      = (lastread & <static_cast>(InputOffsets::MOTOR_RESET)) != 0;
    driverInputs.dashboardRotate = (lastread & <static_cast>(InputOffsets::DASHBOARD_ROTATE)) != 0;
    driverInputs.lapButton       = (lastread & <static_cast>(InputOffsets::LAP_BUTTON)) != 0;
    driverInputs.mechanicalBrake = (lastread & <static_cast>(InputOffsets::MECHANICAL_BRAKE)) != 0;

    const bool fnrLow       = (lastread & static_cast<uint16_t>(InputOffsets::FNR_LOW)) != 0;
    const bool fnrHigh      = (lastread & static_cast<uint16_t>(InputOffsets::FNR_HIGH)) != 0;

    // FNR (MAY CHANGE)
    // 00 reverse
    // 01 neutral
    // 10 forward
    // 11 not implemented
    if (fnrHigh && fnrLow) {
        driverInputs.fnrState = FNRState::Invalid;
    } else if (fnrHigh) {
        driverInputs.fnrState = FNRState::Forward;
    } else if (fnrLow) {
        driverInputs.fnrState = FNRState::Neutral;
    } else {
        driverInputs.fnrState = FNRState::Reverse;
    }

    driverInputsMutex.Unlock();

    return;
}

/**
 * @brief Decodes the lights inputs polled from the IO expander
 * @param lastread inputs last read from the IO expander
 */
static void DecodeLightInputs(uint16_t lastread) 
{  
    if ( !lightInputsMutex.Lock(5) ) {
    return;
    }

    lightInputs.hazard          = (lastread & <static_cast>(InputOffsets::HAZARD)) != 0;
    lightInputs.horn            = (lastread & <static_cast>(InputOffsets::HORN)) != 0;
    lightInputs.rightSignal     = (lastread & <static_cast>(InputOffsets::RIGHT_SIGNAL)) != 0;
    lightInputs.leftSignal      = (lastread & <static_cast>(InputOffsets::LEFT_SIGNAL)) != 0;

    lightInputsMutex.Unlock();

    return;
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

        // Get Last read
        // lasteread = GPA << 8 | GPB
        uint16_t lastread = getLastRead(&hiox);
		
#if 0
        // Test GPIO Inputs
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
        
        DecodeDriverInputs(lastread);
        DecodeLightInputs(lastread);
        
        // Send CAN Messages

		osDelay(50);

	}
}

/**
 * @brief Getter function that returns a copy of DriverInputs state via reference while being thread-safe.
 * @param out reference to inputs state to copy into.
 * @return True or False if copy was successful.
 */
bool GetDriverInputs(DriverInputs& out) {
    if ( !driverInputsMutex.Lock(5) ) {
        return false;
    }
    out = driverInputs;
    driverInputsMutex.Unlock();
    return true;
    

}

/**
 * @brief Getter function that returns a copy of LightInputs state via reference while being thread-safe
 * @param out reference to inputs state to copy into
 */
bool GetLightInputs(LightInputs& out) {
    if ( !lightInputsMutex.Lock(5) ) {
        return false;
    }
    out = lightInputs;
    lightInputsMutex.Unlock();
    return true;
    
}
