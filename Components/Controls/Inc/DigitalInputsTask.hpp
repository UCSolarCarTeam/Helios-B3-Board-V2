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

extern Mutex driverInputsMutex;
extern Mutex lightInputsMutex;

/* Macros/Enums ------------------------------------------------------------*/
enum class InputOffsets : uint16_t {
    DASHBOARD_ROTATE    = (1U << 8),        // GPA0
    LAP_BUTTON          = (1U << 9),        // GPA1
    FNR_LOW             = (1U << 10),       // GPA2
    FNR_HIGH            = (1U << 11),       // GPA3
    MOTOR_RESET         = (1U << 12),       // GPA4
    EXTRA_6             = (1U << 13),       // GPA5
    HORN                = (1U << 14),       // GPA6
    EXTRA_5             = (1U << 15),       // GPA7

    RIGHT_SIGNAL        = (1U << 0),        // GPB0
    MECHANICAL_BRAKE    = (1U << 1),        // GPB1
    HAZARD              = (1U << 2),        // GPB2
    LEFT_SIGNAL         = (1U << 3),        // GPB3
    EXTRA_2             = (1U << 4),        // GPB4
    EXTRA_4             = (1U << 5),        // GPB5
    EXTRA_1             = (1U << 6),        // GPB6
    EXTRA_3             = (1U << 7)         // GPB7
};

enum class FNRState : uint8_t {
    Reverse,    // 0b00
    Neutral,    // 0b01
    Forward,    // 0b10
    Invalid     // 0b11
};

typedef struct DriverInputs {
    FNRState fnrState;
    bool forward;
    bool neutral;
    bool reverse;
    bool mechanicalBrake;
    bool motorReset;
    bool dashboardRotate;
    bool lapButton;
} DriverInputs;

typedef struct LightInputs {
    bool rightSignal;
    bool leftSignal;
    bool hazard;
    bool horn;
} LightInputs;

/* Class ------------------------------------------------------------------*/
class DigitalInputsTask : public Task
{
public:
    static DigitalInputsTask& Inst() {
        static DigitalInputsTask inst;
        return inst;
    }

    void InitTask();

    // Getter for states
    // Pass in reference and return copy of state
    static bool GetDriverInputs(DriverInputs& out);
    static bool GetLightInputs(LightInputs& out);

protected:
    static void RunTask(void* pvParams) { DigitalInputsTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code
    void HandleCommand(Command& cm);

private:
    // Private Functions
    DigitalInputsTask();        // Private constructor
    DigitalInputsTask(const DigitalInputsTask&);                        // Prevent copy-construction
    DigitalInputsTask& operator=(const DigitalInputsTask&);            // Prevent assignment

    DriverInputs driverInputs;
    LightInputs lightInputs;
};

#endif    // HELIOS_Digital_INPUTS_TASK_HPP_
