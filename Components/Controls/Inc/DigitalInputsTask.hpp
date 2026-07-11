/**
 ******************************************************************************
 * File Name          : PCA9685Task.hpp
 * Description        : Task for controlling the PCA9685 PWM driver.
 ******************************************************************************
*/
#ifndef HELIOS_PCA9685TASK_HPP_
#define HELIOS_PCA9685TASK_HPP_

#include <pca9685.h>
#include "Task.hpp"
#include "SystemDefines.hpp"

/* Macros/Enums ------------------------------------------------------------*/
enum PCA9685_COMMANDS {
    PCA9685_NONE = 0,
    PCA9685_SET_DUTY,       // Sets one PCA9685 channel duty cycle
    PCA9685_SET_PWM,        // Sets one PCA9685 channel using raw ON/OFF counts
    PCA9685_SET_ALL_PWM,    // Sets all PCA9685 channels using raw ON/OFF counts
    PCA9685_SET_FREQ,       // Changes the PCA9685 PWM frequency
    PCA9685_SLEEP_CMD,      // Puts the PCA9685 into sleep or wakes it up
    PCA9685_RESTART_CMD     // Restarts the PCA9685 PWM logic
};

class PCA9685Task : public Task
{
public:
    static PCA9685Task& Inst() {
        static PCA9685Task inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { PCA9685Task::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void* pvParams); // Main run code

    void HandleCommand(Command& cm);

private:
    // Private Functions
    PCA9685Task();                                      // Private constructor
    PCA9685Task(const PCA9685Task&);                    // Prevent copy-construction
    PCA9685Task& operator=(const PCA9685Task&);         // Prevent assignment

    // Private Variables
    PCA9685_HandleTypeDef hpca;                         // PCA9685 driver handle
};

#endif // HELIOS_PCA9685TASK_HPP_
