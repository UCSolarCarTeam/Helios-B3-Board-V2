/**
 ******************************************************************************
 * File Name          : MotorControlTask.hpp
 * Description        : Header for the Motor Control Task
 ******************************************************************************
 */
#ifndef HELIOS_MOTOR_CONTROL_TASK_HPP_
#define HELIOS_MOTOR_CONTROL_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "Command.hpp"
#include "SystemDefines.hpp"

#include "CanRxTask.hpp"
#include "CanTxTask.hpp"
#include "PedalsInputTask.hpp"
#include "DigitalInputsTask.hpp"
#include "CAN.hpp"

/* Macros ------------------------------------------------------------------*/

// CAN frames (base address 0x500 + offset)
#define MOTOR_DRIVE_STDID 0x501U
#define MOTOR_DRIVE_DLC 8

#define MOTOR_POWER_STDID 0x502U
#define MOTOR_POWER_DLC 8

#define MOTOR_RESET_STDID 0x503U

// Bus current setpoint, fraction of the controller's bus current limit (0.0 to 1.0)
#define BUS_CURRENT_OUT 1.0f

// Unreachable targets, so current follows the pedal (Torque Control)
#define MAX_FORWARD_RPM 20000
#define MAX_REVERSE_RPM (-20000)
#define REGEN_RPM 0.5f

// Pedal thresholds, in percent (0 to 100) as returned by PedalsInputTask
#define NON_ZERO_THRESHOLD 3.0f
#define MAX_PEDAL_THRESHOLD 100.0f

// Fraction of the controller's max current sent at full pedal
// (comment in the old code said 100 A max config, 69 A at full pedal = 0.69f)
#define MOTOR_PERCENTAGE_REDUCER 1.0f
#define REGEN_INPUT_SCALING 0.175f

// Vehicle velocity gates for changing direction (m/s)
// A small margin around zero allows for things like slow turns.
#define SAFE_VEHICLE_VELOCITY_TO_GO_FORWARD (-1.38f)
#define SAFE_VEHICLE_VELOCITY_TO_GO_REVERSE 1.38f

#define LAP_PIN CONTEXT_Pin
#define LAP_GPIO_PORT CONTEXT_GPIO_Port

/* Class ------------------------------------------------------------------*/

class MotorControlTask : public Task {
public:
    static MotorControlTask &Inst()
    {
        static MotorControlTask inst;
        return inst;
    }

    void InitTask();

    // Getters for CANTx Task
    CANMsg MotorControlTask::getMotorDrive() { return motor_drive_msg; }
    CANMsg MotorControlTask::getMotorPower() { return motor_power_msg; }

    // Setters for CANRx Task
    void SetAllowCharge(bool state) { allowCharge = state; }
    void SetAllowDischarge(bool state) { allowDischarge = state; }
    void SetMotor0VehicleVelocityInput(float value) { motor0VehicleVelocityInput = value; }
    void SetMotor1VehicleVelocityInput(float value) { motor1VehicleVelocityInput = value; }

protected:
    static void RunTask(void *pvParams) { MotorControlTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run()
    void Run(void *pvParams);                                                       // Main run code
    void HandleCommand(Command &cm);

private:
    MotorControlTask();                                      // Private constructor
    MotorControlTask(const MotorControlTask &);              // Prevent copy-construction
    MotorControlTask &operator=(const MotorControlTask &);   // Prevent assignment

    // Written by the CAN Rx task, read by Run(). Fail-safe defaults: no drive or
    // regen until the AuxBMS permits it; zero velocity is "safe" in both directions.
    volatile bool  allowCharge = false;
    volatile bool  allowDischarge = false;
    
    volatile float motor0VehicleVelocityInput = 0.0f;
    volatile float motor1VehicleVelocityInput = 0.0f;

    CANMsg motor_drive_msg;
    CANMsg motor_power_msg;
};

#endif