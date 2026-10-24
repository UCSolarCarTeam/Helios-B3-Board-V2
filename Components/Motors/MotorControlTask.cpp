#include <string.h> // for memcpy
#include "cmsis_os2.h"

#include "MotorControlTask.hpp"
#include "DriveLogic.hpp"
#include "CANTxTask.hpp"

namespace {
    // 10 Hz
    constexpr uint32_t DRIVE_COMMAND_PERIOD_TICKS = 100;

    // True if DigitalInputsTask doesn't flip the active low bits lol
    constexpr bool DRIVER_INPUTS_ACTIVE_LOW = false;

    constexpr uint8_t BIT_FORWARD    = 0;
    constexpr uint8_t BIT_REVERSE    = 2;
    constexpr uint8_t BIT_MECH_BRAKE = 3;
    constexpr uint8_t BIT_RESET      = 4;

    DriveLogic::Config makeConfig(){
        // Please adjust the config in the Header file :)
        DriveLogic::Config cfg;

        cfg.accPedalDeadzone = static_cast<float>(NON_ZERO_THRESHOLD);
        cfg.regPedalDeadzone = static_cast<float>(NON_ZERO_THRESHOLD);

        cfg.pedalMax = static_cast<float>(MAX_PEDAL_THRESHOLD);
        
        cfg.forwardRpm = static_cast<float>(MAX_FORWARD_RPM);
        cfg.reverseRpm = static_cast<float>(MAX_REVERSE_RPM);
        cfg.regenRpm = static_cast<float>(REGEN_RPM); 

        cfg.accelScaling = static_cast<float>(MOTOR_PERCENTAGE_REDUCER);
        cfg.regenScaling = static_cast<float>(MOTOR_PERCENTAGE_REDUCER * REGEN_INPUT_SCALING);

        return cfg;
    }

    DriveLogic::Direction decodeDirection(uint8_t inputs){
        const bool fwd = isPressed(inputs, BIT_FORWARD);
        const bool rev = isPressed(inputs, BIT_REVERSE);

        if (fwd && rev) { 
            return DriveLogic::Direction::Invalid; 
        }

        if (fwd) { 
            return DriveLogic::Direction::Forward; 
        }

        if (rev) { 
            return DriveLogic::Direction::Reverse; 
        }

        return DriveLogic::Direction::Neutral;
    }

    bool isPressed(uint8_t inputs, uint8_t bit){
        const bool raw = (inputs >> bit) & 1U;
        return DRIVER_INPUTS_ACTIVE_LOW ? !raw : raw;
    }

    float clamp(float a){
        if (!(a > 0.0f)) { 
            return 0.0f; 
        }

        if (a > 1.0f){ 
            return 1.0f; 
        }
        
        return a;
    }

    void sendMotor(CANMsg& msg, CANTX_COMMANDS commandID, float lowWord, float highWord)
    {
        float data[2] = {lowWord, highWord};
        memcpy(&msg.data[0], &data[0], sizeof(data));

        Command cmd = Command(TASK_SPECIFIC_COMMAND, commandID);
        cmd.CopyDataToCommand(reinterpret_cast<uint8_t*>(data), sizeof(data));
        CANTxTask::Inst().SendCommandReference(cmd);
    }

} // End namespace

MotorControlTask::MotorControlTask() : Task(MOTOR_CONTROL_TASK_QUEUE_DEPTH_OBJS) {}

void MotorControlTask::InitTask() {
    CUBE_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Motor Control task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)MotorControlTask::RunTask,
                    (const char *)"MotorControlTask",
                    (uint16_t)MOTOR_CONTROL_TASK_STACK_DEPTH_WORDS,
                    (void *)this,
                    (UBaseType_t)MOTOR_CONTROL_TASK_RTOS_PRIORITY,
                    (TaskHandle_t *)&rtTaskHandle);

    CUBE_ASSERT(rtValue == pdPASS, "MotorControlTask::InitTask() - xTaskCreate() failed");
}

void MotorControlTask::Run(void *pvParams){
    motor_drive_msg.ID = 0;
    motor_drive_msg.extendedID = MOTOR_DRIVE_STDID;
    motor_drive_msg.DLC = MOTOR_DRIVE_DLC;

    motor_power_msg.ID = 0;
    motor_power_msg.extendedID = MOTOR_POWER_STDID;
    motor_power_msg.DLC = MOTOR_POWER_DLC;

    const DriveLogic::Config cfg = makeConfig();
    DriveLogic::State state;

    bool prevResetPressed = false;
    uint32_t wakeTime = osKernelGetTickCount();

    for (;;){
        wakeTime += DRIVE_COMMAND_PERIOD_TICKS;
        osDelayUntil(wakeTime);

        // Get Driver Inputs
        const uint8_t driverInputs = DigitalInputsTask::Inst().GetDriverInputs();

        // Velocity Gates for Switching between Fwd and Rev
        const float v0 = motor0VehicleVelocityInput;
        const float v1 = motor1VehicleVelocityInput;

        const bool forwardSafe = v0 >= SAFE_VEHICLE_VELOCITY_TO_GO_FORWARD &&
                                 v1 >= SAFE_VEHICLE_VELOCITY_TO_GO_FORWARD;

        const bool reverseSafe = v0 <= SAFE_VEHICLE_VELOCITY_TO_GO_REVERSE &&
                                 v1 <= SAFE_VEHICLE_VELOCITY_TO_GO_REVERSE;

        // Handle Driver Inputs
        DriveLogic::Inputs in;
        in.direction = decodeDirection(driverInputs);

        in.accelPercent = static_cast<float>(PedalsInputTask::Inst().GetAccelPercentage());
        in.regenPercent = static_cast<float>(PedalsInputTask::Inst().GetRegenPercentage());
        
        in.mechBrake = isPressed(driverInputs, BIT_MECH_BRAKE);
        in.fwdSafe = forwardSafe;
        in.revSafe = reverseSafe;
        
        in.allowCharge = allowCharge;
        in.allowDischarge = allowDischarge;

        const bool resetPressed = isPressed(driverInputs, BIT_RESET);
        const bool resetRequested = resetPressed && !prevResetPressed;
        prevResetPressed = resetPressed;

        DriveLogic::Output out = DriveLogic::DriveLogic(state, in, cfg);
        if (resetRequested){
            out = DriveLogic::Output(); // command zero before resetting
            state.mode = DriveLogic::Mode::Off;
        }

        // Motor Drive: velocity [rpm], current [percent of max 0.0f...1.0f]
        sendMotor(motor_drive_msg, MOTOR_DRIVE_INPUT, out.rpm, clamp(out.current));

        // Motor Power: reserved, bus current
        sendMotor(motor_power_msg, MOTOR_POWER_INPUT, 0.0f, clamp(BUS_CURRENT_OUT));

        if (resetRequested){
            CUBE_PRINT("Resetting Motors\n");
            CANTxTask::Inst().SendCommand(Command(TASK_SPECIFIC_COMMAND, MOTOR_RESET_INPUT));
        }

    }       
}