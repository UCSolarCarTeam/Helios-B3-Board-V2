#pragma once

#include <stdint.h>

enum class Direction : uint8_t {
    Neutral,
    Forward,
    Reverse,
    Error
};

enum class Gear : uint8_t {
    Off,            // Off
    Accelerating,   // Forward or Reverse
    RegenBraking,   // Regen Brake
    MechBraking     // Mechanical Brake
};

struct Config {
    float accPedalDeadzone; // Deadzone for Acceleration Pedal
    float regPedalDeadzone; // Deadzone for Regen Pedal

    float pedalMax;     // ADC Max

    float regenRPM;     // Regen RPM, should be 0
    float maxFwdRPM;    // Max Forward RPM
    float maxRevRPM;    // Max Reverse RPM

    float regenScaling; // Scaling Factor for Regen Braking
    float accelScaling; // Scaling Factor for Acceleration
};

struct Inputs {
    Direction direction;

    float accelPercent; // 0..100
    float regenPercent; // 0..100

    bool mechBrake;
    bool fwdSafe; // velocity allows going forward
    bool revSafe; // velocity allows going reverse

    bool allowCharge;
    bool allowDischarge;
};

struct State {
    Gear gear = Gear::Off;
};

struct Output {
    float rpm = 0.0f;       // target velocity (Usually unreachable, or 0)
    float current = 0.0f    // target current
};

Output DriveLogic(State& state, const Inputs& in, const Config& cfg);