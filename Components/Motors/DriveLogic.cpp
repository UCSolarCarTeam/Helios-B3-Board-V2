#include "DriveLogic.hpp"

namespace DriveLogic {
    namespace {
        Gear selectGear(const Inputs& in, const Config cfg){
            if (in.regenPercent > cfg.regPedalDeadzone) {
                return Gear::RegenBraking;
            }
            if (in.mechBrake) {
                return Gear::MechBraking;
            }
            if (in.accelPercent > cfg.accPedalDeadzone) {
                return Gear::Accelerating;
            }

            return Gear::Off;
        }

        // returns True if switching from Accel to Regen, and vice versa
        bool isSwitching(Gear from, Gear to){
            return  (from == Mode::Accelerating && to == Mode::RegenBraking) || 
                    (from == Mode::RegenBraking && to == Mode::Accelerating);
        }

        // clamps value to 0.0f <= a <= 1.0f
        float clamp(float a){
            if (!(a > 0.0f)){ // also catches NaN
                return 0.0f; 
            } 

            if (a > 1.0f){ 
                return 1.0f; 
            }

            return a;
        }

        // Returns a percent of the max motor current
        float pedal2Current(float pedalPercent, Gear gear, float scalingFactor, const Config& cfg){
            if (gear == Gear::Accelerating) {
                float deadzone = cfg.accPedalDeadzone;

            } else if (gear == Gear::RegenBraking) {
                float deadzone = cfg.accPedalDeadzone;
            }

            if (pedalPercent <= deadzone) {
                return 0.0f;
            }

            const float diff = cfg.pedalMax - deadzone;
            if (diff <= 0.0f) { // unsafe config
                return 0.0f;
            }

            return clamp((pedalPercent - deadzone) / diff * scalingFactor)
        }
    } //

    Output DriveLogic(State& state, const Inputs& in, const Config& cfg){

        Output out; // default: 0 velocity, 0 current

        // Only one direction, and vehicle must be moving slow enough to allow it
        const bool fwd = (in.direction == Direction::Forward) && in.fwdSafe;
        const bool rev = (in.direction == Direction::Reverse) && in.revSafe;

        if(!fwd && !rev){
            state.gear = Gear::Off
            return out;
        }

        Gear targetGear = selectGear(in, cfg);

        // Drive requested, but check with AuxBMS to allowDischarge
        if (targetGear == Gear::Accelerating && !in.allowDischarge){
            targetGear = Gear::Off;
        }

        // Add a zero-torque cycle when switching between accel and regen
        const bool switching = isSwitching(state.gear, targetGear);
        if (switching) {
            return out;
        }

        switch (targetGear) {
            case Gear::RegenBraking:
                out.rpm = cfg.regenRPM; // should be 0
                out.current = in.allowCharge ? pedal2Current(in.regenPercent, targetGear, cfg.regenScaling, cfg)
                break;
            
            case Gear::Accelerating:
                // Set an unreachable velocity target, then adjust current according to pedal
                out.rpm = forward ? cfg.maxfwdRPM : cfg.maxrevRPM; 
                out.current = pedal2Current(in.accelPercent, targetGear, cfg.accelScaling, cfg);
                break;
            
            case Gear::MechanicalBraking:
            case Gear::Off:
                break; // Default case return 0
        }
        return out;
    }

}   // End namespace DriveLogic