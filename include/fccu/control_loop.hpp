// FCCU control: maps state + sensor readings + driver demand to a Command.
//
// Loops:
//   anode pressure -> PI on H2 valve (integrator removes consumption droop)
//   air supply     -> feedforward from current setpoint at lambda_air = 2.0
//                     (commands the compressor ahead of the load, hiding its lag)
//   coolant temp   -> PI on cooling PWM, target 65 degC
//   current        -> demand% mapped to amps, slew-limited so the load can
//                     never outrun reactant supply (starvation protection)
#pragma once

#include "fccu/state_machine.hpp"
#include "fccu/types.hpp"

namespace fccu {

class Pid {
public:
    Pid(double kp, double ki, double kd = 0.0,
        double out_min = 0.0, double out_max = 1.0)
        : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

    void reset() {
        integral_ = 0.0;
        has_prev_ = false;
    }

    double update(double error, double dt);
    double integral() const { return integral_; }

private:
    double kp_, ki_, kd_;
    double out_min_, out_max_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool has_prev_ = false;
};

class ControlLoop {
public:
    static constexpr double ANODE_TARGET_BAR = 2.0;
    static constexpr double COOLANT_TARGET_C = 65.0;
    static constexpr double LAMBDA_AIR = 2.0;
    static constexpr double COMPRESSOR_IDLE = 0.1;
    static constexpr double CURRENT_SLEW = 150.0;   // A/s
    static constexpr double WARMUP_CURRENT = 80.0;  // A
    static constexpr double PURGE_VALVE_OPEN = 0.2;

    Command update(State state, const Readings& readings, double demand_pct,
                   double dt);
    void reset();

    double current_setpoint() const { return current_setpoint_; }

private:
    double slew_current(double target, double dt);
    static double compressor_for(double current_a);

    Pid pressure_pid_{1.0, 2.0};
    Pid cooling_pid_{0.15, 0.02};
    double current_setpoint_ = 0.0;
};

} // namespace fccu
