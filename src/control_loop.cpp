#include "fccu/control_loop.hpp"

#include <algorithm>
#include <cmath>

#include "fccu/actuators.hpp"
#include "fccu/fuel_cell.hpp"

namespace fccu {

namespace fc = fuel_cell;

double Pid::update(double error, double dt) {
    double d = has_prev_ ? (error - prev_error_) / dt : 0.0;
    prev_error_ = error;
    has_prev_ = true;

    integral_ += error * dt;
    double out = kp_ * error + ki_ * integral_ + kd_ * d;
    if (out > out_max_) {
        // anti-windup: freeze the integral at the saturation boundary
        if (ki_ != 0.0) integral_ = (out_max_ - kp_ * error - kd_ * d) / ki_;
        return out_max_;
    }
    if (out < out_min_) {
        if (ki_ != 0.0) integral_ = (out_min_ - kp_ * error - kd_ * d) / ki_;
        return out_min_;
    }
    return out;
}

void ControlLoop::reset() {
    pressure_pid_.reset();
    cooling_pid_.reset();
    current_setpoint_ = 0.0;
}

double ControlLoop::slew_current(double target, double dt) {
    double step = CURRENT_SLEW * dt;
    current_setpoint_ += std::clamp(target - current_setpoint_, -step, step);
    return current_setpoint_;
}

double ControlLoop::compressor_for(double current_a) {
    double o2_needed = fc::o2_consumption(current_a) * LAMBDA_AIR;
    return std::max(o2_needed / AirCompressor::MAX_O2_FLOW, COMPRESSOR_IDLE);
}

// a NaN reading (disconnected sensor) must never reach a command output;
// fall back to the setpoint until the safety monitor trips
static double safe(double value, double fallback) {
    return std::isnan(value) ? fallback : value;
}

Command ControlLoop::update(State state, const Readings& readings,
                            double demand_pct, double dt) {
    double p_anode = safe(readings.anode_pressure, ANODE_TARGET_BAR);
    double t_cool = safe(readings.coolant_temp, COOLANT_TARGET_C);
    // hydrogen admission fails CLOSED on a dead pressure sensor: coasting on
    // the setpoint would mean open-loop H2 flow until the safety monitor
    // trips. The PI integrator freezes for the outage.
    bool anode_ok = !std::isnan(readings.anode_pressure);

    switch (state) {
    case State::off:
    case State::fault:
        reset();
        return {.cooling = state == State::fault ? 1.0 : 0.0};

    case State::purge:
        return {.h2_valve = PURGE_VALVE_OPEN, .compressor = COMPRESSOR_IDLE,
                .recirc_pump = 1.0, .purge_open = true};

    case State::pressurize:
        return {.h2_valve = anode_ok
                    ? pressure_pid_.update(ANODE_TARGET_BAR - p_anode, dt) : 0.0,
                .compressor = COMPRESSOR_IDLE, .recirc_pump = 1.0};

    case State::warmup: {
        double current = slew_current(WARMUP_CURRENT, dt);
        return {.h2_valve = anode_ok
                    ? pressure_pid_.update(ANODE_TARGET_BAR - p_anode, dt) : 0.0,
                .compressor = compressor_for(current), .recirc_pump = 1.0,
                .current_request = current};
    }

    case State::running: {
        double target = std::clamp(demand_pct, 0.0, 100.0) / 100.0 * fc::I_RATED;
        double current = slew_current(target, dt);
        return {.h2_valve = anode_ok
                    ? pressure_pid_.update(ANODE_TARGET_BAR - p_anode, dt) : 0.0,
                .compressor = compressor_for(current), .recirc_pump = 1.0,
                .cooling = cooling_pid_.update(t_cool - COOLANT_TARGET_C, dt),
                .current_request = current};
    }

    case State::shutdown:
        // load off, H2 closed, vent the anode, keep cooling
        current_setpoint_ = 0.0;
        return {.cooling = cooling_pid_.update(t_cool - COOLANT_TARGET_C, dt),
                .purge_open = true};
    }
    return {};
}

} // namespace fccu
