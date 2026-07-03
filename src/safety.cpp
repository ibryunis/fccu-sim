#include "fccu/safety.hpp"

#include <cmath>
#include <format>

#include "fccu/sensors.hpp"

namespace fccu {

std::string FaultRecord::describe() const {
    return std::format("FAULT: {}: {:.1f} vs limit {:.1f} at t={:.1f}s",
                       reason, value, limit, time_s);
}

SafetyMonitor::SafetyMonitor()
    : defs_{{
          {"tank overpressure", TANK_MAX_BAR, PERSIST_FAST},
          {"anode overpressure", ANODE_MAX_BAR, PERSIST_FAST},
          {"stack overcurrent", CURRENT_MAX_A, PERSIST_FAST},
          {"stack undervoltage under load", STACK_MIN_V, PERSIST_FAST},
          {"coolant overtemp", COOLANT_MAX_C, PERSIST_TEMP},
          {"tank overtemp", TANK_TEMP_MAX_C, PERSIST_TEMP},
          {"sensor implausible: tank_pressure", 420.0, PERSIST_FAST},
          {"sensor implausible: tank_temp", 85.0, PERSIST_FAST},
          {"sensor implausible: stack_voltage", 120.0, PERSIST_FAST},
          {"sensor implausible: stack_current", 400.0, PERSIST_FAST},
          {"sensor implausible: coolant_temp", 120.0, PERSIST_FAST},
          {"sensor implausible: anode_pressure", 4.0, PERSIST_FAST},
          {"sensor implausible: h2_flow", 0.5, PERSIST_FAST},
      }} {}

// NaN comparisons are false, so a dead sensor cannot trip a hard limit;
// the plausibility checks own NaN.
static bool over(double v, double limit) {
    return !std::isnan(v) && v > limit;
}

std::array<SafetyMonitor::Evaluation, SafetyMonitor::CHECK_COUNT>
SafetyMonitor::evaluate(const Readings& r) const {
    std::array<Evaluation, CHECK_COUNT> out;
    out[0] = {over(r.tank_pressure, TANK_MAX_BAR), r.tank_pressure};
    out[1] = {over(r.anode_pressure, ANODE_MAX_BAR), r.anode_pressure};
    out[2] = {over(r.stack_current, CURRENT_MAX_A), r.stack_current};

    bool undervolt = !std::isnan(r.stack_voltage) && !std::isnan(r.stack_current)
                  && r.stack_current > UNDERVOLT_MIN_A
                  && r.stack_voltage < STACK_MIN_V;
    out[3] = {undervolt, r.stack_voltage};

    out[4] = {over(r.coolant_temp, COOLANT_MAX_C), r.coolant_temp};
    out[5] = {over(r.tank_temp, TANK_TEMP_MAX_C), r.tank_temp};

    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        double v = r.*READING_FIELDS[i];
        const SensorConfig& cfg = SENSOR_CONFIGS[i];
        // tolerance beyond the physical range: readings a hair past the rail
        // are sensor noise, not a broken wire
        double tol = 0.02 * (cfg.range_hi - cfg.range_lo);
        bool bad = std::isnan(v) || v < cfg.range_lo - tol || v > cfg.range_hi + tol;
        out[6 + i] = {bad, v};
    }
    return out;
}

bool SafetyMonitor::update(double t, double dt, const Readings& r) {
    auto evals = evaluate(r);
    for (std::size_t i = 0; i < CHECK_COUNT; ++i) {
        double before = accum_[i];
        if (evals[i].violated) {
            accum_[i] = std::min(before + dt, defs_[i].persistence_s);
        } else {
            // leak instead of zeroing: a marginal fault dithering across its
            // limit through sensor noise still trips (>75% violated samples);
            // the cap makes the trip fire exactly once per violation episode
            accum_[i] = std::max(before - LEAK_RATE * dt, 0.0);
        }
        if (before < defs_[i].persistence_s && accum_[i] >= defs_[i].persistence_s) {
            trip(defs_[i].name, evals[i].value, defs_[i].limit, t);
        }
    }
    update_stuck(t, dt, r);
    return latched_;
}

void SafetyMonitor::update_stuck(double t, double dt, const Readings& r) {
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        double v = r.*READING_FIELDS[i];
        const SensorConfig& cfg = SENSOR_CONFIGS[i];
        bool frozen = !std::isnan(v) && has_prev_ && v == stuck_prev_[i]
                   && v > cfg.range_lo && v < cfg.range_hi;
        double before = stuck_accum_[i];
        stuck_accum_[i] = frozen ? std::min(before + dt, PERSIST_STUCK)
                                 : std::max(before - LEAK_RATE * dt, 0.0);
        if (before < PERSIST_STUCK && stuck_accum_[i] >= PERSIST_STUCK) {
            trip(std::format("sensor stuck: {}", SENSOR_NAMES[i]), v, v, t);
        }
        if (!std::isnan(v)) stuck_prev_[i] = v;
    }
    has_prev_ = true;
}

void SafetyMonitor::trip(std::string reason, double value, double limit, double t) {
    FaultRecord record{std::move(reason), value, limit, t};
    history_.push_back(record);
    if (!latched_) {
        latched_ = true;
        fault_ = std::move(record);
    }
}

bool SafetyMonitor::any_violation(const Readings& r) const {
    for (const Evaluation& e : evaluate(r)) {
        if (e.violated) return true;
    }
    return false;
}

bool SafetyMonitor::reset(const Readings& r) {
    // persistence-symmetric: one lucky low-noise sample must not re-arm the
    // system - every accumulator has to drain (leak) to zero first
    if (any_violation(r)) return false;
    for (double a : accum_) {
        if (a > 0.0) return false;
    }
    for (double a : stuck_accum_) {
        if (a > 0.0) return false;
    }
    latched_ = false;
    fault_.reset();
    return true;
}

} // namespace fccu
