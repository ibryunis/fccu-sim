// Sensors: gaussian noise on the true value, plus injectable failures.
//
// Failure modes:
//   stuck        - keeps returning the last healthy reading
//   disconnected - returns NaN (open circuit / broken wire)
//   out_of_range - returns a value far above the physical range
//
// SensorSuite maps SensorId -> Readings field through the constexpr
// pointer-to-member table in types.hpp: no string keys, no heap.
#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <random>

#include "fccu/types.hpp"

namespace fccu {

enum class FailureMode { none, stuck, disconnected, out_of_range };

struct SensorConfig {
    double noise_sigma;
    double range_lo;
    double range_hi;
};

inline constexpr std::array<SensorConfig, SENSOR_COUNT> SENSOR_CONFIGS = {{
    {0.5, 0.0, 420.0},      // tank_pressure, bar
    {0.2, -40.0, 85.0},     // tank_temp, degC
    {0.15, 0.0, 120.0},     // stack_voltage, V
    {0.5, 0.0, 400.0},      // stack_current, A
    {0.15, -40.0, 120.0},   // coolant_temp, degC
    {0.01, 0.0, 4.0},       // anode_pressure, bar
    {0.002, 0.0, 0.5},      // h2_flow, mol/s
}};

class Sensor {
public:
    explicit Sensor(const SensorConfig& cfg) : cfg_(cfg) {}

    void fail(FailureMode mode) { mode_ = mode; }
    void repair() { mode_ = FailureMode::none; }
    FailureMode failure() const { return mode_; }

    double read(double true_value, std::mt19937& rng) {
        switch (mode_) {
        case FailureMode::stuck:
            return last_good_;
        case FailureMode::disconnected:
            return std::numeric_limits<double>::quiet_NaN();
        case FailureMode::out_of_range:
            return cfg_.range_hi * 1.5;
        case FailureMode::none:
            break;
        }
        std::normal_distribution<double> noise(0.0, cfg_.noise_sigma);
        // a real transducer rails at its range; genuine over-range physics
        // must trip the hard limits, not the wiring-fault plausibility check
        last_good_ = std::clamp(true_value + noise(rng),
                                cfg_.range_lo, cfg_.range_hi);
        return last_good_;
    }

private:
    SensorConfig cfg_;
    FailureMode mode_ = FailureMode::none;
    double last_good_ = 0.0;
};

class SensorSuite {
public:
    SensorSuite()
        : sensors_{Sensor(SENSOR_CONFIGS[0]), Sensor(SENSOR_CONFIGS[1]),
                   Sensor(SENSOR_CONFIGS[2]), Sensor(SENSOR_CONFIGS[3]),
                   Sensor(SENSOR_CONFIGS[4]), Sensor(SENSOR_CONFIGS[5]),
                   Sensor(SENSOR_CONFIGS[6])},
          rng_(std::random_device{}()) {}

    Readings sample(const Readings& truth) {
        Readings out;
        for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
            out.*READING_FIELDS[i] = sensors_[i].read(truth.*READING_FIELDS[i], rng_);
        }
        return out;
    }

    void fail(SensorId id, FailureMode mode) {
        sensors_[static_cast<std::size_t>(id)].fail(mode);
    }
    void repair(SensorId id) { sensors_[static_cast<std::size_t>(id)].repair(); }
    FailureMode failure(SensorId id) const {
        return sensors_[static_cast<std::size_t>(id)].failure();
    }

private:
    std::array<Sensor, SENSOR_COUNT> sensors_;
    std::mt19937 rng_;
};

} // namespace fccu
