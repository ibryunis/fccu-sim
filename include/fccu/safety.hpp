// Safety monitor, modelled on the Formula Student shutdown circuit (SDC).
//
// Every check has a persistence window: the violation must hold continuously
// for the whole window before it trips. Windows follow the FS EV 5.8.7 split:
// 500 ms for pressure / voltage / current faults, 1000 ms for temperature.
//
// On trip the monitor latches: the FCCU is forced to FAULT, hydrogen is cut,
// and the latch survives until a manual reset - which is only accepted once
// every condition has cleared. De-energized is the safe state; a human must
// re-arm the system.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "fccu/types.hpp"

namespace fccu {

inline constexpr double PERSIST_FAST = 0.5;  // s: pressure, voltage, current
inline constexpr double PERSIST_TEMP = 1.0;  // s: temperature

struct FaultRecord {
    std::string reason;
    double value = 0.0;
    double limit = 0.0;
    double time_s = 0.0;

    std::string describe() const;
};

class SafetyMonitor {
public:
    static constexpr double TANK_MAX_BAR = 400.0;
    static constexpr double ANODE_MAX_BAR = 3.0;
    static constexpr double CURRENT_MAX_A = 360.0;
    static constexpr double STACK_MIN_V = 38.4;   // 0.40 V/cell * 96 cells
    static constexpr double UNDERVOLT_MIN_A = 20.0;
    static constexpr double COOLANT_MAX_C = 80.0;
    static constexpr double TANK_TEMP_MAX_C = 60.0;

    // 6 hard limits + one plausibility check per sensor
    static constexpr std::size_t CHECK_COUNT = 6 + SENSOR_COUNT;

    SafetyMonitor();

    // returns true when latched; call once per tick with sensor readings
    bool update(double t, double dt, const Readings& r);

    // external trip (e.g. FSM startup timeout); latches immediately
    void trip(std::string reason, double value, double limit, double t);

    bool any_violation(const Readings& r) const;

    // manual reset; refused while any condition is still violated
    bool reset(const Readings& r);

    bool latched() const { return latched_; }
    const std::optional<FaultRecord>& fault() const { return fault_; }
    const std::vector<FaultRecord>& history() const { return history_; }

private:
    struct CheckDef {
        const char* name;
        double limit;
        double persistence_s;
    };
    struct Evaluation {
        bool violated;
        double value;
    };

    std::array<Evaluation, CHECK_COUNT> evaluate(const Readings& r) const;

    std::array<CheckDef, CHECK_COUNT> defs_;
    std::array<double, CHECK_COUNT> accum_{};
    bool latched_ = false;
    std::optional<FaultRecord> fault_;   // primary (first) fault
    std::vector<FaultRecord> history_;
};

} // namespace fccu
