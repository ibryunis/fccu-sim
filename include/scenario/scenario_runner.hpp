// Randomized acceptance-test scenario, executed against the live simulation
// through the same public command/inject API the operator uses - failures
// are therefore always reproducible by hand from the report.
//
// This is test tooling that ships in the sim binary, not FCCU firmware, so
// it lives outside fccu/. Dependency arrow: runner -> Simulation, never back
// (runtime.hpp only forward-declares this class).
//
// Script (all timing counted in ticks, so verdicts are host-independent):
//   1. START, expect RUNNING within 60 s
//   2. three random demand steps (20-90%), each held 4 s; after the slew
//      settles, delivered current must track the setpoint within 8%
//   3. one random fault injection; expect the SDC latch + FAULT within the
//      fault's persistence window plus a physics-dependent margin
//   4. reset attempted immediately: must be REFUSED (accumulators not drained)
//   5. cause cleared; reset must then be accepted
//   6. START again, expect RUNNING - system survives a full fault cycle
#pragma once

#include <cstdint>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "fccu/types.hpp"

namespace fccu {

class Simulation;

class ScenarioRunner {
public:
    struct Line {
        double t = 0.0;      // sim time
        bool pass = true;
        std::string text;
    };
    struct Report {
        bool running = false;
        bool done = false;
        bool pass = false;
        std::uint32_t seed = 0;
        std::vector<Line> lines;
    };

    explicit ScenarioRunner(Simulation& sim) : sim_(sim) {}

    bool start(std::uint32_t seed);  // false while a run is active
    void advance();                  // once per tick, sim thread, no sim lock held
    Report report() const;

private:
    enum class Phase {
        idle, wait_running, demand_hold, inject, wait_fault,
        reset_refused, clear_cause, wait_reset, restart, done
    };

    void log(bool pass, std::string text);
    void fail_and_finish(std::string text);
    void enter(Phase p, long deadline_ticks);

    Simulation& sim_;
    mutable std::mutex mutex_;
    Report report_;

    Phase phase_ = Phase::idle;
    std::mt19937 rng_;
    long ticks_in_phase_ = 0;
    long deadline_ = 0;
    int demand_step_ = 0;
    double demand_target_ = 0.0;
    int fault_kind_ = 0;             // 0 overheat, 1 spike, 2 disconnect, 3 stuck
    SensorId fault_sensor_ = SensorId::coolant_temp;
};

} // namespace fccu
