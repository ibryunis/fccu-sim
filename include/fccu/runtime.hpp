// Simulation runtime: 100 Hz tick wiring plant, sensors, safety, FSM and
// control together, plus the thread-safe command/injection API the UI uses.
//
// Tick order (matches what real FCCU firmware does):
//   1. sample sensors            - the controller only ever sees these
//   2. safety monitor            - may latch, on sensor values not truth
//   3. state machine             - latch forces FAULT the same tick
//   4. control law               - commands consistent with the new state
//   5. actuate + integrate plant
//   6. log
//
// Threading: the sim runs in a std::jthread ticking on absolute deadlines
// (sleep_until, so scheduling jitter never accumulates as drift). One mutex
// guards all state. Public API methods lock, copy or mutate, and return -
// the lock is never held across I/O. UI threads get a Snapshot by value.
#pragma once

#include <atomic>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "fccu/control_loop.hpp"
#include "fccu/datalog.hpp"
#include "fccu/plant.hpp"
#include "fccu/safety.hpp"
#include "fccu/sensors.hpp"
#include "fccu/state_machine.hpp"
#include "fccu/types.hpp"

namespace fccu {

inline constexpr double TICK_DT = 0.01;    // 100 Hz
inline constexpr int LOG_DECIMATION = 10;  // log at 10 Hz

// value-type copy of everything the UI shows; crosses the thread boundary
struct Snapshot {
    double t = 0.0;
    State state = State::off;
    bool latched = false;
    std::optional<FaultRecord> fault;
    std::vector<FaultRecord> fault_history;  // last 5; described outside the lock
    long overruns = 0;                       // ticks where the loop fell behind
    double demand_pct = 0.0;
    double current_setpoint = 0.0;
    double power_kw = 0.0;
    Readings readings;
    double h2_valve = 0.0, compressor = 0.0, recirc_pump = 0.0, cooling = 0.0;
    bool cooling_blocked = false;
    std::array<FailureMode, SENSOR_COUNT> failures{};
};

enum class InjectKind {
    pressure_spike, overheat, clear_overheat, sensor_fail, sensor_repair
};

class Simulation {
public:
    // pass a directory to enable CSV logging, nullopt to disable (tests)
    explicit Simulation(std::optional<std::filesystem::path> log_dir = {});
    ~Simulation();  // stops the thread if running (jthread joins)

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    // --- real-time thread ---------------------------------------------
    void start_thread();

    // --- thread-safe UI / command API -----------------------------------
    enum class CommandResult { ok, refused, unknown };

    void set_demand(double pct);  // non-finite values are rejected
    // "start" | "stop" | "reset"; reset is refused while a cause persists
    CommandResult command(std::string_view name);
    void inject(InjectKind kind, SensorId sensor = SensorId::tank_pressure,
                FailureMode mode = FailureMode::none);
    Snapshot snapshot() const;  // locks, copies, unlocks - no I/O under lock

    // --- single-threaded use (tests) -------------------------------------
    void step();
    void run_for(double seconds);

    const Plant& plant() const { return plant_; }
    Plant& plant() { return plant_; }
    const SafetyMonitor& safety() const { return safety_; }
    State state() const { return fsm_.state(); }
    const Readings& readings() const { return readings_; }
    const Command& last_command() const { return last_command_; }
    const std::filesystem::path& log_path() const { return log_path_; }

private:
    std::string step_locked();  // returns the CSV row to write, or empty
    std::string log_row() const;

    Plant plant_;
    SensorSuite sensors_;
    SafetyMonitor safety_;
    StateMachine fsm_;
    ControlLoop control_;

    double t_ = 0.0;
    long tick_ = 0;
    double demand_pct_ = 0.0;
    Readings readings_;
    Command last_command_;

    std::optional<CsvLogger> logger_;
    std::filesystem::path log_path_;
    std::atomic<long> overruns_{0};

    mutable std::mutex mutex_;
    std::jthread thread_;  // last member: joins before the rest is destroyed
};

} // namespace fccu
