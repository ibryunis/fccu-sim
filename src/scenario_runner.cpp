#include "scenario/scenario_runner.hpp"

#include <format>

#include "fccu/fuel_cell.hpp"
#include "fccu/runtime.hpp"

namespace fccu {

namespace {
constexpr long TICKS_PER_S = 100;
// sensors safe for stuck injection: mid-range while RUNNING (h2_flow sits at
// its zero rail at idle - the documented stuck-at-rail blind spot)
constexpr SensorId STUCKABLE[] = {SensorId::tank_pressure, SensorId::stack_voltage,
                                  SensorId::stack_current, SensorId::coolant_temp,
                                  SensorId::anode_pressure};
constexpr const char* FAULT_NAMES[] = {"overheat (cooling blocked)",
                                       "tank pressure spike",
                                       "sensor disconnect", "sensor stuck"};
} // namespace

bool ScenarioRunner::start(std::uint32_t seed) {
    std::scoped_lock lock(mutex_);
    if (report_.running) return false;
    report_ = Report{.running = true, .seed = seed};
    rng_.seed(seed);
    phase_ = Phase::idle;
    demand_step_ = 0;
    return true;
}

void ScenarioRunner::log(bool pass, std::string text) {
    std::scoped_lock lock(mutex_);
    if (report_.lines.size() < 32) {
        report_.lines.push_back({sim_.snapshot().t, pass, std::move(text)});
    }
}

void ScenarioRunner::fail_and_finish(std::string text) {
    log(false, std::move(text));
    std::scoped_lock lock(mutex_);
    report_.running = false;
    report_.done = true;
    report_.pass = false;
    phase_ = Phase::done;
    sim_.unlock_ui();
}

void ScenarioRunner::enter(Phase p, long deadline_ticks) {
    phase_ = p;
    ticks_in_phase_ = 0;
    deadline_ = deadline_ticks;
}

void ScenarioRunner::advance() {
    {
        std::scoped_lock lock(mutex_);
        if (!report_.running) return;
    }
    ++ticks_in_phase_;
    Snapshot s = sim_.snapshot();

    switch (phase_) {
    case Phase::idle:
        sim_.command("start", Simulation::Auth::autotest);
        log(true, std::format("seed {}: START issued", report_.seed));
        enter(Phase::wait_running, 60 * TICKS_PER_S);
        break;

    case Phase::wait_running:
        if (s.state == State::running) {
            log(true, std::format("RUNNING reached in {} ticks", ticks_in_phase_));
            demand_target_ = std::uniform_real_distribution(20.0, 90.0)(rng_);
            sim_.set_demand(demand_target_, Simulation::Auth::autotest);
            enter(Phase::demand_hold, 4 * TICKS_PER_S);
        } else if (ticks_in_phase_ > deadline_) {
            fail_and_finish("startup timeout: never reached RUNNING");
        }
        break;

    case Phase::demand_hold:
        if (ticks_in_phase_ >= deadline_) {
            // settle criterion: 4 s hold covers worst-case slew (330 A at
            // 150 A/s = 2.2 s) plus 1.8 s of settled samples
            double setpoint = s.current_setpoint;
            double delivered = s.readings.stack_current;
            bool ok = std::abs(delivered - setpoint) <= 0.08 * setpoint + 2.0;
            log(ok, std::format("demand {:.0f}%: setpoint {:.0f} A, delivered "
                                "{:.0f} A", demand_target_, setpoint, delivered));
            if (!ok) {
                fail_and_finish("demand tracking out of tolerance");
                break;
            }
            if (++demand_step_ < 3) {
                demand_target_ = std::uniform_real_distribution(20.0, 90.0)(rng_);
                sim_.set_demand(demand_target_, Simulation::Auth::autotest);
                enter(Phase::demand_hold, 4 * TICKS_PER_S);
            } else {
                enter(Phase::inject, 0);
            }
        }
        break;

    case Phase::inject: {
        fault_kind_ = std::uniform_int_distribution(0, 3)(rng_);
        long window;
        switch (fault_kind_) {
        case 0:
            // guarantee heat build-up regardless of the random demand history
            sim_.set_demand(85.0, Simulation::Auth::autotest);
            sim_.inject(InjectKind::overheat, SensorId::tank_pressure,
                        FailureMode::none, Simulation::Auth::autotest);
            window = 120 * TICKS_PER_S;  // physics: climb to 80 C, then 1 s
            break;
        case 1:
            sim_.inject(InjectKind::pressure_spike, SensorId::tank_pressure,
                        FailureMode::none, Simulation::Auth::autotest);
            window = 3 * TICKS_PER_S;    // 500 ms persistence + margin
            break;
        case 2:
            fault_sensor_ = static_cast<SensorId>(
                std::uniform_int_distribution<std::size_t>(0, SENSOR_COUNT - 1)(rng_));
            sim_.inject(InjectKind::sensor_fail, fault_sensor_,
                        FailureMode::disconnected, Simulation::Auth::autotest);
            window = 3 * TICKS_PER_S;
            break;
        default:
            fault_sensor_ = STUCKABLE[std::uniform_int_distribution<std::size_t>(
                0, std::size(STUCKABLE) - 1)(rng_)];
            sim_.inject(InjectKind::sensor_fail, fault_sensor_,
                        FailureMode::stuck, Simulation::Auth::autotest);
            window = 4 * TICKS_PER_S;    // 1 s stuck persistence + margin
            break;
        }
        log(true, std::format("injecting fault: {}", FAULT_NAMES[fault_kind_]));
        enter(Phase::wait_fault, window);
        break;
    }

    case Phase::wait_fault:
        if (s.state == State::fault && s.latched) {
            log(true, std::format("latched in {} ticks: {}", ticks_in_phase_,
                                  s.fault ? s.fault->describe() : "?"));
            enter(Phase::reset_refused, 0);
        } else if (ticks_in_phase_ > deadline_) {
            fail_and_finish("SDC did not latch within the expected window");
        }
        break;

    case Phase::reset_refused: {
        // cause still present: reset must be refused
        auto r = sim_.command("reset", Simulation::Auth::autotest);
        bool ok = r == Simulation::CommandResult::refused;
        log(ok, ok ? "reset correctly refused while cause persists"
                   : "reset was accepted while cause still present");
        if (!ok) {
            fail_and_finish("latch integrity violated");
            break;
        }
        // clear the cause through the same API
        switch (fault_kind_) {
        case 0:
            sim_.inject(InjectKind::clear_overheat, SensorId::tank_pressure,
                        FailureMode::none, Simulation::Auth::autotest);
            break;
        case 1:
            break;  // tank cools toward ambient on its own
        default:
            sim_.inject(InjectKind::sensor_repair, fault_sensor_,
                        FailureMode::none, Simulation::Auth::autotest);
            break;
        }
        enter(Phase::clear_cause, 200 * TICKS_PER_S);
        break;
    }

    case Phase::clear_cause:
        // wait until a reset is accepted (condition gone + accumulators drained)
        if (ticks_in_phase_ % (TICKS_PER_S / 2) == 0) {
            if (sim_.command("reset", Simulation::Auth::autotest)
                == Simulation::CommandResult::ok) {
                log(true, std::format("reset accepted after {} ticks",
                                      ticks_in_phase_));
                enter(Phase::wait_reset, 2 * TICKS_PER_S);
                break;
            }
        }
        if (ticks_in_phase_ > deadline_) {
            fail_and_finish("reset never accepted after cause cleared");
        }
        break;

    case Phase::wait_reset:
        if (s.state == State::off) {
            sim_.command("start", Simulation::Auth::autotest);
            log(true, "restart issued");
            enter(Phase::restart, 60 * TICKS_PER_S);
        } else if (ticks_in_phase_ > deadline_) {
            fail_and_finish("FSM did not return to OFF after reset");
        }
        break;

    case Phase::restart:
        if (s.state == State::running) {
            log(true, "RUNNING reached after fault cycle: PASS");
            {
                std::scoped_lock lock(mutex_);
                report_.running = false;
                report_.done = true;
                report_.pass = true;
            }
            phase_ = Phase::done;
            sim_.set_demand(0.0, Simulation::Auth::autotest);
            sim_.unlock_ui();
        } else if (ticks_in_phase_ > deadline_) {
            fail_and_finish("restart never reached RUNNING");
        }
        break;

    case Phase::done:
        break;
    }
}

ScenarioRunner::Report ScenarioRunner::report() const {
    std::scoped_lock lock(mutex_);
    return report_;
}

} // namespace fccu
