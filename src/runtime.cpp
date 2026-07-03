#include "fccu/runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>

#include "fccu/fuel_cell.hpp"

namespace fccu {

namespace fc = fuel_cell;

static std::string timestamp_name() {
    auto now = std::chrono::system_clock::now();
    return std::format("run_{:%Y%m%d_%H%M%S}.csv",
                       std::chrono::floor<std::chrono::seconds>(now));
}

Simulation::Simulation(std::optional<std::filesystem::path> log_dir) {
    if (log_dir) {
        log_path_ = *log_dir / timestamp_name();
        logger_.emplace(log_path_,
                        "t,state,demand_pct,current_setpoint,latched,"
                        "tank_pressure,tank_temp,stack_voltage,stack_current,"
                        "coolant_temp,anode_pressure,h2_flow,"
                        "h2_valve,compressor,recirc_pump,cooling");
    }
}

Simulation::~Simulation() {
    if (thread_.joinable()) thread_.request_stop();
    // jthread destructor joins
}

std::string Simulation::step_locked() {
    t_ += TICK_DT;
    ++tick_;

    readings_ = sensors_.sample(plant_.truth());

    safety_.update(t_, TICK_DT, readings_);
    fsm_.update(TICK_DT, readings_.anode_pressure, readings_.coolant_temp,
                safety_.latched());
    if (fsm_.state() == State::purge && prev_state_ != State::purge) {
        plant_.reset_energy_counters();  // new run starts at the purge edge
    }
    prev_state_ = fsm_.state();
    if (fsm_.timeout_reason() && !safety_.latched()) {
        safety_.trip(*fsm_.timeout_reason(), readings_.anode_pressure, 0.0, t_);
    }

    last_command_ = control_.update(fsm_.state(), readings_, demand_pct_, TICK_DT);
    plant_.apply(last_command_);
    plant_.step(TICK_DT, last_command_.current_request, last_command_.purge_open);

    return logger_ && tick_ % LOG_DECIMATION == 0 ? log_row() : std::string{};
}

std::string Simulation::log_row() const {
    return std::format(
        "{:.2f},{},{:.0f},{:.2f},{:d},"
        "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"
        "{:.4f},{:.4f},{:.4f},{:.4f}",
        t_, to_string(fsm_.state()), demand_pct_, control_.current_setpoint(),
        safety_.latched(),
        readings_.tank_pressure, readings_.tank_temp, readings_.stack_voltage,
        readings_.stack_current, readings_.coolant_temp, readings_.anode_pressure,
        readings_.h2_flow,
        plant_.h2_valve().position(), plant_.compressor().position(),
        plant_.recirc_pump().position(), plant_.cooling().position());
}

void Simulation::step() {
    std::string row;
    {
        std::scoped_lock lock(mutex_);
        row = step_locked();
    }
    // file I/O outside the lock: a slow flush must never block UI calls.
    // Only this thread writes the log, so no lock is needed for logger_.
    if (!row.empty()) logger_->row(row);
}

void Simulation::run_for(double seconds) {
    int n = static_cast<int>(std::lround(seconds / TICK_DT));
    for (int i = 0; i < n; ++i) step();
}

void Simulation::set_demand(double pct) {
    // NaN passes through std::clamp (comparisons are false) and would poison
    // the whole plant integration irrecoverably - reject it at the boundary
    if (!std::isfinite(pct)) return;
    std::scoped_lock lock(mutex_);
    demand_pct_ = std::clamp(pct, 0.0, 100.0);
}

Simulation::CommandResult Simulation::command(std::string_view name) {
    std::scoped_lock lock(mutex_);
    if (name == "start") {
        fsm_.request_start();
    } else if (name == "stop") {
        fsm_.request_stop();
    } else if (name == "reset") {
        if (!safety_.reset(readings_)) return CommandResult::refused;
        fsm_.request_reset();
    } else {
        return CommandResult::unknown;
    }
    return CommandResult::ok;
}

void Simulation::inject(InjectKind kind, SensorId sensor, FailureMode mode) {
    std::scoped_lock lock(mutex_);
    switch (kind) {
    case InjectKind::pressure_spike: plant_.tank().inject_heat(80.0); break;
    case InjectKind::overheat:       plant_.set_cooling_blocked(true); break;
    case InjectKind::clear_overheat: plant_.set_cooling_blocked(false); break;
    case InjectKind::sensor_fail:    sensors_.fail(sensor, mode); break;
    case InjectKind::sensor_repair:  sensors_.repair(sensor); break;
    }
}

Snapshot Simulation::snapshot() const {
    std::scoped_lock lock(mutex_);  // held for the copy only, never across I/O
    Snapshot s;
    s.t = t_;
    s.state = fsm_.state();
    s.latched = safety_.latched();
    s.fault = safety_.fault();
    const auto& hist = safety_.history();
    std::size_t first = hist.size() > 5 ? hist.size() - 5 : 0;
    for (std::size_t i = first; i < hist.size(); ++i) {
        s.fault_history.push_back(hist[i]);  // describe() happens outside the lock
    }
    s.overruns = overruns_.load();
    std::size_t ne = std::min(exec_n_, TIMING_RING);
    s.exec_us.assign(exec_ring_.begin(), exec_ring_.begin() + ne);
    std::size_t np = std::min(period_n_, TIMING_RING);
    s.period_us.assign(period_ring_.begin(), period_ring_.begin() + np);
    s.demand_pct = demand_pct_;
    s.current_setpoint = control_.current_setpoint();
    s.power_kw = plant_.voltage() * plant_.current() / 1000.0;
    s.h2_consumed_g = plant_.h2_consumed_mol() * 2.016;
    s.energy_wh = plant_.energy_wh();
    // LHV of H2: 241.8 kJ/mol = 67.17 Wh/mol (spec sheets quote LHV)
    double lhv_wh = plant_.h2_consumed_mol() * 67.17;
    s.stack_efficiency_pct = lhv_wh > 1e-6 ? plant_.energy_wh() / lhv_wh * 100.0 : 0.0;
    s.readings = readings_;
    s.h2_valve = plant_.h2_valve().position();
    s.compressor = plant_.compressor().position();
    s.recirc_pump = plant_.recirc_pump().position();
    s.cooling = plant_.cooling().position();
    s.cooling_blocked = plant_.cooling_blocked();
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        s.failures[i] = sensors_.failure(static_cast<SensorId>(i));
    }
    return s;
}

void Simulation::start_thread() {
    thread_ = std::jthread([this](std::stop_token stop) {
        using clock = std::chrono::steady_clock;
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        constexpr auto period =
            duration_cast<clock::duration>(std::chrono::duration<double>(TICK_DT));
        auto next = clock::now();
        clock::time_point prev_wake{};
        while (!stop.stop_requested()) {
            auto t0 = clock::now();
            step();
            auto t1 = clock::now();
            {
                std::scoped_lock lock(mutex_);
                exec_ring_[exec_n_++ % TIMING_RING] = static_cast<std::uint32_t>(
                    duration_cast<microseconds>(t1 - t0).count());
                if (prev_wake != clock::time_point{}) {
                    period_ring_[period_n_++ % TIMING_RING] =
                        static_cast<std::uint32_t>(
                            duration_cast<microseconds>(t0 - prev_wake).count());
                }
            }
            prev_wake = t0;
            next += period;  // absolute deadline: jitter never accumulates
            if (next < clock::now()) {
                next = clock::now();  // fell behind: resync, don't burst-catch-up
                overruns_.fetch_add(1);  // but never silently - telemetry shows it
            }
            std::this_thread::sleep_until(next);
        }
    });
}

} // namespace fccu
