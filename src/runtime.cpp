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

void Simulation::step_locked() {
    t_ += TICK_DT;
    ++tick_;

    readings_ = sensors_.sample(plant_.truth());

    safety_.update(t_, TICK_DT, readings_);
    fsm_.update(TICK_DT, readings_.anode_pressure, readings_.coolant_temp,
                safety_.latched());
    if (fsm_.timeout_reason() && !safety_.latched()) {
        safety_.trip(*fsm_.timeout_reason(), readings_.anode_pressure, 0.0, t_);
    }

    last_command_ = control_.update(fsm_.state(), readings_, demand_pct_, TICK_DT);
    plant_.apply(last_command_);
    plant_.step(TICK_DT, last_command_.current_request, last_command_.purge_open);

    if (logger_ && tick_ % LOG_DECIMATION == 0) log_row();
}

void Simulation::log_row() {
    logger_->row(std::format(
        "{:.2f},{},{:.0f},{:.2f},{:d},"
        "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"
        "{:.4f},{:.4f},{:.4f},{:.4f}",
        t_, to_string(fsm_.state()), demand_pct_, control_.current_setpoint(),
        safety_.latched(),
        readings_.tank_pressure, readings_.tank_temp, readings_.stack_voltage,
        readings_.stack_current, readings_.coolant_temp, readings_.anode_pressure,
        readings_.h2_flow,
        plant_.h2_valve().position(), plant_.compressor().position(),
        plant_.recirc_pump().position(), plant_.cooling().position()));
}

void Simulation::step() {
    std::scoped_lock lock(mutex_);
    step_locked();
}

void Simulation::run_for(double seconds) {
    int n = static_cast<int>(std::lround(seconds / TICK_DT));
    for (int i = 0; i < n; ++i) step();
}

void Simulation::set_demand(double pct) {
    std::scoped_lock lock(mutex_);
    demand_pct_ = std::clamp(pct, 0.0, 100.0);
}

bool Simulation::command(std::string_view name) {
    std::scoped_lock lock(mutex_);
    if (name == "start") {
        fsm_.request_start();
    } else if (name == "stop") {
        fsm_.request_stop();
    } else if (name == "reset") {
        if (safety_.reset(readings_)) fsm_.request_reset();
    } else {
        return false;
    }
    return true;
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
        s.fault_history.push_back(hist[i].describe());
    }
    s.demand_pct = demand_pct_;
    s.current_setpoint = control_.current_setpoint();
    s.power_kw = plant_.voltage() * plant_.current() / 1000.0;
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
        constexpr auto period =
            std::chrono::duration_cast<clock::duration>(
                std::chrono::duration<double>(TICK_DT));
        auto next = clock::now();
        while (!stop.stop_requested()) {
            step();
            next += period;  // absolute deadline: jitter never accumulates
            if (next < clock::now()) next = clock::now();  // fell behind: resync
            std::this_thread::sleep_until(next);
        }
    });
}

} // namespace fccu
