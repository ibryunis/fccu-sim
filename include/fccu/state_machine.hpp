// FCCU state machine.
//
// OFF -> PURGE -> PRESSURIZE -> WARMUP -> RUNNING -> SHUTDOWN -> OFF
// Any state -> FAULT when the safety latch is set. FAULT exits only via
// manual reset, and only after the latch has been cleared.
//
// Startup phases:
//   PURGE       flow H2 through the open purge valve to displace air/nitrogen
//               from the anode (never load a stack with air in the anode)
//   PRESSURIZE  close purge, bring anode manifold to operating pressure
//   WARMUP      draw a small current to self-heat the membrane
#pragma once

#include <optional>
#include <string>

namespace fccu {

enum class State { off, purge, pressurize, warmup, running, shutdown, fault };

const char* to_string(State s);

class StateMachine {
public:
    static constexpr double PURGE_TIME = 2.0;
    static constexpr double PRESSURIZE_TARGET_BAR = 1.8;
    static constexpr double PRESSURIZE_TIMEOUT = 10.0;
    static constexpr double WARMUP_TARGET_C = 35.0;
    static constexpr double WARMUP_TIMEOUT = 120.0;
    static constexpr double SHUTDOWN_TIME = 3.0;

    void request_start() { start_req_ = true; }
    void request_stop() { stop_req_ = true; }
    void request_reset() { reset_req_ = true; }

    State update(double dt, double anode_pressure, double stack_temp,
                 bool fault_latched);

    State state() const { return state_; }
    double time_in_state() const { return time_in_state_; }

    // set when a startup phase times out; runtime records it as a safety trip
    const std::optional<std::string>& timeout_reason() const {
        return timeout_reason_;
    }

private:
    void go(State s);

    State state_ = State::off;
    double time_in_state_ = 0.0;
    std::optional<std::string> timeout_reason_;
    bool start_req_ = false;
    bool stop_req_ = false;
    bool reset_req_ = false;
};

} // namespace fccu
