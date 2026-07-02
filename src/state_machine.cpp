#include "fccu/state_machine.hpp"

namespace fccu {

const char* to_string(State s) {
    switch (s) {
    case State::off:        return "OFF";
    case State::purge:      return "PURGE";
    case State::pressurize: return "PRESSURIZE";
    case State::warmup:     return "WARMUP";
    case State::running:    return "RUNNING";
    case State::shutdown:   return "SHUTDOWN";
    case State::fault:      return "FAULT";
    }
    return "?";
}

void StateMachine::go(State s) {
    state_ = s;
    time_in_state_ = 0.0;
}

State StateMachine::update(double dt, double anode_pressure, double stack_temp,
                           bool fault_latched) {
    time_in_state_ += dt;
    bool start = start_req_, stop = stop_req_, reset = reset_req_;
    start_req_ = stop_req_ = reset_req_ = false;

    if (fault_latched && state_ != State::fault) {
        go(State::fault);
        return state_;
    }

    bool in_active_state = state_ == State::purge || state_ == State::pressurize
                        || state_ == State::warmup || state_ == State::running;
    if (stop && in_active_state) {
        go(State::shutdown);
        return state_;
    }

    switch (state_) {
    case State::off:
        if (start && !fault_latched) go(State::purge);
        break;

    case State::purge:
        if (time_in_state_ >= PURGE_TIME) go(State::pressurize);
        break;

    case State::pressurize:
        if (anode_pressure >= PRESSURIZE_TARGET_BAR) {
            go(State::warmup);
        } else if (time_in_state_ > PRESSURIZE_TIMEOUT) {
            timeout_reason_ = "pressurize timeout: anode pressure not reached";
            go(State::fault);
        }
        break;

    case State::warmup:
        if (stack_temp >= WARMUP_TARGET_C) {
            go(State::running);
        } else if (time_in_state_ > WARMUP_TIMEOUT) {
            timeout_reason_ = "warmup timeout: stack temp not reached";
            go(State::fault);
        }
        break;

    case State::running:
        break;

    case State::shutdown:
        if (time_in_state_ >= SHUTDOWN_TIME) go(State::off);
        break;

    case State::fault:
        if (reset && !fault_latched) {
            timeout_reason_.reset();
            go(State::off);
        }
        break;
    }
    return state_;
}

} // namespace fccu
