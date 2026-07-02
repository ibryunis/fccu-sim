// Actuators: each holds a commanded value and a lagged actual position.
//
// First-order lag: position approaches command with time constant tau.
// Commands are clamped to [0, 1] (fraction open / PWM duty).
//
// Concrete classes, no virtual dispatch: the plant owns each by value and
// calls it directly. Polymorphism would buy nothing here.
#pragma once

#include <algorithm>

namespace fccu {

class Actuator {
public:
    explicit Actuator(double tau_s) : tau_s_(tau_s) {}

    double command() const { return command_; }
    double position() const { return position_; }

    void set(double value) { command_ = std::clamp(value, 0.0, 1.0); }

    void step(double dt) {
        position_ += (command_ - position_) * std::min(dt / tau_s_, 1.0);
    }

private:
    double tau_s_;
    double command_ = 0.0;
    double position_ = 0.0;
};

class H2Valve : public Actuator {
public:
    static constexpr double MAX_FLOW = 0.35;       // mol/s fully open
    static constexpr double MIN_SUPPLY_BAR = 50.0; // below: tank can't feed full flow

    H2Valve() : Actuator(0.05) {}

    double flow(double tank_pressure_bar) const {
        double avail = std::clamp(tank_pressure_bar / MIN_SUPPLY_BAR, 0.0, 1.0);
        return position() * MAX_FLOW * avail;
    }
};

class AirCompressor : public Actuator {
public:
    static constexpr double MAX_O2_FLOW = 0.25;    // mol O2 / s at full PWM

    AirCompressor() : Actuator(0.4) {}

    double o2_flow() const { return position() * MAX_O2_FLOW; }
};

// Anode recirculation: without it, unreacted H2 is lost and local
// starvation costs effective supply.
class RecircPump : public Actuator {
public:
    static constexpr double H2_EFFICIENCY_OFF = 0.85;

    RecircPump() : Actuator(0.1) {}

    double h2_efficiency() const {
        return position() > 0.5 ? 1.0 : H2_EFFICIENCY_OFF;
    }
};

class CoolingSystem : public Actuator {
public:
    static constexpr double UA_MAX = 700.0;        // W/K at full PWM

    CoolingSystem() : Actuator(0.5) {}

    double heat_removed(double stack_temp_c, double ambient_c) const {
        return position() * UA_MAX * std::max(stack_temp_c - ambient_c, 0.0);
    }
};

} // namespace fccu
