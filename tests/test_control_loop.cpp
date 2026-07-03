#include <cmath>

#include "catch_amalgamated.hpp"
#include "fccu/control_loop.hpp"
#include "fccu/fuel_cell.hpp"
#include "fccu/plant.hpp"

using namespace fccu;
using Catch::Approx;

namespace {
constexpr double DT = 0.01;
constexpr Readings NOMINAL{.coolant_temp = 65.0, .anode_pressure = 2.0};

void run_loop(Plant& plant, ControlLoop& ctrl, State state, double demand,
              double seconds) {
    for (int i = 0; i < static_cast<int>(seconds / DT); ++i) {
        Command cmd = ctrl.update(state, plant.truth(), demand, DT);
        plant.apply(cmd);
        plant.step(DT, cmd.current_request, cmd.purge_open);
    }
}
} // namespace

TEST_CASE("PID proportional sign") {
    Pid pid(1.0, 0.0, 0.0, -10.0, 10.0);
    CHECK(pid.update(2.0, DT) == Approx(2.0));
    CHECK(pid.update(-2.0, DT) < 0.0);
}

TEST_CASE("PID output clamped") {
    Pid pid(100.0, 0.0);
    CHECK(pid.update(5.0, DT) == 1.0);
    CHECK(pid.update(-5.0, DT) == 0.0);
}

TEST_CASE("PID integrator removes steady-state error") {
    // first-order plant: dy/dt = u - 0.5*y, target 1.0
    Pid pid(0.5, 2.0, 0.0, 0.0, 5.0);
    double y = 0.0;
    for (int i = 0; i < 2000; ++i) {
        double u = pid.update(1.0 - y, DT);
        y += (u - 0.5 * y) * DT;
    }
    CHECK(y == Approx(1.0).margin(0.01));
}

TEST_CASE("PID anti-windup bounds the integral") {
    Pid pid(0.1, 1.0);
    for (int i = 0; i < 1000; ++i) pid.update(10.0, DT);  // saturated throughout
    // integral must stay near the saturation boundary, not grow to ~100
    CHECK(pid.integral() < 2.0);
}

TEST_CASE("OFF commands everything off") {
    ControlLoop ctrl;
    Command cmd = ctrl.update(State::off, NOMINAL, 50.0, DT);
    CHECK(cmd.h2_valve == 0.0);
    CHECK(cmd.current_request == 0.0);
    CHECK_FALSE(cmd.purge_open);
}

TEST_CASE("FAULT cuts H2 and keeps cooling") {
    ControlLoop ctrl;
    Command cmd = ctrl.update(State::fault, NOMINAL, 50.0, DT);
    CHECK(cmd.h2_valve == 0.0);
    CHECK(cmd.current_request == 0.0);
    CHECK(cmd.cooling == 1.0);
}

TEST_CASE("PURGE opens purge valve, no load") {
    ControlLoop ctrl;
    Command cmd = ctrl.update(State::purge, NOMINAL, 0.0, DT);
    CHECK(cmd.purge_open);
    CHECK(cmd.h2_valve > 0.0);
    CHECK(cmd.current_request == 0.0);
}

TEST_CASE("current setpoint is slew limited") {
    ControlLoop ctrl;
    Command cmd = ctrl.update(State::running, NOMINAL, 100.0, DT);
    CHECK(cmd.current_request <= ControlLoop::CURRENT_SLEW * DT + 1e-9);
}

TEST_CASE("demand maps to current") {
    ControlLoop ctrl;
    Command cmd;
    for (int i = 0; i < 500; ++i) {
        cmd = ctrl.update(State::running, NOMINAL, 50.0, DT);
    }
    CHECK(cmd.current_request == Approx(0.5 * fuel_cell::I_RATED).epsilon(0.01));
}

TEST_CASE("compressor feedforward scales with demand") {
    ControlLoop ctrl;
    Command low, high;
    for (int i = 0; i < 500; ++i) low = ctrl.update(State::running, NOMINAL, 20.0, DT);
    ctrl.reset();
    for (int i = 0; i < 500; ++i) high = ctrl.update(State::running, NOMINAL, 90.0, DT);
    CHECK(high.compressor > low.compressor);
}

TEST_CASE("NaN readings never produce NaN commands") {
    ControlLoop ctrl;
    Readings nan_readings;
    nan_readings.anode_pressure = std::nan("");
    nan_readings.coolant_temp = std::nan("");
    Command cmd = ctrl.update(State::running, nan_readings, 50.0, DT);
    CHECK_FALSE(std::isnan(cmd.h2_valve));
    CHECK_FALSE(std::isnan(cmd.compressor));
    CHECK_FALSE(std::isnan(cmd.cooling));
    CHECK_FALSE(std::isnan(cmd.current_request));
}

TEST_CASE("H2 valve fails closed on a dead anode pressure sensor") {
    ControlLoop ctrl;
    Readings healthy = NOMINAL;
    for (int i = 0; i < 200; ++i) ctrl.update(State::running, healthy, 50.0, DT);

    Readings dead = NOMINAL;
    dead.anode_pressure = std::nan("");
    Command cmd = ctrl.update(State::running, dead, 50.0, DT);
    CHECK(cmd.h2_valve == 0.0);  // never coast open-loop on the setpoint
}

TEST_CASE("closed loop: pressure settles on target") {
    Plant plant;
    ControlLoop ctrl;
    plant.set_stack_temp(65.0);
    run_loop(plant, ctrl, State::running, 50.0, 10.0);
    CHECK(plant.anode_pressure() == Approx(2.0).margin(0.1));
}

TEST_CASE("closed loop: pressure holds under full load") {
    Plant plant;
    ControlLoop ctrl;
    plant.set_stack_temp(65.0);
    run_loop(plant, ctrl, State::running, 100.0, 15.0);
    CHECK(plant.anode_pressure() == Approx(2.0).margin(0.15));
    CHECK(plant.current() > 300.0);
}

TEST_CASE("closed loop: delivered current tracks demand") {
    Plant plant;
    ControlLoop ctrl;
    plant.set_stack_temp(65.0);
    run_loop(plant, ctrl, State::running, 50.0, 10.0);
    CHECK(plant.current() == Approx(0.5 * fuel_cell::I_RATED).epsilon(0.05));
}

TEST_CASE("closed loop: cooling regulates temperature") {
    Plant plant;
    ControlLoop ctrl;
    plant.set_stack_temp(75.0);
    run_loop(plant, ctrl, State::running, 60.0, 120.0);
    CHECK(plant.stack_temp() == Approx(65.0).margin(3.0));
}

TEST_CASE("closed loop: full demand step never collapses cell voltage") {
    Plant plant;
    ControlLoop ctrl;
    plant.set_stack_temp(65.0);
    run_loop(plant, ctrl, State::running, 0.0, 3.0);
    double min_v = 999.0;
    for (int i = 0; i < static_cast<int>(10.0 / DT); ++i) {
        Command cmd = ctrl.update(State::running, plant.truth(), 100.0, DT);
        plant.apply(cmd);
        plant.step(DT, cmd.current_request, cmd.purge_open);
        if (plant.current() > 20.0) {
            min_v = std::min(min_v, plant.voltage() / fuel_cell::N_CELLS);
        }
    }
    CHECK(min_v > 0.4);
}
