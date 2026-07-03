// End-to-end: full startup, load tracking, every fault injection,
// latch + reset flow. Runs the real Simulation single-threaded.
#include <cmath>
#include <string>

#include "catch_amalgamated.hpp"
#include "fccu/fuel_cell.hpp"
#include "fccu/runtime.hpp"

using namespace fccu;
using Catch::Approx;

namespace {
template <typename Pred>
bool run_until(Simulation& sim, Pred pred, double timeout_s) {
    int n = static_cast<int>(timeout_s * 100);
    for (int i = 0; i < n; ++i) {
        sim.step();
        if (pred(sim)) return true;
    }
    return false;
}

void start_to_running(Simulation& sim) {
    sim.command("start");
    bool ok = run_until(
        sim, [](Simulation& s) { return s.state() == State::running; }, 60.0);
    REQUIRE(ok);
}
} // namespace

TEST_CASE("full startup reaches RUNNING through all phases") {
    Simulation sim;
    sim.command("start");
    bool saw_purge = false, saw_pressurize = false, saw_warmup = false;
    bool ok = run_until(sim, [&](Simulation& s) {
        saw_purge |= s.state() == State::purge;
        saw_pressurize |= s.state() == State::pressurize;
        saw_warmup |= s.state() == State::warmup;
        return s.state() == State::running;
    }, 60.0);
    REQUIRE(ok);
    CHECK(saw_purge);
    CHECK(saw_pressurize);
    CHECK(saw_warmup);
    CHECK_FALSE(sim.safety().latched());
}

TEST_CASE("delivered current tracks demand") {
    Simulation sim;
    start_to_running(sim);
    sim.set_demand(50.0);
    sim.run_for(10.0);
    CHECK(sim.plant().current() == Approx(0.5 * fuel_cell::I_RATED).epsilon(0.05));
    CHECK(sim.plant().voltage() * sim.plant().current() > 8000.0);
}

TEST_CASE("stop returns to OFF") {
    Simulation sim;
    start_to_running(sim);
    sim.command("stop");
    CHECK(run_until(sim, [](Simulation& s) { return s.state() == State::off; }, 10.0));
}

TEST_CASE("overheat injection faults and cuts hydrogen") {
    Simulation sim;
    start_to_running(sim);
    sim.set_demand(80.0);
    sim.run_for(5.0);
    sim.inject(InjectKind::overheat);
    REQUIRE(run_until(sim, [](Simulation& s) { return s.state() == State::fault; },
                      120.0));
    REQUIRE(sim.safety().fault().has_value());
    CHECK(sim.safety().fault()->reason == "coolant overtemp");
    sim.run_for(1.0);
    CHECK(sim.last_command().h2_valve == 0.0);
    CHECK(sim.last_command().current_request == 0.0);
}

TEST_CASE("pressure spike injection faults") {
    Simulation sim;
    start_to_running(sim);
    sim.inject(InjectKind::pressure_spike);
    REQUIRE(run_until(sim, [](Simulation& s) { return s.state() == State::fault; },
                      5.0));
    CHECK(sim.safety().fault()->reason == "tank overpressure");
}

TEST_CASE("sensor disconnect faults") {
    Simulation sim;
    start_to_running(sim);
    sim.inject(InjectKind::sensor_fail, SensorId::coolant_temp,
               FailureMode::disconnected);
    REQUIRE(run_until(sim, [](Simulation& s) { return s.state() == State::fault; },
                      5.0));
    CHECK(sim.safety().fault()->reason == "sensor implausible: coolant_temp");
}

TEST_CASE("reset refused while fault present, accepted after repair") {
    Simulation sim;
    start_to_running(sim);
    sim.inject(InjectKind::sensor_fail, SensorId::tank_pressure,
               FailureMode::disconnected);
    REQUIRE(run_until(sim, [](Simulation& s) { return s.state() == State::fault; },
                      5.0));

    sim.command("reset");  // sensor still dead: must be refused
    sim.run_for(0.5);
    CHECK(sim.state() == State::fault);
    CHECK(sim.safety().latched());

    sim.inject(InjectKind::sensor_repair, SensorId::tank_pressure);
    sim.run_for(0.5);
    sim.command("reset");
    sim.run_for(0.5);
    CHECK(sim.state() == State::off);
    CHECK_FALSE(sim.safety().latched());

    start_to_running(sim);  // and the car can start again
}

TEST_CASE("NaN demand is rejected, plant stays finite") {
    Simulation sim;
    start_to_running(sim);
    sim.set_demand(50.0);
    sim.run_for(3.0);
    sim.set_demand(std::nan(""));  // must not poison the plant
    sim.run_for(2.0);
    CHECK(std::isfinite(sim.plant().current()));
    CHECK(std::isfinite(sim.plant().anode_pressure()));
    CHECK(std::isfinite(sim.plant().stack_temp()));
    CHECK(sim.plant().current() > 100.0);  // still tracking the last demand
}

TEST_CASE("empty tank: pressurize timeout latches the SDC") {
    Simulation sim;
    double taken = sim.plant().tank().draw(1e9);
    // tank model cools linearly per mol drawn; restore temp so only the
    // empty-tank effect (no flow -> no pressure rise) is under test
    sim.plant().tank().inject_heat(taken * HydrogenTank::COOL_PER_MOL);
    sim.command("start");
    REQUIRE(run_until(sim, [](Simulation& s) { return s.state() == State::fault; },
                      20.0));
    REQUIRE(sim.safety().latched());
    CHECK(sim.safety().fault()->reason.find("pressurize timeout")
          != std::string::npos);
}

TEST_CASE("stuck sensor is detected while running") {
    Simulation sim;
    start_to_running(sim);
    sim.run_for(2.0);
    sim.inject(InjectKind::sensor_fail, SensorId::coolant_temp, FailureMode::stuck);
    REQUIRE(run_until(sim, [](Simulation& s) { return s.state() == State::fault; },
                      5.0));
    CHECK(sim.safety().fault()->reason == "sensor stuck: coolant_temp");
}

TEST_CASE("energy accounting tracks a run and resets on the next purge") {
    Simulation sim;
    start_to_running(sim);
    sim.set_demand(60.0);
    sim.run_for(10.0);
    CHECK(sim.plant().h2_consumed_mol() > 0.5);
    CHECK(sim.plant().energy_wh() > 20.0);
    // PEM stacks run roughly 45-65% of LHV at moderate load
    double eff = sim.plant().energy_wh() / (sim.plant().h2_consumed_mol() * 67.17);
    CHECK(eff > 0.35);
    CHECK(eff < 0.75);

    sim.command("stop");
    REQUIRE(run_until(sim, [](Simulation& s) { return s.state() == State::off; },
                      10.0));
    sim.command("start");
    sim.run_for(0.1);  // entered PURGE: counters must have reset
    CHECK(sim.plant().h2_consumed_mol() < 0.01);
}

TEST_CASE("overheat recovery needs cooldown first") {
    Simulation sim;
    start_to_running(sim);
    sim.set_demand(80.0);
    sim.run_for(5.0);
    sim.inject(InjectKind::overheat);
    REQUIRE(run_until(sim, [](Simulation& s) { return s.state() == State::fault; },
                      120.0));

    sim.command("reset");  // still hot: must be refused
    sim.run_for(0.5);
    CHECK(sim.state() == State::fault);

    sim.inject(InjectKind::clear_overheat);  // FAULT runs cooling at 100%
    REQUIRE(run_until(
        sim, [](Simulation& s) { return s.readings().coolant_temp < 79.0; }, 120.0));
    sim.run_for(2.0);  // persistence accumulators settle
    sim.command("reset");
    sim.run_for(0.5);
    CHECK(sim.state() == State::off);
}
