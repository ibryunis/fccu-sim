#include "catch_amalgamated.hpp"
#include "fccu/actuators.hpp"

using namespace fccu;
using Catch::Approx;

namespace {
void settle(Actuator& a, double seconds, double dt = 0.01) {
    for (int i = 0; i < static_cast<int>(seconds / dt); ++i) a.step(dt);
}
} // namespace

TEST_CASE("command clamped to [0,1]") {
    Actuator a(0.1);
    a.set(1.7);
    CHECK(a.command() == 1.0);
    a.set(-3.0);
    CHECK(a.command() == 0.0);
}

TEST_CASE("first-order lag reaches 63% after tau") {
    Actuator a(0.5);
    a.set(1.0);
    settle(a, 0.5);
    CHECK(a.position() == Approx(0.632).margin(0.02));
}

TEST_CASE("position settles on command") {
    Actuator a(0.1);
    a.set(0.7);
    settle(a, 2.0);
    CHECK(a.position() == Approx(0.7).margin(1e-3));
}

TEST_CASE("valve flow scales with opening") {
    H2Valve v;
    v.set(1.0);
    settle(v, 1.0);
    double full = v.flow(350.0);
    v.set(0.5);
    settle(v, 1.0);
    CHECK(v.flow(350.0) == Approx(full / 2).epsilon(0.02));
}

TEST_CASE("valve flow drops with a nearly empty tank") {
    H2Valve v;
    v.set(1.0);
    settle(v, 1.0);
    CHECK(v.flow(25.0) == Approx(v.flow(350.0) / 2).epsilon(0.02));
    CHECK(v.flow(0.0) == 0.0);
}

TEST_CASE("compressor flow proportional to PWM") {
    AirCompressor c;
    c.set(0.4);
    settle(c, 5.0);
    CHECK(c.o2_flow() == Approx(0.4 * AirCompressor::MAX_O2_FLOW).epsilon(0.01));
}

TEST_CASE("recirc efficiency") {
    RecircPump p;
    CHECK(p.h2_efficiency() == RecircPump::H2_EFFICIENCY_OFF);
    p.set(1.0);
    settle(p, 1.0);
    CHECK(p.h2_efficiency() == 1.0);
}

TEST_CASE("cooling power scales with delta T") {
    CoolingSystem c;
    c.set(1.0);
    settle(c, 5.0);
    CHECK(c.heat_removed(65.0, 25.0) == Approx(700.0 * 40).epsilon(0.01));
    CHECK(c.heat_removed(20.0, 25.0) == 0.0);
}
