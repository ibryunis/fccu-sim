#include <cmath>

#include "catch_amalgamated.hpp"
#include "fccu/safety.hpp"

using namespace fccu;
using Catch::Approx;

namespace {
constexpr double DT = 0.01;
constexpr Readings HEALTHY{.tank_pressure = 350.0, .tank_temp = 25.0,
                           .stack_voltage = 60.0, .stack_current = 150.0,
                           .coolant_temp = 65.0, .anode_pressure = 2.0,
                           .h2_flow = 0.1};

double feed(SafetyMonitor& mon, const Readings& r, double seconds, double t0 = 0.0) {
    double t = t0;
    int n = static_cast<int>(std::lround(seconds / DT));
    for (int i = 0; i < n; ++i) {
        t += DT;
        mon.update(t, DT, r);
    }
    return t;
}

Readings with(double Readings::* field, double value) {
    Readings r = HEALTHY;
    r.*field = value;
    return r;
}
} // namespace

TEST_CASE("healthy readings never trip") {
    SafetyMonitor mon;
    feed(mon, HEALTHY, 5.0);
    CHECK_FALSE(mon.latched());
}

TEST_CASE("overpressure persistence window") {
    SafetyMonitor mon;
    Readings bad = with(&Readings::tank_pressure, 450.0);

    SECTION("below persistence: no trip") {
        feed(mon, bad, PERSIST_FAST - 0.05);
        CHECK_FALSE(mon.latched());
    }
    SECTION("past persistence: trips with full record") {
        feed(mon, bad, PERSIST_FAST + 0.05);
        REQUIRE(mon.latched());
        REQUIRE(mon.fault().has_value());
        CHECK(mon.fault()->reason == "tank overpressure");
        CHECK(mon.fault()->value == Approx(450.0));
        CHECK(mon.fault()->limit == 400.0);
    }
}

TEST_CASE("temperature uses the one-second window") {
    SafetyMonitor mon;
    Readings hot = with(&Readings::coolant_temp, 90.0);
    // 0.7 s over limit: would trip a 500 ms window, must not trip the 1 s one
    feed(mon, hot, 0.7);
    CHECK_FALSE(mon.latched());
    feed(mon, hot, 0.4);
    REQUIRE(mon.latched());
    CHECK(mon.fault()->reason == "coolant overtemp");
}

TEST_CASE("pressure uses the half-second window") {
    SafetyMonitor mon;
    feed(mon, with(&Readings::anode_pressure, 3.5), 0.7);
    CHECK(mon.latched());
}

TEST_CASE("intermittent violation resets the accumulator") {
    SafetyMonitor mon;
    Readings bad = with(&Readings::tank_pressure, 450.0);
    for (int i = 0; i < 10; ++i) {
        feed(mon, bad, 0.3);
        feed(mon, HEALTHY, 0.1);
    }
    CHECK_FALSE(mon.latched());
}

TEST_CASE("fault record describes the cause") {
    SafetyMonitor mon;
    feed(mon, with(&Readings::coolant_temp, 82.0), 1.1, 12.0);
    REQUIRE(mon.fault().has_value());
    std::string text = mon.fault()->describe();
    CHECK(text.find("coolant overtemp") != std::string::npos);
    CHECK(text.find("82.0") != std::string::npos);
    CHECK(text.find("80.0") != std::string::npos);
    // trips exactly when the 1 s window fills
    CHECK(text.find("t=13.0") != std::string::npos);
}

TEST_CASE("latch survives the condition clearing") {
    SafetyMonitor mon;
    feed(mon, with(&Readings::tank_pressure, 450.0), 1.0);
    feed(mon, HEALTHY, 10.0);
    CHECK(mon.latched());
}

TEST_CASE("reset flow") {
    SafetyMonitor mon;
    Readings bad = with(&Readings::tank_pressure, 450.0);
    feed(mon, bad, 1.0);
    REQUIRE(mon.latched());

    SECTION("refused while violated") {
        CHECK_FALSE(mon.reset(bad));
        CHECK(mon.latched());
    }
    SECTION("accepted after clear") {
        CHECK(mon.reset(HEALTHY));
        CHECK_FALSE(mon.latched());
        CHECK_FALSE(mon.fault().has_value());
    }
    SECTION("history survives reset") {
        mon.reset(HEALTHY);
        CHECK(mon.history().size() >= 1);
    }
}

TEST_CASE("first fault is primary") {
    SafetyMonitor mon;
    Readings bad = with(&Readings::tank_pressure, 450.0);
    double t = feed(mon, bad, 1.0);
    Readings worse = bad;
    worse.coolant_temp = 95.0;
    feed(mon, worse, 2.0, t);
    CHECK(mon.fault()->reason == "tank overpressure");
}

TEST_CASE("undervoltage only under load") {
    SafetyMonitor mon;
    Readings low_v = HEALTHY;
    low_v.stack_voltage = 30.0;
    low_v.stack_current = 5.0;
    feed(mon, low_v, 1.0);
    CHECK_FALSE(mon.latched());
    low_v.stack_current = 100.0;
    feed(mon, low_v, 1.0);
    CHECK(mon.latched());
}

TEST_CASE("NaN cannot trip a hard limit") {
    // NaN pressure must not satisfy "pressure > limit"; plausibility owns NaN
    SafetyMonitor mon;
    Readings r = with(&Readings::tank_pressure, std::nan(""));
    feed(mon, r, 0.3);  // below plausibility persistence too
    CHECK_FALSE(mon.latched());
}

TEST_CASE("disconnected sensor trips plausibility") {
    SafetyMonitor mon;
    feed(mon, with(&Readings::tank_pressure, std::nan("")), PERSIST_FAST + 0.05);
    REQUIRE(mon.latched());
    CHECK(mon.fault()->reason == "sensor implausible: tank_pressure");
}

TEST_CASE("out-of-range sensor trips plausibility") {
    SafetyMonitor mon;
    feed(mon, with(&Readings::coolant_temp, 180.0), PERSIST_FAST + 0.05);
    CHECK(mon.latched());
}

TEST_CASE("noise just past the rail is tolerated") {
    // slightly negative flow reading = noise around zero, not a broken wire
    SafetyMonitor mon;
    feed(mon, with(&Readings::h2_flow, -0.005), 5.0);
    CHECK_FALSE(mon.latched());
    CHECK_FALSE(mon.any_violation(with(&Readings::h2_flow, -0.005)));
}

TEST_CASE("external trip latches") {
    SafetyMonitor mon;
    mon.trip("pressurize timeout", 1.2, 1.8, 5.0);
    CHECK(mon.latched());
    CHECK(mon.fault()->reason == "pressurize timeout");
}
