#include <cmath>
#include <random>
#include <set>

#include "catch_amalgamated.hpp"
#include "fccu/sensors.hpp"

using namespace fccu;
using Catch::Approx;

namespace {
Sensor make_sensor() {
    return Sensor(SensorConfig{0.1, 0.0, 100.0});
}
std::mt19937 rng(42);
} // namespace

TEST_CASE("healthy reading near truth") {
    Sensor s = make_sensor();
    double sum = 0.0;
    for (int i = 0; i < 500; ++i) sum += s.read(50.0, rng);
    CHECK(sum / 500 == Approx(50.0).margin(0.05));
}

TEST_CASE("noise present") {
    Sensor s = make_sensor();
    std::set<double> values;
    for (int i = 0; i < 20; ++i) values.insert(s.read(50.0, rng));
    CHECK(values.size() > 1);
}

TEST_CASE("stuck holds last good value") {
    Sensor s = make_sensor();
    s.read(50.0, rng);
    s.fail(FailureMode::stuck);
    double stuck = s.read(99.0, rng);
    CHECK(stuck == Approx(50.0).margin(1.0));
    CHECK(s.read(0.0, rng) == stuck);
}

TEST_CASE("disconnected returns NaN") {
    Sensor s = make_sensor();
    s.fail(FailureMode::disconnected);
    CHECK(std::isnan(s.read(50.0, rng)));
}

TEST_CASE("out of range exceeds physical max") {
    Sensor s = make_sensor();
    s.fail(FailureMode::out_of_range);
    CHECK(s.read(50.0, rng) > 100.0);
}

TEST_CASE("repair restores readings") {
    Sensor s = make_sensor();
    s.fail(FailureMode::disconnected);
    s.repair();
    CHECK(s.read(50.0, rng) == Approx(50.0).margin(1.0));
}

TEST_CASE("suite maps every channel") {
    SensorSuite suite;
    Readings truth{.tank_pressure = 350.0, .tank_temp = 25.0,
                   .stack_voltage = 60.0, .stack_current = 150.0,
                   .coolant_temp = 65.0, .anode_pressure = 2.0, .h2_flow = 0.1};
    Readings out = suite.sample(truth);
    CHECK(out.tank_pressure == Approx(350.0).margin(5.0));
    CHECK(out.stack_current == Approx(150.0).margin(5.0));
    CHECK(out.h2_flow == Approx(0.1).margin(0.05));
}

TEST_CASE("suite failure injection by id") {
    SensorSuite suite;
    suite.fail(SensorId::coolant_temp, FailureMode::disconnected);
    Readings out = suite.sample(Readings{});
    CHECK(std::isnan(out.coolant_temp));
    CHECK_FALSE(std::isnan(out.tank_pressure));
    CHECK(suite.failure(SensorId::coolant_temp) == FailureMode::disconnected);

    suite.repair(SensorId::coolant_temp);
    out = suite.sample(Readings{.coolant_temp = 65.0});
    CHECK(out.coolant_temp == Approx(65.0).margin(1.0));
}
