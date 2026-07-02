#include "catch_amalgamated.hpp"
#include "fccu/tank.hpp"

using fccu::HydrogenTank;
using Catch::Approx;

TEST_CASE("initial pressure matches spec") {
    HydrogenTank tank(20.0, 350.0, 25.0);
    CHECK(tank.pressure_bar() == Approx(350.0));
}

TEST_CASE("pressure drops when drawing") {
    HydrogenTank tank;
    double p0 = tank.pressure_bar();
    tank.draw(10.0);
    CHECK(tank.pressure_bar() < p0);
}

TEST_CASE("cannot draw more than content") {
    HydrogenTank tank(1.0, 10.0);
    tank.draw(1e6);
    CHECK(tank.moles() == 0.0);
    CHECK(tank.pressure_bar() == 0.0);
}

TEST_CASE("temperature relaxes to ambient") {
    HydrogenTank tank;
    tank.inject_heat(50.0);
    for (int i = 0; i < 100000; ++i) tank.step(0.01);
    CHECK(tank.temp_c() == Approx(25.0).margin(0.5));
}

TEST_CASE("heating raises pressure") {
    HydrogenTank tank;
    double p0 = tank.pressure_bar();
    tank.inject_heat(80.0);
    CHECK(tank.pressure_bar() > p0 * 1.2);
}
