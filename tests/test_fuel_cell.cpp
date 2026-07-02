#include "catch_amalgamated.hpp"
#include "fccu/fuel_cell.hpp"

using namespace fccu::fuel_cell;
using Catch::Approx;

constexpr double P_H2 = 2.0, P_O2 = 0.21, T_OPER = 65.0;

TEST_CASE("open circuit voltage is physical") {
    double v = cell_voltage(0.0, P_H2, P_O2, T_OPER);
    CHECK(v > 0.9);
    CHECK(v < 1.3);
}

TEST_CASE("voltage decreases monotonically with current") {
    double currents[] = {0, 50, 100, 200, 300, 330};
    double prev = 1e9;
    for (double i : currents) {
        double v = cell_voltage(i, P_H2, P_O2, T_OPER);
        CHECK(v < prev);
        prev = v;
    }
}

TEST_CASE("warm stack outperforms cold") {
    CHECK(cell_voltage(200, P_H2, P_O2, 65.0) > cell_voltage(200, P_H2, P_O2, 25.0));
}

TEST_CASE("higher pressure raises voltage") {
    CHECK(cell_voltage(100, 2.5, P_O2, T_OPER) > cell_voltage(100, 1.2, P_O2, T_OPER));
}

TEST_CASE("starvation collapses voltage") {
    double healthy = cell_voltage(200, P_H2, P_O2, T_OPER, 1.0);
    double starved = cell_voltage(200, P_H2, P_O2, T_OPER, 0.3);
    CHECK(starved < healthy * 0.5);
}

TEST_CASE("voltage is zero at limiting current") {
    CHECK(cell_voltage(I_LIM * AREA_CM2, P_H2, P_O2, T_OPER) == 0.0);
}

TEST_CASE("rated power is around 20 kW") {
    double power = stack_voltage(I_RATED, P_H2, P_O2, T_OPER) * I_RATED;
    CHECK(power > 15e3);
    CHECK(power < 25e3);
}

TEST_CASE("Faraday H2 consumption") {
    // I*N/(2F): 330 * 96 / (2 * 96485) ~= 0.164 mol/s
    CHECK(h2_consumption(330) == Approx(0.164).margin(0.005));
}

TEST_CASE("O2 consumption is half of H2") {
    CHECK(o2_consumption(200) == Approx(h2_consumption(200) / 2));
}

TEST_CASE("waste heat positive under load") {
    double v = cell_voltage(200, P_H2, P_O2, T_OPER);
    CHECK(waste_heat(200, v) > 0.0);
}
