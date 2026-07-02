#include "fccu/fuel_cell.hpp"

#include <algorithm>
#include <cmath>

namespace fccu::fuel_cell {

double membrane_resistance(double temp_c) {
    // linear drop from cold to warm resistance as the membrane hydrates/heats
    double frac = std::clamp((temp_c - T_AMBIENT) / (T_OP - T_AMBIENT), 0.0, 1.0);
    return R_COLD + (R_HOT - R_COLD) * frac;
}

double nernst(double p_h2_bar, double p_o2_bar, double temp_c) {
    double t_k = temp_c + 273.15;
    double p_h2 = std::max(p_h2_bar, 0.01);
    double p_o2 = std::max(p_o2_bar, 0.01);
    return E0 - 0.85e-3 * (t_k - 298.15)
         + (GAS_R * t_k / (2 * FARADAY)) * std::log(p_h2 * std::sqrt(p_o2));
}

double cell_voltage(double current_a, double p_h2_bar, double p_o2_bar,
                    double temp_c, double supply_ratio) {
    double e = nernst(p_h2_bar, p_o2_bar, temp_c);
    double i = std::max(current_a, 0.0) / AREA_CM2;
    if (i <= I0) return e;

    double i_lim = I_LIM * std::clamp(supply_ratio, 0.0, 1.0);
    if (i >= i_lim) return 0.0;

    double v_act = TAFEL_A * std::log(i / I0);
    double v_ohm = i * membrane_resistance(temp_c);
    double v_conc = -B_CONC * std::log(1.0 - i / i_lim);
    return std::max(e - v_act - v_ohm - v_conc, 0.0);
}

double stack_voltage(double current_a, double p_h2_bar, double p_o2_bar,
                     double temp_c, double supply_ratio) {
    return N_CELLS * cell_voltage(current_a, p_h2_bar, p_o2_bar, temp_c, supply_ratio);
}

double h2_consumption(double current_a) {
    return std::max(current_a, 0.0) * N_CELLS / (2 * FARADAY);
}

double o2_consumption(double current_a) {
    return std::max(current_a, 0.0) * N_CELLS / (4 * FARADAY);
}

double waste_heat(double current_a, double v_cell) {
    return std::max(E_THERMAL - v_cell, 0.0) * std::max(current_a, 0.0) * N_CELLS;
}

} // namespace fccu::fuel_cell
