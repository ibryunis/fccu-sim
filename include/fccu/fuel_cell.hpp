// PEM fuel cell stack model.
//
// Static polarization curve (Larminie & Dicks, "Fuel Cell Systems Explained"):
//
//     V_cell = E_nernst - V_activation - V_ohmic - V_concentration
//
// Sized as a simplified single Formula Student stack: 96 cells, 300 cm2,
// ~20 kW peak. Real FCEVs stack higher voltage and boost through a DC/DC;
// that converter is abstracted away as the current control loop.
//
// The model is stateless math, so these are free functions, not a class.
#pragma once

namespace fccu::fuel_cell {

inline constexpr double GAS_R = 8.314;      // gas constant, J/(mol*K)
inline constexpr double FARADAY = 96485.0;  // C/mol

inline constexpr int N_CELLS = 96;
inline constexpr double AREA_CM2 = 300.0;
inline constexpr double I_RATED = 330.0;    // rated stack current, A

inline constexpr double E0 = 1.229;         // reversible potential at STP, V
inline constexpr double TAFEL_A = 0.04;     // activation (Tafel) slope, V
inline constexpr double I0 = 3e-4;          // exchange current density, A/cm2
inline constexpr double B_CONC = 0.05;      // concentration loss constant, V
inline constexpr double I_LIM = 1.4;        // limiting current density, A/cm2
inline constexpr double R_HOT = 0.12;       // membrane resistance warm, ohm*cm2
inline constexpr double R_COLD = 0.30;      // membrane resistance cold, ohm*cm2
inline constexpr double T_OP = 65.0;        // design operating temp, degC
inline constexpr double T_AMBIENT = 25.0;

// heating value potential: current * (E_THERMAL - V_cell) is waste heat
inline constexpr double E_THERMAL = 1.25;

double membrane_resistance(double temp_c);
double nernst(double p_h2_bar, double p_o2_bar, double temp_c);

// supply_ratio < 1 models reactant starvation: it shrinks the limiting
// current, collapsing voltage exactly like a starved real stack
double cell_voltage(double current_a, double p_h2_bar, double p_o2_bar,
                    double temp_c, double supply_ratio = 1.0);
double stack_voltage(double current_a, double p_h2_bar, double p_o2_bar,
                     double temp_c, double supply_ratio = 1.0);

double h2_consumption(double current_a);  // Faraday: I*N/(2F), mol/s
double o2_consumption(double current_a);  // I*N/(4F), mol/s
double waste_heat(double current_a, double v_cell);  // W

} // namespace fccu::fuel_cell
