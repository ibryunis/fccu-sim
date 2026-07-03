// Plant: the physical system the FCCU controls.
//
// Wires tank, stack and actuators together and integrates the two dynamic
// states: anode manifold pressure and stack temperature. The FCCU only sees
// this through sensors and only touches it through apply(Command).
#pragma once

#include "fccu/actuators.hpp"
#include "fccu/tank.hpp"
#include "fccu/types.hpp"

namespace fccu {

class Plant {
public:
    static constexpr double ANODE_VOLUME_M3 = 0.002;  // 2 L supply manifold
    static constexpr double ANODE_TARGET_BAR = 2.0;   // design anode pressure, abs
    static constexpr double PURGE_FLOW = 0.05;        // mol/s vented while purging
    static constexpr double THERMAL_MASS = 8000.0;    // J/K, lowered for demo pacing
    static constexpr double AIR_MARGIN = 1.05;        // min O2 excess before starving

    // how well reactant supply covers the demanded current, 0..1
    double supply_ratio(double current_a) const;

    void apply(const Command& cmd);
    void step(double dt, double current_request, bool purge_open);

    Readings truth() const;  // ground truth the sensor suite samples from

    double anode_pressure() const { return anode_pressure_; }
    double stack_temp() const { return stack_temp_; }
    double current() const { return current_; }
    double voltage() const { return voltage_; }

    HydrogenTank& tank() { return tank_; }
    const H2Valve& h2_valve() const { return h2_valve_; }
    const AirCompressor& compressor() const { return compressor_; }
    const RecircPump& recirc_pump() const { return recirc_pump_; }
    const CoolingSystem& cooling() const { return cooling_; }

    void set_cooling_blocked(bool blocked) { cooling_blocked_ = blocked; }
    bool cooling_blocked() const { return cooling_blocked_; }

    // initial-condition setters (also used by tests to skip warm-up)
    void set_stack_temp(double c) { stack_temp_ = c; }
    void set_anode_pressure(double bar) { anode_pressure_ = bar; }

    // run energy accounting: H2 drawn by the stack (reaction + recirc loss,
    // purge vent excluded) and electrical energy delivered
    double h2_consumed_mol() const { return h2_consumed_mol_; }
    double energy_wh() const { return energy_wh_; }
    void reset_energy_counters() { h2_consumed_mol_ = 0.0; energy_wh_ = 0.0; }

private:
    HydrogenTank tank_;
    H2Valve h2_valve_;
    AirCompressor compressor_;
    RecircPump recirc_pump_;
    CoolingSystem cooling_;

    double anode_pressure_ = 1.0;  // bar abs, starts at atmosphere
    double stack_temp_ = 25.0;
    double ambient_ = 25.0;
    double current_ = 0.0;
    double voltage_ = 0.0;
    double h2_flow_ = 0.0;
    double h2_consumed_mol_ = 0.0;
    double energy_wh_ = 0.0;
    bool cooling_blocked_ = false; // fault injection: coolant loop failure
};

} // namespace fccu
