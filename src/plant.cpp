#include "fccu/plant.hpp"

#include <algorithm>

#include "fccu/fuel_cell.hpp"

namespace fccu {

namespace fc = fuel_cell;

double Plant::supply_ratio(double current_a) const {
    double h2 = std::clamp((anode_pressure_ - 1.0) / (ANODE_TARGET_BAR - 1.0),
                           0.0, 1.0);
    double o2_needed = fc::o2_consumption(current_a) * AIR_MARGIN;
    double air = o2_needed <= 0.0 ? 1.0
                                  : std::min(compressor_.o2_flow() / o2_needed, 1.0);
    return std::min(h2, air);
}

void Plant::apply(const Command& cmd) {
    h2_valve_.set(cmd.h2_valve);
    compressor_.set(cmd.compressor);
    recirc_pump_.set(cmd.recirc_pump);
    cooling_.set(cmd.cooling);
}

void Plant::step(double dt, double current_request, bool purge_open) {
    h2_valve_.step(dt);
    compressor_.step(dt);
    recirc_pump_.step(dt);
    cooling_.step(dt);

    // current actually drawn: capped just under the starvation-limited maximum
    double i_lim = fc::I_LIM * fc::AREA_CM2 * supply_ratio(current_request);
    current_ = std::clamp(current_request, 0.0, 0.98 * i_lim);

    double p_o2 = 0.21 * (1.0 + 0.5 * compressor_.position());
    voltage_ = fc::stack_voltage(current_, anode_pressure_, p_o2, stack_temp_,
                                 supply_ratio(current_));

    // anode manifold mass balance
    h2_flow_ = h2_valve_.flow(tank_.pressure_bar());
    double drawn = dt > 0.0 ? tank_.draw(h2_flow_ * dt) / dt : 0.0;
    double consumed = fc::h2_consumption(current_) / recirc_pump_.h2_efficiency();
    double vent = purge_open ? PURGE_FLOW : 0.0;
    double t_k = stack_temp_ + 273.15;
    double dp_pa = (drawn - consumed - vent) * fc::GAS_R * t_k / ANODE_VOLUME_M3 * dt;
    anode_pressure_ = std::max(anode_pressure_ + dp_pa / 1e5, 0.0);

    // lumped thermal mass
    double v_cell = voltage_ / fc::N_CELLS;
    double heat_in = fc::waste_heat(current_, v_cell);
    double heat_out = cooling_blocked_ ? 0.0
                                       : cooling_.heat_removed(stack_temp_, ambient_);
    stack_temp_ += (heat_in - heat_out) * dt / THERMAL_MASS;

    tank_.step(dt);
}

Readings Plant::truth() const {
    return {
        .tank_pressure = tank_.pressure_bar(),
        .tank_temp = tank_.temp_c(),
        .stack_voltage = voltage_,
        .stack_current = current_,
        .coolant_temp = stack_temp_,
        .anode_pressure = anode_pressure_,
        .h2_flow = h2_flow_,
    };
}

} // namespace fccu
