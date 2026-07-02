// Hydrogen tank: ideal gas, pressure falls as moles are drawn.
//
// Discharge cools the gas slightly (expansion work); temperature relaxes
// back to ambient. Fault injection can heat the tank to force overpressure.
//
// Small value type with trivial logic: header-only by design.
#pragma once

#include <algorithm>

#include "fccu/fuel_cell.hpp"

namespace fccu {

class HydrogenTank {
public:
    static constexpr double TEMP_RELAX_TAU = 60.0; // s, relaxation to ambient
    static constexpr double COOL_PER_MOL = 1.0;    // K drop per mol drawn

    explicit HydrogenTank(double volume_l = 20.0, double pressure_bar = 350.0,
                          double temp_c = 25.0, double ambient_c = 25.0)
        : volume_m3_(volume_l / 1000.0), temp_c_(temp_c), ambient_c_(ambient_c),
          moles_((pressure_bar * 1e5) * volume_m3_
                 / (fuel_cell::GAS_R * (temp_c + 273.15))) {}

    double pressure_bar() const {
        return moles_ * fuel_cell::GAS_R * (temp_c_ + 273.15) / volume_m3_ / 1e5;
    }
    double moles() const { return moles_; }
    double temp_c() const { return temp_c_; }

    double draw(double mol) {
        double taken = std::min(mol, moles_);
        moles_ -= taken;
        temp_c_ -= COOL_PER_MOL * taken;
        return taken;
    }

    void step(double dt) {
        temp_c_ += (ambient_c_ - temp_c_) * dt / TEMP_RELAX_TAU;
    }

    void inject_heat(double delta_c) { temp_c_ += delta_c; }

private:
    double volume_m3_;
    double temp_c_;
    double ambient_c_;
    double moles_;
};

} // namespace fccu
