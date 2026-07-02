// Shared plain types. Zero dependencies: both the plant (consumer of
// Command, producer of truth Readings) and the controller (the reverse)
// include this without depending on each other.
#pragma once

#include <array>
#include <cstddef>

namespace fccu {

// one sample of every sensor channel; also used for plant ground truth
struct Readings {
    double tank_pressure = 0.0;   // bar
    double tank_temp = 0.0;       // degC
    double stack_voltage = 0.0;   // V
    double stack_current = 0.0;   // A
    double coolant_temp = 0.0;    // degC
    double anode_pressure = 0.0;  // bar absolute
    double h2_flow = 0.0;         // mol/s
};

enum class SensorId : std::size_t {
    tank_pressure, tank_temp, stack_voltage, stack_current,
    coolant_temp, anode_pressure, h2_flow,
};

inline constexpr std::size_t SENSOR_COUNT = 7;

// pointer-to-member table: lets code iterate Readings fields by SensorId
// without string keys or reflection
inline constexpr std::array<double Readings::*, SENSOR_COUNT> READING_FIELDS = {
    &Readings::tank_pressure, &Readings::tank_temp, &Readings::stack_voltage,
    &Readings::stack_current, &Readings::coolant_temp, &Readings::anode_pressure,
    &Readings::h2_flow,
};

inline constexpr std::array<const char*, SENSOR_COUNT> SENSOR_NAMES = {
    "tank_pressure", "tank_temp", "stack_voltage", "stack_current",
    "coolant_temp", "anode_pressure", "h2_flow",
};

// what the control loop commands each tick
struct Command {
    double h2_valve = 0.0;        // fraction open 0..1
    double compressor = 0.0;      // PWM 0..1
    double recirc_pump = 0.0;     // PWM 0..1
    double cooling = 0.0;         // PWM 0..1
    double current_request = 0.0; // A drawn from the stack
    bool purge_open = false;
};

} // namespace fccu
