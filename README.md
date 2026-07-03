# FCCU Simulator (C++)

Software simulator of a hydrogen-electric Formula Student car's fuel cell
system and its Fuel Cell Control Unit (FCCU), in C++20. The plant (stack,
tank, actuators) is simulated at 100 Hz; the FCCU (state machine, control
loops, safety monitor) runs against it the way firmware runs against
hardware: it only sees sensor readings and only acts through commands.

## Run it

Windows (MinGW g++ and CMake on PATH):

```
run.bat
```

Linux:

```
./run.sh
```

Both build on first run, start the server, and open http://localhost:8000.
Press START, move the demand slider, inject faults, reset the latch.
Every run logs CSV at 10 Hz to `logs/`.

Tests:

```
cmake --build build && build\fccu_tests
```

## Architecture

```
include/fccu/          public headers - no UI include anywhere in here
  types.hpp            Readings, Command, SensorId (shared PODs, zero deps)
  fuel_cell.hpp        polarization model (stateless -> free functions)
  tank.hpp             ideal-gas H2 tank                     (header-only)
  actuators.hpp        valve/compressor/pump/cooling, 1st-order lag
  sensors.hpp          noise + failure injection per SensorId
  plant.hpp            wires sim together, integrates pressure + temp
  state_machine.hpp    OFF->PURGE->PRESSURIZE->WARMUP->RUNNING->SHUTDOWN, FAULT
  control_loop.hpp     PID + per-state Command generation
  safety.hpp           SDC-style latching safety monitor
  datalog.hpp          RAII CSV logger                       (header-only)
  runtime.hpp          100 Hz tick + thread-safe API + Snapshot
src/                   implementations + main.cpp (HTTP/SSE/JSON boundary)
tests/                 Catch2: 182 assertions in 73 test cases
ui/static/index.html   dashboard, single file, no build step, offline
third_party/           cpp-httplib, Catch2 amalgamated (vendored)
```

Tick order each 10 ms: sensors -> safety -> FSM -> control -> plant -> log.
Safety runs on sensor readings, never plant truth - the controller only
knows what a real ECU knows.

## C++ design decisions

- **`struct Readings` + `enum class SensorId`, not string keys.** Type-safe,
  zero heap, typos fail at compile time. A constexpr pointer-to-member table
  (`READING_FIELDS`) lets the sensor suite and JSON serializer iterate the
  fields without reflection. Strings exist only at the REST boundary.
- **Ownership by value, no smart pointers.** `Simulation` owns every
  subsystem as a plain member; nothing is shared or polymorphic across a
  boundary, so there is nothing for a pointer to manage. RAII handles the
  log file and the thread.
- **Threading.** The sim ticks in a `std::jthread` on absolute deadlines:
  `sleep_until(next += period)` - `sleep_for` would accumulate scheduling
  jitter as drift, `sleep_until` re-targets an absolute deadline each tick.
  One mutex guards sim state. HTTP handlers lock only to copy a `Snapshot`
  struct or drop a command; JSON serialization and the network write happen
  outside the lock, so a slow client can stall only its own connection
  thread, never the control loop. `thread_` is the last member, so it is
  destroyed (joined) first - the thread is dead before anything it uses is
  torn down.
- **SSE, not WebSocket.** Telemetry is strictly one-way at 20 Hz and
  commands are request/response POSTs, so Server-Sent Events over
  cpp-httplib match the data flow with zero extra dependency. The browser
  `EventSource` reconnects automatically.
- **Catch2 amalgamated** (vs GoogleTest): two vendored files, no separate
  framework build; `SECTION`s map cleanly onto the FSM transition table.

## Fuel cell model

Static polarization curve from Larminie & Dicks, *Fuel Cell Systems
Explained*. Per-cell voltage at current density `i`:

```
V = E_nernst - V_act - V_ohm - V_conc

E_nernst = E0 - 0.85e-3 (T - 298.15) + (RT/2F) ln(pH2 * sqrt(pO2))
V_act    = A ln(i / i0)              activation loss (reaction kinetics)
V_ohm    = i * r(T)                  membrane resistance, falls as stack warms
V_conc   = -B ln(1 - i / i_lim)      diffusion starvation near limiting current
```

- Nernst: thermodynamic ceiling (~1.2 V), rises weakly with reactant
  pressure - where tank pressure and compressor output enter.
- Activation loss dominates at low current (steep initial drop).
- Ohmic loss is why WARMUP exists: `r(T)` runs from 0.30 ohm cm2 cold to
  0.12 ohm cm2 at 65 degC.
- Concentration loss couples actuators to performance: `i_lim` scales with
  the supply ratio, so a starved valve or lagging compressor collapses
  stack voltage.

Fuel drain is Faraday's law: `n_H2 = I N / 2F` (two electrons per H2
molecule), O2 at half that. Sizing: 96 cells, 300 cm2, ~0.65 V/cell at the
rated 330 A - about 20 kW at ~62 V.

**Sizing caveat, on purpose:** real FCEVs stack to ~200 V and boost through
a DC/DC converter into a ~600 V tractive system. This is a simplified
single low-voltage stack with the DC/DC abstracted away as the current
control loop.

## Control

| Loop | Strategy | Why |
|------|----------|-----|
| Anode pressure -> H2 valve | PI, target 2.0 bar | integrator removes consumption droop; plant gain is large (~4.5 bar/s full valve into 2 L) so kp stays small |
| Air -> compressor | feedforward from current *setpoint*, lambda 2.0 | the disturbance is known ahead of time; commanding early hides the 0.4 s spool-up lag |
| Coolant -> cooling PWM | PI, target 65 degC | classic regulation, disturbance (waste heat) varies with load |
| Demand -> current | linear map, slew-limited 150 A/s | the load must never outrun reactant supply |

Anti-windup by integrator clamping: on saturation the integral freezes at
the boundary, otherwise a long saturated phase would wind up and badly
overshoot on recovery.

## Safety

Modelled on the FS shutdown circuit: de-energized is the safe state, a
human must re-arm.

- Persistence per FS EV 5.8.7: **500 ms** for pressure/voltage/current,
  **1000 ms** for temperature. Accumulators are *leaky*, not hard-reset: a
  clean sample drains at 3x the fill rate, so a brief transient never trips
  but a marginal fault dithering across its limit through sensor noise
  (>75% of samples violated) still does. A hard reset-to-zero would let a
  real fault sitting just above a limit evade the window forever.
- Frozen-signal detection: consecutive bit-identical readings for 1 s while
  strictly inside the sensor range trip `sensor stuck: <name>`. Healthy
  channels always wiggle with noise; rail-clamped values legitimately
  repeat, so rails are exempt. Two documented blind spots: a sensor stuck
  exactly at a rail is invisible to this check, and bit-identical
  comparison only works with continuous noise - a real quantized ADC
  produces identical consecutive codes routinely, so real firmware needs a
  delta-band-over-time check instead.
- Monitored: tank overpressure (400 bar), anode overpressure (3.0 bar),
  overcurrent (360 A), undervoltage under load (0.40 V/cell), coolant
  overtemp (80 degC), tank overtemp (60 degC), plus per-sensor plausibility
  (NaN = broken wire, out of range, with 2% tolerance past the rail so
  noise near zero is not a broken wire). Startup timeouts trip it too.
- On trip: latch set, FSM forced to FAULT, hydrogen cut, cooling to 100%.
  Reset is persistence-symmetric: refused until every accumulator has
  drained to zero AND no instantaneous violation remains - one lucky
  low-noise sample cannot re-arm the system. `update()` and `reset()` share
  one `evaluate()` so they cannot disagree.
- Fail-closed: the H2 valve commands 0 the moment the anode pressure
  reading is NaN, instead of coasting open-loop on the setpoint for the
  500 ms until plausibility trips.
- Every trip records reason, value, limit, timestamp:
  `FAULT: coolant overtemp: 81.3 vs limit 80.0 at t=150.1s`. Vague fault
  reporting costs real teams days; this makes the cause readable at a glance.

## Known limitations

- Thermal mass is set low (8 kJ/K) so warm-up takes ~25 s instead of
  minutes; same dynamics, compressed timescale.
- Gas composition is not tracked: the anode is assumed pure H2, so the
  stack shows open-circuit voltage even in OFF (physically: residual H2,
  which would decay over minutes), and there is no nitrogen crossover -
  which is why RUNNING has no periodic purge cycle and the air path has no
  cathode feedback. Adding either would be dead code the plant model cannot
  exercise.
- Single lumped thermal node - stack and coolant share one temperature.
- The telemetry `overruns` counter typically shows a few percent of ticks
  missing their 10 ms deadline on Windows (scheduler granularity). The
  loop resyncs rather than burst-catching-up, and the control law always
  integrates a fixed TICK_DT, so behavior is unaffected - but sim time can
  lag wall time slightly under a coarse system timer.
- A START pressed during SHUTDOWN is deliberately discarded, not queued:
  the car must never restart itself after a commanded stop. Covered by a
  test that documents the decision.
