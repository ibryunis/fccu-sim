# LinkedIn media captions

Numbers come from real runs on this machine; regenerate with
docs/make_plots.py and a fresh campaign after any retune.

## dashboard_fault.png

Dashboard at the moment the shutdown circuit latches. Cooling was
blocked at t~150 s; coolant crossed the 80 C limit, the monitor waited
out the 1 s persistence window and cut load and hydrogen in one 10 ms
tick. The banner carries the full fault record: reason, measured value,
limit, timestamp. Reset stays refused until the cause clears.

## campaign_summary.png

Console output of the verification campaign: 2000 randomized fault
scenarios in parallel on 12 workers, all passed. 37.3 h of simulated
operation in 1.1 s wall time (125,572x realtime). Every scenario checks
demand tracking, latch timing against the FS windows, refused reset
while the cause persists, and a clean restart. A failing seed
reproduces the exact run.

## fault_timeline.png

Full fault cycle from the simulator's own 10 Hz telemetry log. Cold
start with 80 A warm-up, three demand steps, then a cooling failure.
The annotation shows the crossing-to-latch gap: the FS 1 s persistence
window as measured from the log. Load and hydrogen valve cut to zero at
the latch; FAULT-state cooling recovers the stack; the latch releases
only on manual reset after every accumulator has drained.

## latch_latency.png

2000 randomized fault scenarios, each worker owning a full simulation.
Pressure spike and sensor disconnect latch at exactly 500 ms; stuck
sensor at exactly 1000 ms, tick-accurate to the FS rule windows across
~500 draws each. Overheat is physics-dominated: 13-23 s to reach the
limit depending on demand history, which is why its panel shows time to
trip, not detection latency.

## step_response.png

Demand tracking from a live session log. Top: slew-limited setpoint
(150 A/s, reactant supply must never be outrun) and delivered current
measured through the noisy sensor model. Bottom: tracking error stays
within a few amps of sensor noise except during commanded ramps. Air is
feedforward from the setpoint so the compressor spools before the load
arrives; anode pressure is a PI loop holding 2.00 bar through every
step.

## architecture.png

The controller only sees sensor Readings and only acts through actuator
Commands, the same boundary a real ECU has. One 100 Hz jthread ticks
sensors, safety, state machine, control, plant on absolute deadlines;
one mutex, never held across I/O. Telemetry is Server-Sent Events:
one-way data flow needs no WebSocket dependency.

## state_machine.png

Startup is sequenced by guards, not timers: purge displaces air from
the anode, pressurize exits on measured manifold pressure, warm-up on
membrane temperature, each with a timeout that fails into the latched
FAULT state. The latch mirrors the FS shutdown circuit: de-energized
safe state, hydrogen cut on entry, manual re-arm only after the cause
clears and every persistence accumulator has drained.

## dashboard_running.png

Steady state at 65% demand: 214 A delivered against a 214.5 A setpoint,
73 V, 15.7 kW, coolant held at 65 C by the cooling PI. The footer shows
loop health: period p50 exactly 10.0 ms after raising the Windows timer
resolution to 1 ms.
