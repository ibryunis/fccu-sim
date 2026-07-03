# LinkedIn media captions

Ready-to-paste captions for each artifact. Numbers are from real runs on
this machine; regenerate with `docs/make_plots.py` after any retune.

## fault_timeline.png

FCCU sim - full fault cycle from the simulator's own 10 Hz telemetry log.
Cold start (80 A warm-up), three demand steps, then a cooling failure
injected at t~70 s. Coolant crosses the 80 degC limit at t~100 s, the
safety monitor waits out its 1 s persistence window (FS EV 5.8.7) and
latches: load and hydrogen valve cut to zero within one 10 ms tick.
FAULT-state cooling recovers the stack; the latch only releases on manual
reset after every persistence accumulator has drained.

## latch_latency.png

2000 randomized fault scenarios against the fuel cell control stack, run
in parallel (each worker owns a complete simulation). Pressure spike and
sensor disconnect latch at exactly 500 ms, stuck sensor at exactly 1000 ms
- tick-accurate to the FS rule windows across ~500 draws each. Overheat is
the physics-dominated case: 13-23 s depending on the random demand history
before the fault. 37 hours of simulated operation verified in 1.0 s of
wall time (132,000x realtime). Any failing seed reproduces the exact run.

## step_response.png

Demand tracking from a live session log: the slew-limited setpoint
(150 A/s - reactant supply must never be outrun) and delivered current
measured through the noisy sensor model. Air is feedforward from the
setpoint so the compressor spools before the load arrives; anode pressure
is a PI loop that holds 2.00 bar through every step.

## architecture.svg

The controller only sees sensor Readings and only acts through actuator
Commands - the same boundary a real ECU has. One 100 Hz jthread ticks
sensors -> safety -> FSM -> control -> plant on absolute deadlines; one
mutex, never held across I/O. Telemetry is Server-Sent Events (one-way
data flow needs no WebSocket dependency).

## state_machine.svg

Startup is sequenced by guards, not timers: purge displaces air from the
anode, pressurize exits on measured manifold pressure, warm-up on membrane
temperature - each with a timeout that fails into the latched FAULT state.
The latch mirrors the FS shutdown circuit: de-energized safe state,
hydrogen cut on entry, manual re-arm only after the cause clears.

## dashboard screenshot (take live: run.bat, START, AUTO TEST)

Live dashboard during a seeded AUTO TEST: the scenario runner drives the
sim through the same REST API an operator uses (manual controls are locked
out with HTTP 409 while it runs), checks demand tracking, injects a random
fault, verifies the latch fires inside its persistence window, and proves
reset is refused until the cause clears. Every run is reproducible from
the seed in the report.
