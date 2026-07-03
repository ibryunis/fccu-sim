# Renders LinkedIn media from real sim data:
#   docs/media/step_response.png   demand tracking, from a logged live session
#   docs/media/fault_timeline.png  overheat -> latch -> H2 cut, same session
#   docs/media/latch_latency.png   histograms from a 2000-run campaign CSV
#
# usage: python docs/make_plots.py <telemetry_log.csv> <campaign_runs.csv>
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DARK = {"figure.facecolor": "#0d1117", "axes.facecolor": "#161b22",
        "axes.edgecolor": "#30363d", "axes.labelcolor": "#e6edf3",
        "text.color": "#e6edf3", "xtick.color": "#8b949e",
        "ytick.color": "#8b949e", "grid.color": "#21262d",
        "font.family": "Segoe UI", "axes.grid": True}
plt.rcParams.update(DARK)

OUT = Path(__file__).parent / "media"
OUT.mkdir(exist_ok=True)


def load_log(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in r:
            if k != "state":
                r[k] = float(r[k])
    return rows


def step_response(rows):
    t = [r["t"] for r in rows]
    fig, ax = plt.subplots(figsize=(10, 4.5), dpi=150)
    ax.plot(t, [r["current_setpoint"] for r in rows], color="#8b949e",
            lw=1.5, label="current setpoint (slew-limited, 150 A/s)")
    ax.plot(t, [r["stack_current"] for r in rows], color="#58a6ff",
            lw=1.2, label="delivered current (measured, with sensor noise)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("stack current [A]")
    ax.set_title("Demand step tracking - 100 Hz control loop, PI anode pressure + feedforward air")
    ax.legend(loc="upper left", framealpha=0.2)
    fig.tight_layout()
    fig.savefig(OUT / "step_response.png")


def fault_timeline(rows):
    t = [r["t"] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), dpi=150, sharex=True)
    ax = axes[0]
    ax.plot(t, [r["coolant_temp"] for r in rows], color="#f85149", lw=1.4)
    ax.axhline(80, color="#d29922", ls="--", lw=1, label="80 degC limit (1 s persistence)")
    ax.set_ylabel("coolant [degC]")
    ax.legend(loc="upper left", framealpha=0.2)
    ax.set_title("Cooling failure: overtemp trip, latched shutdown, hydrogen cut")

    ax = axes[1]
    ax.plot(t, [r["h2_valve"] * 100 for r in rows], color="#3fb950", lw=1.4,
            label="H2 valve [%]")
    ax.plot(t, [r["stack_current"] for r in rows], color="#58a6ff", lw=1.0,
            label="stack current [A]")
    ax.set_ylabel("valve % / current A")
    ax.legend(loc="upper left", framealpha=0.2)

    ax = axes[2]
    ax.plot(t, [r["latched"] for r in rows], color="#f85149", lw=1.6,
            drawstyle="steps-post")
    ax.set_ylabel("SDC latch")
    ax.set_yticks([0, 1], ["armed", "LATCHED"])
    ax.set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(OUT / "fault_timeline.png")


def latency_hist(path):
    per_fault = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if int(r["latch_ms"]) >= 0:
                per_fault.setdefault(r["fault"], []).append(int(r["latch_ms"]))
    fig, axes = plt.subplots(1, len(per_fault), figsize=(13, 3.6), dpi=150)
    order = ["pressure spike", "sensor disconnect", "sensor stuck", "overheat"]
    colors = ["#58a6ff", "#3fb950", "#d29922", "#f85149"]
    for ax, name, color in zip(axes, order, colors):
        v = per_fault[name]
        ax.hist([x / 1000 for x in v], bins=24, color=color)
        ax.set_title(f"{name}\nn={len(v)}", fontsize=10)
        ax.set_xlabel("latch latency [s]")
    axes[0].set_ylabel("scenarios")
    fig.suptitle("SDC latch latency, 2000 randomized fault scenarios "
                 "(FS EV 5.8.7 windows: 500 ms / 1 s)", y=1.02)
    fig.tight_layout()
    fig.savefig(OUT / "latch_latency.png", bbox_inches="tight")


rows = load_log(sys.argv[1])
step_response(rows)
fault_timeline(rows)
latency_hist(sys.argv[2])
print("wrote", [p.name for p in OUT.glob("*.png")])
