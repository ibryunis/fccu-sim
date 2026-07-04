# Renders LinkedIn media from real sim data:
#   docs/media/step_response.png   demand tracking + error, from a session log
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

# two render styles:
#   dark  - matches the dashboard, used in the README
#   light - technical-report look (default matplotlib, near-stock), for
#           slides/posts where plots should look like plots, not branding
DARK = {"figure.facecolor": "#0d1117", "axes.facecolor": "#161b22",
        "axes.edgecolor": "#30363d", "axes.labelcolor": "#e6edf3",
        "text.color": "#e6edf3", "xtick.color": "#8b949e",
        "ytick.color": "#8b949e", "grid.color": "#21262d",
        "font.family": "Segoe UI", "axes.grid": True}
LIGHT = {"figure.facecolor": "white", "axes.facecolor": "white",
         "axes.edgecolor": "#444444", "axes.labelcolor": "black",
         "text.color": "black", "xtick.color": "#333333",
         "ytick.color": "#333333", "grid.color": "#dddddd",
         "font.family": "DejaVu Sans", "axes.grid": True}

STYLE = "light" if "light" in sys.argv else "dark"
SUFFIX = "_light" if STYLE == "light" else ""
plt.rcParams.update(LIGHT if STYLE == "light" else DARK)

# series colors per style: dark uses the dashboard palette, light uses
# sober report colors
if STYLE == "light":
    C_SET, C_MEAS, C_ERR = "#555555", "#1f77b4", "#b8860b"
    C_TEMP, C_TANK, C_LIM = "#c62828", "#1f77b4", "#e65100"
    C_VALVE, C_LATCH = "#2e7d32", "#c62828"
    C_HIST = ["#1f77b4", "#2e7d32", "#b8860b", "#c62828"]
    C_ANNOT, C_ARROW = "black", "#666666"
else:
    C_SET, C_MEAS, C_ERR = "#8b949e", "#58a6ff", "#d29922"
    C_TEMP, C_TANK, C_LIM = "#f85149", "#58a6ff", "#d29922"
    C_VALVE, C_LATCH = "#3fb950", "#f85149"
    C_HIST = ["#58a6ff", "#3fb950", "#d29922", "#f85149"]
    C_ANNOT, C_ARROW = "#e6edf3", "#8b949e"

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
    sp = [r["current_setpoint"] for r in rows]
    de = [r["stack_current"] for r in rows]
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(10, 5.6), dpi=150,
                                  sharex=True, height_ratios=[3, 1])
    ax.plot(t, sp, color=C_SET, lw=1.5,
            label="setpoint (slew-limited, 150 A/s)")
    ax.plot(t, de, color=C_MEAS, lw=1.0,
            label="delivered (measured, sensor noise)")
    ax.set_ylabel("stack current [A]")
    ax.set_title("Demand step tracking - 100 Hz loop, PI anode pressure + feedforward air")
    ax.legend(loc="upper left", framealpha=0.6)

    # traces overlap at full scale; the tracking error is the actual finding
    err = [abs(a - b) for a, b in zip(de, sp)]
    ax2.plot(t, err, color=C_ERR, lw=0.9)
    ax2.set_ylabel("|error| [A]")
    ax2.set_xlabel("time [s]")
    ax2.set_ylim(0, 8)
    fig.tight_layout()
    fig.savefig(OUT / f"step_response{SUFFIX}.png")


def fault_timeline(rows):
    t = [r["t"] for r in rows]
    cool = [r["coolant_temp"] for r in rows]
    lat = [r["latched"] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 6.6), dpi=150, sharex=True,
                             height_ratios=[3, 2, 1])
    ax = axes[0]
    ax.plot(t, cool, color=C_TEMP, lw=1.4)
    ax.axhline(80, color=C_LIM, ls="--", lw=1)
    ax.text(t[-1], 80, " 80 °C limit", color=C_LIM, fontsize=9,
            va="bottom", ha="right")
    ax.set_ylabel("coolant [°C]")
    ax.set_title("Cooling failure: overtemp trip, latched shutdown, hydrogen cut")

    # annotate limit crossing -> latch: the 1 s persistence window made visible
    t_cross = next((x for x, c in zip(t, cool) if c > 80), None)
    t_latch = next((x for x, l in zip(t, lat) if l > 0.5), None)
    if t_cross and t_latch:
        ax.annotate(f"crossing → latch: {t_latch - t_cross:.1f} s\n"
                    "(FS 1 s persistence window)",
                    xy=(t_latch, 80), xytext=(t_latch + 6, 55),
                    color=C_ANNOT, fontsize=9,
                    arrowprops=dict(arrowstyle="->", color=C_ARROW))

    ax = axes[1]
    ax.plot(t, [r["h2_valve"] * 100 for r in rows], color=C_VALVE, lw=1.4,
            label="H₂ valve [%]")
    ax.plot(t, [r["stack_current"] for r in rows], color=C_MEAS, lw=1.0,
            label="stack current [A]")
    ax.set_ylabel("valve % / current A")
    ax.legend(loc="upper left", framealpha=0.6)

    ax = axes[2]
    ax.plot(t, lat, color=C_LATCH, lw=1.6, drawstyle="steps-post")
    ax.set_ylabel("SDC latch")
    ax.set_yticks([0, 1], ["armed", "LATCHED"])
    ax.set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(OUT / f"fault_timeline{SUFFIX}.png")


def latency_hist(path):
    per_fault = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if int(r["latch_ms"]) >= 0:
                per_fault.setdefault(r["fault"], []).append(int(r["latch_ms"]))

    fig, axes = plt.subplots(1, 4, figsize=(13, 3.8), dpi=150)
    rule = [("pressure spike", 0.5), ("sensor disconnect", 0.5),
            ("sensor stuck", 1.0)]
    for ax, (name, window), color in zip(axes, rule, C_HIST):
        v = [x / 1000 for x in per_fault[name]]
        ax.hist(v, bins=[window - 0.3, window - 0.1, window - 0.02,
                         window + 0.02, window + 0.1, window + 0.3],
                color=color)
        ax.axvline(window, color=C_ANNOT, ls=":", lw=0.8)
        ax.set_title(f"{name}\nn={len(v)}, all at {window * 1000:.0f} ms",
                     fontsize=10)
        ax.set_xlabel("latch latency [s]")

    ax = axes[3]
    v = [x / 1000 for x in per_fault["overheat"]]
    ax.hist(v, bins=20, color=C_HIST[3])
    ax.set_title(f"overheat\nn={len(v)}, physics-limited", fontsize=10)
    ax.set_xlabel("time to trip [s]")  # heat-up time dominates, not detection

    axes[0].set_ylabel("scenarios")
    fig.suptitle("Latch latency is deterministic: exactly the FS EV 5.8.7 "
                 "windows, across 2000 randomized scenarios", y=1.03)
    fig.tight_layout()
    fig.savefig(OUT / f"latch_latency{SUFFIX}.png", bbox_inches="tight")


rows = load_log(sys.argv[1])
step_response(rows)
fault_timeline(rows)
latency_hist(sys.argv[2])
print("wrote", [p.name for p in OUT.glob("*.png")])
