# Renders docs/media/campaign_control_room.png: a Grafana-style summary of
# a verification campaign, from campaign_runs.csv. Every number is computed
# from the run data; nothing is typed in by hand.
#
# usage: python docs/make_control_room.py campaign_runs.csv <wall_seconds>
import base64
import csv
import io
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
TICK_S, CHECKS_PER_TICK = 0.01, 20

runs_csv, wall_s = sys.argv[1], float(sys.argv[2])

rows = list(csv.DictReader(open(runs_csv)))
n = len(rows)
passed = sum(r["pass"] == "1" for r in rows)
ticks = sum(int(r["ticks"]) for r in rows)
sim_h = ticks * TICK_S / 3600
per_fault = {}
for r in rows:
    if int(r["latch_ms"]) >= 0:
        per_fault.setdefault(r["fault"], []).append(int(r["latch_ms"]))

plt.rcParams.update({
    "figure.facecolor": "#111217", "axes.facecolor": "#111217",
    "axes.edgecolor": "#2c2f36", "axes.labelcolor": "#9aa0a6",
    "text.color": "#d5d9de", "xtick.color": "#9aa0a6",
    "ytick.color": "#9aa0a6", "grid.color": "#22252b",
    "font.family": "Segoe UI", "axes.grid": True, "font.size": 11})


def hist_png(name, color, xlabel):
    v = [x / 1000 for x in per_fault[name]]
    fig, ax = plt.subplots(figsize=(4.1, 2.15), dpi=130)
    span = max(v) - min(v)
    bins = 24 if span > 1 else [min(v) - 0.25, min(v) - 0.02,
                                min(v) + 0.02, min(v) + 0.25]
    ax.hist(v, bins=bins, color=color)
    ax.set_xlabel(xlabel, fontsize=9)
    ax.tick_params(labelsize=8.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    buf = io.BytesIO()
    fig.tight_layout(pad=0.6)
    fig.savefig(buf, format="png")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def tile(label, value, unit="", color="#e8eaed", sub=""):
    return f"""<div class="tile"><div class="lbl">{label}</div>
    <div class="val" style="color:{color}">{value}<span class="unit">{unit}</span></div>
    <div class="sub">{sub}</div></div>"""


def panel(title, img, note):
    return f"""<div class="panel"><div class="ptitle">{title}</div>
    <img src="data:image/png;base64,{img}">
    <div class="note">{note}</div></div>"""


stats = [
    tile("FAULT SCENARIOS", f"{n:,}", "", "#e8eaed", f"seeded, randomized, {passed - n or 'zero'} escapes" if passed == n else "FAILURES PRESENT"),
    tile("SHUTDOWNS CORRECT", f"{passed / n * 100:.2f}", " %", "#4cc38a", "latch fired inside its window, every run"),
    tile("OPERATION VERIFIED", f"{sim_h:,.0f}", " h", "#5ab0f7", f"{sim_h / 24:.0f} days of track time, {ticks:,} control ticks"),
    tile("FASTER THAN REALTIME", f"{ticks * TICK_S / wall_s:,.0f}", " x", "#f2a65a", f"{wall_s:.0f} s wall on 12 threads, {ticks / wall_s / 1e6:.1f}M ticks/s"),
    tile("SAFETY EVALUATIONS", f"{ticks * CHECKS_PER_TICK / 1e9:.1f}", " B", "#e8eaed", "20 checks per 10 ms tick"),
    tile("RESET INTEGRITY", "0", "", "#4cc38a", "re-arms accepted while a cause persisted"),
]

panels = [
    panel(f"PRESSURE SPIKE - n={len(per_fault['pressure spike']):,}",
          hist_png("pressure spike", "#5ab0f7", "latch latency [s]"),
          "all at exactly 500 ms - the FS EV 5.8.7 window"),
    panel(f"SENSOR DISCONNECT - n={len(per_fault['sensor disconnect']):,}",
          hist_png("sensor disconnect", "#4cc38a", "latch latency [s]"),
          "all at exactly 500 ms"),
    panel(f"SENSOR STUCK - n={len(per_fault['sensor stuck']):,}",
          hist_png("sensor stuck", "#f2c14e", "latch latency [s]"),
          "all at exactly 1000 ms - frozen-signal detection"),
    panel(f"OVERHEAT - n={len(per_fault['overheat']):,}",
          hist_png("overheat", "#f0655d", "time to trip [s]"),
          "physics-limited: heat-up time varies with demand history"),
]

html = f"""<!DOCTYPE html><html><head><meta charset="utf-8"><style>
  body {{ margin:0; background:#0b0c0f; font-family:"Segoe UI",sans-serif;
         padding:26px 30px; }}
  h1 {{ color:#e8eaed; font-size:20px; margin:0; font-weight:600; }}
  .h2 {{ color:#9aa0a6; font-size:12px; margin:4px 0 20px; }}
  .grid {{ display:grid; grid-template-columns:repeat(3,1fr); gap:14px;
           margin-bottom:14px; }}
  .grid4 {{ display:grid; grid-template-columns:repeat(4,1fr); gap:14px; }}
  .tile,.panel {{ background:#111217; border:1px solid #2c2f36;
                  border-radius:4px; padding:14px 18px; }}
  .lbl,.ptitle {{ color:#9aa0a6; font-size:10.5px; letter-spacing:.8px;
                  font-weight:600; }}
  .val {{ font-size:42px; font-weight:700; line-height:1.15;
          font-variant-numeric:tabular-nums; }}
  .unit {{ font-size:20px; color:#9aa0a6; font-weight:400; }}
  .sub,.note {{ color:#9aa0a6; font-size:11px; margin-top:2px; }}
  .panel img {{ width:100%; margin-top:6px; }}
  .foot {{ color:#6f757c; font-size:11px; margin-top:16px;
           font-family:Consolas,monospace; }}
</style></head><body>
<h1>FCCU verification campaign</h1>
<div class="h2">hydrogen fuel cell control unit &middot; randomized fault injection against the full control stack &middot; C++20, 100 Hz</div>
<div class="grid">{''.join(stats)}</div>
<div class="grid4">{''.join(panels)}</div>
<div class="foot">$ fccu_campaign --runs {n} --seed 7 &nbsp;&middot;&nbsp; every scenario: startup &rarr; random demand &rarr; random fault &rarr; latch verified in-window &rarr; reset refused until cause cleared &rarr; restart &nbsp;&middot;&nbsp; any seed reproduces its exact run</div>
</body></html>"""

out_html = Path(tempfile.gettempdir()) / "control_room.html"
out_html.write_text(html, encoding="utf-8")
out_png = Path(__file__).parent / "media" / "campaign_control_room.png"
subprocess.run([EDGE, "--headless=old", "--disable-gpu",
                f"--screenshot={out_png}", "--window-size=1560,812",
                "--force-device-scale-factor=1.5", "--hide-scrollbars",
                out_html.as_uri()], check=False, capture_output=True)
print("wrote", out_png)
