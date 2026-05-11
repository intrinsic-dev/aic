"""Deploy-time diagnostic: parse [TICK] lines from a compose log and plot
where motion gets killed (or doesn't) in the deploy pipeline.

Expects a compose log produced by a RunACT.py temporarily modified to emit
per-tick `[TICK] t=... raw_lz=... tgt_z=... tcp_z=... vel_lz=... F=...`
lines. The current RunACT.py does NOT emit these by default — re-add the
log line in `insert_cable()` before rebuilding the debug image. The TICK
format expected by this parser is documented in the TICK_RE regex below.

Run from /home/saivemu/code/aic-train under pixi:
  pixi run --as-is python ../aic/analysis/plot_deploy_diagnostic.py [LOG_PATH]

Output: analysis/plots/deploy_diagnostic.png with 4 rows × N_trials cols:
  row 1: tcp_z vs tgt_z over trial-time (does target lead, does tcp follow)
  row 2: commanded action_lz vs observed tcp_lin_vel.z (does gripper accept)
  row 3: |target − tcp| deflection (impedance spring extension)
  row 4: wrist force magnitude vs baseline + backoff threshold
"""

import re
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/aic_compose_debug.log")
OUT = Path(__file__).parent / "plots" / "deploy_diagnostic.png"

TICK_RE = re.compile(
    r"\[TICK\]\s+t=(?P<t>[-\d.]+)\s+"
    r"raw_lx=(?P<raw_lx>[-+\d.]+)\s+raw_ly=(?P<raw_ly>[-+\d.]+)\s+raw_lz=(?P<raw_lz>[-+\d.]+)\s+"
    r"raw_ax=(?P<raw_ax>[-+\d.]+)\s+raw_ay=(?P<raw_ay>[-+\d.]+)\s+raw_az=(?P<raw_az>[-+\d.]+)\s+"
    r"tgt_x=(?P<tgt_x>[-+\d.]+)\s+tgt_y=(?P<tgt_y>[-+\d.]+)\s+tgt_z=(?P<tgt_z>[-+\d.]+)\s+"
    r"tcp_x=(?P<tcp_x>[-+\d.]+)\s+tcp_y=(?P<tcp_y>[-+\d.]+)\s+tcp_z=(?P<tcp_z>[-+\d.]+)\s+"
    r"vel_lx=(?P<vel_lx>[-+\d.]+)\s+vel_ly=(?P<vel_ly>[-+\d.]+)\s+vel_lz=(?P<vel_lz>[-+\d.]+)\s+"
    r"F=(?P<F>[-+\d.]+)\s+mode=(?P<mode>\w+)"
)

if not LOG.exists():
    print(f"ERROR: log file not found: {LOG}")
    print("Re-run the diagnostic compose to generate it (see module docstring).")
    sys.exit(1)

ticks = []
for line in LOG.read_text().splitlines():
    m = TICK_RE.search(line)
    if m:
        d = m.groupdict()
        for k in d:
            if k != "mode":
                d[k] = float(d[k])
        ticks.append(d)
print(f"parsed {len(ticks)} ticks")
if not ticks:
    print(f"ERROR: no [TICK] lines in {LOG}. Did the debug image's RunACT.py emit them?")
    sys.exit(1)

# Split into trials by detecting t reset (current t < previous t - 1s)
trials, current, prev_t = [], [], -1
for tk in ticks:
    if tk["t"] < prev_t - 1.0:
        if current:
            trials.append(current)
        current = []
    current.append(tk)
    prev_t = tk["t"]
if current:
    trials.append(current)
print(f"split into {len(trials)} trials with sizes {[len(t) for t in trials]}")

fig, axes = plt.subplots(4, len(trials), figsize=(6 * len(trials), 14), squeeze=False)

for col, trial in enumerate(trials):
    t = np.array([x["t"] for x in trial])
    tcp_z = np.array([x["tcp_z"] for x in trial])
    tgt_z = np.array([x["tgt_z"] for x in trial])
    raw_lz = np.array([x["raw_lz"] for x in trial])
    vel_lz = np.array([x["vel_lz"] for x in trial])
    tcp_x = np.array([x["tcp_x"] for x in trial])
    tcp_y = np.array([x["tcp_y"] for x in trial])
    tgt_x = np.array([x["tgt_x"] for x in trial])
    tgt_y = np.array([x["tgt_y"] for x in trial])
    F = np.array([x["F"] for x in trial])

    ax = axes[0][col]
    ax.plot(t, tgt_z, "r-", lw=1.5, label="target_z (commanded)")
    ax.plot(t, tcp_z, "b-", lw=1.5, label="tcp_z (observed)")
    tgt_delta = (tgt_z[-1] - tgt_z[0]) * 1000
    tcp_delta = (tcp_z[-1] - tcp_z[0]) * 1000
    ax.set_title(f"Trial {col+1}: z position\ntarget Δ {tgt_delta:+.1f}mm, tcp Δ {tcp_delta:+.1f}mm")
    ax.set_xlabel("trial time [s]"); ax.set_ylabel("z [m]")
    ax.legend(loc="best", fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[1][col]
    ax.plot(t, raw_lz * 1000, "r-", lw=1.0, alpha=0.8, label="commanded action[2] mm/s")
    ax.plot(t, vel_lz * 1000, "b-", lw=1.0, alpha=0.8, label="observed tcp_lin_vel.z mm/s")
    ax.axhline(0, color="k", lw=0.5, alpha=0.4)
    ax.set_title(f"Trial {col+1}: z velocity — cmd vs observed\n"
                 f"mean cmd={raw_lz.mean()*1000:+.2f}mm/s, mean vel={vel_lz.mean()*1000:+.2f}mm/s")
    ax.set_xlabel("trial time [s]"); ax.set_ylabel("velocity [mm/s]")
    ax.legend(loc="best", fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[2][col]
    deflection_xy = np.sqrt((tgt_x - tcp_x)**2 + (tgt_y - tcp_y)**2) * 1000
    deflection_z = (tgt_z - tcp_z) * 1000
    deflection_3d = np.sqrt((tgt_x - tcp_x)**2 + (tgt_y - tcp_y)**2 + (tgt_z - tcp_z)**2) * 1000
    ax.plot(t, deflection_3d, "k-", lw=1.5, label="|tgt-tcp| 3D (mm)")
    ax.plot(t, deflection_z, "r-", lw=1.0, alpha=0.7, label="z component (signed)")
    ax.plot(t, deflection_xy, "g-", lw=1.0, alpha=0.7, label="xy component")
    ax.axhline(20, color="orange", lw=1, ls="--", alpha=0.5, label="2cm clamp")
    ax.set_title(f"Trial {col+1}: spring deflection (target − tcp)\n"
                 f"max 3D = {deflection_3d.max():.1f}mm, mean = {deflection_3d.mean():.1f}mm")
    ax.set_xlabel("trial time [s]"); ax.set_ylabel("deflection [mm]")
    ax.legend(loc="best", fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[3][col]
    ax.plot(t, F, "k-", lw=1.0)
    baseline = F[:6].mean()
    ax.axhline(baseline, color="g", lw=1, ls="--", alpha=0.5, label=f"baseline ~{baseline:.1f}N")
    ax.axhline(baseline + 15, color="r", lw=1, ls="--", alpha=0.5, label="backoff threshold")
    ax.set_title(f"Trial {col+1}: wrist force\n"
                 f"min={F.min():.1f}N, mean={F.mean():.1f}N, max={F.max():.1f}N")
    ax.set_xlabel("trial time [s]"); ax.set_ylabel("|force| [N]")
    ax.legend(loc="best", fontsize=8); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(OUT, dpi=110, bbox_inches="tight")
print(f"saved: {OUT}")

print("\n=== per-trial summary ===")
for i, trial in enumerate(trials):
    raw_lz = np.array([x["raw_lz"] for x in trial])
    vel_lz = np.array([x["vel_lz"] for x in trial])
    tcp_z = np.array([x["tcp_z"] for x in trial])
    tgt_z = np.array([x["tgt_z"] for x in trial])
    tcp_x = np.array([x["tcp_x"] for x in trial])
    tcp_y = np.array([x["tcp_y"] for x in trial])
    F = np.array([x["F"] for x in trial])
    print(f"Trial {i+1}: {len(trial)} ticks, duration {trial[-1]['t']:.1f}s")
    print(f"  Commanded: lz mean={raw_lz.mean()*1000:+.2f}mm/s  std={raw_lz.std()*1000:.2f}")
    print(f"  Observed:  vz mean={vel_lz.mean()*1000:+.2f}mm/s  std={vel_lz.std()*1000:.2f}")
    print(f"  tcp_z delta {(tcp_z[-1]-tcp_z[0])*1000:+.2f}mm  (range [{tcp_z.min():.4f}, {tcp_z.max():.4f}])")
    print(f"  tgt_z delta {(tgt_z[-1]-tgt_z[0])*1000:+.2f}mm")
    print(f"  tcp xy cumulative path: "
          f"{np.sum(np.linalg.norm(np.diff(np.column_stack([tcp_x, tcp_y]), axis=0), axis=1))*1000:.2f}mm")
    print(f"  Force: mean={F.mean():.1f}N  max={F.max():.1f}N  baseline (first 6 ticks)={F[:6].mean():.1f}N")
