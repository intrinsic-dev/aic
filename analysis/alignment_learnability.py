"""Test whether the alignment-phase xy signal is *learnable* or *aleatoric noise*.

For each val alignment-phase frame, find K nearest training alignment frames
in normalized observation space. Compute the standard deviation of GT actions
across those neighbors. That standard deviation is the lower bound on MAE for
any model — even one that perfectly predicts the conditional mean given that
observation. If neighbor std ≈ GT action magnitude, the alignment xy signal
is dominated by aleatoric noise (CheatCode's own variability) and no choice of
loss function can recover it from this data.

This script reads the LeRobot parquet files directly (state + action only) so
it doesn't pay for AV1 video decoding on 180k frames. Should finish in <1 min.

Run from /home/saivemu/code/aic-train under pixi:
  pixi run --as-is python ../aic/analysis/alignment_learnability.py

Output: analysis/plots/alignment_learnability.png + console summary.
"""

import glob
from pathlib import Path
import numpy as np
import pyarrow.parquet as pq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors

# ---------------------------------------------------------------- config
DATA_DIR = Path("/home/saivemu/.cache/huggingface/lerobot/saivemu/aic_act_v1/data")
TRAIN_EPISODES = set(range(0, 255))
VAL_EPISODES = set(range(255, 300))

# Phase boundary detection
DESCENT_LINZ_THRESHOLD_MS = 0.005      # 5 mm/s
SMOOTH_WINDOW_FRAMES = 20               # 1 second @ 20Hz
SUSTAIN_FRAMES = 20

# k-NN params: K neighbors per query
K = 10
N_TRAIN_BANK = 60000                    # subsample train alignment frames
N_VAL_QUERIES = 5000                    # subsample val alignment queries
RNG = np.random.default_rng(0)

OUT_PNG = Path(__file__).parent / "plots" / "alignment_learnability.png"

DIM_NAMES = ["lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z", "pad"]
DIM_SCALE = [1000, 1000, 1000, 180/np.pi, 180/np.pi, 180/np.pi, 1.0]
DIM_UNIT = ["mm/s", "mm/s", "mm/s", "deg/s", "deg/s", "deg/s", ""]


def detect_phase_boundary(actions: np.ndarray) -> int:
    lin_z = np.abs(actions[:, 2])
    if len(lin_z) < SMOOTH_WINDOW_FRAMES:
        return len(actions)
    kernel = np.ones(SMOOTH_WINDOW_FRAMES) / SMOOTH_WINDOW_FRAMES
    smooth = np.convolve(lin_z, kernel, mode="same")
    for i in range(len(smooth) - SUSTAIN_FRAMES):
        if np.all(smooth[i:i + SUSTAIN_FRAMES] < DESCENT_LINZ_THRESHOLD_MS):
            return i
    return len(actions)


# ---------------------------------------------------------------- load parquet
print("Loading parquet files...")
files = sorted(glob.glob(str(DATA_DIR / "chunk-000" / "*.parquet")))
print(f"  {len(files)} files")

tables = [pq.read_table(f, columns=["episode_index", "frame_index", "observation.state", "action"])
          for f in files]
table = tables[0]
for t in tables[1:]:
    import pyarrow as pa
    table = pa.concat_tables([table, t])

ep_idx = np.asarray(table.column("episode_index"))
frame_idx = np.asarray(table.column("frame_index"))
# observation.state and action come back as list[float], length 26 and 7
states = np.array(table.column("observation.state").to_pylist(), dtype=np.float32)
actions = np.array(table.column("action").to_pylist(), dtype=np.float32)
print(f"  total frames: {len(states)}  states shape: {states.shape}  actions shape: {actions.shape}")

# ---------------------------------------------------------------- per-episode phase boundaries
print("Detecting per-episode phase boundaries...")
alignment_mask = np.zeros(len(states), dtype=bool)
unique_eps, ep_first = np.unique(ep_idx, return_index=True)
ep_boundaries = list(zip(ep_first, np.append(ep_first[1:], len(states))))
ep_to_range = dict(zip(unique_eps, ep_boundaries))

for ep, (from_i, to_i) in ep_to_range.items():
    bdry = detect_phase_boundary(actions[from_i:to_i])
    alignment_mask[from_i + bdry:to_i] = True

# Split train/val alignment masks
train_mask = np.array([ep in TRAIN_EPISODES for ep in ep_idx])
val_mask = np.array([ep in VAL_EPISODES for ep in ep_idx])
train_align = np.where(train_mask & alignment_mask)[0]
val_align = np.where(val_mask & alignment_mask)[0]
print(f"  train alignment frames: {len(train_align)}")
print(f"  val   alignment frames: {len(val_align)}")

# Subsample
n_bank = min(N_TRAIN_BANK, len(train_align))
n_queries = min(N_VAL_QUERIES, len(val_align))
bank = RNG.choice(train_align, size=n_bank, replace=False)
queries = RNG.choice(val_align, size=n_queries, replace=False)
print(f"  bank: {n_bank} train frames | queries: {n_queries} val frames")

# ---------------------------------------------------------------- k-NN over normalized state
print(f"Building k-NN (K={K})...")
state_std = states.std(axis=0)
state_std[state_std < 1e-6] = 1.0
states_n = states / state_std

nn = NearestNeighbors(n_neighbors=K, algorithm="auto", n_jobs=-1)
nn.fit(states_n[bank])
print("Querying val alignment frames...")
_, neighbor_pos = nn.kneighbors(states_n[queries])
neighbor_actions = actions[bank[neighbor_pos]]              # (n_queries, K, 7)
query_gt = actions[queries]                                  # (n_queries, 7)
neighbor_std = neighbor_actions.std(axis=1)                  # (n_queries, 7)
neighbor_mean = neighbor_actions.mean(axis=1)
mean_residual = np.abs(query_gt - neighbor_mean)             # (n_queries, 7)

# ---------------------------------------------------------------- report
print("\n=== Alignment-phase learnability per dim ===\n")
print(f"{'dim':>6} {'unit':>6} {'GT_|mean|':>10} {'neigh_std':>10} {'NB-std/|GT|':>12} {'cond-mean MAE':>15} {'CMM/|GT|':>10}")
print(f"{'':>6} {'':>6} {'(scaled)':>10} {'(scaled)':>10} {'ratio':>12} {'(scaled)':>15} {'ratio':>10}")
for d in range(7):
    if DIM_UNIT[d] == "":
        continue
    scale = DIM_SCALE[d]
    gt_mag = np.abs(query_gt[:, d]).mean() * scale
    ns = neighbor_std[:, d].mean() * scale
    cmm = mean_residual[:, d].mean() * scale
    print(f"{DIM_NAMES[d]:>6} {DIM_UNIT[d]:>6} {gt_mag:>10.3f} {ns:>10.3f} "
          f"{ns/max(gt_mag, 1e-9):>12.2f} {cmm:>15.3f} {cmm/max(gt_mag, 1e-9):>10.2f}")
print()
print("Reading:")
print("  GT_|mean|       = mean |GT action| on val alignment queries (physical units)")
print("  neigh_std       = std of GT actions across K nearest train neighbors per query")
print("                    = noise floor; no model can do better than this on MAE")
print("  NB-std/|GT|     = if ≈1, neighbor actions are as variable as the GT magnitude itself →")
print("                    aleatoric noise dominates; no learnable signal")
print("  cond-mean MAE   = MAE of using the neighbor-mean as the prediction (best perfect-model MAE)")
print("  CMM/|GT|        = if ≈1, even a perfect conditional-mean predictor fails to match the magnitude")


# Compare to Plan B's measured per-dim alignment MAE/GT ratio (from phase_and_openloop.py)
plan_b_alignment_mae = {
    "lin_x":  0.626,    # MAE in mm/s
    "lin_y":  0.651,
    "lin_z":  2.814,
    "ang_x":  0.013,    # MAE in deg/s
    "ang_y":  0.002,
    "ang_z":  0.004,
}
plan_b_gt_mag = {
    "lin_x":  0.653,
    "lin_y":  0.724,
    "lin_z":  9.272,
    "ang_x":  0.034,
    "ang_y":  0.005,
    "ang_z":  0.017,
}

print("\n=== Comparison: Plan B's actual alignment MAE vs the noise floor we just measured ===")
print(f"{'dim':>6} {'unit':>6} {'plan_b_MAE':>10} {'noise_floor':>12} {'plan_b/floor':>13} {'closable gap':>14}")
for d, name in enumerate(["lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"]):
    scale = DIM_SCALE[d]
    pb = plan_b_alignment_mae[name]
    floor = neighbor_std[:, d].mean() * scale
    # The closable gap is how much we COULD potentially improve if we hit the noise floor
    gap = pb - floor
    print(f"{name:>6} {DIM_UNIT[d]:>6} {pb:>10.3f} {floor:>12.3f} {pb/max(floor, 1e-9):>13.2f}x {gap:>14.3f}")
print()
print("If plan_b/floor ≈ 1: Plan B is already at the noise floor; no better model can help")
print("If plan_b/floor >> 1: there's headroom; a better model (e.g. L2 loss) could reduce MAE by closable_gap")


# ---------------------------------------------------------------- plot
fig, axes = plt.subplots(2, 3, figsize=(18, 10))
fig.suptitle("Alignment-phase signal vs noise floor (per dim)\n"
             "Each point: a val alignment frame.  X = |GT action|.  Y = std of GT actions across "
             f"K={K} nearest train neighbors.\n"
             f"y=x line: above means neighbor variance dominates → aleatoric noise, no learnable signal.\n"
             f"Bank: {n_bank} train frames, queries: {n_queries} val frames.",
             fontsize=12)

for i, d in enumerate([0, 1, 2, 3, 4, 5]):
    ax = axes[i // 3][i % 3]
    scale = DIM_SCALE[d]
    gt_abs = np.abs(query_gt[:, d]) * scale
    ns = neighbor_std[:, d] * scale
    ax.scatter(gt_abs, ns, s=2, alpha=0.3, color="C0")
    max_val = max(gt_abs.max(), ns.max()) * 1.05
    ax.plot([0, max_val], [0, max_val], "r--", lw=1, label="neighbor-std = |GT|")
    ax.scatter([gt_abs.mean()], [ns.mean()], color="red", s=120, marker="X",
               label=f"mean: |GT|={gt_abs.mean():.3f}, nb-std={ns.mean():.3f}", zorder=10)
    ratio = ns.mean()/max(gt_abs.mean(), 1e-9)
    verdict = "NOISE FLOOR" if ratio > 0.9 else ("partial signal" if ratio > 0.5 else "LEARNABLE SIGNAL")
    ax.set_title(f"{DIM_NAMES[d]} ({DIM_UNIT[d]}) — NB-std/|GT| = {ratio:.2f}  [{verdict}]",
                 fontsize=11)
    ax.set_xlabel(f"|GT action| [{DIM_UNIT[d]}]")
    ax.set_ylabel(f"neighbor std [{DIM_UNIT[d]}]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, max_val)
    ax.set_ylim(0, max_val)

plt.tight_layout()
plt.savefig(OUT_PNG, dpi=110, bbox_inches="tight")
print(f"\nSaved: {OUT_PNG}")
