"""Phase-sliced training-data analysis + open-loop per-frame model replay.

Combines two diagnostics that interlock:

1. PHASE SLICING: detect the descent/alignment phase boundary in each CheatCode
   demo by finding where |lin_z action| smoothed over 1s first drops below 5 mm/s
   and stays there. Aggregate per-phase action statistics across many training
   episodes. Answers: "does CheatCode actively command small corrections during
   alignment, or does it go quiet and let physics finish?"

2. OPEN-LOOP REPLAY: for held-out val episodes, run policy.predict_action_chunk
   on every frame and take chunk slot 0 as the per-frame prediction. Plot against
   ground-truth action with the phase boundary marked. Answers: "where in the
   episode timeline does the model's prediction quality degrade?"

Together they isolate whether the alignment-phase failure lives in the data, the
model, or the closed-loop deploy interaction:
  - Signal exists in data, model fails to predict it    -> model-side fix
  - Signal exists in data, model predicts it correctly  -> deploy/closed-loop fix
  - Signal isn't in data                                -> data-side fix (new oracle)

Run from /home/saivemu/code/aic-train under pixi:
  pixi run --as-is python ../aic/analysis/phase_and_openloop.py

Outputs in analysis/plots/:
  - phase_action_stats.png      (training-data per-phase histograms + boundary distribution)
  - openloop_replay_grid.png    (val-episode timeseries, predicted vs GT, phase-marked)
And a console summary with phase-sliced MAE comparison.
"""

from pathlib import Path
import json
import numpy as np
import torch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from safetensors.torch import load_file
import draccus

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.configuration_act import ACTConfig

# ---------------------------------------------------------------- config
REPO_ID = "saivemu/aic_act_v1"
CKPT = Path("/home/saivemu/code/aic-train/outputs/train/act_aic_v1_planb/checkpoints/last/pretrained_model")
# Sample broadly across training data for phase stats (every 25th of 255 train eps)
TRAIN_EPISODES = list(range(0, 250, 25))
# Held-out val for open-loop replay (4 episodes spread across [255..299])
VAL_EPISODES = [255, 270, 285, 299]
OUT_DIR = Path(__file__).parent / "plots"
OUT_DIR.mkdir(exist_ok=True)

# Phase-boundary detection: |action_lz| smoothed over 1s; alignment starts where
# this stays below threshold for a sustained window.
DESCENT_LINZ_THRESHOLD_MS = 0.005    # 5 mm/s
SMOOTH_WINDOW_FRAMES = 20             # 20 Hz * 1 s = 20 frames
SUSTAIN_FRAMES = 20                   # must stay quiet for at least 1s to count

DIM_NAMES = ["lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z", "pad"]
DIM_SCALE = [1000, 1000, 1000, 180 / np.pi, 180 / np.pi, 180 / np.pi, 1.0]
DIM_UNIT = ["mm/s", "mm/s", "mm/s", "deg/s", "deg/s", "deg/s", ""]

# ---------------------------------------------------------------- setup
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

with open(CKPT / "config.json") as f:
    cfg_dict = json.load(f)
cfg_dict.pop("type", None)
cfg = draccus.decode(ACTConfig, cfg_dict)
policy = ACTPolicy(cfg)
policy.load_state_dict(load_file(CKPT / "model.safetensors"))
policy.eval().to(device)

stats = load_file(CKPT / "policy_preprocessor_step_3_normalizer_processor.safetensors")
state_mean = stats["observation.state.mean"].to(device).view(1, -1)
state_std = stats["observation.state.std"].to(device).view(1, -1)
action_mean = stats["action.mean"].to(device).view(1, -1)
action_std = stats["action.std"].to(device).view(1, -1)
img_stats = {
    cam: (
        stats[f"observation.images.{cam}_camera.mean"].to(device).view(1, 3, 1, 1),
        stats[f"observation.images.{cam}_camera.std"].to(device).view(1, 3, 1, 1),
    )
    for cam in ("left", "center", "right")
}

# Load both splits (one dataset object, episode filter spans both groups)
all_episodes = sorted(set(TRAIN_EPISODES) | set(VAL_EPISODES))
ds = LeRobotDataset(repo_id=REPO_ID, episodes=all_episodes, video_backend="pyav")
print(f"Loaded {ds.num_episodes} episodes, {ds.num_frames} frames")

# Build local frame ranges per requested episode (LeRobotDataset reindexes when
# `episodes=` is set; we need to walk and group by sample["episode_index"]).
ep_idx_seq = [int(ds[i]["episode_index"].item()) for i in range(ds.num_frames)]
local_ranges = {}
for ep in all_episodes:
    matches = [i for i, e in enumerate(ep_idx_seq) if e == ep]
    local_ranges[ep] = (matches[0], matches[-1] + 1) if matches else None


def detect_phase_boundary(actions: np.ndarray) -> int:
    """Return frame index where the descent ends and alignment begins.

    Heuristic: smooth |action[:, 2]| (lin_z magnitude) with a 1-second moving
    average, then find the first sustained-quiet region where the smoothed
    value stays below DESCENT_LINZ_THRESHOLD_MS for SUSTAIN_FRAMES consecutive
    frames. Returns the start of that region.

    If actions never quiet down, returns len(actions) (entire episode is descent).
    """
    lin_z = np.abs(actions[:, 2])
    if len(lin_z) < SMOOTH_WINDOW_FRAMES:
        return len(actions)
    kernel = np.ones(SMOOTH_WINDOW_FRAMES) / SMOOTH_WINDOW_FRAMES
    smooth = np.convolve(lin_z, kernel, mode="same")
    for i in range(len(smooth) - SUSTAIN_FRAMES):
        if np.all(smooth[i:i + SUSTAIN_FRAMES] < DESCENT_LINZ_THRESHOLD_MS):
            return i
    return len(actions)


# ---------------------------------------------------------------- (1) phase stats over training episodes
print("\n=== Phase analysis: training episodes ===")
descent_actions, align_actions, boundary_times = [], [], []
per_episode = {}
for ep in TRAIN_EPISODES:
    if local_ranges.get(ep) is None:
        continue
    from_idx, to_idx = local_ranges[ep]
    actions = np.array([ds[i]["action"].numpy() for i in range(from_idx, to_idx)])
    bdry = detect_phase_boundary(actions)
    n_frames = to_idx - from_idx
    boundary_times.append(bdry * 0.05)
    per_episode[ep] = {"n_frames": n_frames, "boundary": bdry, "actions": actions}
    descent_actions.append(actions[:bdry])
    align_actions.append(actions[bdry:])
    print(f"  ep {ep:>3}: {n_frames} frames, boundary at frame {bdry} ({bdry*0.05:.1f}s)"
          f"  descent={bdry}, alignment={n_frames - bdry}")

descent_all = np.concatenate(descent_actions, axis=0)
align_all = np.concatenate(align_actions, axis=0)
print(f"\nTotal: {len(descent_all)} descent frames, {len(align_all)} alignment frames")
print(f"Mean boundary time: {np.mean(boundary_times):.1f}s (std {np.std(boundary_times):.1f})")

print("\n--- Per-dim action stats (descent vs alignment) ---")
print(f"{'dim':>6} {'unit':>6} {'desc_mean':>10} {'desc_|q01|':>10} {'desc_|q99|':>10} {'align_mean':>11} {'align_|q01|':>11} {'align_|q99|':>11} {'|cmd|_align':>11}")
for d in range(7):
    if DIM_UNIT[d] == "":
        continue
    desc = descent_all[:, d] * DIM_SCALE[d]
    alig = align_all[:, d] * DIM_SCALE[d]
    cmd_mag_align = np.mean(np.abs(alig))
    print(f"{DIM_NAMES[d]:>6} {DIM_UNIT[d]:>6} {desc.mean():+10.3f} {np.abs(np.quantile(desc, 0.01)):10.3f} {np.abs(np.quantile(desc, 0.99)):10.3f} "
          f"{alig.mean():+11.3f} {np.abs(np.quantile(alig, 0.01)):11.3f} {np.abs(np.quantile(alig, 0.99)):11.3f} {cmd_mag_align:11.3f}")

# Phase stats plot
fig, axes = plt.subplots(2, 3, figsize=(18, 10))
fig.suptitle(f"Training-data action distribution: descent vs alignment\n"
             f"Boundary detected where smoothed |lin_z| stays < 5 mm/s for >1s\n"
             f"Sampled {len(TRAIN_EPISODES)} training episodes "
             f"({len(descent_all)} descent + {len(align_all)} alignment frames)",
             fontsize=13)
for i, d in enumerate([0, 1, 2, 3, 4, 5]):
    ax = axes[i // 3][i % 3]
    desc = descent_all[:, d] * DIM_SCALE[d]
    alig = align_all[:, d] * DIM_SCALE[d]
    # Robust range
    lo, hi = np.quantile(np.concatenate([desc, alig]), [0.01, 0.99])
    bins = np.linspace(lo, hi, 60)
    ax.hist(desc, bins=bins, alpha=0.5, label=f"descent (n={len(desc)})", color="C0", density=True)
    ax.hist(alig, bins=bins, alpha=0.5, label=f"alignment (n={len(alig)})", color="C1", density=True)
    ax.axvline(0, color="k", lw=0.5, alpha=0.5)
    ax.set_title(f"{DIM_NAMES[d]} ({DIM_UNIT[d]})")
    ax.set_xlabel(f"action [{DIM_UNIT[d]}]")
    ax.set_ylabel("density")
    ax.legend(fontsize=9, loc="best")
    ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(OUT_DIR / "phase_action_stats.png", dpi=110, bbox_inches="tight")
print(f"\nSaved: {OUT_DIR / 'phase_action_stats.png'}")

# Boundary distribution + per-episode timeline overlay
fig, axes = plt.subplots(2, 1, figsize=(14, 8))
fig.suptitle("Phase boundary distribution + per-episode |lin_z| timelines", fontsize=13)

ax = axes[0]
ax.hist(boundary_times, bins=20, edgecolor="k", alpha=0.7)
ax.axvline(np.mean(boundary_times), color="r", lw=2, ls="--",
           label=f"mean = {np.mean(boundary_times):.1f}s")
ax.set_xlabel("phase boundary time [s] (descent → alignment)")
ax.set_ylabel("# episodes")
ax.set_title(f"Per-episode boundary time (n={len(boundary_times)} sampled training episodes)")
ax.legend(); ax.grid(True, alpha=0.3)

ax = axes[1]
for ep, d in per_episode.items():
    t = np.arange(d["n_frames"]) * 0.05
    lin_z_mag = np.abs(d["actions"][:, 2]) * 1000
    ax.plot(t, lin_z_mag, lw=0.8, alpha=0.5)
    bdry_t = d["boundary"] * 0.05
    ax.scatter([bdry_t], [DESCENT_LINZ_THRESHOLD_MS * 1000], color="r", marker="v", s=40, zorder=5)
ax.axhline(DESCENT_LINZ_THRESHOLD_MS * 1000, color="r", lw=1, ls="--",
           label=f"threshold {DESCENT_LINZ_THRESHOLD_MS*1000:.1f} mm/s")
ax.set_xlabel("episode time [s]")
ax.set_ylabel("|lin_z| [mm/s]")
ax.set_title("Per-episode |lin_z| over time (red ▼ marks detected boundary)")
ax.legend(); ax.grid(True, alpha=0.3)
ax.set_ylim(0, 60)
plt.tight_layout()
plt.savefig(OUT_DIR / "phase_boundaries.png", dpi=110, bbox_inches="tight")
print(f"Saved: {OUT_DIR / 'phase_boundaries.png'}")


# ---------------------------------------------------------------- (2) open-loop replay on val episodes
def normalize_obs_sample(sample):
    state = sample["observation.state"].unsqueeze(0).to(device).float()
    state_n = (state - state_mean) / state_std
    batch = {"observation.state": state_n}
    for cam in ("left", "center", "right"):
        key = f"observation.images.{cam}_camera"
        img = sample[key].unsqueeze(0).to(device).float()
        if img.max() > 1.5:
            img = img / 255.0
        m, s = img_stats[cam]
        batch[key] = (img - m) / s
    return batch


print("\n=== Open-loop replay: val episodes ===")
val_replays = {}
for ep in VAL_EPISODES:
    if local_ranges.get(ep) is None:
        print(f"  ep {ep}: not in dataset (skipping)")
        continue
    from_idx, to_idx = local_ranges[ep]
    n_frames = to_idx - from_idx
    print(f"  ep {ep}: {n_frames} frames, running predict_action_chunk per frame...")

    gt = []
    pred = []
    for f in range(from_idx, to_idx):
        sample = ds[f]
        gt.append(sample["action"].cpu().numpy())
        with torch.no_grad():
            batch = normalize_obs_sample(sample)
            chunk_norm = policy.predict_action_chunk(batch)         # (1, 100, 7)
            chunk_unnorm = chunk_norm * action_std + action_mean    # back to physical units
            pred.append(chunk_unnorm[0, 0].cpu().numpy())            # slot 0
    gt = np.array(gt)
    pred = np.array(pred)
    bdry = detect_phase_boundary(gt)
    val_replays[ep] = {"gt": gt, "pred": pred, "boundary": bdry, "n_frames": n_frames}

    # Per-phase MAE
    if bdry > 5 and bdry < n_frames - 5:
        desc_mae = np.abs(pred[:bdry] - gt[:bdry]).mean(axis=0)
        alig_mae = np.abs(pred[bdry:] - gt[bdry:]).mean(axis=0)
        print(f"    boundary={bdry} ({bdry*0.05:.1f}s)")
        print(f"    descent MAE   (mm/s, deg/s):"
              f" lz {desc_mae[2]*1000:.2f}  ang_mean {np.mean(desc_mae[3:6])*180/np.pi:.3f}")
        print(f"    alignment MAE (mm/s, deg/s):"
              f" lz {alig_mae[2]*1000:.2f}  ang_mean {np.mean(alig_mae[3:6])*180/np.pi:.3f}")
        for d in range(6):
            scale = DIM_SCALE[d]
            print(f"      {DIM_NAMES[d]:>5}: desc_MAE={desc_mae[d]*scale:.3f}  align_MAE={alig_mae[d]*scale:.3f}"
                  f"  desc_GT_mean={gt[:bdry, d].mean()*scale:+.3f}  align_GT_mean={gt[bdry:, d].mean()*scale:+.3f}")
    else:
        print(f"    boundary at frame {bdry} — no clear alignment phase, skipping per-phase split")

# Aggregate per-phase MAE across val episodes
agg_pred = {"descent": [], "alignment": []}
agg_gt = {"descent": [], "alignment": []}
for ep, r in val_replays.items():
    bdry = r["boundary"]
    if bdry < 5 or bdry > r["n_frames"] - 5:
        continue
    agg_pred["descent"].append(r["pred"][:bdry])
    agg_pred["alignment"].append(r["pred"][bdry:])
    agg_gt["descent"].append(r["gt"][:bdry])
    agg_gt["alignment"].append(r["gt"][bdry:])
print("\n=== Aggregated per-phase MAE on val episodes ===")
for phase in ("descent", "alignment"):
    if not agg_pred[phase]:
        continue
    p = np.concatenate(agg_pred[phase], axis=0)
    g = np.concatenate(agg_gt[phase], axis=0)
    mae = np.abs(p - g).mean(axis=0)
    n = len(p)
    print(f"  {phase:>10} (n={n}):")
    for d in range(6):
        scale = DIM_SCALE[d]
        gt_mag = np.abs(g[:, d]).mean() * scale
        print(f"    {DIM_NAMES[d]:>5}: MAE={mae[d]*scale:.3f}  GT_|mean|={gt_mag:.3f}  ratio={(mae[d]*scale)/max(gt_mag, 1e-9):.2f}")

# Plot: per-val-episode predicted vs GT per dim, with phase boundary
n_val = len([e for e in VAL_EPISODES if local_ranges.get(e) is not None])
fig, axes = plt.subplots(n_val, 2, figsize=(14, 4 * n_val), squeeze=False)
fig.suptitle("Open-loop replay on val episodes: predicted (dashed) vs GT (solid)\n"
             "Red dashed line = detected descent/alignment phase boundary",
             fontsize=13)
row = 0
for ep in VAL_EPISODES:
    if local_ranges.get(ep) is None:
        continue
    r = val_replays[ep]
    t = np.arange(r["n_frames"]) * 0.05
    bdry_t = r["boundary"] * 0.05

    # Linear actions
    ax = axes[row][0]
    for d, name in enumerate(["lin_x", "lin_y", "lin_z"]):
        ax.plot(t, r["gt"][:, d] * 1000, "-", color=f"C{d}", lw=1.2, alpha=0.9, label=f"GT {name}")
        ax.plot(t, r["pred"][:, d] * 1000, "--", color=f"C{d}", lw=1.0, alpha=0.7,
                label=f"pred {name}")
    ax.axvline(bdry_t, color="r", lw=1.5, ls="--", alpha=0.6, label=f"phase boundary @ {bdry_t:.1f}s")
    ax.set_title(f"Episode {ep} — linear action (mm/s)")
    ax.set_xlabel("time [s]"); ax.set_ylabel("velocity [mm/s]")
    ax.legend(fontsize=7, ncol=2, loc="best")
    ax.grid(True, alpha=0.3)

    # Angular actions
    ax = axes[row][1]
    for d, name in enumerate(["ang_x", "ang_y", "ang_z"]):
        ax.plot(t, r["gt"][:, 3 + d] * 180 / np.pi, "-", color=f"C{d}", lw=1.2, alpha=0.9,
                label=f"GT {name}")
        ax.plot(t, r["pred"][:, 3 + d] * 180 / np.pi, "--", color=f"C{d}", lw=1.0, alpha=0.7,
                label=f"pred {name}")
    ax.axvline(bdry_t, color="r", lw=1.5, ls="--", alpha=0.6, label=f"phase boundary")
    ax.set_title(f"Episode {ep} — angular action (deg/s)")
    ax.set_xlabel("time [s]"); ax.set_ylabel("angular velocity [deg/s]")
    ax.legend(fontsize=7, ncol=2, loc="best")
    ax.grid(True, alpha=0.3)
    row += 1
plt.tight_layout()
plt.savefig(OUT_DIR / "openloop_replay_grid.png", dpi=110, bbox_inches="tight")
print(f"\nSaved: {OUT_DIR / 'openloop_replay_grid.png'}")

print("\nDone.")
