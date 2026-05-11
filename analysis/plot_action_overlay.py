"""Training-data action overlay: ground-truth velocity actions from CheatCode
demos, overlaid with Plan-B ACT model's predicted action chunks at the same
observations.

Output: analysis/plots/training_action_overlay.png

Reveals whether the trained model can reproduce CheatCode's action sequences
when given training-set observations. Spoiler from the data:
  - Linear dims (esp. lin_z) are consistently non-zero ~-10 mm/s; model tracks GT.
  - Angular dims are mostly zero with sparse bursts; model also predicts ~zero.

Run from /home/saivemu/code/aic-train under pixi:
  pixi run --as-is python ../aic/analysis/plot_action_overlay.py
"""

import sys
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

REPO_ID = "saivemu/aic_act_v1"
CKPT = Path("/home/saivemu/code/aic-train/outputs/train/act_aic_v1_planb/checkpoints/last/pretrained_model")
EPISODES = [0, 50, 100, 200]
N_CHUNK_OVERLAYS = 6
OUT_PNG = Path(__file__).parent / "plots" / "training_action_overlay.png"

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

ds = LeRobotDataset(repo_id=REPO_ID, episodes=EPISODES, video_backend="pyav")
print(f"Loaded {ds.num_episodes} episodes, {ds.num_frames} frames")

ep_idx_seq = [int(ds[i]["episode_index"].item()) for i in range(ds.num_frames)]
local_ranges = {}
for ep in EPISODES:
    matches = [i for i, e in enumerate(ep_idx_seq) if e == ep]
    if matches:
        local_ranges[ep] = (matches[0], matches[-1] + 1)
    else:
        local_ranges[ep] = None

fig, axes = plt.subplots(len(EPISODES), 3, figsize=(20, 4 * len(EPISODES)), squeeze=False)

for row, ep_idx in enumerate(EPISODES):
    if local_ranges.get(ep_idx) is None:
        continue
    from_idx, to_idx = local_ranges[ep_idx]
    n_frames = to_idx - from_idx

    tcp_xyz, gt_actions = [], []
    for f in range(from_idx, to_idx):
        sample = ds[f]
        tcp_xyz.append(sample["observation.state"][:3].cpu().numpy())
        gt_actions.append(sample["action"].cpu().numpy())
    tcp_xyz = np.array(tcp_xyz)
    gt_actions = np.array(gt_actions)

    chunk_t = np.linspace(0, n_frames - 1, N_CHUNK_OVERLAYS + 2)[1:-1].astype(int)
    chunk_predictions = []
    for t in chunk_t:
        sample = ds[from_idx + t]
        with torch.no_grad():
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
            chunk_norm = policy.predict_action_chunk(batch)
            chunk_unnorm = chunk_norm * action_std + action_mean
            chunk_predictions.append((t, chunk_unnorm[0].cpu().numpy()))

    # Col 1: xy trajectory
    ax = axes[row][0]
    ax.plot(tcp_xyz[:, 0], tcp_xyz[:, 1], color="k", lw=1.5, label="TCP path")
    ax.scatter(tcp_xyz[0, 0], tcp_xyz[0, 1], color="g", s=80, marker="o", label="start", zorder=5)
    ax.scatter(tcp_xyz[-1, 0], tcp_xyz[-1, 1], color="r", s=80, marker="*", label="end", zorder=5)
    path_len = np.sum(np.linalg.norm(np.diff(tcp_xyz, axis=0), axis=1))
    ax.set_title(f"Episode {ep_idx} — TCP xy path (length {path_len*1000:.1f} mm)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    # Col 2: linear action timeseries
    ax = axes[row][1]
    times = np.arange(n_frames) * 0.05
    for d, name in enumerate(["lin_x", "lin_y", "lin_z"]):
        ax.plot(times, gt_actions[:, d] * 1000, label=f"GT {name}", lw=1.2, alpha=0.9)
    cmap = plt.cm.viridis
    for idx, (t0, chunk) in enumerate(chunk_predictions):
        color = cmap(idx / max(1, len(chunk_predictions) - 1))
        chunk_t_axis = (t0 + np.arange(chunk.shape[0])) * 0.05
        for d in range(3):
            ax.plot(chunk_t_axis, chunk[:, d] * 1000, "--",
                    color=color, lw=0.8, alpha=0.55,
                    label=f"chunk@t={t0}" if d == 0 else None)
    ax.set_title("Linear action (mm/s) — solid=GT, dashed=ACT chunk")
    ax.set_xlabel("episode time [s]"); ax.set_ylabel("velocity [mm/s]")
    ax.grid(True, alpha=0.3); ax.legend(loc="upper right", fontsize=7, ncol=2)

    # Col 3: angular action timeseries
    ax = axes[row][2]
    for d, name in enumerate(["ang_x", "ang_y", "ang_z"]):
        ax.plot(times, gt_actions[:, 3 + d] * 180 / np.pi, label=f"GT {name}", lw=1.2, alpha=0.9)
    for idx, (t0, chunk) in enumerate(chunk_predictions):
        color = cmap(idx / max(1, len(chunk_predictions) - 1))
        chunk_t_axis = (t0 + np.arange(chunk.shape[0])) * 0.05
        for d in range(3, 6):
            ax.plot(chunk_t_axis, chunk[:, d] * 180 / np.pi, "--",
                    color=color, lw=0.8, alpha=0.55,
                    label=f"chunk@t={t0}" if d == 3 else None)
    ax.set_title("Angular action (deg/s) — solid=GT, dashed=ACT chunk")
    ax.set_xlabel("episode time [s]"); ax.set_ylabel("angular velocity [deg/s]")
    ax.grid(True, alpha=0.3); ax.legend(loc="upper right", fontsize=7, ncol=2)

plt.tight_layout()
plt.savefig(OUT_PNG, dpi=110, bbox_inches="tight")
print(f"Saved: {OUT_PNG}")
