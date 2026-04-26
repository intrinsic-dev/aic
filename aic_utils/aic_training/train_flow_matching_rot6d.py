#!/usr/bin/env python3
"""
Training scaffold for FlowMatchingPolicy — 6D rotation action representation.

Identical to train_diffusion_rot6d.py except:
  - Uses FlowMatchingPolicy (flow_matching.py) instead of DiffusionPolicy.
  - No noise schedule: the rectified flow loss is self-contained in
    policy.compute_loss() — samples t ~ U(0,1), builds the linear
    interpolation x_t = (1-t)*x_noise + t*x_1, and trains v_θ to predict
    x_1 - x_noise.
  - No --num_timesteps CLI argument.

Dataset and action representation are unchanged:
  - tcp_pose.orientation quaternion converted to 6D rotation (action_dim=9).
  - Quaternion validity checked at load time.

Usage:
    pixi run python3 aic_training/train_flow_matching_rot6d.py \
        --dataset_dirs /path/to/dataset1 /path/to/dataset2 \
        --output_dir outputs/flow_matching_rot6d
"""

import argparse
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless backend — no display required
import matplotlib.pyplot as plt

_aic_utils = str(Path(__file__).resolve().parent.parent)
if _aic_utils not in sys.path:
    sys.path.insert(0, _aic_utils)

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, ConcatDataset

from aic_training.flow_matching import (
    FlowMatchingPolicy,
    TOKEN_DIM,
    ROBOT_STATE_DIM,
)


# ---------------------------------------------------------------------------
# Rotation conversion utilities
# (ported from pytorch3d/transforms/rotation_conversions.py, no dependency)
# ---------------------------------------------------------------------------

def quaternion_to_matrix(quaternions: torch.Tensor) -> torch.Tensor:
    """Convert quaternions to rotation matrices.

    Args:
        quaternions: (..., 4) — real part first, i.e. (w, x, y, z)
    Returns:
        rotation matrices: (..., 3, 3)
    """
    if quaternions.shape[-1] != 4:
        raise ValueError("quaternions must have last dimension 4")

    r, i, j, k = torch.unbind(quaternions, dim=-1)   # w, x, y, z

    two_s = torch.div(2.0, (quaternions * quaternions).sum(dim=-1))

    o = torch.stack([
        1 - two_s * (j * j + k * k),
        two_s * (i * j - k * r),
        two_s * (i * k + j * r),
        two_s * (i * j + k * r),
        1 - two_s * (i * i + k * k),
        two_s * (j * k - i * r),
        two_s * (i * k - j * r),
        two_s * (j * k + i * r),
        1 - two_s * (i * i + j * j),
    ], dim=-1)

    return o.reshape(quaternions.shape[:-1] + (3, 3))


def matrix_to_rotation_6d(matrix: torch.Tensor) -> torch.Tensor:
    """Convert rotation matrices to the 6D continuous representation.

    Reference: Zhou et al. "On the Continuity of Rotation Representations
    in Neural Networks", CVPR 2019.

    Args:
        matrix: (..., 3, 3)
    Returns:
        6D vectors: (..., 6)
    """
    batch_dim = matrix.shape[:-2]
    return matrix[..., :2, :].clone().reshape(batch_dim + (6,))


# ---------------------------------------------------------------------------
# Dataset wrapper  (identical to train_diffusion_rot6d.py)
# ---------------------------------------------------------------------------

_STATE_NAMES = [
    "tcp_pose.position.x", "tcp_pose.position.y", "tcp_pose.position.z",
    "tcp_pose.orientation.x", "tcp_pose.orientation.y", "tcp_pose.orientation.z", "tcp_pose.orientation.w",
    "tcp_velocity.linear.x", "tcp_velocity.linear.y", "tcp_velocity.linear.z",
    "tcp_velocity.angular.x", "tcp_velocity.angular.y", "tcp_velocity.angular.z",
    "tcp_error.x", "tcp_error.y", "tcp_error.z", "tcp_error.rx", "tcp_error.ry", "tcp_error.rz",
    "joint_positions.0", "joint_positions.1", "joint_positions.2", "joint_positions.3",
    "joint_positions.4", "joint_positions.5", "joint_positions.6",
    "wrench.force.x", "wrench.force.y", "wrench.force.z",
    "wrench.torque.x", "wrench.torque.y", "wrench.torque.z",
    "fts_tare_offset.force.x", "fts_tare_offset.force.y", "fts_tare_offset.force.z",
    "fts_tare_offset.torque.x", "fts_tare_offset.torque.y", "fts_tare_offset.torque.z",
    "max_force_magnitude",
    "insertion_event", "task.target_module", "task.port_name", "episode_number",
]
_SIDX = {name: i for i, name in enumerate(_STATE_NAMES)}

_ROBOT_STATE_COLS = list(range(0, 7)) + list(range(19, 26)) + list(range(26, 32))

_COL_EPISODE_NUMBER  = _SIDX["episode_number"]
_COL_INSERTION_EVENT = _SIDX["insertion_event"]
_COL_PORT_NAME       = _SIDX["task.port_name"]
_COL_TARGET_MODULE   = _SIDX["task.target_module"]

_COL_QUAT_XYZW = slice(3, 7)
QUAT_NORM_TOL  = 1e-3

CAMERA_KEYS = ("left_camera", "center_camera", "right_camera")


def _quat_xyzw_to_rot6d(quat_xyzw: np.ndarray) -> torch.Tensor:
    quat_wxyz = np.concatenate([quat_xyzw[:, 3:4], quat_xyzw[:, :3]], axis=1)
    quat_t = torch.from_numpy(quat_wxyz.astype(np.float32))
    R = quaternion_to_matrix(quat_t)
    return matrix_to_rotation_6d(R)


class AICLeRobotDataset(torch.utils.data.Dataset):
    """Wraps a single LeRobot dataset for flow matching training.

    Action representation: (pred_horizon, 9) — [x, y, z, r6d_0..r6d_5]
    Quaternion validity is checked at load time.
    """

    def __init__(self, dataset_dir: str | Path, obs_horizon: int = 2,
                 pred_horizon: int = 16, action_dim: int = 9):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self.obs_horizon  = obs_horizon
        self.pred_horizon = pred_horizon
        self.action_dim   = action_dim
        self._window_len  = obs_horizon + pred_horizon

        self._lerobot = LeRobotDataset(str(dataset_dir))

        state_arr, global_indices = self._load_state()
        self._global_indices = global_indices
        self._state_arr      = state_arr

        self._windows = self._build_windows(state_arr)
        print(
            f"[AICLeRobotDataset] {Path(dataset_dir).name}: "
            f"{len(self._windows)} windows from {state_arr.shape[0]} frames"
        )

    def _load_state(self):
        hf = self._lerobot.hf_dataset
        state_arr      = np.stack(hf["observation.state"]).astype(np.float32)
        global_indices = np.array(hf["index"], dtype=np.int64)

        quats  = state_arr[:, _COL_QUAT_XYZW]
        norms  = np.linalg.norm(quats, axis=1)
        bad    = np.abs(norms - 1.0) > QUAT_NORM_TOL
        n_bad  = int(bad.sum())
        if n_bad > 0:
            bad_indices = np.where(bad)[0]
            print(
                f"[AICLeRobotDataset] WARNING: {n_bad}/{len(state_arr)} frames have "
                f"non-unit tcp_pose.orientation (|norm-1| > {QUAT_NORM_TOL}). "
                f"First 5 bad local indices: {bad_indices[:5].tolist()}, "
                f"norms: {norms[bad_indices[:5]].tolist()}"
            )
        else:
            print(
                f"[AICLeRobotDataset] All {len(state_arr)} quaternions are valid "
                f"(norm within {QUAT_NORM_TOL} of 1.0)."
            )
        return state_arr, global_indices

    def _build_windows(self, state_arr: np.ndarray) -> list[int]:
        ep_num = state_arr[:, _COL_EPISODE_NUMBER]
        changes   = np.where(np.diff(ep_num) != 0)[0] + 1
        ep_starts = np.concatenate([[0], changes])
        ep_ends   = np.concatenate([changes, [len(ep_num)]])

        windows: list[int] = []
        for i in range(len(ep_starts) - 1):
            s, e = int(ep_starts[i]), int(ep_ends[i])
            chunk = state_arr[s:e]
            port_name     = int(round(chunk[0, _COL_PORT_NAME]))
            max_insertion = float(chunk[:, _COL_INSERTION_EVENT].max())
            if port_name != 2 and max_insertion < 2:
                continue
            for w in range((e - s) - self._window_len + 1):
                windows.append(s + w)
        return windows

    def __len__(self) -> int:
        return len(self._windows)

    def __getitem__(self, idx: int) -> dict:
        local_start = self._windows[idx]
        gidx = self._global_indices[local_start : local_start + self._window_len]

        obs_state   = self._state_arr[local_start : local_start + self.obs_horizon]
        robot_state = torch.from_numpy(obs_state[:, _ROBOT_STATE_COLS])

        target_module = torch.tensor(int(round(obs_state[0, _COL_TARGET_MODULE])), dtype=torch.int64)
        port_name     = torch.tensor(int(round(obs_state[0, _COL_PORT_NAME])),     dtype=torch.int64)

        pred_state = self._state_arr[
            local_start + self.obs_horizon : local_start + self.obs_horizon + self.pred_horizon
        ]
        pos   = torch.from_numpy(pred_state[:, :3].astype(np.float32))
        rot6d = _quat_xyzw_to_rot6d(pred_state[:, 3:7])
        action = torch.cat([pos, rot6d], dim=-1)            # (pred_horizon, 9)

        images: dict[str, torch.Tensor] = {k: [] for k in CAMERA_KEYS}
        for gi in gidx[: self.obs_horizon]:
            frame = self._lerobot[int(gi)]
            for k in CAMERA_KEYS:
                images[k].append(frame[f"observation.images.{k}"])
        images = {k: torch.stack(v) for k, v in images.items()}

        episode_number = torch.tensor(int(round(obs_state[0, _COL_EPISODE_NUMBER])), dtype=torch.int64)

        return {
            "images":         images,
            "robot_state":    robot_state,
            "target_module":  target_module,
            "port_name":      port_name,
            "action":         action,
            "episode_number": episode_number,
        }


def build_dataloader(dataset_dirs, obs_horizon, pred_horizon, action_dim,
                     batch_size, num_workers=4) -> DataLoader:
    datasets = [
        AICLeRobotDataset(d, obs_horizon=obs_horizon, pred_horizon=pred_horizon,
                          action_dim=action_dim)
        for d in dataset_dirs
    ]
    return DataLoader(
        ConcatDataset(datasets),
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        pin_memory=True,
        drop_last=True,
    )


# ---------------------------------------------------------------------------
# Training step
# ---------------------------------------------------------------------------

def flow_matching_loss(
    policy: FlowMatchingPolicy,
    batch: dict,
    device: torch.device,
) -> torch.Tensor:
    """Thin wrapper around policy.compute_loss() for the training loop."""
    return policy.compute_loss(
        images        = {k: v.to(device) for k, v in batch["images"].items()},
        robot_state   = batch["robot_state"].to(device),
        target_module = batch["target_module"].to(device),
        port_name     = batch["port_name"].to(device),
        actions       = batch["action"].to(device),
    )


# ---------------------------------------------------------------------------
# Main training loop
# ---------------------------------------------------------------------------

def train(args):
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Training on {device}")

    policy = FlowMatchingPolicy(
        obs_horizon      = args.obs_horizon,
        action_dim       = args.action_dim,
        robot_state_dim  = ROBOT_STATE_DIM,
        d_model          = TOKEN_DIM,
        n_heads          = args.n_heads,
        n_layers         = args.n_layers,
        ffn_dim          = args.ffn_dim,
        dropout          = args.dropout,
        cfg_dropout_prob = args.cfg_dropout_prob,
    ).to(device)

    for p in policy.image_encoder.parameters():
        p.requires_grad = False

    total     = sum(p.numel() for p in policy.parameters())
    trainable = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"\nModel parameters: {total:,} total, {trainable:,} trainable")
    for name, mod in policy.named_children():
        n = sum(p.numel() for p in mod.parameters())
        print(f"  {name:<30} {n:>12,}")

    optimizer = torch.optim.AdamW(policy.parameters(), lr=args.lr, weight_decay=1e-4)

    loader = build_dataloader(
        dataset_dirs = args.dataset_dirs,
        obs_horizon  = args.obs_horizon,
        pred_horizon = args.pred_horizon,
        action_dim   = args.action_dim,
        batch_size   = args.batch_size,
        num_workers  = args.num_workers,
    )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    policy.train()
    global_step = 0
    step_losses: list[float] = []
    epoch_losses: list[float] = []

    for epoch in range(args.num_epochs):
        epoch_loss = 0.0
        epoch_start = time.perf_counter()

        for batch in loader:
            optimizer.zero_grad()
            loss = flow_matching_loss(policy, batch, device)
            loss.backward()
            nn.utils.clip_grad_norm_(policy.parameters(), max_norm=1.0)
            optimizer.step()

            step_losses.append(loss.item())
            epoch_loss += loss.item()
            global_step += 1

            if global_step % args.log_every == 0:
                print(f"  step {global_step:6d}  loss={loss.item():.4f}")

        avg_loss    = epoch_loss / max(len(loader), 1)
        elapsed_sec = time.perf_counter() - epoch_start
        epoch_losses.append(avg_loss)
        print(f"Epoch {epoch + 1}/{args.num_epochs}  avg_loss={avg_loss:.4f}  time={elapsed_sec:.1f}s")

        is_last_epoch = (epoch + 1) == args.num_epochs
        if (epoch + 1) % args.save_every == 0 or is_last_epoch:
            ckpt_path = output_dir / f"checkpoint_epoch{epoch + 1:04d}.pt"
            torch.save({
                "epoch":        epoch + 1,
                "global_step":  global_step,
                "model_state":  policy.state_dict(),
                "optim_state":  optimizer.state_dict(),
                "args":         vars(args),
            }, ckpt_path)
            print(f"  Saved checkpoint: {ckpt_path}")

    print("Training complete.")

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    axes[0].plot(step_losses, linewidth=0.8, alpha=0.8)
    axes[0].set_xlabel("Step")
    axes[0].set_ylabel("Loss")
    axes[0].set_title("Training loss (per step)")
    axes[0].grid(True, linewidth=0.5)
    axes[1].plot(range(1, len(epoch_losses) + 1), epoch_losses, marker="o", markersize=3)
    axes[1].set_xlabel("Epoch")
    axes[1].set_ylabel("Avg loss")
    axes[1].set_title("Training loss (per epoch average)")
    axes[1].grid(True, linewidth=0.5)
    fig.tight_layout()
    plot_path = output_dir / "loss_curve.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"Loss curve saved to {plot_path}")


def debug_dataset(args):
    loader = build_dataloader(
        dataset_dirs = args.dataset_dirs,
        obs_horizon  = args.obs_horizon,
        pred_horizon = args.pred_horizon,
        action_dim   = args.action_dim,
        batch_size   = args.batch_size,
        num_workers  = args.num_workers,
    )
    for batch in loader:
        print(batch)
        break


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Train FlowMatchingPolicy with 6D rotation action representation"
    )

    # Data
    p.add_argument("--dataset_dirs", nargs="+", required=True,
                   help="Paths to one or more LeRobot dataset directories")
    p.add_argument("--output_dir", default="outputs/flow_matching_rot6d",
                   help="Directory for checkpoints and logs")

    # Sequence lengths
    p.add_argument("--obs_horizon",  type=int, default=2)
    p.add_argument("--pred_horizon", type=int, default=8)
    p.add_argument("--action_dim",   type=int, default=9,
                   help="Action dimension: 3 position + 6D rotation = 9")

    # Model
    p.add_argument("--n_heads",          type=int,   default=8)
    p.add_argument("--n_layers",         type=int,   default=4)
    p.add_argument("--ffn_dim",          type=int,   default=1024)
    p.add_argument("--dropout",          type=float, default=0.0)
    p.add_argument("--cfg_dropout_prob", type=float, default=0.1,
                   help="Probability of replacing task token with null token (CFG dropout)")

    # Training
    p.add_argument("--num_epochs", type=int,   default=100)
    p.add_argument("--batch_size", type=int,   default=32)
    p.add_argument("--lr",         type=float, default=1e-4)
    p.add_argument("--num_workers",type=int,   default=4)
    p.add_argument("--device",     default="cuda")
    p.add_argument("--log_every",  type=int,   default=100)
    p.add_argument("--save_every", type=int,   default=10)

    return p.parse_args()


if __name__ == "__main__":
    #debug_dataset(parse_args())
    train(parse_args())
