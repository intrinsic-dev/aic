#!/usr/bin/env python3
"""
Training scaffold for DiffusionPolicy — 6D rotation action representation.

Identical to train_diffusion.py except:
  - tcp_pose.orientation quaternion (4D) is converted to the 6D continuous
    rotation representation (Zhou et al., 2019).
  - action_dim is therefore 9  (3 position + 6D rotation) instead of 7.
  - Quaternion validity is checked at dataset load time.

The quaternion_to_matrix and matrix_to_rotation_6d functions are ported
directly from pytorch3d/transforms/rotation_conversions.py (no pytorch3d
dependency required).

6D rotation reference:
    "On the Continuity of Rotation Representations in Neural Networks"
    Zhou et al., CVPR 2019.  https://arxiv.org/abs/1812.07035

Usage:
    pixi run python3 aic_training/train_diffusion_rot6d.py \
        --dataset_dirs /path/to/dataset1 /path/to/dataset2 \
        --output_dir outputs/diffusion_rot6d
"""

import argparse
import sys
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


# ---------------------------------------------------------------------------
# Rotation conversion utilities
# (ported from pytorch3d/transforms/rotation_conversions.py, no dependency)
# ---------------------------------------------------------------------------

def quaternion_to_matrix(quaternions: torch.Tensor) -> torch.Tensor:
    """Convert quaternions to rotation matrices.

    Matches pytorch3d.transforms.quaternion_to_matrix exactly.

    Args:
        quaternions: (..., 4) — real part first, i.e. (w, x, y, z)

    Returns:
        rotation matrices: (..., 3, 3)
    """
    if quaternions.shape[-1] != 4:
        raise ValueError("quaternions must have last dimension 4")

    r, i, j, k = torch.unbind(quaternions, dim=-1)   # w, x, y, z

    # two_s = 2 / |q|²  (= 2 for unit quaternions)
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

    Takes the first two columns of each rotation matrix and flattens them.
    Matches pytorch3d.transforms.matrix_to_rotation_6d exactly.

    Reference: Zhou et al. "On the Continuity of Rotation Representations
    in Neural Networks", CVPR 2019. https://arxiv.org/abs/1812.07035

    Args:
        matrix: (..., 3, 3)

    Returns:
        6D vectors: (..., 6) — [R[:,0], R[:,1]] flattened row-major
    """
    batch_dim = matrix.shape[:-2]
    return matrix[..., :2, :].clone().reshape(batch_dim + (6,))

from aic_training.diffusion_policy import (
    DiffusionPolicy,
    TOKEN_DIM,
    ROBOT_STATE_DIM,
)


# ---------------------------------------------------------------------------
# DDPM noise schedule  (unchanged)
# ---------------------------------------------------------------------------

class DDPMSchedule:
    """Linear beta schedule and forward-process utilities (q(x_t | x_0)).

    Reference: Ho et al. "Denoising Diffusion Probabilistic Models" (2020).
    """

    def __init__(
        self,
        num_timesteps: int = 50,
        beta_start: float = 1e-4,
        beta_end: float = 0.02,
        device: torch.device = torch.device("cpu"),
    ):
        self.num_timesteps = num_timesteps

        betas = torch.linspace(beta_start, beta_end, num_timesteps, device=device)
        alphas = 1.0 - betas
        alpha_bar = torch.cumprod(alphas, dim=0)          # ᾱ_t

        self.betas     = betas
        self.alpha_bar = alpha_bar                        # (T,)

    def to(self, device):
        self.betas     = self.betas.to(device)
        self.alpha_bar = self.alpha_bar.to(device)
        return self

    def add_noise(
        self,
        x0: torch.Tensor,
        noise: torch.Tensor,
        t: torch.Tensor,
    ) -> torch.Tensor:
        """Sample q(x_t | x_0) = sqrt(ᾱ_t) * x0 + sqrt(1 - ᾱ_t) * noise."""
        ab = self.alpha_bar[t]                            # (B,)
        ab = ab[:, None, None]                            # broadcast over horizon & dim
        return ab.sqrt() * x0 + (1.0 - ab).sqrt() * noise


# ---------------------------------------------------------------------------
# Dataset wrapper
# ---------------------------------------------------------------------------

# observation.state column layout (43 dims, from info.json)
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

# Columns fed into the policy as robot_state (ROBOT_STATE_DIM=20):
#   tcp_pose(7) + joint_positions(7) + wrench.force(3) + wrench.torque(3)
_ROBOT_STATE_COLS = list(range(0, 7)) + list(range(19, 26)) + list(range(26, 32))

_COL_EPISODE_NUMBER  = _SIDX["episode_number"]     # 42
_COL_INSERTION_EVENT = _SIDX["insertion_event"]    # 39
_COL_PORT_NAME       = _SIDX["task.port_name"]     # 41
_COL_TARGET_MODULE   = _SIDX["task.target_module"] # 40

# Quaternion columns in the state vector: (qx, qy, qz, qw)
_COL_QUAT_XYZW = slice(3, 7)

QUAT_NORM_TOL = 1e-3   # frames whose quaternion norm deviates more than this are flagged

CAMERA_KEYS = ("left_camera", "center_camera", "right_camera")


def _quat_xyzw_to_rot6d(quat_xyzw: np.ndarray) -> torch.Tensor:
    """Convert an array of (qx, qy, qz, qw) quaternions to 6D rotation vectors.

    Args:
        quat_xyzw: (N, 4) float32 — quaternions in (x, y, z, w) order
    Returns:
        rot6d: (N, 6) float32 — first two columns of the rotation matrix, flattened
    """
    # pytorch3d uses (w, x, y, z) convention
    quat_wxyz = np.concatenate([quat_xyzw[:, 3:4], quat_xyzw[:, :3]], axis=1)
    quat_t = torch.from_numpy(quat_wxyz.astype(np.float32))  # (N, 4)
    R = quaternion_to_matrix(quat_t)                          # (N, 3, 3)
    return matrix_to_rotation_6d(R)                           # (N, 6)


class AICLeRobotDataset(torch.utils.data.Dataset):
    """Wraps a single LeRobot dataset directory for diffusion policy training.

    Identical to AICLeRobotDataset in train_diffusion.py except the action
    representation uses 6D rotations instead of quaternions:

        action: (pred_horizon, 9)  — tcp_pose [x, y, z, r6d_0..r6d_5]

    The 6D rotation is the first two columns of the SO(3) rotation matrix
    (Zhou et al. 2019), converted from the recorded quaternion via pytorch3d.

    Quaternion validity is checked at load time: frames whose orientation
    quaternion has norm outside [1 − QUAT_NORM_TOL, 1 + QUAT_NORM_TOL] are
    flagged with a warning.

    Chunking logic (unchanged):
      - Rows are grouped into episode chunks by changes in episode_number.
      - The last chunk is always discarded (may be incomplete).
      - Chunks where task.port_name != 2 and insertion_event never reaches 2
        are discarded as failed episodes.
      - Valid chunks are sliced into sliding windows of length
        obs_horizon + pred_horizon.
    """

    def __init__(
        self,
        dataset_dir: str | Path,
        obs_horizon: int = 2,
        pred_horizon: int = 16,
        action_dim: int = 9,   # 3 position + 6D rotation
    ):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self.obs_horizon  = obs_horizon
        self.pred_horizon = pred_horizon
        self.action_dim   = action_dim
        self._window_len  = obs_horizon + pred_horizon

        self._lerobot = LeRobotDataset(str(dataset_dir))

        state_arr, global_indices = self._load_state()
        self._global_indices = global_indices    # (N,) — maps local row → global frame idx
        self._state_arr      = state_arr         # (N, 43)

        self._windows = self._build_windows(state_arr)
        print(
            f"[AICLeRobotDataset] {Path(dataset_dir).name}: "
            f"{len(self._windows)} windows from {state_arr.shape[0]} frames"
        )

    # ------------------------------------------------------------------
    # Data loading helpers
    # ------------------------------------------------------------------

    def _load_state(self):
        """Read state array and global indices; validate orientation quaternions."""
        hf = self._lerobot.hf_dataset
        state_arr      = np.stack(hf["observation.state"]).astype(np.float32)
        global_indices = np.array(hf["index"], dtype=np.int64)

        # Check tcp_pose.orientation is a valid unit quaternion for every frame.
        # Columns 3:7 are (qx, qy, qz, qw).
        quats = state_arr[:, _COL_QUAT_XYZW]           # (N, 4)
        norms = np.linalg.norm(quats, axis=1)           # (N,)
        bad   = np.abs(norms - 1.0) > QUAT_NORM_TOL
        n_bad = int(bad.sum())
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
        """Return list of valid window start positions (local indices).

        Rules:
          1. Split rows into chunks wherever episode_number changes.
          2. Always discard the last chunk (may be incomplete).
          3. Discard chunks where port_name != 2 and insertion_event never reaches 2.
          4. Slide a window of length obs_horizon + pred_horizon over each kept chunk.
        """
        ep_num = state_arr[:, _COL_EPISODE_NUMBER]

        changes   = np.where(np.diff(ep_num) != 0)[0] + 1
        ep_starts = np.concatenate([[0], changes])
        ep_ends   = np.concatenate([changes, [len(ep_num)]])
        num_episodes = len(ep_starts)

        windows: list[int] = []
        for i in range(num_episodes - 1):
            s, e = int(ep_starts[i]), int(ep_ends[i])
            chunk = state_arr[s:e]

            port_name     = int(round(chunk[0, _COL_PORT_NAME]))
            max_insertion = float(chunk[:, _COL_INSERTION_EVENT].max())
            if port_name != 2 and max_insertion < 2:
                continue

            chunk_len = e - s
            for w in range(chunk_len - self._window_len + 1):
                windows.append(s + w)

        return windows

    # ------------------------------------------------------------------
    # Dataset interface
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._windows)

    def __getitem__(self, idx: int) -> dict:
        local_start = self._windows[idx]
        local_end   = local_start + self._window_len

        gidx = self._global_indices[local_start:local_end]  # (window_len,)

        # --- Robot state: first obs_horizon frames ---
        obs_state   = self._state_arr[local_start : local_start + self.obs_horizon]
        robot_state = torch.from_numpy(obs_state[:, _ROBOT_STATE_COLS])  # (obs_horizon, 20)

        target_module = torch.tensor(
            int(round(obs_state[0, _COL_TARGET_MODULE])), dtype=torch.int64
        )
        port_name = torch.tensor(
            int(round(obs_state[0, _COL_PORT_NAME])), dtype=torch.int64
        )

        # --- Actions: tcp_pose for pred_horizon frames after the obs window ---
        pred_state = self._state_arr[
            local_start + self.obs_horizon : local_start + self.obs_horizon + self.pred_horizon
        ]                                                   # (pred_horizon, 43)

        # Position: cols 0:3  →  (pred_horizon, 3)
        pos = torch.from_numpy(pred_state[:, :3].astype(np.float32))

        # Orientation: cols 3:7 are (qx, qy, qz, qw) → 6D rotation (pred_horizon, 6)
        rot6d = _quat_xyzw_to_rot6d(pred_state[:, 3:7])    # (pred_horizon, 6)

        action = torch.cat([pos, rot6d], dim=-1)            # (pred_horizon, 9)

        # --- Images: obs_horizon frames ---
        images: dict[str, torch.Tensor] = {k: [] for k in CAMERA_KEYS}
        for gi in gidx[: self.obs_horizon]:
            frame = self._lerobot[int(gi)]
            for k in CAMERA_KEYS:
                img = frame[f"observation.images.{k}"]
                images[k].append(img)
        images = {k: torch.stack(v) for k, v in images.items()}  # (obs_horizon, C, H, W)

        episode_number = torch.tensor(
            int(round(obs_state[0, _COL_EPISODE_NUMBER])), dtype=torch.int64
        )

        return {
            "images":         images,
            "robot_state":    robot_state,
            "target_module":  target_module,
            "port_name":      port_name,
            "action":         action,           # (pred_horizon, 9)
            "episode_number": episode_number,
        }


def build_dataloader(
    dataset_dirs: list[str | Path],
    obs_horizon: int,
    pred_horizon: int,
    action_dim: int,
    batch_size: int,
    num_workers: int = 4,
) -> DataLoader:
    """Concatenate multiple LeRobot datasets and return a DataLoader."""
    datasets = [
        AICLeRobotDataset(d, obs_horizon=obs_horizon, pred_horizon=pred_horizon, action_dim=action_dim)
        for d in dataset_dirs
    ]
    combined = ConcatDataset(datasets)
    return DataLoader(
        combined,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        pin_memory=True,
        drop_last=True,
    )


# ---------------------------------------------------------------------------
# Training step  (unchanged)
# ---------------------------------------------------------------------------

def ddpm_loss(
    policy: DiffusionPolicy,
    schedule: DDPMSchedule,
    batch: dict,
    device: torch.device,
) -> torch.Tensor:
    """Single DDPM training step: sample t and noise, compute MSE loss."""
    action = batch["action"].to(device)               # (B, pred_horizon, action_dim)
    B = action.shape[0]

    t     = torch.randint(0, schedule.num_timesteps, (B,), device=device)
    noise = torch.randn_like(action)
    noisy_action = schedule.add_noise(action, noise, t)

    images        = {k: v.to(device) for k, v in batch["images"].items()}
    robot_state   = batch["robot_state"].to(device)
    target_module = batch["target_module"].to(device)
    port_name     = batch["port_name"].to(device)

    noise_pred = policy(
        images=images,
        robot_state=robot_state,
        target_module=target_module,
        port_name=port_name,
        noisy_action=noisy_action,
        timestep=t,
    )

    return nn.functional.mse_loss(noise_pred, noise)


# ---------------------------------------------------------------------------
# Main training loop  (unchanged except action_dim default)
# ---------------------------------------------------------------------------

def train(args):
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Training on {device}")

    policy = DiffusionPolicy(
        obs_horizon=args.obs_horizon,
        action_dim=args.action_dim,
        robot_state_dim=ROBOT_STATE_DIM,
        d_model=TOKEN_DIM,
        n_heads=args.n_heads,
        n_layers=args.n_layers,
        ffn_dim=args.ffn_dim,
        dropout=args.dropout,
        cfg_dropout_prob=args.cfg_dropout_prob,
    ).to(device)

    for p in policy.image_encoder.parameters():
        p.requires_grad = False

    total     = sum(p.numel() for p in policy.parameters())
    trainable = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"\nModel parameters: {total:,} total, {trainable:,} trainable")
    for name, mod in policy.named_children():
        n = sum(p.numel() for p in mod.parameters())
        print(f"  {name:<30} {n:>12,}")

    schedule  = DDPMSchedule(num_timesteps=args.num_timesteps, device=device)
    optimizer = torch.optim.AdamW(policy.parameters(), lr=args.lr, weight_decay=1e-4)

    loader = build_dataloader(
        dataset_dirs=args.dataset_dirs,
        obs_horizon=args.obs_horizon,
        pred_horizon=args.pred_horizon,
        action_dim=args.action_dim,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
    )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    policy.train()
    global_step = 0
    step_losses: list[float] = []
    epoch_losses: list[float] = []

    for epoch in range(args.num_epochs):
        epoch_loss = 0.0

        for batch in loader:
            optimizer.zero_grad()
            loss = ddpm_loss(policy, schedule, batch, device)
            loss.backward()
            nn.utils.clip_grad_norm_(policy.parameters(), max_norm=1.0)
            optimizer.step()

            step_losses.append(loss.item())
            epoch_loss += loss.item()
            global_step += 1

            if global_step % args.log_every == 0:
                print(f"  step {global_step:6d}  loss={loss.item():.4f}")

        avg_loss = epoch_loss / max(len(loader), 1)
        epoch_losses.append(avg_loss)
        print(f"Epoch {epoch + 1}/{args.num_epochs}  avg_loss={avg_loss:.4f}")

        if (epoch + 1) % args.save_every == 0:
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
        dataset_dirs=args.dataset_dirs,
        obs_horizon=args.obs_horizon,
        pred_horizon=args.pred_horizon,
        action_dim=args.action_dim,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
    )
    for batch in loader:
        print(batch)
        break


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Train DiffusionPolicy with 6D rotation action representation"
    )

    # Data
    p.add_argument("--dataset_dirs", nargs="+", required=True,
                   help="Paths to one or more LeRobot dataset directories")
    p.add_argument("--output_dir", default="outputs/diffusion_rot6d",
                   help="Directory for checkpoints and logs")

    # Sequence lengths
    p.add_argument("--obs_horizon",  type=int, default=2)
    p.add_argument("--pred_horizon", type=int, default=8)
    p.add_argument("--action_dim",   type=int, default=9,
                   help="Action dimension: 3 position + 6D rotation = 9")

    # Diffusion
    p.add_argument("--num_timesteps", type=int, default=50)

    # Model
    p.add_argument("--n_heads",          type=int,   default=8)
    p.add_argument("--n_layers",         type=int,   default=4)
    p.add_argument("--ffn_dim",          type=int,   default=1024)
    p.add_argument("--dropout",          type=float, default=0.0)
    p.add_argument("--cfg_dropout_prob", type=float, default=0.1,
                   help="Probability of replacing task token with null token during training (CFG dropout)")

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
