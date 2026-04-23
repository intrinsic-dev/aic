#!/usr/bin/env python3
"""
Training scaffold for DiffusionPolicy on AIC cable-insertion datasets.

High-level flow:
  1. Load one or more LeRobot datasets from disk.
  2. Build a DataLoader that yields observation/action batches.
  3. Train the noise prediction network with the DDPM objective:
       loss = MSE(noise_pred, noise)
     where noisy_action = sqrt(alpha_bar) * action + sqrt(1 - alpha_bar) * noise.

Usage:
    pixi run python3 aic_training/train_diffusion.py \
        --dataset_dirs /path/to/dataset1 /path/to/dataset2 \
        --output_dir outputs/diffusion
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

from aic_training.diffusion_policy import (
    DiffusionPolicy,
    TOKEN_DIM,
    ROBOT_STATE_DIM,
)


# ---------------------------------------------------------------------------
# DDPM noise schedule
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

        # Register as plain tensors (not nn.Parameters) — move to device manually
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
        """Sample q(x_t | x_0) = sqrt(ᾱ_t) * x0 + sqrt(1 - ᾱ_t) * noise.

        Args:
            x0:    (B, pred_horizon, action_dim) — clean action
            noise: (B, pred_horizon, action_dim) — standard Gaussian noise
            t:     (B,) int64 — diffusion timestep indices

        Returns:
            noisy_action: (B, pred_horizon, action_dim)
        """
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

_COL_EPISODE_NUMBER  = _SIDX["episode_number"]    # 42
_COL_INSERTION_EVENT = _SIDX["insertion_event"]   # 39
_COL_PORT_NAME       = _SIDX["task.port_name"]    # 41
_COL_TARGET_MODULE   = _SIDX["task.target_module"] # 40

CAMERA_KEYS = ("left_camera", "center_camera", "right_camera")


class AICLeRobotDataset(torch.utils.data.Dataset):
    """Wraps a single LeRobot dataset directory for diffusion policy training.

    Chunking logic:
      - Rows are grouped into episode chunks by changes in observation.state.episode_number.
      - The last chunk is always discarded (may be incomplete).
      - Chunks where task.port_name != 2 (SFP task) and insertion_event never reaches 2
        (no successful insertion) are discarded as failed episodes.
      - Valid chunks are sliced into sliding windows of length obs_horizon + pred_horizon.

    Each sample contains:
        images:        dict {camera_key: (obs_horizon, C, H, W)}  float32 [0, 1]
        robot_state:   (obs_horizon, ROBOT_STATE_DIM)              float32
        target_module: ()  int64
        port_name:     ()  int64
        action:        (pred_horizon, 7)                           float32  tcp_pose [x,y,z,qx,qy,qz,qw]
    """

    def __init__(
        self,
        dataset_dir: str | Path,
        obs_horizon: int = 2,
        pred_horizon: int = 16,
        action_dim: int = 7,
    ):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self.obs_horizon  = obs_horizon
        self.pred_horizon = pred_horizon
        self.action_dim   = action_dim
        self._window_len  = obs_horizon + pred_horizon

        # LeRobot dataset handles image/video decoding; indexed by global frame index.
        self._lerobot = LeRobotDataset(str(dataset_dir))

        # Load all state arrays and global indices from parquet for fast chunking.
        state_arr, global_indices = self._load_state()
        self._global_indices = global_indices    # shape (N,) — maps local row → global frame idx
        self._state_arr      = state_arr         # shape (N, 43)

        # Build list of valid window start positions (local indices into state_arr).
        self._windows = self._build_windows(state_arr)
        print(
            f"[AICLeRobotDataset] {Path(dataset_dir).name}: "
            f"{len(self._windows)} windows from {state_arr.shape[0]} frames"
        )

    # ------------------------------------------------------------------
    # Data loading helpers
    # ------------------------------------------------------------------

    def _load_state(self):
        """Read state array and global indices from the already-loaded lerobot dataset."""
        hf = self._lerobot.hf_dataset
        state_arr      = np.stack(hf["observation.state"]).astype(np.float32)
        global_indices = np.array(hf["index"], dtype=np.int64)
        return state_arr, global_indices

    def _build_windows(self, state_arr: np.ndarray) -> list[int]:
        """Return list of valid window start positions (local indices).

        Rules:
          1. Split rows into chunks wherever episode_number increases.
          2. Always discard the last chunk (may be incomplete).
          3. Discard chunks where port_name != 2 and insertion_event never reaches 2.
          4. Slide a window of length obs_horizon + pred_horizon over each kept chunk.
        """
        ep_num = state_arr[:, _COL_EPISODE_NUMBER]

        # Episode boundaries: row indices where episode_number changes
        changes   = np.where(np.diff(ep_num) != 0)[0] + 1
        ep_starts = np.concatenate([[0], changes])
        ep_ends   = np.concatenate([changes, [len(ep_num)]])
        num_episodes = len(ep_starts)

        windows: list[int] = []
        # Discard last chunk → iterate only up to num_episodes - 1
        for i in range(num_episodes - 1):
            s, e = int(ep_starts[i]), int(ep_ends[i])
            chunk = state_arr[s:e]

            # Filter: discard failed SFP episodes (port_name != 2 and no insertion)
            port_name     = int(round(chunk[0, _COL_PORT_NAME]))
            max_insertion = float(chunk[:, _COL_INSERTION_EVENT].max())
            if port_name != 2 and max_insertion < 2:
                continue

            # Slide window over the kept chunk
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

        # Global frame indices for this window
        gidx = self._global_indices[local_start:local_end]  # (window_len,)

        # --- Robot state: first obs_horizon frames ---
        obs_state = self._state_arr[local_start : local_start + self.obs_horizon]
        robot_state = torch.from_numpy(obs_state[:, _ROBOT_STATE_COLS])  # (obs_horizon, 20)

        # Task identity (constant within episode; read from first obs frame)
        target_module = torch.tensor(
            int(round(obs_state[0, _COL_TARGET_MODULE])), dtype=torch.int64
        )
        port_name = torch.tensor(
            int(round(obs_state[0, _COL_PORT_NAME])), dtype=torch.int64
        )

        # --- Actions: tcp_pose for pred_horizon frames after the obs window ---
        # Cols 0:7 = tcp_pose.position(3) + tcp_pose.orientation(4)
        pred_state = self._state_arr[
            local_start + self.obs_horizon : local_start + self.obs_horizon + self.pred_horizon
        ]
        action = torch.from_numpy(pred_state[:, :7])  # (pred_horizon, 7)

        # --- Images: obs_horizon frames ---
        images: dict[str, torch.Tensor] = {k: [] for k in CAMERA_KEYS}
        for gi in gidx[: self.obs_horizon]:
            frame = self._lerobot[int(gi)]
            for k in CAMERA_KEYS:
                img = frame[f"observation.images.{k}"]   # (C, H, W) float32 [0, 1]
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
            "action":         action,
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
# Training step
# ---------------------------------------------------------------------------

def ddpm_loss(
    policy: DiffusionPolicy,
    schedule: DDPMSchedule,
    batch: dict,
    device: torch.device,
) -> torch.Tensor:
    """Single DDPM training step: sample t and noise, compute MSE loss.

    DDPM objective:
        L = E_{t, x0, ε} [ || ε - ε_θ(sqrt(ᾱ_t)*x0 + sqrt(1-ᾱ_t)*ε, t) ||² ]

    Args:
        policy:   DiffusionPolicy model
        schedule: DDPMSchedule with noise schedule tensors
        batch:    dict of tensors from the DataLoader
        device:   torch device

    Returns:
        loss: scalar tensor
    """
    action = batch["action"].to(device)               # (B, pred_horizon, action_dim)
    B = action.shape[0]

    # Sample random diffusion timesteps for each item in the batch
    t = torch.randint(0, schedule.num_timesteps, (B,), device=device)

    # Sample Gaussian noise
    noise = torch.randn_like(action)                  # (B, pred_horizon, action_dim)

    # Forward diffusion: corrupt the clean action
    noisy_action = schedule.add_noise(action, noise, t)

    # Move all conditioning inputs to device
    images = {k: v.to(device) for k, v in batch["images"].items()}
    robot_state   = batch["robot_state"].to(device)   # (B, obs_horizon, ROBOT_STATE_DIM)
    target_module = batch["target_module"].to(device)
    port_name     = batch["port_name"].to(device)

    # Predict noise
    noise_pred = policy(
        images=images,
        robot_state=robot_state,
        target_module=target_module,
        port_name=port_name,
        noisy_action=noisy_action,
        timestep=t,
    )                                                 # (B, pred_horizon, action_dim)

    return nn.functional.mse_loss(noise_pred, noise)


# ---------------------------------------------------------------------------
# Main training loop
# ---------------------------------------------------------------------------

def train(args):
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Training on {device}")

    # --- Model ---
    policy = DiffusionPolicy(
        obs_horizon=args.obs_horizon,
        action_dim=args.action_dim,
        robot_state_dim=ROBOT_STATE_DIM,
        d_model=TOKEN_DIM,
        n_heads=args.n_heads,
        n_layers=args.n_layers,
        ffn_dim=args.ffn_dim,
        dropout=args.dropout,
        cfg_dropout_prob=0.1,
    ).to(device)

    # --- Freeze image encoder ---
    for p in policy.image_encoder.parameters():
        p.requires_grad = False

    # --- Model summary (temporary) ---
    total = sum(p.numel() for p in policy.parameters())
    trainable = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"\nModel parameters: {total:,} total, {trainable:,} trainable")
    for name, mod in policy.named_children():
        n = sum(p.numel() for p in mod.parameters())
        print(f"  {name:<30} {n:>12,}")

    # --- Noise schedule ---
    schedule = DDPMSchedule(num_timesteps=args.num_timesteps, device=device)

    # --- Optimizer ---
    optimizer = torch.optim.AdamW(policy.parameters(), lr=args.lr, weight_decay=1e-4)

    # --- Data ---
    loader = build_dataloader(
        dataset_dirs=args.dataset_dirs,
        obs_horizon=args.obs_horizon,
        pred_horizon=args.pred_horizon,
        action_dim=args.action_dim,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
    )

    # --- Training loop ---
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    policy.train()
    global_step = 0
    step_losses: list[float] = []   # loss at every optimizer step
    epoch_losses: list[float] = []  # average loss per epoch

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

        # Save checkpoint
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

    # --- Loss curves ---
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
    # --- Data ---
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
    p = argparse.ArgumentParser(description="Train DiffusionPolicy on AIC datasets")

    # Data
    p.add_argument("--dataset_dirs", nargs="+", required=True,
                   help="Paths to one or more LeRobot dataset directories")
    p.add_argument("--output_dir", default="outputs/diffusion",
                   help="Directory for checkpoints and logs")

    # Sequence lengths
    p.add_argument("--obs_horizon",  type=int, default=2)
    p.add_argument("--pred_horizon", type=int, default=8)
    p.add_argument("--action_dim",   type=int, default=7)

    # Diffusion
    p.add_argument("--num_timesteps", type=int, default=50)

    # Model
    p.add_argument("--n_heads",  type=int,   default=8)
    p.add_argument("--n_layers", type=int,   default=4)
    p.add_argument("--ffn_dim",  type=int,   default=1024)
    p.add_argument("--dropout",  type=float, default=0.0)

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
