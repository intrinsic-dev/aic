"""
train_diffusion.py
Trains a Diffusion Policy on the large AIC cable-insertion dataset using LeRobot.

Reference: https://github.com/huggingface/lerobot

Architecture
────────────
  Observation encoder:
    - ResNet-18 × 3 cameras (center, left, right) → spatial softmax features
    - Linear(26) for proprioceptive state
    - Features concatenated and used as global FiLM conditioning
  Denoising backbone : 1-D U-Net (down_dims=256,512,1024)
  Noise scheduler    : DDPM, 100 train timesteps, ε-prediction

AIC robot settings
──────────────────
  state dim      : 26   (TCP pose/vel/err + joints)
  images         : 3 × (128×144)  center, left, right cameras
  action dim     : 6   (cartesian twist)
  horizon        : 16  (predict 16 future steps)
  n_obs_steps    : 2   (condition on last 2 observations)
  n_action_steps : 8   (execute 8 of the 16 predicted steps)

Dataset
───────
  local/aic_cable_insertion_large
  500 train + 100 val episodes, 100 steps each → 60,000 frames
  5 trajectory strategies for coverage

Usage
─────
    python train_diffusion.py [--steps 10000] [--batch_size 8] [--lr 1e-4]
"""

import argparse
import csv
import json
import time
from pathlib import Path

import torch
import torch.nn as nn
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader, random_split

from lerobot.configs.types import FeatureType, NormalizationMode, PolicyFeature
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

REPO_ID  = "local/aic_cable_insertion_large"
OUT_DIR  = Path(__file__).parent / "outputs_diffusion"
CKPT_DIR = Path(__file__).parent / "checkpoints_diffusion"

FPS        = 20
N_OBS      = 2    # temporal context: condition on 2 consecutive observations
HORIZON    = 16   # full prediction horizon
N_ACTION   = 8    # steps executed per inference call


# ─── Config ───────────────────────────────────────────────────────────────────

def make_diffusion_config() -> DiffusionConfig:
    """
    DiffusionConfig for AIC robot with 3 cameras.

    All three cameras are encoded by separate ResNet-18 backbones whose spatial
    softmax features are concatenated with the state embedding before being
    passed as FiLM conditioning to the U-Net denoising backbone.
    """
    return DiffusionConfig(
        # ── I/O shapes ────────────────────────────────────────────────────────
        input_features={
            "observation.state": PolicyFeature(
                type=FeatureType.STATE, shape=(26,)
            ),
            "observation.images.center_camera": PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, 128, 144)
            ),
            "observation.images.left_camera": PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, 128, 144)
            ),
            "observation.images.right_camera": PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, 128, 144)
            ),
        },
        output_features={
            "action": PolicyFeature(type=FeatureType.ACTION, shape=(6,))
        },
        normalization_mapping={
            "STATE":  NormalizationMode.MIN_MAX,
            "VISUAL": NormalizationMode.MEAN_STD,
            "ACTION": NormalizationMode.MIN_MAX,
        },
        # ── Temporal structure ────────────────────────────────────────────────
        n_obs_steps        = N_OBS,
        horizon            = HORIZON,
        n_action_steps     = N_ACTION,
        drop_n_last_frames = HORIZON - N_ACTION,   # 8
        # ── Vision backbone ───────────────────────────────────────────────────
        vision_backbone               = "resnet18",
        pretrained_backbone_weights   = None,
        use_group_norm                = True,
        spatial_softmax_num_keypoints = 32,
        resize_shape                  = None,
        crop_shape                    = None,
        crop_ratio                    = 1.0,
        # ── U-Net ─────────────────────────────────────────────────────────────
        down_dims                 = (256, 512, 1024),
        kernel_size               = 5,
        n_groups                  = 8,
        diffusion_step_embed_dim  = 128,
        use_film_scale_modulation = True,
        # ── DDPM noise schedule ───────────────────────────────────────────────
        noise_scheduler_type  = "DDPM",
        num_train_timesteps   = 100,
        beta_schedule         = "squaredcos_cap_v2",
        beta_start            = 0.0001,
        beta_end              = 0.02,
        prediction_type       = "epsilon",
        clip_sample           = True,
        clip_sample_range     = 1.0,
        do_mask_loss_for_padding = False,
    )


# ─── Trainer ──────────────────────────────────────────────────────────────────

class DiffusionTrainer:
    def __init__(self, cfg: dict):
        self.cfg    = cfg
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Device : {self.device}")

        # ── Dataset ──────────────────────────────────────────────────────────
        print(f"Loading dataset: {REPO_ID} …")
        self.full_ds = LeRobotDataset(
            repo_id=REPO_ID,
            delta_timestamps={
                "observation.state": [0.0, -1 / FPS],
                "observation.images.center_camera": [0.0, -1 / FPS],
                "observation.images.left_camera":   [0.0, -1 / FPS],
                "observation.images.right_camera":  [0.0, -1 / FPS],
                "action": [i / FPS for i in range(HORIZON)],
            },
        )
        n_total = len(self.full_ds)
        n_val   = max(1, int(0.15 * n_total))
        n_train = n_total - n_val
        train_ds, val_ds = random_split(
            self.full_ds, [n_train, n_val],
            generator=torch.Generator().manual_seed(42),
        )
        self.train_loader = DataLoader(
            train_ds, batch_size=cfg["batch_size"],
            shuffle=True, num_workers=4, pin_memory=True, drop_last=True,
        )
        self.val_loader = DataLoader(
            val_ds, batch_size=cfg["batch_size"],
            shuffle=False, num_workers=4, pin_memory=True,
        )
        print(f"Train: {n_train:,}  |  Val: {n_val:,}")

        # ── Model ────────────────────────────────────────────────────────────
        diff_cfg = make_diffusion_config()
        self.model = DiffusionPolicy(diff_cfg, dataset_stats=self.full_ds.meta.stats)
        self.model.to(self.device)
        n_params = sum(p.numel() for p in self.model.parameters() if p.requires_grad)
        print(f"DiffusionPolicy parameters: {n_params:,}\n")

        self.preprocessor, _ = make_pre_post_processors(
            diff_cfg, dataset_stats=self.full_ds.meta.stats
        )

        # ── Optimiser ────────────────────────────────────────────────────────
        self.optimizer = AdamW(
            self.model.parameters(), lr=cfg["lr"],
            betas=(0.95, 0.999), eps=1e-8, weight_decay=cfg["weight_decay"],
        )
        self.scheduler = CosineAnnealingLR(
            self.optimizer, T_max=cfg["steps"], eta_min=cfg["lr"] * 0.01
        )

        # ── Logging ──────────────────────────────────────────────────────────
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        CKPT_DIR.mkdir(parents=True, exist_ok=True)
        self.history: list[dict] = []
        self.best_val = float("inf")
        self.global_step = 0

        json.dump({
            "model": {
                "policy":    "Diffusion Policy (lerobot)",
                "reference": "https://github.com/huggingface/lerobot",
                "backbone":  "ResNet-18 × 3 cameras",
                "unet_dims": [256, 512, 1024],
                "n_obs_steps": N_OBS,
                "horizon":     HORIZON,
                "n_action_steps": N_ACTION,
                "noise_scheduler": "DDPM",
                "num_train_timesteps": 100,
                "n_params": n_params,
            },
            "training": cfg,
            "dataset": {
                "repo_id": REPO_ID,
                "n_train": n_train,
                "n_val":   n_val,
                "cameras": ["center_camera", "left_camera", "right_camera"],
            },
        }, open(OUT_DIR / "run_config.json", "w"), indent=2)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _to_device(self, batch: dict) -> dict:
        return {k: v.to(self.device) if isinstance(v, torch.Tensor) else v
                for k, v in batch.items()}

    def _prep_batch(self, batch: dict) -> dict:
        """
        Normalise inputs and collapse obs time-dim for state/images.
        Diffusion policy uses the *last* obs step as the conditioning frame.
        The action tensor stays (B, horizon, 6).
        """
        batch = self.preprocessor(batch)
        if "observation.state" in batch and batch["observation.state"].ndim == 3:
            batch["observation.state"] = batch["observation.state"][:, -1, :]
        for k in list(batch.keys()):
            if k.startswith("observation.images.") and batch[k].ndim == 5:
                batch[k] = batch[k][:, -1, ...]   # (B, n_obs, C, H, W) → (B, C, H, W)
        return batch

    # ── Epoch ─────────────────────────────────────────────────────────────────

    def _run_epoch(self, loader: DataLoader, train: bool) -> dict:
        self.model.train(True)
        total_loss, n = 0.0, 0
        with torch.set_grad_enabled(train):
            for batch in loader:
                batch = self._to_device(batch)
                batch = self._prep_batch(batch)

                out  = self.model(batch)
                loss = out[0] if isinstance(out, tuple) else out

                if train:
                    self.optimizer.zero_grad()
                    loss.backward()
                    nn.utils.clip_grad_norm_(self.model.parameters(), 1.0)
                    self.optimizer.step()
                    self.scheduler.step()
                    self.global_step += 1

                B = next(v.size(0) for v in batch.values() if isinstance(v, torch.Tensor))
                total_loss += loss.item() * B
                n += B

        return {"loss": total_loss / n}

    # ── Training ──────────────────────────────────────────────────────────────

    def train(self):
        steps_per_epoch = len(self.train_loader)
        n_epochs = max(1, self.cfg["steps"] // steps_per_epoch)
        print(f"Training for {n_epochs} epochs ({self.cfg['steps']} steps, {steps_per_epoch} steps/epoch)")
        print(f"\n{'Epoch':>6}  {'LR':>9}  {'T-loss':>9}  {'V-loss':>9}  {'Best':>5}")
        print("─" * 55)

        for epoch in range(1, n_epochs + 1):
            t0    = time.time()
            t_m   = self._run_epoch(self.train_loader, train=True)
            v_m   = self._run_epoch(self.val_loader,   train=False)
            lr    = self.scheduler.get_last_lr()[0]
            is_best = v_m["loss"] < self.best_val
            if is_best:
                self.best_val = v_m["loss"]
                self.model.save_pretrained(str(CKPT_DIR / "best_model"))

            if epoch % self.cfg.get("save_every", 5) == 0:
                self.model.save_pretrained(str(CKPT_DIR / f"checkpoint_epoch_{epoch:03d}"))

            self.history.append({
                "epoch": epoch, "step": self.global_step, "lr": lr,
                "train_loss": t_m["loss"], "val_loss": v_m["loss"],
            })
            self._save_metrics()

            print(f"{epoch:>6}  {lr:>9.2e}  {t_m['loss']:>9.5f}  {v_m['loss']:>9.5f}  "
                  f"{'✓' if is_best else '':>5}   {time.time()-t0:.1f}s")

        self.model.save_pretrained(str(CKPT_DIR / "final_model"))
        print(f"\nDone. Best val loss: {self.best_val:.5f}")
        print(f"Artifacts → {OUT_DIR}  and  {CKPT_DIR}")

    def _save_metrics(self):
        with open(OUT_DIR / "metrics.json", "w") as f:
            json.dump(self.history, f, indent=2)
        if self.history:
            with open(OUT_DIR / "metrics.csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(self.history[0].keys()))
                w.writeheader()
                w.writerows(self.history)


# ─── Entry point ─────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Train Diffusion Policy on AIC cable-insertion dataset (lerobot)"
    )
    p.add_argument("--steps",        type=int,   default=10000,
                   help="Total training gradient steps")
    p.add_argument("--batch_size",   type=int,   default=8)
    p.add_argument("--lr",           type=float, default=1e-4)
    p.add_argument("--weight_decay", type=float, default=1e-6)
    p.add_argument("--save_every",   type=int,   default=5,
                   help="Save checkpoint every N epochs")
    return vars(p.parse_args())


if __name__ == "__main__":
    cfg = parse_args()
    trainer = DiffusionTrainer(cfg)
    trainer.train()
