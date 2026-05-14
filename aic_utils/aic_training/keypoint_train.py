"""
Training script for PortKeypointNet.

Usage:
    python3 -m keypoint_model.train \
        --dataset /home/lhphanto/hf_dataset_temp \
        --output  /tmp/keypoint_ckpts \
        --epochs  60 \
        --batch-size 16

Splits are done per-episode (not per-frame) to avoid leakage.
"""

import argparse
import logging
import math
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless — no display needed
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import ConcatDataset, DataLoader, Subset

from .constants import CAMERA_NAMES, OUTPUT_KEYS
from .dataset import (
    LeRobotKeypointDataset,
    DEFAULT_TRANSFORM, TRAIN_TRANSFORM,
    LEROBOT_TRANSFORM, LEROBOT_TRAIN_TRANSFORM,
)
from .model import PortKeypointNet

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Visualisation helpers
# ---------------------------------------------------------------------------

_ENTITY_COLORS = {
    "nic_card_mount_0": "#FF4444",
    "nic_card_mount_1": "#FF8800",
    "nic_card_mount_2": "#FFEE00",
    "nic_card_mount_3": "#00FFAA",
    "nic_card_mount_4": "#FF44FF",
    "sc_port_0":        "#44FF44",
    "sc_port_1":        "#44AAFF",
}


def _denormalize(tensor: torch.Tensor) -> np.ndarray:
    """(3, H, W) float tensor normalised with mean=std=0.5 → H×W×3 uint8."""
    img = (tensor * 0.5 + 0.5).clamp(0, 1)
    return (img.permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)


def _plot_points(
    ax: plt.Axes,
    b: int,
    xy: torch.Tensor,
    conf_visible: torch.Tensor,
    img_h: int,
    img_w: int,
    threshold: float = 0.5,
) -> None:
    """Plot keypoints onto ax for batch element b. xy: (B, 36, 2)."""
    for out_idx, (entity, port, cam) in enumerate(OUTPUT_KEYS):
        if conf_visible[b, out_idx].item() < threshold:
            continue
        cam_idx = CAMERA_NAMES.index(cam)
        xn, yn  = xy[b, out_idx].tolist()
        px = xn * img_w + cam_idx * img_w   # offset into concatenated image
        py = yn * img_h
        color = _ENTITY_COLORS.get(entity, "white")
        ax.plot(px, py, "o", color=color, markersize=6,
                markeredgecolor="black", markeredgewidth=0.5)
        ax.annotate(
            f"{entity}\n{port}",
            xy=(px, py), xytext=(4, 4), textcoords="offset points",
            fontsize=4, color=color,
            bbox=dict(boxstyle="round,pad=0.1", fc="black", alpha=0.6),
        )


def save_sample_images(
    model: PortKeypointNet,
    sample_batch: tuple,
    device: torch.device,
    output_dir: Path,
    epoch: int,
) -> None:
    """
    For each sample in sample_batch, save a side-by-side figure:
      left  — ground truth visible points on concatenated (left|center|right) image
      right — predicted points with conf_visible > 0.5
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    model.eval()

    imgs_list, gt = sample_batch
    imgs_dev = [img.to(device) for img in imgs_list]

    with torch.no_grad():
        pred = model(imgs_dev)

    B      = imgs_list[0].shape[0]
    img_h  = imgs_list[0].shape[2]
    img_w  = imgs_list[0].shape[3]

    for b in range(B):
        cam_imgs   = [_denormalize(imgs_list[c][b]) for c in range(len(CAMERA_NAMES))]
        concat_img = np.concatenate(cam_imgs, axis=1)  # H × (3W) × 3

        fig, (ax_gt, ax_pred) = plt.subplots(1, 2, figsize=(20, 5))
        fig.suptitle(f"Epoch {epoch}  sample {b}", fontsize=10)

        for ax, title in ((ax_gt, "Ground truth"), (ax_pred, "Predicted (conf_visible > 0.5)")):
            ax.imshow(concat_img)
            ax.set_title(title, fontsize=9)
            ax.axis("off")
            # Camera boundary lines
            for ci in (1, 2):
                ax.axvline(ci * img_w, color="white", linewidth=0.8, linestyle="--", alpha=0.6)
            for ci, name in enumerate(CAMERA_NAMES):
                ax.text(ci * img_w + 4, 10, name, fontsize=5, color="white",
                        bbox=dict(fc="black", alpha=0.5, pad=1))

        _plot_points(ax_gt,   b, gt["xy"],   gt["conf_visible"],   img_h, img_w, threshold=0.5)
        _plot_points(ax_pred, b, pred["xy"], pred["conf_visible"], img_h, img_w, threshold=0.5)

        out_path = output_dir / f"epoch_{epoch:03d}_sample_{b}.png"
        fig.savefig(out_path, dpi=130, bbox_inches="tight")
        plt.close(fig)

    logger.info(f"  Samples → {output_dir}/epoch_{epoch:03d}_sample_*.png")


# ---------------------------------------------------------------------------
# Loss
# ---------------------------------------------------------------------------

def compute_loss(
    pred: dict[str, torch.Tensor],
    gt: dict[str, torch.Tensor],
    lambda_xy: float = 5.0,
    lambda_dist: float = 2.0,
) -> tuple[torch.Tensor, dict[str, float]]:
    """
    Weighted sum of four terms:
      - BCE for conf_visible / conf_present
      - Smooth L1 for normalised xy  (masked to visible samples)
      - Smooth L1 for log_dist       (masked to present samples)
    """
    vis_loss  = F.binary_cross_entropy(pred["conf_visible"], gt["conf_visible"])
    pres_loss = F.binary_cross_entropy(pred["conf_present"], gt["conf_present"])

    # Regression losses only where ground-truth is meaningful
    vis_mask  = gt["conf_visible"].bool()   # (B, 36) mask
    pres_mask = gt["conf_present"].bool()

    xy_loss   = torch.tensor(0.0, device=vis_loss.device)
    dist_loss = torch.tensor(0.0, device=vis_loss.device)

    if vis_mask.any():
        xy_loss = F.smooth_l1_loss(pred["xy"][vis_mask], gt["xy"][vis_mask])

    if pres_mask.any():
        dist_loss = F.smooth_l1_loss(
            pred["log_dist"][pres_mask], gt["log_dist"][pres_mask]
        )

    total = vis_loss + pres_loss + lambda_xy * xy_loss + lambda_dist * dist_loss

    breakdown = {
        "vis":  vis_loss.item(),
        "pres": pres_loss.item(),
        "xy":   xy_loss.item(),
        "dist": dist_loss.item(),
    }
    return total, breakdown


# ---------------------------------------------------------------------------
# Epoch loops
# ---------------------------------------------------------------------------

def run_epoch(
    model: PortKeypointNet,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer | None,
    scheduler: torch.optim.lr_scheduler.LRScheduler | None,
    device: torch.device,
) -> dict[str, float]:
    training = optimizer is not None
    model.train(training)

    totals: dict[str, float] = {"total": 0.0, "vis": 0.0, "pres": 0.0, "xy": 0.0, "dist": 0.0}

    ctx = torch.enable_grad() if training else torch.no_grad()
    with ctx:
        for imgs, gt in loader:
            imgs = [img.to(device, non_blocking=True) for img in imgs]
            gt   = {k: v.to(device, non_blocking=True) for k, v in gt.items()}

            pred = model(imgs)
            loss, bd = compute_loss(pred, gt)

            if training:
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                optimizer.step()
                if scheduler is not None:
                    scheduler.step()

            totals["total"] += loss.item()
            for k in ("vis", "pres", "xy", "dist"):
                totals[k] += bd[k]

    n = len(loader)
    return {k: v / n for k, v in totals.items()}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def train(args: argparse.Namespace) -> None:
    # The default 'file_descriptor' sharing strategy opens one fd per tensor crossing
    # the process boundary, exhausting the OS limit under spawn + large batches.
    # 'file_system' uses temp files instead — no fd pressure, slightly more I/O.
    if args.num_workers > 0:
        torch.multiprocessing.set_sharing_strategy("file_system")

    device = torch.device(args.device)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    # --- Datasets ---
    logger.info(
        f"Loading {len(args.datasets)} dataset(s) (backend={args.dataset_type}) …"
    )
    train_transform = LEROBOT_TRAIN_TRANSFORM if args.dataset_type == "lerobot" else TRAIN_TRANSFORM
    val_transform   = LEROBOT_TRANSFORM       if args.dataset_type == "lerobot" else DEFAULT_TRANSFORM

    # lerobot backend streams frames on demand via HF datasets — pre-decoding all
    # frames as float32 tensors typically causes OOM on large datasets.
    # local backend benefits from caching since av video seeking is slow.
    if args.dataset_type == "lerobot":
        cache_frames = args.cache_frames   # off by default; opt-in only if dataset fits in RAM
        if not cache_frames:
            logger.info("lerobot backend: streaming frames on demand (use --cache-frames to preload)")
    else:
        cache_frames = not args.no_cache

    train_ds, val_ds = LeRobotKeypointDataset.multi_episode_split(
        args.datasets,
        val_fraction=args.val_split,
        seed=args.seed,
        backend=args.dataset_type,
        train_transform=train_transform,
        val_transform=val_transform,
        cache_frames=cache_frames,
    )
    logger.info(f"  Total: {len(train_ds)} train frames, {len(val_ds)} val frames")

    # multiprocessing_context="spawn" is required when num_workers > 0 with lerobot/torchcodec:
    # the default "fork" copies the parent's open video decoder state into each worker,
    # causing races and "Invalid data" decoder crashes. spawn starts each worker fresh.
    mp_ctx = "spawn" if args.num_workers > 0 else None

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device.type == "cuda"),
        persistent_workers=(args.num_workers > 0),
        multiprocessing_context=mp_ctx,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=(device.type == "cuda"),
        persistent_workers=(args.num_workers > 0),
        multiprocessing_context=mp_ctx,
    )

    # --- Model ---
    model = PortKeypointNet(
        pretrained=not args.no_pretrain,
        dropout=args.dropout,
    ).to(device)
    n_params = sum(p.numel() for p in model.parameters()) / 1e6
    logger.info(f"PortKeypointNet: {n_params:.1f}M parameters")

    # --- Optimiser: lower LR for ViT backbone, higher for head ---
    optimizer = torch.optim.AdamW([
        {"params": model.vit.parameters(),  "lr": args.lr * 0.1},
        {"params": model.head.parameters(), "lr": args.lr},
    ], weight_decay=1e-4)

    # Per-batch cosine schedule with a short linear warm-up
    total_steps  = args.epochs * len(train_loader)
    warmup_steps = max(1, total_steps // 20)
    scheduler = torch.optim.lr_scheduler.SequentialLR(
        optimizer,
        schedulers=[
            torch.optim.lr_scheduler.LinearLR(
                optimizer, start_factor=0.01, end_factor=1.0, total_iters=warmup_steps
            ),
            torch.optim.lr_scheduler.CosineAnnealingLR(
                optimizer, T_max=total_steps - warmup_steps, eta_min=1e-6
            ),
        ],
        milestones=[warmup_steps],
    )

    # --- Fixed samples for per-epoch visualisation ---
    sample_batch = None
    if args.save_samples:
        n = len(val_ds)
        # Spread 3 indices across the val set so they cover different episodes
        sample_indices = [0, n // 2, n - 1]
        sample_loader  = DataLoader(
            Subset(val_ds, sample_indices),
            batch_size=3, shuffle=False, num_workers=0,
        )
        sample_batch = next(iter(sample_loader))
        sample_dir   = output / "samples"
        logger.info(f"Sample images will be saved to {sample_dir}/")

    # --- Resume ---
    start_epoch = 1
    best_val    = math.inf
    if args.resume:
        ckpt_r = torch.load(args.resume, map_location=device)
        model.load_state_dict(ckpt_r["model"])
        if "optimizer" in ckpt_r:
            optimizer.load_state_dict(ckpt_r["optimizer"])
        if "scheduler" in ckpt_r:
            scheduler.load_state_dict(ckpt_r["scheduler"])
        if "best_val" in ckpt_r:
            best_val = ckpt_r["best_val"]
        start_epoch = ckpt_r.get("epoch", 0) + 1
        logger.info(
            f"Resumed from {args.resume}  "
            f"(epoch {start_epoch - 1} → continuing from epoch {start_epoch}  "
            f"best_val={best_val:.4f})"
        )

    # --- Training loop ---
    for epoch in range(start_epoch, args.epochs + 1):
        t0      = time.monotonic()
        train_m = run_epoch(model, train_loader, optimizer, scheduler, device)
        val_m   = run_epoch(model, val_loader,   None,      None,      device)
        elapsed = time.monotonic() - t0

        logger.info(
            f"Epoch {epoch:3d}/{args.epochs}  ({elapsed:.0f}s)"
            f"  train={train_m['total']:.4f}"
            f" (vis={train_m['vis']:.3f} pres={train_m['pres']:.3f}"
            f"  xy={train_m['xy']:.4f} dist={train_m['dist']:.4f})"
            f"  val={val_m['total']:.4f}"
        )

        ckpt = {
            "epoch":     epoch,
            "model":     model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "scheduler": scheduler.state_dict(),
            "val_loss":  val_m["total"],
            "best_val":  best_val,
        }
        torch.save(ckpt, output / "last.pt")

        if val_m["total"] < best_val:
            best_val       = val_m["total"]
            ckpt["best_val"] = best_val
            torch.save(ckpt, output / "best.pt")
            logger.info(f"  → new best  val={best_val:.4f}")

        if sample_batch is not None:
            save_sample_images(model, sample_batch, device, sample_dir, epoch)

    logger.info(f"Done. Best val loss: {best_val:.4f}  checkpoints in {output}/")


def main() -> None:
    parser = argparse.ArgumentParser(description="Train PortKeypointNet")
    parser.add_argument("--datasets", nargs="+", required=True,
                        help="One or more LeRobot dataset paths (or HF repo IDs with --dataset-type lerobot). "
                             "Each is split by episode independently, then concatenated.")
    parser.add_argument("--output",       default="/tmp/keypoint_ckpts")
    parser.add_argument("--epochs",       type=int,   default=60)
    parser.add_argument("--batch-size",   type=int,   default=16)
    parser.add_argument("--lr",           type=float, default=3e-4)
    parser.add_argument("--dropout",      type=float, default=0.3)
    parser.add_argument("--val-split",    type=float, default=0.15)
    parser.add_argument("--seed",         type=int,   default=42)
    parser.add_argument("--num-workers",  type=int,   default=4)
    parser.add_argument(
        "--device",
        default="cuda" if torch.cuda.is_available() else "cpu",
    )
    parser.add_argument("--dataset-type", default="local", choices=["local", "lerobot"],
                        help="'local': read parquet+mp4 directly; "
                             "'lerobot': use LeRobotDataset (supports HF remote datasets)")
    parser.add_argument("--resume",       type=str, default=None,
                        help="Path to a checkpoint (e.g. last.pt) to resume training from.")
    parser.add_argument("--no-pretrain",  action="store_true", help="Random backbone init")
    parser.add_argument("--no-cache",     action="store_true",
                        help="Don't preload frames into RAM (local backend: slow seeks; lerobot: default)")
    parser.add_argument("--cache-frames", action="store_true",
                        help="Force frame caching for lerobot backend (only if dataset fits in RAM)")
    parser.add_argument("--save-samples", action="store_true",
                        help="Save 3 GT-vs-predicted visualisation images per epoch to <output>/samples/")
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
