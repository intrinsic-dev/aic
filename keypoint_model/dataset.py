"""
LeRobot dataset wrapper for PortKeypointNet training.

Each sample:
    images  : list of 3 tensors (3, H, W), ordered [left, center, right]
    targets : dict
        conf_visible : (36,)    1.0 if u,v projected inside image bounds
        conf_present : (36,)    1.0 if port was in front of camera (Z > 0, dist > 0)
        xy           : (36, 2)  normalised pixel coords (u/W, v/H) — valid only when visible
        log_dist     : (36,)    log(dist in metres) — valid only when present

The state vector stores port_center.{cam}.{entity}.{port}.{u|v|dist} for every combo.
Sentinels: u = v = -1.0 (not visible), dist = -1.0 (not present / behind camera).
"""

import json
import logging
import math
from pathlib import Path
from typing import Optional

import av
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset
from torchvision import transforms as T

from .constants import (
    CAMERA_NAMES,
    NUM_OUTPUTS,
    OUTPUT_KEYS,
)

logger = logging.getLogger(__name__)

# u,v in the state vector are in full-resolution camera space (e.g. 1152×1024).
# Images are saved at this fraction of full resolution.
IMAGE_SCALE = 0.25

# Normalization stats for vit_small_patch16_224 (from timm.data.resolve_model_data_config)
_VITIMAGE_MEAN = [0.5, 0.5, 0.5]
_VITIMAGE_STD  = [0.5, 0.5, 0.5]

# ---------------------------------------------------------------------------
# "local" backend transforms  — input is H×W×3 uint8 numpy array (from av)
# ---------------------------------------------------------------------------

DEFAULT_TRANSFORM = T.Compose([
    T.ToPILImage(),
    T.Resize((256, 256)),
    T.CenterCrop((224, 224)),
    T.ToTensor(),
    T.Normalize(mean=_VITIMAGE_MEAN, std=_VITIMAGE_STD),
])

TRAIN_TRANSFORM = T.Compose([
    T.ToPILImage(),
    T.Resize((256, 256)),
    T.CenterCrop((224, 224)),
    #T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.2, hue=0.05),
    T.ToTensor(),
    T.Normalize(mean=_VITIMAGE_MEAN, std=_VITIMAGE_STD),
])

# ---------------------------------------------------------------------------
# "lerobot" backend transforms — input is (C, H, W) float32 tensor in [0, 1]
# (LeRobotDataset decodes and scales to [0,1] internally — no ToPILImage/ToTensor needed)
# ---------------------------------------------------------------------------

LEROBOT_TRANSFORM = T.Compose([
    T.Resize((256, 256), antialias=True),
    T.CenterCrop((224, 224)),
    T.Normalize(mean=_VITIMAGE_MEAN, std=_VITIMAGE_STD),
])

LEROBOT_TRAIN_TRANSFORM = T.Compose([
    T.Resize((256, 256), antialias=True),
    T.CenterCrop((224, 224)),
    #T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.2, hue=0.05),
    T.Normalize(mean=_VITIMAGE_MEAN, std=_VITIMAGE_STD),
])


# ---------------------------------------------------------------------------
# Video decoding helpers
# ---------------------------------------------------------------------------

def decode_all_frames(video_path: Path) -> dict[int, np.ndarray]:
    """Decode every frame in a chunk video → {global_frame_idx: H×W×3 uint8}."""
    frames: dict[int, np.ndarray] = {}
    with av.open(str(video_path)) as container:
        stream = container.streams.video[0]
        for i, frame in enumerate(container.decode(stream)):
            frames[i] = frame.to_ndarray(format="rgb24")
    return frames


def decode_single_frame(video_path: Path, target_idx: int) -> np.ndarray:
    """Decode one frame by sequential scan (slow; use only when cache_frames=False)."""
    with av.open(str(video_path)) as container:
        stream = container.streams.video[0]
        for i, frame in enumerate(container.decode(stream)):
            if i == target_idx:
                return frame.to_ndarray(format="rgb24")
    raise IndexError(f"Frame {target_idx} not found in {video_path}")


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------

class LeRobotKeypointDataset(Dataset):
    """
    Args:
        dataset_path : path to a LeRobot dataset root (contains meta/, data/, videos/).
                       For the 'lerobot' backend this can also be a HuggingFace repo-id
                       (e.g. "lhphanto/my_dataset") — LeRobotDataset will download it.
        transform    : per-image callable.  Defaults depend on backend:
                         local    → DEFAULT_TRANSFORM  (numpy uint8 → normalised tensor)
                         lerobot  → LEROBOT_TRANSFORM  (float tensor [0,1] → normalised)
                       Pass the corresponding TRAIN_TRANSFORM / LEROBOT_TRAIN_TRANSFORM
                       for augmentation during training.
        cache_frames : preload all frames into RAM for fast iteration.
                       For 'local' backend this pre-decodes the chunk mp4 with av.
                       For 'lerobot' backend this pre-calls LeRobotDataset.__getitem__.
        episodes     : restrict to a subset of episode indices (for train/val split).
        backend      : 'local'   — read parquet + mp4 directly (default, no extra deps).
                       'lerobot' — use LeRobotDataset from the lerobot package; supports
                                   remote HuggingFace datasets and avoids raw av decoding.
    """

    def __init__(
        self,
        dataset_path: str | Path,
        transform=None,
        cache_frames: bool = True,
        episodes: Optional[list[int]] = None,
        backend: str = "local",
    ):
        if backend not in ("local", "lerobot"):
            raise ValueError(f"backend must be 'local' or 'lerobot', got {backend!r}")

        self.backend      = backend
        self.dataset_path = Path(dataset_path)
        self._lerobot_ds  = None

        # --- Read metadata ---
        # For the lerobot backend, create LeRobotDataset first (handles HF download),
        # then read from its .meta.info dict.  For the local backend, read info.json directly.
        if backend == "lerobot":
            try:
                from lerobot.datasets.lerobot_dataset import LeRobotDataset as _LRDataset
            except ImportError as e:
                raise ImportError(
                    "lerobot package is required for backend='lerobot'. "
                    "Install it with: pip install lerobot"
                ) from e
            self._lerobot_ds = _LRDataset(str(dataset_path))
            info = self._lerobot_ds.meta.info
        else:
            with open(self.dataset_path / "meta" / "info.json") as f:
                info = json.load(f)

        state_names: list[str] = info["features"]["observation.state"]["names"]
        self.img_shapes: dict[str, list[int]] = {
            cam: info["features"][f"observation.images.{cam}"]["shape"]
            for cam in CAMERA_NAMES
        }
        self._state_idx = self._build_state_index(state_names)

        # --- Default transform depends on backend ---
        if transform is not None:
            self.transform = transform
        elif backend == "lerobot":
            self.transform = LEROBOT_TRANSFORM
        else:
            self.transform = DEFAULT_TRANSFORM

        # --- Load frame index / state data ---
        if backend == "lerobot":
            self._init_lerobot_backend(episodes, cache_frames)
        else:
            self._init_local_backend(episodes, cache_frames)

    # ------------------------------------------------------------------
    # Backend-specific initialisation
    # ------------------------------------------------------------------

    def _init_local_backend(
        self,
        episodes: Optional[list[int]],
        cache_frames: bool,
    ) -> None:
        parquet_files = sorted(
            (self.dataset_path / "data" / "chunk-000").glob("*.parquet")
        )
        df = pd.concat([pd.read_parquet(p) for p in parquet_files], ignore_index=True)
        if episodes is not None:
            df = df[df["episode_index"].isin(episodes)].reset_index(drop=True)
        self.df = df

        self._frame_cache: Optional[dict] = None
        if cache_frames:
            logger.info("Pre-loading video frames into RAM …")
            self._frame_cache = {}
            for cam in CAMERA_NAMES:
                vp = self._video_path(cam)
                logger.info(f"  {vp.name} ({cam})")
                self._frame_cache[cam] = decode_all_frames(vp)
            logger.info(
                f"Frame cache ready — {sum(len(v) for v in self._frame_cache.values())} frames."
            )

    def _init_lerobot_backend(
        self,
        episodes: Optional[list[int]],
        cache_frames: bool,
    ) -> None:
        # _lerobot_ds is already created in __init__ before this is called.
        hf = self._lerobot_ds.hf_dataset

        df = pd.DataFrame({
            "index":             hf["index"],
            "frame_index":       hf["frame_index"],
            "episode_index":     hf["episode_index"],
            "observation.state": hf["observation.state"],
        })
        if episodes is not None:
            df = df[df["episode_index"].isin(set(episodes))].reset_index(drop=True)
        self.df = df

        self._frame_cache: Optional[dict] = None
        if cache_frames:
            logger.info("Pre-loading frames via LeRobotDataset …")
            self._frame_cache = {cam: {} for cam in CAMERA_NAMES}
            for global_idx in sorted(self.df["index"].unique()):
                frame = self._lerobot_ds[int(global_idx)]
                for cam in CAMERA_NAMES:
                    self._frame_cache[cam][global_idx] = frame[f"observation.images.{cam}"]
            n = sum(len(v) for v in self._frame_cache.values())
            logger.info(f"Frame cache ready — {n} frames.")

        # Release the main-process _lerobot_ds so DataLoader workers inherit None
        # at fork time (fork copies the parent's decoder state which is not safe).
        # Each worker recreates its own instance lazily in _get_frame().
        self._lerobot_ds = None
        self._lerobot_dataset_path = str(self.dataset_path)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _video_path(self, cam: str) -> Path:
        return (
            self.dataset_path
            / "videos"
            / f"observation.images.{cam}"
            / "chunk-000"
            / "file-000.mp4"
        )

    def _build_state_index(
        self, state_names: list[str]
    ) -> dict[tuple[str, str, str], tuple[int, int, int]]:
        """Map (entity, port, cam) → (u_col, v_col, dist_col) in the state vector."""
        by_key: dict[tuple, dict[str, int]] = {}
        for i, name in enumerate(state_names):
            if not name.startswith("port_center."):
                continue
            parts = name.split(".")
            if len(parts) != 5:
                continue
            _, cam, entity, port, coord = parts
            key = (entity, port, cam)
            by_key.setdefault(key, {})[coord] = i
        return {
            k: (v["u"], v["v"], v["dist"])
            for k, v in by_key.items()
            if "u" in v and "v" in v and "dist" in v
        }

    def _get_frame(self, cam: str, global_idx: int):
        """Return a frame for the given camera and global frame index.

        local   backend → H×W×3 uint8 numpy array
        lerobot backend → (C, H, W) float32 tensor in [0, 1]
        """
        if self._frame_cache is not None:
            return self._frame_cache[cam][global_idx]
        if self.backend == "lerobot":
            if self._lerobot_ds is None:
                # Lazy init per DataLoader worker. _lerobot_ds is set to None
                # before fork so each worker creates its own decoder-safe instance.
                from lerobot.datasets.lerobot_dataset import LeRobotDataset as _LRDataset
                self._lerobot_ds = _LRDataset(self._lerobot_dataset_path)
            return self._lerobot_ds[global_idx][f"observation.images.{cam}"]
        return decode_single_frame(self._video_path(cam), global_idx)

    # ------------------------------------------------------------------
    # Dataset interface
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self.df)

    def __getitem__(self, idx: int):
        row = self.df.iloc[idx]
        global_idx = int(row["index"])
        state = np.asarray(row["observation.state"], dtype=np.float32)

        # --- Images ---
        imgs = [
            self.transform(self._get_frame(cam, global_idx))
            for cam in CAMERA_NAMES
        ]  # list of (3, H, W) tensors

        # --- Targets ---
        conf_visible = torch.zeros(NUM_OUTPUTS, dtype=torch.float32)
        conf_present = torch.zeros(NUM_OUTPUTS, dtype=torch.float32)
        xy           = torch.zeros(NUM_OUTPUTS, 2, dtype=torch.float32)
        log_dist     = torch.zeros(NUM_OUTPUTS, dtype=torch.float32)

        for out_idx, (entity, port, cam) in enumerate(OUTPUT_KEYS):
            key = (entity, port, cam)
            if key not in self._state_idx:
                continue

            u_i, v_i, d_i = self._state_idx[key]
            u    = float(state[u_i])
            v    = float(state[v_i])
            dist = float(state[d_i])

            img_h, img_w = self.img_shapes[cam][:2]

            if dist > 0.0:
                conf_present[out_idx] = 1.0
                log_dist[out_idx]     = math.log(dist)

            if u >= 0.0 and v >= 0.0:
                conf_visible[out_idx] = 1.0
                # u,v are full-res pixels; scale to saved-image space, then normalise
                xy[out_idx, 0] = (u * IMAGE_SCALE) / img_w
                xy[out_idx, 1] = (v * IMAGE_SCALE) / img_h

        return imgs, {
            "conf_visible": conf_visible,   # (36,)
            "conf_present": conf_present,   # (36,)
            "xy":           xy,             # (36, 2)
            "log_dist":     log_dist,       # (36,)
        }

    # ------------------------------------------------------------------
    # Split helpers
    # ------------------------------------------------------------------

    @classmethod
    def episode_split(
        cls,
        dataset_path: str | Path,
        val_fraction: float = 0.15,
        seed: int = 42,
        backend: str = "local",
        train_transform=None,
        val_transform=None,
        **kwargs,   # forwarded to __init__ (e.g. cache_frames)
    ) -> tuple["LeRobotKeypointDataset", "LeRobotKeypointDataset"]:
        """Split by episode so no episode appears in both sets. Returns (train_ds, val_ds)."""
        import random

        if backend == "lerobot":
            from lerobot.datasets.lerobot_dataset import LeRobotDataset as _LRDataset
            tmp_ds   = _LRDataset(str(dataset_path))
            episodes = sorted(set(tmp_ds.hf_dataset["episode_index"]))
            del tmp_ds   # free before the two cls() constructors each create their own instance
        else:
            with open(Path(dataset_path) / "meta" / "info.json") as f:
                info = json.load(f)
            episodes = list(range(info["total_episodes"]))

        rng   = random.Random(seed)
        rng.shuffle(episodes)
        split     = max(1, int(len(episodes) * val_fraction))
        val_eps   = episodes[:split]
        train_eps = episodes[split:]
        train_ds = cls(dataset_path, episodes=train_eps, backend=backend,
                       transform=train_transform, **kwargs)
        val_ds   = cls(dataset_path, episodes=val_eps,   backend=backend,
                       transform=val_transform,   **kwargs)
        return train_ds, val_ds

    @classmethod
    def multi_episode_split(
        cls,
        dataset_paths: list[str | Path],
        val_fraction: float = 0.15,
        seed: int = 42,
        backend: str = "local",
        train_transform=None,
        val_transform=None,
        **kwargs,
    ) -> tuple["ConcatDataset", "ConcatDataset"]:
        """
        Load multiple datasets, split each by episode, then concatenate.
        Each dataset is split independently so val_fraction is respected per-dataset.
        Returns (train_ConcatDataset, val_ConcatDataset).
        """
        from torch.utils.data import ConcatDataset
        train_list, val_list = [], []
        for path in dataset_paths:
            tr, va = cls.episode_split(
                path,
                val_fraction=val_fraction,
                seed=seed,
                backend=backend,
                train_transform=train_transform,
                val_transform=val_transform,
                **kwargs,
            )
            train_list.append(tr)
            val_list.append(va)
            logger.info(
                f"  {Path(path).name}: {len(tr)} train frames, {len(va)} val frames"
            )
        return ConcatDataset(train_list), ConcatDataset(val_list)
