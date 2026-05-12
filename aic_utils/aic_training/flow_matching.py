"""
Flow Matching policy for the AIC cable-insertion task.

Architecture uses a π0-style transformer (Black et al. 2024):
  - Conditioning tokens (image, robot state, task) are prepended as prefix tokens.
  - Noisy action tokens (with timestep embedding added) are appended.
  - A single transformer with full bidirectional self-attention processes the
    entire [prefix | action] sequence jointly — no separate cross-attention.
  - Only the action positions are read out and projected to the velocity field.

Loss  — Rectified Flow (conditional flow matching with straight paths):
    x_t  = (1 - t) * x_0  +  t * x_1        t ~ Uniform(0, 1)
    u_t  = x_1 - x_0                         constant velocity field
    L    = E[ ||v_θ(x_t, t) - u_t||² ]
  where x_0 ~ N(0, I) is noise and x_1 is the clean action.

Sampling  — ODE integration from t=0 (noise) to t=1 (action):
    "euler"    : x_{t+h} = x_t + h * v_θ(x_t, t)
    "midpoint" : 2nd-order midpoint, same NFE×2 as Euler but much more accurate

References:
  Black et al.  "π0: A Vision-Language-Action Flow Model" (2024).
  Lipman et al. "Flow Matching for Generative Modeling" (ICLR 2023).
  Liu et al.    "Flow Straight and Fast: Rectified Flow" (ICLR 2023).

All tokens share TOKEN_DIM = 128.
CFG is supported: train with cfg_dropout_prob > 0, sample with guidance_scale > 1.
"""

import math
import re
import time

import timm
from timm.data import create_transform, resolve_data_config
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
import torchvision.transforms.functional as TF
from torchvision.models.feature_extraction import create_feature_extractor
from typing import Callable, Union, List, Optional, Dict, Tuple, Any, Sequence
from transformers import AutoImageProcessor, AutoModel
import numpy as np


# ---------------------------------------------------------------------------
# Constants  (identical to diffusion_policy.py)
# ---------------------------------------------------------------------------

NUM_TARGET_MODULES = 10
NUM_PORT_NAMES = 3

ROBOT_STATE_DIM = 20  # tcp_pose(7) + joint_positions(7) + wrench(6)

TOKEN_DIM = 128
RESNET_FEATURE_DIM = 512   # layer4 output channels for ResNet18/34

# Flow matching uses continuous t ∈ [0, 1].  Multiply by this factor before
# the sinusoidal embedding so the input covers a wider frequency range
# (same effect as using T=1000 in DDPM).
_T_EMBED_SCALE = 1000.0


# ---------------------------------------------------------------------------
# Vision encoder helpers  (identical to diffusion_policy.py)
# ---------------------------------------------------------------------------

def get_resnet(name: str, weights=None, local_weights_path: str | None = None, **kwargs) -> nn.Module:
    func = getattr(torchvision.models, name)
    if local_weights_path is not None:
        # Load from a local .pth file — avoids any network download.
        # strict=False: the original fc.weight / fc.bias in the ImageNet checkpoint
        # are silently ignored after we replace fc with nn.Identity().
        resnet = func(weights=None, **kwargs)
        resnet.fc = nn.Identity()
        sd = torch.load(local_weights_path, map_location="cpu")
        resnet.load_state_dict(sd, strict=False)
    else:
        resnet = func(weights=weights, **kwargs)
        resnet.fc = nn.Identity()
    return resnet


def replace_submodules(
    root_module: nn.Module,
    predicate: Callable[[nn.Module], bool],
    func: Callable[[nn.Module], nn.Module],
) -> nn.Module:
    if predicate(root_module):
        return func(root_module)
    bn_list = [
        k.split(".")
        for k, m in root_module.named_modules(remove_duplicate=True)
        if predicate(m)
    ]
    for *parent, k in bn_list:
        parent_module = root_module
        if len(parent) > 0:
            parent_module = root_module.get_submodule(".".join(parent))
        if isinstance(parent_module, nn.Sequential):
            src_module = parent_module[int(k)]
        else:
            src_module = getattr(parent_module, k)
        tgt_module = func(src_module)
        if isinstance(parent_module, nn.Sequential):
            parent_module[int(k)] = tgt_module
        else:
            setattr(parent_module, k, tgt_module)
    remaining = [
        k for k, m in root_module.named_modules(remove_duplicate=True)
        if predicate(m)
    ]
    assert len(remaining) == 0
    return root_module


def replace_bn_with_gn(root_module: nn.Module, features_per_group: int = 16) -> nn.Module:
    replace_submodules(
        root_module=root_module,
        predicate=lambda x: isinstance(x, nn.BatchNorm2d),
        func=lambda x: nn.GroupNorm(
            num_groups=x.num_features // features_per_group,
            num_channels=x.num_features,
        ),
    )
    return root_module

# ---------------------------------------------------------------------------
# Image Encoder  (identical to diffusion_policy.py)
# ---------------------------------------------------------------------------

class ImageEncoder(nn.Module):
    """Image encoder using a ResNet18 backbone (torchvision).

    Extracts the layer4 spatial feature map (7×7 for 224×224 input, 512 ch) via
    create_feature_extractor, then projects each spatial cell to TOKEN_DIM,
    giving 49 patch tokens per camera per timestep.

    Output shape: (B, T * num_cameras * 49, TOKEN_DIM)
    """

    CAMERA_KEYS = ("left_camera", "center_camera", "right_camera")

    # Full ResNet18_Weights.IMAGENET1K_V1.transforms() pipeline on float tensors:
    #   resize shortest edge → 256, center-crop → 224×224, ImageNet normalize.
    _IMAGENET_MEAN = [0.485, 0.456, 0.406]
    _IMAGENET_STD  = [0.229, 0.224, 0.225]
    _RESIZE_SIZE   = 256
    _CROP_SIZE     = 224

    def __init__(self, resnet_name: str = "resnet18", weights="IMAGENET1K_V1",
                 features_per_group: int = 16,
                 camera_keys: tuple[str, ...] | None = None,
                 local_weights_path: str | None = None,
                 obs_horizon: int = 2,
                 pos_enc: str = "rope"):
        super().__init__()
        if camera_keys is not None:
            self.CAMERA_KEYS = camera_keys
        self.num_cameras = len(self.CAMERA_KEYS)

        backbone = get_resnet(resnet_name, weights=weights, local_weights_path=local_weights_path)
        backbone = replace_bn_with_gn(backbone, features_per_group=features_per_group)
        self.backbone = create_feature_extractor(backbone, return_nodes={"layer4": "feat"})

        self.proj       = nn.Linear(RESNET_FEATURE_DIM, TOKEN_DIM)
        if pos_enc == "rope":
            self.pos_enc2d = RoPE2D(dim=TOKEN_DIM)
        else:
            self.pos_enc2d = SinCos2D(dim=TOKEN_DIM)
        self.cam_id_emb = nn.Embedding(self.num_cameras, TOKEN_DIM)
        self.time_emb   = nn.Embedding(obs_horizon, TOKEN_DIM)

        self.register_buffer(
            "img_mean", torch.tensor(self._IMAGENET_MEAN, dtype=torch.float32).view(1, 3, 1, 1)
        )
        self.register_buffer(
            "img_std",  torch.tensor(self._IMAGENET_STD,  dtype=torch.float32).view(1, 3, 1, 1)
        )

    def forward(self, images: dict) -> torch.Tensor:
        B, T = next(iter(images.values())).shape[:2]
        device = next(iter(images.values())).device
        per_camera = []
        for cam_idx, key in enumerate(self.CAMERA_KEYS):
            img_flat = images[key].flatten(end_dim=1)                          # (B*T, C, H, W)
            img_flat = TF.resize(img_flat, self._RESIZE_SIZE, antialias=True)  # shortest edge → 256
            img_flat = TF.center_crop(img_flat, self._CROP_SIZE)               # → 224×224
            img_flat = (img_flat - self.img_mean) / self.img_std               # ImageNet normalize
            feat_map = self.backbone(img_flat)["feat"]                         # (B*T, 512, H, W)
            _, C, H, W = feat_map.shape
            patches = feat_map.permute(0, 2, 3, 1).reshape(B * T, H * W, C)  # (B*T, H*W, 512)
            tokens  = self.proj(patches)                                       # (B*T, H*W, TOKEN_DIM)
            tokens  = self.pos_enc2d(tokens, H, W)                            # 2D spatial PE
            tokens  = tokens + self.cam_id_emb.weight[cam_idx]                # camera ID
            tokens  = tokens.reshape(B, T, H * W, -1)
            t_idx   = torch.arange(T, device=device)
            tokens  = tokens + self.time_emb(t_idx).unsqueeze(0).unsqueeze(2) # (1,T,1,d)
            per_camera.append(tokens.reshape(B, T * H * W, TOKEN_DIM))
        return torch.cat(per_camera, dim=1)                                    # (B, cams*T*H*W, TOKEN_DIM)


# ---------------------------------------------------------------------------
# 2-D Rotary Position Embedding  (for image patch tokens)
# ---------------------------------------------------------------------------

class RoPE2D(nn.Module):
    """2D Rotary Position Embedding for row-major image patch tokens.

    Row positions modulate the first half of the embedding dimension;
    column positions modulate the second half. Each half uses standard
    1-D RoPE: x * cos(θ) + rotate_half(x) * sin(θ).

    dim must be divisible by 4 (two axes × two elements per rotation pair).
    """

    def __init__(self, dim: int, base: float = 10000.0):
        super().__init__()
        assert dim % 4 == 0, "dim must be divisible by 4 for 2D RoPE"
        half = dim // 2
        inv_freq = 1.0 / (base ** (torch.arange(0, half, 2).float() / half))
        self.register_buffer("inv_freq", inv_freq)  # (half//2,)

    @staticmethod
    def _rotate_half(x: torch.Tensor) -> torch.Tensor:
        x1, x2 = x[..., 0::2], x[..., 1::2]
        return torch.stack([-x2, x1], dim=-1).flatten(-2)

    def _apply_1d(self, x: torch.Tensor, positions: torch.Tensor) -> torch.Tensor:
        """Apply 1-D RoPE.  x: (..., N, d), positions: (N,)"""
        angles = positions.float().unsqueeze(-1) * self.inv_freq   # (N, d//2)
        cos = angles.cos().repeat_interleave(2, dim=-1)            # (N, d)
        sin = angles.sin().repeat_interleave(2, dim=-1)            # (N, d)
        return x * cos + self._rotate_half(x) * sin

    def forward(self, x: torch.Tensor, grid_h: int, grid_w: int) -> torch.Tensor:
        """
        Args:
            x:             (..., grid_h * grid_w, dim) — patches in row-major order
            grid_h, grid_w: spatial patch grid dimensions
        Returns:
            x with 2D RoPE applied in-place on a copy
        """
        device = x.device
        rows = torch.arange(grid_h, device=device).repeat_interleave(grid_w)  # (N,)
        cols = torch.arange(grid_w, device=device).repeat(grid_h)             # (N,)
        half = x.shape[-1] // 2
        x_row = self._apply_1d(x[..., :half], rows)
        x_col = self._apply_1d(x[..., half:], cols)
        return torch.cat([x_row, x_col], dim=-1)


# ---------------------------------------------------------------------------
# 2-D Sinusoidal Position Embedding  (for image patch tokens)
# ---------------------------------------------------------------------------

class SinCos2D(nn.Module):
    """Additive 2D sinusoidal positional encoding for row-major patch tokens.

    Encodes row in the first half of the embedding dimension and column in the
    second half, using the standard ViT/BERT sinusoidal formula:
        PE[pos, 2i]   = sin(pos / 10000^(2i / d))
        PE[pos, 2i+1] = cos(pos / 10000^(2i / d))

    The encoding is computed on the fly and not stored as a parameter, so it
    generalises to arbitrary grid sizes without retraining.
    dim must be divisible by 4 (two axes × even sinusoidal pairs).
    """

    def __init__(self, dim: int, base: float = 10000.0):
        super().__init__()
        assert dim % 4 == 0, "dim must be divisible by 4 for 2D SinCos"
        half = dim // 2
        # inv_freq shape: (half//2,) — shared for row and col axes
        inv_freq = 1.0 / (base ** (torch.arange(0, half, 2).float() / half))
        self.register_buffer("inv_freq", inv_freq)

    def _encode_1d(self, positions: torch.Tensor) -> torch.Tensor:
        """Return sinusoidal encoding for (N,) integer positions → (N, half_dim)."""
        angles = positions.float().unsqueeze(-1) * self.inv_freq  # (N, half//2)
        return torch.cat([angles.sin(), angles.cos()], dim=-1)    # (N, half)

    def forward(self, x: torch.Tensor, grid_h: int, grid_w: int) -> torch.Tensor:
        """
        Args:
            x:              (..., grid_h * grid_w, dim) — patches in row-major order
            grid_h, grid_w: spatial patch grid dimensions
        Returns:
            x + sinusoidal 2D position encoding (same shape as x)
        """
        device = x.device
        rows = torch.arange(grid_h, device=device).repeat_interleave(grid_w)  # (N,)
        cols = torch.arange(grid_w, device=device).repeat(grid_h)             # (N,)
        pe = torch.cat([self._encode_1d(rows), self._encode_1d(cols)], dim=-1)  # (N, dim)
        return x + pe


# ---------------------------------------------------------------------------
# DINO Image Encoder
# ---------------------------------------------------------------------------

class DINOImageEncoder(nn.Module):
    """Image encoder using a frozen DINOv2 ViT-S/14 backbone with 4 registers.

    Extracts the CLS token per camera per timestep and projects it to TOKEN_DIM.
    Output shape: (B, T * num_cameras, TOKEN_DIM)  —  vs ResNet's (B, T*cams*4, TOKEN_DIM).

    Usage:
        encoder = DINOImageEncoder("/path/to/dinov2_vits14_reg4_pretrain.pth")
    """

    CAMERA_KEYS = ImageEncoder.CAMERA_KEYS
    DINO_DIM = 384  # ViT-S embed_dim

    def __init__(self, checkpoint_path: str, freeze_backbone: bool = True,
                 camera_keys: tuple[str, ...] | None = None,
                 obs_horizon: int = 2,
                 pos_enc: str = "rope"):
        super().__init__()
        self._freeze_backbone = freeze_backbone
        if camera_keys is not None:
            self.CAMERA_KEYS = camera_keys

        self.processor = AutoImageProcessor.from_pretrained(checkpoint_path)
        self.backbone = AutoModel.from_pretrained(checkpoint_path)

        if freeze_backbone:
            for p in self.backbone.parameters():
                p.requires_grad_(False)

        self.num_cameras = len(self.CAMERA_KEYS)
        self._num_registers = getattr(self.backbone.config, "num_register_tokens", 0)

        self.proj        = nn.Linear(self.DINO_DIM, TOKEN_DIM)
        if pos_enc == "rope":
            self.pos_enc2d = RoPE2D(dim=TOKEN_DIM)
        else:
            self.pos_enc2d = SinCos2D(dim=TOKEN_DIM)
        self.cam_id_emb  = nn.Embedding(self.num_cameras, TOKEN_DIM)
        self.time_emb    = nn.Embedding(obs_horizon, TOKEN_DIM)

    def train(self, mode: bool = True):
        super().train(mode)
        if self._freeze_backbone:
            self.backbone.eval()
        return self

    def forward(self, images: dict) -> torch.Tensor:
        B, T = next(iter(images.values())).shape[:2]
        device = next(iter(images.values())).device
        per_camera = []
        for cam_idx, key in enumerate(self.CAMERA_KEYS):
            img_flat = images[key].flatten(end_dim=1)              # (B*T, C, H, W)
            inputs = self.processor(images=img_flat, return_tensors="pt")
            inputs = {k: v.to(device) for k, v in inputs.items()}
            with torch.no_grad():
                outputs = self.backbone(**inputs)
                # Skip CLS token and register tokens; remaining patches are row-major
                patches = outputs.last_hidden_state[
                    :, 1 + self._num_registers:, :
                ]                                                  # (B*T, N, DINO_DIM)
            N = patches.shape[1]
            #print("LXH debug patch dim:", N)
            grid_h = grid_w = int(math.isqrt(N))
            assert grid_h * grid_w == N, \
                f"Non-square patch grid for camera {key!r}: N={N}"
            tokens = self.proj(patches)                            # (B*T, N, TOKEN_DIM)
            tokens = self.pos_enc2d(tokens, grid_h, grid_w)       # 2D spatial PE
            tokens = tokens + self.cam_id_emb.weight[cam_idx]     # camera ID: (TOKEN_DIM,)
            # Temporal embedding: reshape to (B, T, N, d), add (T, d), reshape back
            tokens = tokens.reshape(B, T, N, -1)
            t_idx  = torch.arange(T, device=device)
            tokens = tokens + self.time_emb(t_idx).unsqueeze(0).unsqueeze(2)  # (1,T,1,d)
            per_camera.append(tokens.reshape(B, T * N, TOKEN_DIM))
        return torch.cat(per_camera, dim=1)                        # (B, cams*T*N, TOKEN_DIM)


# ---------------------------------------------------------------------------
# EfficientFormer Image Encoder
# ---------------------------------------------------------------------------

class EfformerImageEncoder(nn.Module):
    """Image encoder using a frozen EfficientFormerV2-S1 backbone (timm).

    Extracts the stage-2 feature map (14×14 spatial grid, 120 channels) and
    projects each spatial cell to TOKEN_DIM, giving 196 patch tokens per
    camera per timestep.

    Output shape: (B, T * num_cameras * 196, TOKEN_DIM)

    The stage-2 feature map sits at a good mid-level semantic resolution —
    finer than the final 7×7 stage but already past the pure-texture early
    stages.

    Key remapping from the original checkpoint format (stem.conv1, stages.0.*)
    to timm's FeatureListNet format (stem_conv1, stages_0.*) is done at load
    time; the classification head keys are silently dropped (strict=False).
    """

    CAMERA_KEYS  = ImageEncoder.CAMERA_KEYS
    STAGE2_DIM   = 120   # EfficientFormerV2-S1 stage-2 output channels
    STAGE2_IDX   = 2     # 0-based stage index

    def __init__(self, checkpoint_path: str, freeze_backbone: bool = True,
                 camera_keys: tuple[str, ...] | None = None,
                 obs_horizon: int = 2,
                 pos_enc: str = "rope"):
        super().__init__()
        self._freeze_backbone = freeze_backbone
        if camera_keys is not None:
            self.CAMERA_KEYS = camera_keys

        backbone = timm.create_model("efficientformerv2_s1", pretrained=False, features_only=True)
        raw_sd = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        remapped = {}
        for k, v in raw_sd.items():
            k = k.replace("stem.conv1", "stem_conv1").replace("stem.conv2", "stem_conv2")
            k = re.sub(r"stages\.(\d+)", r"stages_\1", k)
            remapped[k] = v
        backbone.load_state_dict(remapped, strict=False)
        self.backbone = backbone

        if freeze_backbone:
            for p in self.backbone.parameters():
                p.requires_grad_(False)

        # Derive preprocessing transform and normalisation stats from the model itself.
        # self.transform is the canonical PIL pipeline; _input_* / _resize_* replicate
        # the same resize → center-crop → normalize sequence on tensors in forward().
        config = resolve_data_config({}, model=self.backbone)
        transform = create_transform(**config)
        print("EfficientTransformer transform loaded:", transform)
        print("EfficientTransformer config loaded:", config)
        # Extract resize/crop sizes directly from the transform to stay in sync:
        #   Resize(shorter-edge → _resize_size)  →  CenterCrop(_input_h, _input_w)
        resize_step = transform.transforms[0]
        self._resize_size = (resize_step.size if isinstance(resize_step.size, int)
                             else resize_step.size[0])
        _, self._input_h, self._input_w = config["input_size"]
        self.register_buffer(
            "img_mean", torch.tensor(list(config["mean"]), dtype=torch.float32).view(1, 3, 1, 1)
        )
        self.register_buffer(
            "img_std",  torch.tensor(list(config["std"]),  dtype=torch.float32).view(1, 3, 1, 1)
        )

        self.num_cameras = len(self.CAMERA_KEYS)

        self.proj        = nn.Linear(self.STAGE2_DIM, TOKEN_DIM)
        if pos_enc == "rope":
            self.pos_enc2d = RoPE2D(dim=TOKEN_DIM)
        else:
            self.pos_enc2d = SinCos2D(dim=TOKEN_DIM)
        self.cam_id_emb  = nn.Embedding(self.num_cameras, TOKEN_DIM)
        self.time_emb    = nn.Embedding(obs_horizon, TOKEN_DIM)

    def train(self, mode: bool = True):
        super().train(mode)
        if self._freeze_backbone:
            self.backbone.eval()
        return self

    def forward(self, images: dict) -> torch.Tensor:
        B, T = next(iter(images.values())).shape[:2]
        device = next(iter(images.values())).device
        per_camera = []
        for cam_idx, key in enumerate(self.CAMERA_KEYS):
            img_flat = images[key].flatten(end_dim=1).float()        # (B*T, C, H, W)
            # Replicate self.transform on tensors:
            #   Resize(shorter-edge → _resize_size, bicubic) → CenterCrop → Normalize
            img_flat = TF.resize(img_flat, self._resize_size,
                                 interpolation=TF.InterpolationMode.BICUBIC, antialias=True)
            img_flat = TF.center_crop(img_flat, [self._input_h, self._input_w])
            img_flat = (img_flat - self.img_mean) / self.img_std
            with torch.no_grad():
                feat = self.backbone(img_flat)[self.STAGE2_IDX]     # (B*T, STAGE2_DIM, H, W)
            BT, C, H, W = feat.shape # for stage 2 it should be BT x 120 x 14 x 14
            # Row-major flatten: (B*T, H*W, C)
            patches = feat.permute(0, 2, 3, 1).reshape(BT, H * W, C)
            tokens = self.proj(patches)                             # (B*T, N, TOKEN_DIM)
            tokens = self.pos_enc2d(tokens, H, W)                  # 2D spatial PE
            tokens = tokens + self.cam_id_emb.weight[cam_idx]      # camera ID: (TOKEN_DIM,)
            # Temporal embedding
            tokens = tokens.reshape(B, T, H * W, -1)
            t_idx  = torch.arange(T, device=device)
            tokens = tokens + self.time_emb(t_idx).unsqueeze(0).unsqueeze(2)  # (1,T,1,d)
            per_camera.append(tokens.reshape(B, T * H * W, TOKEN_DIM))
        return torch.cat(per_camera, dim=1)                         # (B, cams*T*N, TOKEN_DIM)


# ---------------------------------------------------------------------------
# Timestep Encoder  (identical to diffusion_policy.py; t is scaled internally)
# ---------------------------------------------------------------------------

class TimestepEncoder(nn.Module):
    """Encodes a continuous time t ∈ [0, 1] into a TOKEN_DIM embedding.

    t is multiplied by _T_EMBED_SCALE before the sinusoidal embedding so the
    frequencies cover the same range as a discrete DDPM with T=1000 steps.
    """

    def __init__(self, embed_dim: int = TOKEN_DIM):
        super().__init__()
        self.embed_dim = embed_dim
        assert embed_dim % 2 == 0

        self.mlp = nn.Sequential(
            nn.Linear(embed_dim, embed_dim * 4),
            nn.SiLU(),
            nn.Linear(embed_dim * 4, embed_dim),
        )

    def _sinusoidal_embedding(self, t: torch.Tensor) -> torch.Tensor:
        half = self.embed_dim // 2
        freqs = torch.exp(
            -torch.arange(half, dtype=torch.float32, device=t.device)
            * (torch.log(torch.tensor(10000.0)) / (half - 1))
        )
        args = t.float().unsqueeze(1) * freqs.unsqueeze(0)
        return torch.cat([args.sin(), args.cos()], dim=-1)

    def forward(self, t: torch.Tensor) -> torch.Tensor:
        """
        Args:
            t: (B,) continuous time in [0, 1]
        Returns:
            emb: (B, TOKEN_DIM)
        """
        return self.mlp(self._sinusoidal_embedding(t * _T_EMBED_SCALE))


# ---------------------------------------------------------------------------
# Task Encoder  (identical to diffusion_policy.py)
# ---------------------------------------------------------------------------

class TaskEncoder(nn.Module):
    NUM_TASKS = 12

    _LOOKUP = torch.full((NUM_TARGET_MODULES, NUM_PORT_NAMES), fill_value=-1, dtype=torch.long)
    for _m in range(5):
        _LOOKUP[_m, 0] = _m * 2
        _LOOKUP[_m, 1] = _m * 2 + 1
    _LOOKUP[5, 2] = 10
    _LOOKUP[6, 2] = 11

    def __init__(self, task_embed_dim: int = TOKEN_DIM):
        super().__init__()
        self.task_embed_dim = task_embed_dim
        self.embedding = nn.Embedding(self.NUM_TASKS, task_embed_dim)
        self.null_token = nn.Parameter(torch.zeros(task_embed_dim))
        self.register_buffer("lookup", self._LOOKUP.clone())

    def uncond_token(self, batch_size: int, device: torch.device) -> torch.Tensor:
        return self.null_token.view(1, 1, -1).expand(batch_size, 1, -1)

    def forward(self, target_module: torch.Tensor, port_name: torch.Tensor) -> torch.Tensor:
        task_idx = self.lookup[target_module, port_name]
        if (task_idx < 0).any():
            bad_tm = target_module[task_idx < 0].tolist()
            bad_pn = port_name[task_idx < 0].tolist()
            raise ValueError(
                f"Unknown (target_module, port_name) pairs (no task index assigned): "
                f"{list(zip(bad_tm, bad_pn))}"
            )
        return self.embedding(task_idx).unsqueeze(1)


# ---------------------------------------------------------------------------
# AdaLN-Zero  (DiT: Peebles & Xie 2023)
# ---------------------------------------------------------------------------

class AdaLNZero(nn.Module):
    """Adaptive LayerNorm-Zero conditioning for one transformer layer.

    A single conditioning vector c (timestep + task) is projected to
    shift / scale / gate parameters for both the self-attention and FFN
    sub-layers.  The output linear is zero-initialized so the model starts
    as an identity mapping (training-stability trick from DiT).
    """

    def __init__(self, d_model: int, cond_dim: int | None = None):
        super().__init__()
        if cond_dim is None:
            cond_dim = d_model
        self.silu   = nn.SiLU()
        self.linear = nn.Linear(cond_dim, 6 * d_model)
        self.norm   = nn.LayerNorm(d_model, elementwise_affine=False)
        nn.init.zeros_(self.linear.weight)
        nn.init.zeros_(self.linear.bias)

    def forward(self, x: torch.Tensor, c: torch.Tensor):
        """
        Args:
            x: (B, L, d)    — token sequence
            c: (B, cond_dim) — conditioning embedding (e.g. cat([timestep, task]))
        Returns:
            x_norm:   (B, L, d) — adaptively normalized x for self-attention
            gate_sa:  (B, 1, d) — residual gate for self-attention
            shift_ff: (B, 1, d) — shift for FFN pre-norm
            scale_ff: (B, 1, d) — scale for FFN pre-norm
            gate_ff:  (B, 1, d) — residual gate for FFN
        """
        p = self.linear(self.silu(c)).unsqueeze(1)            # (B, 1, 6*d)
        shift_sa, scale_sa, gate_sa, shift_ff, scale_ff, gate_ff = p.chunk(6, dim=-1)
        x_norm = self.norm(x) * (1 + scale_sa) + shift_sa
        return x_norm, gate_sa, shift_ff, scale_ff, gate_ff


# ---------------------------------------------------------------------------
# Transformer  (π0-style prefix attention + AdaLN-Zero conditioning)
# ---------------------------------------------------------------------------

class TransformerLayer(nn.Module):
    """Transformer layer with AdaLN-Zero conditioning.

    Self-attention and FFN norms are both driven by a combined
    (timestep + task) conditioning vector via AdaLN-Zero.
    No cross-attention — image/state tokens are part of the sequence.
    """

    def __init__(self, d_model: int, n_heads: int, ffn_dim: int, dropout: float = 0.0,
                 cond_dim: int | None = None):
        super().__init__()
        self.self_attn = nn.MultiheadAttention(d_model, n_heads, dropout=dropout, batch_first=True)
        self.ffn = nn.Sequential(
            nn.Linear(d_model, ffn_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(ffn_dim, d_model),
        )
        self.adaLN = AdaLNZero(d_model, cond_dim=cond_dim)
        self.drop  = nn.Dropout(dropout)

    def forward(self, x: torch.Tensor, cond: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x:    (B, L, d) — full [prefix | action] token sequence
            cond: (B, d)    — conditioning embedding (timestep + task)
        """
        x_norm, gate_sa, shift_ff, scale_ff, gate_ff = self.adaLN(x, cond)

        # Self-attention with gated residual
        attn_out, _ = self.self_attn(x_norm, x_norm, x_norm)
        x = x + gate_sa * self.drop(attn_out)

        # FFN with adaptive pre-norm and gated residual
        x_ff = self.adaLN.norm(x) * (1 + scale_ff) + shift_ff
        x = x + gate_ff * self.drop(self.ffn(x_ff))
        return x


class VectorFieldTransformer(nn.Module):
    """π0-style transformer predicting the velocity field v_θ(x_t, t).

    Image and robot-state tokens are prepended as a prefix and attended to
    jointly with action tokens via full bidirectional self-attention.

    Timestep and task identity condition every layer via AdaLN-Zero: their
    embeddings are summed into a single vector c that drives the adaptive
    LayerNorm scale / shift / gate in each TransformerLayer.

    Sequence layout per forward call:
        [ img_tokens   (B, T*cams*4, d) |   ← prefix (attend freely)
          state_tokens (B, T, d)        |
          action_tokens (B, H, d)       ]   ← only these projected to output

    Conditioning (AdaLN-Zero, not in the sequence):
        c = cat([timestep_emb, task_token], dim=-1)  → shift / scale / gate per layer
    """

    def __init__(self, action_dim: int, pred_horizon: int, d_model: int = TOKEN_DIM,
                 n_heads: int = 8, n_layers: int = 4, ffn_dim: int = 1024, dropout: float = 0.0):
        super().__init__()
        self.norm_img   = nn.LayerNorm(d_model)
        self.norm_state = nn.LayerNorm(d_model)
        self.layers = nn.ModuleList([
            TransformerLayer(d_model=d_model, n_heads=n_heads, ffn_dim=ffn_dim, dropout=dropout,
                             cond_dim=2 * d_model)
            for _ in range(n_layers)
        ])
        self.norm_out = nn.LayerNorm(d_model)
        self.out_proj = nn.Linear(d_model, action_dim)

        # Pre-compute sinusoidal PE for the fixed prediction horizon (1, H, d_model)
        pos = torch.arange(pred_horizon).unsqueeze(1)              # (H, 1)
        i   = torch.arange(d_model // 2).unsqueeze(0)             # (1, d/2)
        angles = pos / (10000.0 ** (2.0 * i / d_model))           # (H, d/2)
        pe = torch.cat([angles.sin(), angles.cos()], dim=-1)      # (H, d_model)
        self.register_buffer("pos_emb", pe.unsqueeze(0))          # (1, H, d_model)

    def encode_prefix(self, img_tokens: torch.Tensor, state_tokens: torch.Tensor) -> torch.Tensor:
        """Normalise and concatenate image and state tokens into the prefix.

        Call once per sample() invocation; the result is constant across all ODE steps.
        """
        return torch.cat([self.norm_img(img_tokens), self.norm_state(state_tokens)], dim=1)

    def forward(self, action_tokens, timestep_emb, task_token,
                img_tokens=None, state_tokens=None, prefix_cache=None):
        # AdaLN conditioning: concatenate timestep and task embeddings so each
        # signal stays independent before the AdaLNZero linear mixes them
        cond = torch.cat([timestep_emb, task_token.squeeze(1)], dim=-1)  # (B, 2*d_model)

        # Prefix: use pre-computed cache when available (constant across ODE steps)
        if prefix_cache is not None:
            prefix = prefix_cache
        else:
            prefix = torch.cat([self.norm_img(img_tokens), self.norm_state(state_tokens)], dim=1)
        n_prefix = prefix.shape[1]

        # Add pre-computed sinusoidal PE to action tokens
        action_tokens = action_tokens + self.pos_emb[:, :action_tokens.shape[1]]

        # Full sequence: prefix + action tokens
        x = torch.cat([prefix, action_tokens], dim=1)         # (B, n_prefix + H, d_model)

        for layer in self.layers:
            x = layer(x, cond)

        # Read out only the action positions
        x = x[:, n_prefix:]                                   # (B, H, d_model)
        return self.out_proj(self.norm_out(x))


# ---------------------------------------------------------------------------
# Flow Matching Policy
# ---------------------------------------------------------------------------

class FlowMatchingPolicy(nn.Module):
    """Flow matching policy for the AIC cable-insertion task.

    Uses a π0-style transformer: all conditioning tokens (images, robot state,
    task) are prepended as a prefix and processed jointly with the noisy action
    tokens via full bidirectional self-attention — no separate cross-attention.

    Training loss  — rectified flow objective:
        x_t  = (1 - t) * x_0  +  t * x_1     (straight interpolation)
        u_t  = x_1 - x_0                       (target velocity, constant)
        L    = MSE(v_θ(x_t, t), u_t)

    Sampling  — ODE integration from t=0 (noise) to t=1 (action):
        "euler"    : first-order,   1 NFE per step
        "midpoint" : second-order,  2 NFE per step, much better accuracy

    CFG is supported: train with cfg_dropout_prob > 0, sample with guidance_scale > 1.
    """

    def __init__(
        self,
        obs_horizon: int = 2,
        pred_horizon: int = 16,
        action_dim: int = 7,
        robot_state_dim: int = ROBOT_STATE_DIM,
        d_model: int = TOKEN_DIM,
        n_heads: int = 8,
        n_layers: int = 4,
        ffn_dim: int = 1024,
        dropout: float = 0.0,
        image_encoder_type: str = "resnet",
        resnet_name: str = "resnet18",
        resnet_weights: str = "IMAGENET1K_V1",
        local_ckpt_path: str | None = None,
        camera_keys: tuple[str, ...] | None = None,
        cfg_dropout_prob: float = 0.0,
        pos_enc: str = "rope",
    ):
        super().__init__()
        self.obs_horizon        = obs_horizon
        self.pred_horizon       = pred_horizon
        self.action_dim         = action_dim
        self.cfg_dropout_prob   = cfg_dropout_prob
        self.image_encoder_type = image_encoder_type

        # Profiling (disabled by default; set policy.profiling = True to enable)
        self.profiling       = False
        self._prof_log_every = 50
        self._prof_n         = 0
        self._prof_acc: dict[str, float] = {}

        if image_encoder_type == "resnet":
            self.image_encoder = ImageEncoder(resnet_name=resnet_name, weights=resnet_weights,
                                              local_weights_path=local_ckpt_path,
                                              camera_keys=camera_keys,
                                              obs_horizon=obs_horizon,
                                              pos_enc=pos_enc)
        elif image_encoder_type == "dino":
            if local_ckpt_path is None:
                raise ValueError("dino_checkpoint must be provided when image_encoder_type='dino'")
            self.image_encoder = DINOImageEncoder(checkpoint_path=local_ckpt_path,
                                                  camera_keys=camera_keys,
                                                  obs_horizon=obs_horizon,
                                                  pos_enc=pos_enc)
        elif image_encoder_type == "efformer":
            if local_ckpt_path is None:
                raise ValueError(
                    "efformer_checkpoint must be provided when image_encoder_type='efformer'"
                )
            self.image_encoder = EfformerImageEncoder(checkpoint_path=local_ckpt_path,
                                                      camera_keys=camera_keys,
                                                      obs_horizon=obs_horizon,
                                                      pos_enc=pos_enc)
        else:
            raise ValueError(
                f"Unknown image_encoder_type {image_encoder_type!r}. "
                f"Choose 'resnet', 'dino', or 'efformer'."
            )
        self.timestep_encoder  = TimestepEncoder(embed_dim=d_model)
        self.task_encoder      = TaskEncoder(task_embed_dim=d_model)
        self.state_encoder     = nn.Sequential(
            nn.Linear(robot_state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, d_model),
        )
        # Separate temporal PE for state tokens (independent from image temporal PE
        # in DINOImageEncoder — the two modalities have very different value ranges)
        self.state_time_emb = nn.Embedding(obs_horizon, d_model)
        self.action_in_proj = nn.Linear(action_dim, d_model)
        self.vector_field_net = VectorFieldTransformer(
            action_dim=action_dim,
            pred_horizon=pred_horizon,
            d_model=d_model,
            n_heads=n_heads,
            n_layers=n_layers,
            ffn_dim=ffn_dim,
            dropout=dropout,
        )

    # ------------------------------------------------------------------
    # Profiling helpers  (identical to DiffusionPolicy)
    # ------------------------------------------------------------------

    def _tock(self, t0: float, name: str, device: torch.device) -> float:
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        t1 = time.perf_counter()
        self._prof_acc[name] = self._prof_acc.get(name, 0.0) + (t1 - t0) * 1e3
        return t1

    def _prof_report(self) -> None:
        n = max(self._prof_n, 1)
        lines = [f"FlowMatchingPolicy forward profile (avg over last {n} calls):"]
        total = 0.0
        for name, acc in self._prof_acc.items():
            avg = acc / n
            total += avg
            lines.append(f"  {name:<20} {avg:6.2f} ms")
        lines.append(f"  {'TOTAL':<20} {total:6.2f} ms")
        print("\n".join(lines), flush=True)
        self._prof_n   = 0
        self._prof_acc = {}

    # ------------------------------------------------------------------
    # Forward  (condition encoding identical to DiffusionPolicy)
    # ------------------------------------------------------------------

    def forward(
        self,
        images: dict,
        robot_state: torch.Tensor,
        target_module: torch.Tensor,
        port_name: torch.Tensor,
        x_t: torch.Tensor,
        t: torch.Tensor,
        use_uncond: bool = False,
        img_tokens_cache: torch.Tensor | None = None,
        state_tokens_cache: torch.Tensor | None = None,
        task_token_cache: torch.Tensor | None = None,
        prefix_cache: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Predict velocity field v_θ(x_t, t).

        Args:
            images:             {camera_key: (B, obs_horizon, C, H, W)}
            robot_state:        (B, obs_horizon, ROBOT_STATE_DIM)
            target_module:      (B,) int64
            port_name:          (B,) int64
            x_t:                (B, pred_horizon, action_dim) — interpolated action at time t
            t:                  (B,) float32 in [0, 1] — flow matching time
            use_uncond:         if True, zero all conditions for the CFG unconditional pass.
            img_tokens_cache:   pre-computed image tokens; skips image encoder if provided
            state_tokens_cache: pre-computed state tokens; skips state encoder if provided
            task_token_cache:   pre-computed task token (cond or null); skips task encoder
            prefix_cache:       pre-computed encode_prefix() result; skips both token
                                encoders and the LayerNorm inside vector_field_net.
                                Inference-only — CFG dropout during training will not work.

        Returns:
            v_pred: (B, pred_horizon, action_dim) — predicted velocity field
        """
        B   = x_t.shape[0]
        dev = x_t.device

        if self.profiling:
            if dev.type == "cuda":
                torch.cuda.synchronize(dev)
            _t = time.perf_counter()

        # When the normed prefix is pre-computed, skip all img/state encoding.
        if prefix_cache is None:
            if img_tokens_cache is not None:
                img_tokens = img_tokens_cache
            else:
                img_tokens = self.image_encoder(images)

            if self.profiling and img_tokens_cache is None:
                _t = self._tock(_t, "image_encoder", dev)
            elif self.profiling:
                if dev.type == "cuda":
                    torch.cuda.synchronize(dev)
                _t = time.perf_counter()

            if state_tokens_cache is not None:
                state_tokens = state_tokens_cache
            else:
                state_tokens = self.state_encoder(
                    robot_state.flatten(end_dim=1)
                ).reshape(B, self.obs_horizon, -1)
                t_idx = torch.arange(self.obs_horizon, device=dev)
                state_tokens = state_tokens + self.state_time_emb(t_idx).unsqueeze(0)

            if self.profiling:
                _t = self._tock(_t, "state_encoder", dev)

            if use_uncond:
                img_tokens   = torch.zeros_like(img_tokens)
                state_tokens = torch.zeros_like(state_tokens)
        else:
            img_tokens   = None  # prefix_cache already encodes these
            state_tokens = None
            if self.profiling:
                if dev.type == "cuda":
                    torch.cuda.synchronize(dev)
                _t = time.perf_counter()

        # Task token
        if use_uncond:
            task_token = (task_token_cache if task_token_cache is not None
                          else self.task_encoder.uncond_token(B, dev))
        else:
            if task_token_cache is not None:
                task_token = task_token_cache
            else:
                task_token = self.task_encoder(target_module, port_name)
            if self.cfg_dropout_prob > 0.0 and self.training:
                # Drop all conditions jointly with the same mask.
                drop_mask    = (torch.rand(B, device=dev) < self.cfg_dropout_prob).view(B, 1, 1)
                img_tokens   = torch.where(drop_mask, torch.zeros_like(img_tokens),   img_tokens)
                state_tokens = torch.where(drop_mask, torch.zeros_like(state_tokens), state_tokens)
                null_token   = self.task_encoder.uncond_token(B, dev)
                task_token   = torch.where(drop_mask, null_token, task_token)

        timestep_emb  = self.timestep_encoder(t)
        action_tokens = self.action_in_proj(x_t)

        if self.profiling:
            _t = self._tock(_t, "cond_encoders", dev)

        v_pred = self.vector_field_net(
            action_tokens=action_tokens,
            timestep_emb=timestep_emb,
            task_token=task_token,
            img_tokens=img_tokens,
            state_tokens=state_tokens,
            prefix_cache=prefix_cache,
        )

        if self.profiling:
            self._tock(_t, "vector_field_net", dev)
            self._prof_n += 1
            if self._prof_n >= self._prof_log_every:
                self._prof_report()

        return v_pred

    # ------------------------------------------------------------------
    # Loss  — rectified flow objective
    # ------------------------------------------------------------------

    def compute_loss(
        self,
        images: dict,
        robot_state: torch.Tensor,
        target_module: torch.Tensor,
        port_name: torch.Tensor,
        actions: torch.Tensor,
    ) -> torch.Tensor:
        """Compute the rectified flow matching loss.

        Samples a random time t ~ Uniform(0, 1) per batch element, constructs
        the linear interpolation x_t = (1-t)*x_0 + t*x_1, and trains the
        network to predict the constant velocity u_t = x_1 - x_0.

        Args:
            images:        {camera_key: (B, obs_horizon, C, H, W)}
            robot_state:   (B, obs_horizon, ROBOT_STATE_DIM)
            target_module: (B,) int64
            port_name:     (B,) int64
            actions:       (B, pred_horizon, action_dim) — clean action x_1

        Returns:
            loss: scalar MSE loss
        """
        B      = actions.shape[0]
        device = actions.device

        ## Log image size and value range (first call only, gated by a counter)
        #if not hasattr(self, '_img_log_count'):
        #    self._img_log_count = 0
        #if self._img_log_count < 3:
        #    for key, img in images.items():
        #        print(
        #            f"[compute_loss] images[{key!r}] shape={tuple(img.shape)}  "
        #            f"dtype={img.dtype}  min={img.min().item():.4f}  "
        #            f"max={img.max().item():.4f}  mean={img.mean().item():.4f}",
        #            flush=True,
        #        )
        #    self._img_log_count += 1

        # Sample t ~ Uniform(0, 1) per batch element
        t = torch.rand(B, device=device)                          # (B,)

        # Sample noise x_noise ~ N(0, I)
        x_noise = torch.randn_like(actions)                       # (B, pred_horizon, action_dim)

        # Linear interpolation: x_t = (1-t)*x_noise + t*x_1
        t_bc = t[:, None, None]                                   # broadcast over horizon & dim
        x_t  = (1.0 - t_bc) * x_noise + t_bc * actions

        # Constant target velocity field: u_t = x_1 - x_noise
        u_t = actions - x_noise                                   # (B, pred_horizon, action_dim)

        # Predict velocity and compute MSE
        v_pred = self(
            images=images,
            robot_state=robot_state,
            target_module=target_module,
            port_name=port_name,
            x_t=x_t,
            t=t,
        )

        return F.mse_loss(v_pred, u_t)

    # ------------------------------------------------------------------
    # Sampling  — ODE integration from t=0 (noise) to t=1 (action)
    # ------------------------------------------------------------------

    @torch.no_grad()
    def sample(
        self,
        images: dict,
        robot_state: torch.Tensor,
        target_module: torch.Tensor,
        port_name: torch.Tensor,
        num_steps: int = 10,
        guidance_scale: float = 1.0,
        solver: str = "midpoint",
    ) -> torch.Tensor:
        """Integrate the learned ODE from t=0 (noise) to t=1 (clean action).

        "euler"    (1st order): x_{t+h} = x_t + h * v_θ(x_t, t)
        "midpoint" (2nd order): evaluate v_θ at the midpoint of each step.
                                2× NFE of Euler but much better accuracy;
                                recommended for num_steps ≤ 20.

        CFG: when guidance_scale != 1.0, two forward passes are run per
        evaluation point and the velocity fields are combined as:
            v = v_uncond + guidance_scale * (v_cond - v_uncond)

        Args:
            images:         {camera_key: (B, obs_horizon, C, H, W)}
            robot_state:    (B, obs_horizon, ROBOT_STATE_DIM)
            target_module:  (B,) int64
            port_name:      (B,) int64
            num_steps:      number of ODE integration steps
            guidance_scale: CFG scale (1.0 = disabled)
            solver:         "euler" or "midpoint"

        Returns:
            actions: (B, pred_horizon, action_dim) — denoised action sequence
        """
        device  = robot_state.device
        B       = robot_state.shape[0]
        use_cfg = guidance_scale != 1.0
        h       = 1.0 / num_steps                                 # step size in [0, 1]

        # Pre-encode everything constant across ODE steps (images, state, task, prefix)
        if self.profiling:
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            _t_enc = time.perf_counter()
        img_tokens      = self.image_encoder(images)
        state_tokens    = self.state_encoder(
            robot_state.flatten(end_dim=1)
        ).reshape(B, self.obs_horizon, -1)
        t_idx        = torch.arange(self.obs_horizon, device=device)
        state_tokens = state_tokens + self.state_time_emb(t_idx).unsqueeze(0)
        task_token_cond = self.task_encoder(target_module, port_name)
        prefix_cond     = self.vector_field_net.encode_prefix(img_tokens, state_tokens)
        if self.profiling:
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            print(f"[profile] pre-encode (once per sample): "
                  f"{(time.perf_counter() - _t_enc)*1e3:.2f} ms", flush=True)

        if use_cfg:
            prefix_uncond     = self.vector_field_net.encode_prefix(
                torch.zeros_like(img_tokens),
                torch.zeros_like(state_tokens),
            )
            task_token_uncond = self.task_encoder.uncond_token(B, device)
            # Stack cond/uncond once so the hot loop runs a single 2B forward pass
            # instead of two sequential B-size passes.
            prefix_2b = torch.cat([prefix_cond, prefix_uncond], dim=0)
            task_2b   = torch.cat([task_token_cond, task_token_uncond], dim=0)

        def eval_v(xt: torch.Tensor, t_val: float) -> torch.Tensor:
            """Evaluate velocity field (with optional CFG) at a given t."""
            t_tensor = torch.full((B,), t_val, dtype=torch.float32, device=device)
            if use_cfg:
                # Single 2B forward pass: first B = cond, second B = uncond
                v_out = self(
                    images=images, robot_state=robot_state,
                    target_module=target_module, port_name=port_name,
                    x_t=torch.cat([xt, xt], dim=0),
                    t=torch.cat([t_tensor, t_tensor], dim=0),
                    task_token_cache=task_2b,
                    prefix_cache=prefix_2b,
                )
                v_cond, v_uncond = v_out[:B], v_out[B:]
                return v_uncond + guidance_scale * (v_cond - v_uncond)
            return self(
                images=images, robot_state=robot_state,
                target_module=target_module, port_name=port_name,
                x_t=xt, t=t_tensor,
                task_token_cache=task_token_cond,
                prefix_cache=prefix_cond,
            )

        # Start from pure noise at t=0
        x = torch.randn(B, self.pred_horizon, self.action_dim, device=device)

        if solver == "euler":
            # 1st-order Euler integration
            for i in range(num_steps):
                t_val = i * h
                x = x + h * eval_v(x, t_val)

        elif solver == "midpoint":
            # 2nd-order midpoint (Heun's predictor-corrector variant):
            #   x_mid = x_t + (h/2) * v(x_t, t)
            #   x_{t+h} = x_t + h * v(x_mid, t + h/2)
            for i in range(num_steps):
                t_val = i * h
                v1    = eval_v(x, t_val)
                x_mid = x + (h / 2.0) * v1
                v2    = eval_v(x_mid, t_val + h / 2.0)
                x     = x + h * v2

        else:
            raise ValueError(f"Unknown solver {solver!r}. Choose 'euler' or 'midpoint'.")

        return x  # (B, pred_horizon, action_dim)
