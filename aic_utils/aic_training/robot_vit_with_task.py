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

import copy
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
                 obs_horizon: int = 2):
        super().__init__()
        if camera_keys is not None:
            self.CAMERA_KEYS = camera_keys
        self.num_cameras = len(self.CAMERA_KEYS)

        backbone = get_resnet(resnet_name, weights=weights, local_weights_path=local_weights_path)
        backbone = replace_bn_with_gn(backbone, features_per_group=features_per_group)
        self.backbone = create_feature_extractor(backbone, return_nodes={"layer4": "feat"})

        self.proj       = nn.Linear(RESNET_FEATURE_DIM, TOKEN_DIM)
        self.pos_enc2d  = SinCos2D(dim=TOKEN_DIM)
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


# ---------------------------------------------------------------------------
# Image Conditioner  (global-pooled ResNet18 → AdaLN conditioning vector)
# ---------------------------------------------------------------------------

class ImageConditioner(nn.Module):
    """Shared-backbone + per-camera-head image conditioner for AdaLN-Zero.

    Shared frozen backbone (BN→GN):
        conv1 → bn1 → relu → maxpool → layer1 → layer2 → layer3
        output: (B, 256, 14×14) for 224×224 input

    Per-camera trainable head (layer4 conv weights initialised from ImageNet1K):
        layer4 (BN→GN) → AdaptiveAvgPool2d(1,1) → (B, 512) → Linear → (B, TOKEN_DIM)

    Camera embeddings are concatenated so each camera keeps a distinct channel in
    the conditioning vector:
        output: (B, num_cameras * TOKEN_DIM)
    """

    CAMERA_KEYS = ImageEncoder.CAMERA_KEYS
    _IMAGENET_MEAN = ImageEncoder._IMAGENET_MEAN
    _IMAGENET_STD  = ImageEncoder._IMAGENET_STD
    _RESIZE_SIZE   = ImageEncoder._RESIZE_SIZE
    _CROP_SIZE     = ImageEncoder._CROP_SIZE

    def __init__(
        self,
        resnet_name: str = "resnet18",
        weights: str = "IMAGENET1K_V1",
        features_per_group: int = 16,
        camera_keys: tuple[str, ...] | None = None,
        local_weights_path: str | None = None,
    ):
        super().__init__()
        if camera_keys is not None:
            self.CAMERA_KEYS = camera_keys
        self.num_cameras = len(self.CAMERA_KEYS)

        # Load full ResNet18 with ImageNet weights (BN still intact at this point)
        resnet = get_resnet(resnet_name, weights=weights, local_weights_path=local_weights_path)

        # Shared backbone: conv1 .. layer3 — apply BN→GN, then freeze
        shared = nn.Sequential(
            resnet.conv1, resnet.bn1, resnet.relu, resnet.maxpool,
            resnet.layer1, resnet.layer2, resnet.layer3,
        )
        replace_bn_with_gn(shared, features_per_group=features_per_group)
        for p in shared.parameters():
            p.requires_grad_(False)
        self.shared_backbone = shared

        # Per-camera heads: independent copy of layer4 (conv weights from ImageNet),
        # BN→GN applied, followed by global pool and TOKEN_DIM projection.
        self.camera_heads = nn.ModuleList()
        for _ in range(self.num_cameras):
            layer4 = copy.deepcopy(resnet.layer4)
            replace_bn_with_gn(layer4, features_per_group=features_per_group)
            self.camera_heads.append(nn.Sequential(
                layer4,
                nn.AdaptiveAvgPool2d((1, 1)),
                nn.Flatten(),
                nn.Linear(RESNET_FEATURE_DIM, TOKEN_DIM),
            ))

        self.register_buffer(
            "img_mean", torch.tensor(self._IMAGENET_MEAN, dtype=torch.float32).view(1, 3, 1, 1)
        )
        self.register_buffer(
            "img_std", torch.tensor(self._IMAGENET_STD, dtype=torch.float32).view(1, 3, 1, 1)
        )

    def forward(self, images: dict) -> torch.Tensor:
        """
        Args:
            images: {camera_key: (B, T, C, H, W) or (B, C, H, W)}
        Returns:
            cond: (B, num_cameras * TOKEN_DIM) — per-camera embeddings concatenated
        """
        cam_embeds = []
        for cam_idx, key in enumerate(self.CAMERA_KEYS):
            img = images[key]
            if img.dim() == 5:
                img = img[:, -1]                                          # most-recent timestep
            img = TF.resize(img, self._RESIZE_SIZE, antialias=True)
            img = TF.center_crop(img, self._CROP_SIZE)
            img = (img - self.img_mean) / self.img_std
            with torch.no_grad():
                feat = self.shared_backbone(img)                          # (B, 256, 14, 14)
            cam_embeds.append(self.camera_heads[cam_idx](feat))           # (B, TOKEN_DIM)
        return torch.cat(cam_embeds, dim=-1)                              # (B, num_cameras * TOKEN_DIM)


# ---------------------------------------------------------------------------
# Robot Policy Transformer  (task-conditioned; images in sequence)
# ---------------------------------------------------------------------------

class RobotPolicyTransformer(nn.Module):
    """Transformer for RobotWithTaskPolicy.

    Sequence: [image_tokens (N_img) | state_token (1) | prev_action_token (1)]
    Image patch tokens (with spatial PE, camera-ID and temporal embeddings) are
    prepended as a prefix so every layer attends freely over vision and
    proprioception jointly.

    Task identity conditions every layer via AdaLN-Zero (cond_dim = TOKEN_DIM).
    Only the last token (prev_action_token) is read out.
    """

    def __init__(
        self,
        d_model: int = TOKEN_DIM,
        n_heads: int = 4,
        n_layers: int = 4,
        ffn_dim: int = 512,
        dropout: float = 0.0,
        cond_dim: int = TOKEN_DIM,    # task token dimension
    ):
        super().__init__()
        self.layers = nn.ModuleList([
            TransformerLayer(
                d_model=d_model, n_heads=n_heads, ffn_dim=ffn_dim,
                dropout=dropout, cond_dim=cond_dim,
            )
            for _ in range(n_layers)
        ])
        self.norm_out = nn.LayerNorm(d_model)

    def forward(
        self,
        img_tokens:   torch.Tensor,
        state_token:  torch.Tensor,
        action_token: torch.Tensor,
        cond:         torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            img_tokens:   (B, N_img, d_model) — image patch tokens from ImageEncoder
            state_token:  (B, 1, d_model)
            action_token: (B, 1, d_model)
            cond:         (B, TOKEN_DIM)      — task conditioning vector
        Returns:
            out: (B, d_model) — output at the prev_action_token position (last token)
        """
        x = torch.cat([img_tokens, state_token, action_token], dim=1)  # (B, N_img+2, d)
        for layer in self.layers:
            x = layer(x, cond)
        return self.norm_out(x[:, -1])                                  # read out action token


# ---------------------------------------------------------------------------
# RobotWithTaskPolicy  (RLPD-compatible actor: Gaussian mean + log std output)
# ---------------------------------------------------------------------------

_LOG_STD_MIN = -20.0
_LOG_STD_MAX = 2.0


class RobotWithTaskPolicy(nn.Module):
    """RLPD-compatible robot actor with task-conditioned AdaLN and image tokens in sequence.

    Architecture:
        ImageEncoder (ResNet18, patch tokens with spatial/camera/temporal PE)
            → (B, obs_horizon * num_cameras * N_patches, d_model)  image prefix tokens
        TaskEncoder (target_module, port_name)
            → (B, d_model)  AdaLN-Zero conditioning vector
        state_encoder  robot_state → (B, 1, d_model)
        action_in_proj prev_action → (B, 1, d_model)
        RobotPolicyTransformer
            sequence = [img_tokens | state_token | action_token]
            cond     = task_token  (AdaLN-Zero)
        output_head MLP → (B, 2 * action_dim)  →  [means | log_stds]

    Inputs:
        images:        {camera_key: (B, obs_horizon, C, H, W)}
        robot_state:   (B, robot_state_dim)
        prev_action:   (B, action_dim)
        target_module: (B,) int64
        port_name:     (B,) int64

    Outputs:
        means:    (B, action_dim)  — action_dim = 9  (3 pos + 6 rot6d)
        log_stds: (B, action_dim)  — clamped to [_LOG_STD_MIN, _LOG_STD_MAX]
    """

    def __init__(
        self,
        robot_state_dim:    int = ROBOT_STATE_DIM,
        action_dim:         int = 9,
        obs_horizon:        int = 1,
        d_model:            int = TOKEN_DIM,
        n_heads:            int = 4,
        n_layers:           int = 4,
        ffn_dim:            int = 512,
        dropout:            float = 0.0,
        resnet_name:        str = "resnet18",
        resnet_weights:     str = "IMAGENET1K_V1",
        camera_keys: tuple[str, ...] | None = None,
        local_ckpt_path:    str | None = None,
        features_per_group: int = 16,
    ):
        super().__init__()
        self.action_dim  = action_dim
        self.obs_horizon = obs_horizon

        self.image_encoder = ImageEncoder(
            resnet_name=resnet_name,
            weights=resnet_weights,
            features_per_group=features_per_group,
            camera_keys=camera_keys,
            local_weights_path=local_ckpt_path,
            obs_horizon=obs_horizon,
        )
        self.task_encoder = TaskEncoder(task_embed_dim=d_model)
        self.state_encoder = nn.Sequential(
            nn.Linear(robot_state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, d_model),
        )
        self.action_in_proj = nn.Linear(action_dim, d_model)
        self.transformer = RobotPolicyTransformer(
            d_model=d_model,
            n_heads=n_heads,
            n_layers=n_layers,
            ffn_dim=ffn_dim,
            dropout=dropout,
            cond_dim=d_model,   # task token dimension
        )
        self.output_head = nn.Sequential(
            nn.Linear(d_model, d_model),
            nn.ReLU(),
            nn.Linear(d_model, 2 * action_dim),
        )

    def forward(
        self,
        images:        dict,
        robot_state:   torch.Tensor,
        prev_action:   torch.Tensor,
        target_module: torch.Tensor,
        port_name:     torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Returns:
            means:    (B, action_dim)
            log_stds: (B, action_dim) clamped to [_LOG_STD_MIN, _LOG_STD_MAX]
        """
        img_tokens = self.image_encoder(images)                        # (B, N_img, d)
        task_cond  = self.task_encoder(target_module, port_name).squeeze(1)  # (B, d)
        state_tok  = self.state_encoder(robot_state).unsqueeze(1)      # (B, 1, d)
        action_tok = self.action_in_proj(prev_action).unsqueeze(1)     # (B, 1, d)
        out        = self.transformer(img_tokens, state_tok, action_tok, task_cond)  # (B, d)
        params     = self.output_head(out)                             # (B, 2*action_dim)
        means, log_stds = params.chunk(2, dim=-1)
        return means, log_stds.clamp(_LOG_STD_MIN, _LOG_STD_MAX)
