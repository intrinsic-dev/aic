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

import time

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
from typing import Callable


# ---------------------------------------------------------------------------
# Constants  (identical to diffusion_policy.py)
# ---------------------------------------------------------------------------

NUM_TARGET_MODULES = 10
NUM_PORT_NAMES = 3

ROBOT_STATE_DIM = 20  # tcp_pose(7) + joint_positions(7) + wrench(6)

TOKEN_DIM = 128
RESNET_FEATURE_DIM = 512
IMG_TOKENS_PER_VIEW = RESNET_FEATURE_DIM // TOKEN_DIM   # 4

# Flow matching uses continuous t ∈ [0, 1].  Multiply by this factor before
# the sinusoidal embedding so the input covers a wider frequency range
# (same effect as using T=1000 in DDPM).
_T_EMBED_SCALE = 1000.0


# ---------------------------------------------------------------------------
# Vision encoder helpers  (identical to diffusion_policy.py)
# ---------------------------------------------------------------------------

def get_resnet(name: str, weights=None, **kwargs) -> nn.Module:
    func = getattr(torchvision.models, name)
    resnet = func(weights=weights, **kwargs)
    resnet.fc = nn.Identity()
    #debug_weights = torchvision.models.ResNet18_Weights.IMAGENET1K_V1
    #print("LXH:", debug_weights.transforms())
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
    CAMERA_KEYS = ("left_camera", "center_camera", "right_camera")

    # ImageNet normalization — matches ResNet18_Weights.IMAGENET1K_V1.transforms()
    _IMAGENET_MEAN = [0.485, 0.456, 0.406]
    _IMAGENET_STD  = [0.229, 0.224, 0.225]

    def __init__(self, resnet_name: str = "resnet18", weights="IMAGENET1K_V1",
                 features_per_group: int = 16):
        super().__init__()
        backbone = get_resnet(resnet_name, weights=weights)
        self.backbone = replace_bn_with_gn(backbone, features_per_group=features_per_group)
        self.num_cameras = len(self.CAMERA_KEYS)
        self.register_buffer(
            "img_mean", torch.tensor(self._IMAGENET_MEAN, dtype=torch.float32).view(1, 3, 1, 1)
        )
        self.register_buffer(
            "img_std",  torch.tensor(self._IMAGENET_STD,  dtype=torch.float32).view(1, 3, 1, 1)
        )

    def forward(self, images: dict) -> torch.Tensor:
        B, T = next(iter(images.values())).shape[:2]
        per_camera = []
        for key in self.CAMERA_KEYS:
            img = images[key]
            img_flat = img.flatten(end_dim=1)                        # (B*T, C, H, W)
            img_flat = (img_flat - self.img_mean) / self.img_std     # ImageNet normalize
            feat = self.backbone(img_flat)
            feat = feat.reshape(B, T, RESNET_FEATURE_DIM)
            per_camera.append(feat)
        stacked = torch.stack(per_camera, dim=2)
        tokens = stacked.reshape(B, T * self.num_cameras * IMG_TOKENS_PER_VIEW, TOKEN_DIM)
        return tokens


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
        return self.embedding(task_idx).unsqueeze(1)


# ---------------------------------------------------------------------------
# Transformer  (π0-style: full self-attention over concatenated tokens)
# ---------------------------------------------------------------------------

class TransformerLayer(nn.Module):
    """Pre-norm transformer layer: full self-attention + FFN.

    Operates on the entire [prefix | action] sequence. No cross-attention —
    conditioning tokens are part of the sequence and attended to directly.
    """

    def __init__(self, d_model: int, n_heads: int, ffn_dim: int, dropout: float = 0.0):
        super().__init__()
        self.self_attn = nn.MultiheadAttention(d_model, n_heads, dropout=dropout, batch_first=True)
        self.ffn = nn.Sequential(
            nn.Linear(d_model, ffn_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(ffn_dim, d_model),
        )
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.drop  = nn.Dropout(dropout)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        residual = x
        x = self.norm1(x)
        x, _ = self.self_attn(x, x, x)
        x = residual + self.drop(x)

        residual = x
        x = self.norm2(x)
        x = self.ffn(x)
        x = residual + self.drop(x)
        return x


class VectorFieldTransformer(nn.Module):
    """π0-style transformer predicting the velocity field v_θ(x_t, t).

    Conditioning tokens (image, state, task) are prepended as a prefix and
    attended to jointly with action tokens via full bidirectional self-attention.
    Only the action positions are read out and projected to the output.

    Sequence layout per forward call:
        [ img_tokens   (B, T*cams*4, d) |
          state_tokens (B, T, d)        |
          task_token   (B, 1, d)        |
          timestep_emb (B, 1, d)        |
          action_tokens (B, H, d)       ]   ← only these are projected to output
    """

    def __init__(self, action_dim: int, d_model: int = TOKEN_DIM, n_heads: int = 8,
                 n_layers: int = 4, ffn_dim: int = 1024, dropout: float = 0.0):
        super().__init__()
        self.norm_img   = nn.LayerNorm(d_model)
        self.norm_state = nn.LayerNorm(d_model)
        self.norm_task  = nn.LayerNorm(d_model)
        self.layers = nn.ModuleList([
            TransformerLayer(d_model=d_model, n_heads=n_heads, ffn_dim=ffn_dim, dropout=dropout)
            for _ in range(n_layers)
        ])
        self.norm_out = nn.LayerNorm(d_model)
        self.out_proj = nn.Linear(d_model, action_dim)

    @staticmethod
    def _sinusoidal_pos_emb(length: int, dim: int, device: torch.device) -> torch.Tensor:
        """Sinusoidal positional encoding for positions 0 .. length-1, shape (1, length, dim)."""
        pos = torch.arange(length, device=device).unsqueeze(1)       # (L, 1)
        i   = torch.arange(dim // 2, device=device).unsqueeze(0)     # (1, d/2)
        angles = pos / (10000.0 ** (2.0 * i / dim))                  # (L, d/2)
        emb = torch.cat([angles.sin(), angles.cos()], dim=-1)        # (L, d)
        return emb.unsqueeze(0)                                       # (1, L, d)

    def forward(self, action_tokens, timestep_emb, img_tokens, state_tokens, task_token):
        # Build prefix: all conditioning tokens including timestep
        prefix = torch.cat([
            self.norm_img(img_tokens),
            self.norm_state(state_tokens),
            self.norm_task(task_token),
            timestep_emb.unsqueeze(1),                        # (B, 1, d_model)
        ], dim=1)                                              # (B, n_prefix, d_model)
        n_prefix = prefix.shape[1]

        # Add sinusoidal positional encoding to action tokens so the transformer
        # knows their temporal order within the prediction horizon
        L, d = action_tokens.shape[1], action_tokens.shape[2]
        pos_emb = self._sinusoidal_pos_emb(L, d, action_tokens.device)  # (1, L, d)
        action_tokens = action_tokens + pos_emb

        # Full sequence: prefix tokens + action tokens attend to everything jointly
        x = torch.cat([prefix, action_tokens], dim=1)         # (B, n_prefix + pred_horizon, d_model)

        for layer in self.layers:
            x = layer(x)

        # Read out only the action positions
        x = x[:, n_prefix:]                                   # (B, pred_horizon, d_model)
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
        action_dim: int = 7,
        robot_state_dim: int = ROBOT_STATE_DIM,
        d_model: int = TOKEN_DIM,
        n_heads: int = 8,
        n_layers: int = 4,
        ffn_dim: int = 1024,
        dropout: float = 0.0,
        resnet_name: str = "resnet18",
        resnet_weights="IMAGENET1K_V1",
        cfg_dropout_prob: float = 0.0,
    ):
        super().__init__()
        self.obs_horizon      = obs_horizon
        self.action_dim       = action_dim
        self.cfg_dropout_prob = cfg_dropout_prob

        # Profiling (disabled by default; set policy.profiling = True to enable)
        self.profiling       = False
        self._prof_log_every = 50
        self._prof_n         = 0
        self._prof_acc: dict[str, float] = {}

        self.image_encoder = ImageEncoder(resnet_name=resnet_name, weights=resnet_weights)
        self.timestep_encoder = TimestepEncoder(embed_dim=d_model)
        self.task_encoder  = TaskEncoder(task_embed_dim=d_model)
        self.state_encoder = nn.Sequential(
            nn.Linear(robot_state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, d_model),
        )
        self.action_in_proj = nn.Linear(action_dim, d_model)
        self.vector_field_net = VectorFieldTransformer(
            action_dim=action_dim,
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
    ) -> torch.Tensor:
        """Predict velocity field v_θ(x_t, t).

        Args:
            images:           {camera_key: (B, obs_horizon, C, H, W)}
            robot_state:      (B, obs_horizon, ROBOT_STATE_DIM)
            target_module:    (B,) int64
            port_name:        (B,) int64
            x_t:              (B, pred_horizon, action_dim) — interpolated action at time t
            t:                (B,) float32 in [0, 1] — flow matching time
            use_uncond:       if True, zero all conditions (img, state, task) for the CFG
                              unconditional pass.
            img_tokens_cache: pre-computed image tokens; skips image encoder if provided

        Returns:
            v_pred: (B, pred_horizon, action_dim) — predicted velocity field
        """
        B   = x_t.shape[0]
        dev = x_t.device

        if self.profiling:
            if dev.type == "cuda":
                torch.cuda.synchronize(dev)
            _t = time.perf_counter()

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

        state_tokens = self.state_encoder(
            robot_state.flatten(end_dim=1)
        ).reshape(B, self.obs_horizon, -1)

        if self.profiling:
            _t = self._tock(_t, "state_encoder", dev)

        if use_uncond:
            # Unconditional pass: zero image and state tokens, use learnable null task token.
            img_tokens   = torch.zeros_like(img_tokens)
            state_tokens = torch.zeros_like(state_tokens)
            task_token   = self.task_encoder.uncond_token(B, dev)
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
            img_tokens=img_tokens,
            state_tokens=state_tokens,
            task_token=task_token,
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
        pred_horizon: int = 16,
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
            pred_horizon:   number of action steps to generate
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

        # Pre-encode images once — they are constant across all ODE steps
        if self.profiling:
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            _t_img = time.perf_counter()
        img_tokens_cache = self.image_encoder(images)
        if self.profiling:
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            print(f"[profile] image_encoder (once per sample): "
                  f"{(time.perf_counter() - _t_img)*1e3:.2f} ms", flush=True)

        # Pre-compute zero image cache for the unconditional CFG pass so we
        # don't allocate it inside the hot ODE loop.
        img_tokens_cache_zeros = (
            torch.zeros_like(img_tokens_cache) if use_cfg else None
        )

        def eval_v(xt: torch.Tensor, t_val: float) -> torch.Tensor:
            """Evaluate velocity field (with optional CFG) at a given t."""
            t_tensor = torch.full((B,), t_val, dtype=torch.float32, device=device)
            v_cond = self(
                images=images, robot_state=robot_state,
                target_module=target_module, port_name=port_name,
                x_t=xt, t=t_tensor, use_uncond=False,
                img_tokens_cache=img_tokens_cache,
            )
            if use_cfg:
                v_uncond = self(
                    images=images, robot_state=robot_state,
                    target_module=target_module, port_name=port_name,
                    x_t=xt, t=t_tensor, use_uncond=True,
                    img_tokens_cache=img_tokens_cache_zeros,
                )
                return v_uncond + guidance_scale * (v_cond - v_uncond)
            return v_cond

        # Start from pure noise at t=0
        x = torch.randn(B, pred_horizon, self.action_dim, device=device)

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
