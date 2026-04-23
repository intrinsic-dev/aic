"""
Diffusion policy for the AIC cable-insertion task.

Architecture overview:
  observation (images + robot state + task info)  [obs_horizon steps]
      │
      ├─ ImageEncoder    → obs_horizon × num_cameras × 4 image tokens  (128-d each)
      ├─ StateEncoder    → obs_horizon              state tokens        (128-d each)
      └─ TaskEncoder     → 1                        task token          (128-d, one of 12 combos)
      │
      └─► condition token sequence  (keys & values for cross-attention)
                │
  noisy_action ─┤                                            ┌── DiTLayer (self-attn)
  + timestep    └─► action tokens (queries) ────────────────┤   DiTLayer (cross-attn → cond)
    embedding                                                └── × n_layers
                                                               │
                                                               ▼
                                                         predicted noise
                                                    (used by DDPM/DDIM sampler)

All tokens share a unified dimension TOKEN_DIM = 128.  No projection layers are
needed inside DiffusionTransformer — each encoder outputs directly at that dim.

Classifier-Free Guidance (CFG) on task identity:
  Training: with probability cfg_dropout_prob, replace the task token with a
  learnable null token (TaskEncoder.null_token).  The model learns to denoise
  both conditionally and unconditionally from the same weights.

  Inference (sample): run two forward passes per denoising step —
    eps_uncond = forward(..., use_uncond_task=True)
    eps_cond   = forward(..., use_uncond_task=False)
    eps        = eps_uncond + guidance_scale * (eps_cond - eps_uncond)
  guidance_scale=1.0 disables CFG (single conditional pass).

  ImageEncoder:   ResNet18 feature (512-d) split into 4 × 128-d tokens per view
  StateEncoder:   MLP  20 → 256 → 128
  TaskEncoder:    nn.Embedding(12, 128)
  TimestepEncoder: sinusoidal → MLP, output 128-d
  action_in_proj:  Linear(action_dim, 128) in DiffusionPolicy

Task integer encodings (from aic_robot_aic_controller.py):

    TASK_TARGET_MODULE_ENCODING:
        "nic_card_mount_0" → 0,  "nic_card_mount_1" → 1,  "nic_card_mount_2" → 2,
        "nic_card_mount_3" → 3,  "nic_card_mount_4" → 4,  "sc_port_0"        → 5,
        "sc_port_1"        → 6,  "sc_port_2"        → 7,  "sc_port_3"        → 8,
        "sc_port_4"        → 9

    TASK_PORT_NAME_ENCODING:
        "sfp_port_0" → 0,  "sfp_port_1" → 1,  "sc_port_base" → 2
"""

import time

import torch
import torch.nn as nn
import torchvision
from typing import Callable


# ---------------------------------------------------------------------------
# Constants (must match aic_robot_aic_controller.py)
# ---------------------------------------------------------------------------

NUM_TARGET_MODULES = 10   # nic_card_mount_{0..4} + sc_port_{0..4}
NUM_PORT_NAMES = 3        # sfp_port_0, sfp_port_1, sc_port_base

# Robot state vector layout (tcp_velocity removed):
#   tcp_pose.position    (3)  — x, y, z  [meters, raw/unnormalised]
#   tcp_pose.orientation (4)  — qx, qy, qz, qw  [unit quaternion]
#   joint_positions      (7)  — 6 arm joints + 1 gripper  [rad]
#   wrench.force         (3)  — fx, fy, fz  [N]  (tare-corrected)
#   wrench.torque        (3)  — tx, ty, tz  [Nm] (tare-corrected)
ROBOT_STATE_DIM = 20  # 3+4+7+3+3

# Unified token dimension — every encoder outputs vectors of this size.
# No projection layers are needed in DiffusionTransformer.
TOKEN_DIM = 128

# ResNet18 global-average-pool output dimension (after removing the fc layer).
RESNET_FEATURE_DIM = 512
# Each 512-d ResNet feature is split into this many TOKEN_DIM tokens.
IMG_TOKENS_PER_VIEW = RESNET_FEATURE_DIM // TOKEN_DIM   # 4


# ---------------------------------------------------------------------------
# Vision encoder helpers
# ---------------------------------------------------------------------------

def get_resnet(name: str, weights=None, **kwargs) -> nn.Module:
    """Initialize a standard ResNet vision encoder without its final fc layer.

    Args:
        name:    "resnet18", "resnet34", or "resnet50"
        weights: "IMAGENET1K_V1" for pretrained ImageNet weights, None for random init
    Returns:
        ResNet with fc replaced by nn.Identity().
        Output dim: 512 for resnet18/34, 2048 for resnet50.
    """
    func = getattr(torchvision.models, name)
    resnet = func(weights=weights, **kwargs)
    resnet.fc = nn.Identity()
    return resnet


def replace_submodules(
    root_module: nn.Module,
    predicate: Callable[[nn.Module], bool],
    func: Callable[[nn.Module], nn.Module],
) -> nn.Module:
    """Replace all submodules selected by predicate with the output of func."""
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


def replace_bn_with_gn(
    root_module: nn.Module,
    features_per_group: int = 16,
) -> nn.Module:
    """Replace all BatchNorm2d layers with GroupNorm.

    IMPORTANT: must be called before using the encoder with EMA, otherwise
    training performance will degrade significantly.
    """
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
# Image Encoder
# ---------------------------------------------------------------------------

class ImageEncoder(nn.Module):
    """Encodes images from all three cameras using a single shared ResNet18.

    Each 512-d ResNet feature is split into IMG_TOKENS_PER_VIEW (=4) tokens of
    TOKEN_DIM (=128) each, so no projection layer is needed downstream.

    Input:  images dict {camera_key: (B, obs_horizon, C, H, W)}
            pixel values expected in [0, 1].
    Output: (B, obs_horizon * num_cameras * IMG_TOKENS_PER_VIEW, TOKEN_DIM)
              = (B, obs_horizon * 12, 128)  when obs_horizon=1

    Value range of output tokens: unbounded floats (GroupNorm + ReLU internals,
    but the final avgpool output has no explicit bound).
    """

    CAMERA_KEYS = ("left_camera", "center_camera", "right_camera")

    def __init__(
        self,
        resnet_name: str = "resnet18",
        weights="IMAGENET1K_V1",
        features_per_group: int = 16,
    ):
        super().__init__()

        # Single shared backbone — all cameras go through the same weights
        backbone = get_resnet(resnet_name, weights=weights)
        # Replace BatchNorm with GroupNorm so the encoder is compatible with EMA
        self.backbone = replace_bn_with_gn(backbone, features_per_group=features_per_group)

        self.num_cameras = len(self.CAMERA_KEYS)

    def forward(self, images: dict) -> torch.Tensor:
        """
        Args:
            images: {camera_key: (B, obs_horizon, C, H, W)}, pixels in [0, 1]

        Returns:
            tokens: (B, obs_horizon * num_cameras * IMG_TOKENS_PER_VIEW, TOKEN_DIM)
        """
        B, T = next(iter(images.values())).shape[:2]

        per_camera = []
        for key in self.CAMERA_KEYS:
            img = images[key]                              # (B, T, C, H, W)
            img_flat = img.flatten(end_dim=1)              # (B*T, C, H, W)
            feat = self.backbone(img_flat)                 # (B*T, 512)
            feat = feat.reshape(B, T, RESNET_FEATURE_DIM) # (B, T, 512)
            per_camera.append(feat)

        # (B, T, num_cameras, 512)
        stacked = torch.stack(per_camera, dim=2)
        # Split each 512-d feature into 4 × 128-d tokens
        # → (B, T * num_cameras * IMG_TOKENS_PER_VIEW, TOKEN_DIM)
        tokens = stacked.reshape(B, T * self.num_cameras * IMG_TOKENS_PER_VIEW, TOKEN_DIM)
        return tokens


# ---------------------------------------------------------------------------
# Diffusion Timestep Encoder
# ---------------------------------------------------------------------------

class DiffusionTimestepEncoder(nn.Module):
    """Encodes the diffusion timestep t into a sinusoidal + MLP embedding.

    Output dim is TOKEN_DIM (=128) so it can be added directly to action tokens
    without a projection layer.

    Input:  (B,) integer timestep tensor
    Output: (B, TOKEN_DIM)

    Value range: unbounded (SiLU MLP with no output activation).
    """

    def __init__(self, timestep_embed_dim: int = TOKEN_DIM):
        super().__init__()
        self.timestep_embed_dim = timestep_embed_dim
        assert timestep_embed_dim % 2 == 0, "timestep_embed_dim must be even"

        self.mlp = nn.Sequential(
            nn.Linear(timestep_embed_dim, timestep_embed_dim * 4),
            nn.SiLU(),
            nn.Linear(timestep_embed_dim * 4, timestep_embed_dim),
        )

    def _sinusoidal_embedding(self, t: torch.Tensor) -> torch.Tensor:
        half = self.timestep_embed_dim // 2
        freqs = torch.exp(
            -torch.arange(half, dtype=torch.float32, device=t.device)
            * (torch.log(torch.tensor(10000.0)) / (half - 1))
        )                                    # (half,)
        args = t.float().unsqueeze(1) * freqs.unsqueeze(0)  # (B, half)
        return torch.cat([args.sin(), args.cos()], dim=-1)   # (B, timestep_embed_dim)

    def forward(self, t: torch.Tensor) -> torch.Tensor:
        """
        Args:
            t: (B,) diffusion timestep indices (integers 0 … T-1)
        Returns:
            emb: (B, TOKEN_DIM)
        """
        sin_emb = self._sinusoidal_embedding(t)   # (B, TOKEN_DIM)
        return self.mlp(sin_emb)                  # (B, TOKEN_DIM)


# ---------------------------------------------------------------------------
# Task Encoder
# ---------------------------------------------------------------------------

class TaskEncoder(nn.Module):
    """Encodes task identity as a single 128-d token from 12 valid (module, port) combos.

    Valid combinations (from aic_robot_aic_controller.py encodings):

        task_idx  target_module        port_name
        ────────  ───────────────────  ──────────────
          0       nic_card_mount_0 (0)  sfp_port_0 (0)
          1       nic_card_mount_0 (0)  sfp_port_1 (1)
          2       nic_card_mount_1 (1)  sfp_port_0 (0)
          3       nic_card_mount_1 (1)  sfp_port_1 (1)
          4       nic_card_mount_2 (2)  sfp_port_0 (0)
          5       nic_card_mount_2 (2)  sfp_port_1 (1)
          6       nic_card_mount_3 (3)  sfp_port_0 (0)
          7       nic_card_mount_3 (3)  sfp_port_1 (1)
          8       nic_card_mount_4 (4)  sfp_port_0 (0)
          9       nic_card_mount_4 (4)  sfp_port_1 (1)
         10       sc_port_0        (5)  sc_port_base (2)
         11       sc_port_1        (6)  sc_port_base (2)

    Input:  target_module (B,) int, port_name (B,) int
    Output: (B, 1, TOKEN_DIM)

    Value range: unbounded (standard embedding lookup, no activation).
    Typical magnitude after random init: ~N(0, 1).
    """

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
        # Learnable unconditional token used for CFG (replaces the task token
        # with probability cfg_dropout_prob during training, and is used for the
        # unconditional forward pass during CFG inference).
        self.null_token = nn.Parameter(torch.zeros(task_embed_dim))
        self.register_buffer("lookup", self._LOOKUP.clone())

    def uncond_token(self, batch_size: int, device: torch.device) -> torch.Tensor:
        """Return the learnable null token expanded to (batch_size, 1, TOKEN_DIM)."""
        return self.null_token.view(1, 1, -1).expand(batch_size, 1, -1)

    def forward(
        self,
        target_module: torch.Tensor,
        port_name: torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            target_module: (B,) int64 — index from TASK_TARGET_MODULE_ENCODING
            port_name:     (B,) int64 — index from TASK_PORT_NAME_ENCODING
        Returns:
            task_token: (B, 1, TOKEN_DIM)
        """
        task_idx = self.lookup[target_module, port_name]   # (B,)
        return self.embedding(task_idx).unsqueeze(1)        # (B, 1, TOKEN_DIM)


# ---------------------------------------------------------------------------
# Diffusion Transformer (noise prediction network)
# ---------------------------------------------------------------------------

class DiTLayer(nn.Module):
    """Single Diffusion Transformer layer.

    Each action token:
      1. Self-attends to other action tokens.
      2. Cross-attends to the condition token sequence
         (image tokens + state tokens + task token).
      3. Passes through a feed-forward network.

    Pre-norm (LayerNorm before each sub-layer) for training stability.
    """

    def __init__(self, d_model: int, n_heads: int, ffn_dim: int, dropout: float = 0.0):
        super().__init__()
        self.self_attn  = nn.MultiheadAttention(d_model, n_heads, dropout=dropout, batch_first=True)
        self.cross_attn = nn.MultiheadAttention(d_model, n_heads, dropout=dropout, batch_first=True)
        self.ffn = nn.Sequential(
            nn.Linear(d_model, ffn_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(ffn_dim, d_model),
        )
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.norm3 = nn.LayerNorm(d_model)
        self.drop  = nn.Dropout(dropout)

    def forward(self, x: torch.Tensor, cond: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x:    (B, pred_horizon, d_model)    — noisy action tokens
            cond: (B, num_cond_tokens, d_model) — condition token sequence
        Returns:
            x:    (B, pred_horizon, d_model)
        """
        residual = x
        x = self.norm1(x)
        x, _ = self.self_attn(x, x, x)
        x = residual + self.drop(x)

        residual = x
        x = self.norm2(x)
        x, _ = self.cross_attn(query=x, key=cond, value=cond)
        x = residual + self.drop(x)

        residual = x
        x = self.norm3(x)
        x = self.ffn(x)
        x = residual + self.drop(x)

        return x


class DiffusionTransformer(nn.Module):
    """Transformer-based noise prediction network for diffusion policy.

    All inputs are expected to be pre-projected to d_model (= TOKEN_DIM = 128).
    No internal projection layers — callers are responsible for dimensionality.

    Condition token sequence (keys/values for cross-attention):
      ┌──────────────────────────────────────────────────────────────────────┐
      │  image tokens  (obs_horizon × num_cameras × IMG_TOKENS_PER_VIEW, 128) │
      │  state tokens  (obs_horizon tokens, 128)                              │
      │  task token    (1 token for the combined task identity, 128)           │
      └──────────────────────────────────────────────────────────────────────┘

    Query tokens (action sequence):
      pre-projected noisy action tokens + diffusion timestep embedding added.
    """

    def __init__(
        self,
        action_dim: int,
        d_model: int = TOKEN_DIM,
        n_heads: int = 8,
        n_layers: int = 4,
        ffn_dim: int = 1024,
        dropout: float = 0.0,
    ):
        super().__init__()

        # Per-stream LayerNorms applied before concatenating condition tokens.
        # Each encoder has a different output scale; these bring them into the
        # same range before the first cross-attention layer sees them.
        self.norm_img   = nn.LayerNorm(d_model)
        self.norm_state = nn.LayerNorm(d_model)
        self.norm_task  = nn.LayerNorm(d_model)

        self.layers = nn.ModuleList([
            DiTLayer(d_model=d_model, n_heads=n_heads, ffn_dim=ffn_dim, dropout=dropout)
            for _ in range(n_layers)
        ])
        self.norm_out = nn.LayerNorm(d_model)
        self.out_proj = nn.Linear(d_model, action_dim)

    def forward(
        self,
        action_tokens: torch.Tensor,
        timestep_emb: torch.Tensor,
        img_tokens: torch.Tensor,
        state_tokens: torch.Tensor,
        task_token: torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            action_tokens: (B, pred_horizon, d_model)  — noisy actions, pre-projected
            timestep_emb:  (B, d_model)                — diffusion timestep embedding
            img_tokens:    (B, obs_horizon * num_cameras * IMG_TOKENS_PER_VIEW, d_model)
            state_tokens:  (B, obs_horizon, d_model)
            task_token:    (B, 1, d_model)

        Returns:
            noise_pred: (B, pred_horizon, action_dim)
        """
        # Normalise each condition stream independently before concatenating.
        # This compensates for the different output scales of image, state, and task encoders.
        cond = torch.cat([
            self.norm_img(img_tokens),
            self.norm_state(state_tokens),
            self.norm_task(task_token),
        ], dim=1)

        # Add timestep embedding to every action token (broadcast over pred_horizon)
        x = action_tokens + timestep_emb.unsqueeze(1)   # (B, pred_horizon, d_model)

        for layer in self.layers:
            x = layer(x, cond)

        x = self.norm_out(x)
        return self.out_proj(x)                          # (B, pred_horizon, action_dim)


# ---------------------------------------------------------------------------
# Diffusion Policy (top-level module)
# ---------------------------------------------------------------------------

class DiffusionPolicy(nn.Module):
    """Diffusion policy for the AIC cable-insertion task.

    Combines image, robot state, and task embeddings as a condition token
    sequence, then uses a Diffusion Transformer to predict the noise added
    to the action at each denoising step.

    All internal token dimensions are unified at TOKEN_DIM = 128.

    Inputs at each denoising step:
        images          dict[str, (B, obs_horizon, C, H, W)]  — one entry per camera, pixels in [0,1]
        robot_state     (B, obs_horizon, ROBOT_STATE_DIM)     — tcp_pose(7), joint_positions(7), wrench(6)
        target_module   (B,)  int                              — task target module index
        port_name       (B,)  int                              — task port name index
        noisy_action    (B, pred_horizon, action_dim)          — action corrupted with noise
                        action layout: [x, y, z, qx, qy, qz, qw]
        timestep        (B,)  int                              — diffusion step index

    Output:
        noise_pred      (B, pred_horizon, action_dim)          — predicted noise

    Condition tokens fed into cross-attention:
        obs_horizon × num_cameras × IMG_TOKENS_PER_VIEW  image tokens  (128-d each)
        obs_horizon                                       state tokens  (128-d each)
        1                                                 task token    (128-d)
    """

    def __init__(
        self,
        obs_horizon: int = 2,
        action_dim: int = 7,        # tcp_pose: 3 position (x,y,z) + 4 orientation (qx,qy,qz,qw)
        robot_state_dim: int = ROBOT_STATE_DIM,
        # Diffusion Transformer hyperparameters
        d_model: int = TOKEN_DIM,
        n_heads: int = 8,
        n_layers: int = 4,
        ffn_dim: int = 1024,
        dropout: float = 0.0,
        # Vision backbone
        resnet_name: str = "resnet18",
        resnet_weights="IMAGENET1K_V1",
        # Classifier-Free Guidance
        cfg_dropout_prob: float = 0.0,
    ):
        super().__init__()
        self.obs_horizon = obs_horizon
        self.action_dim  = action_dim
        self.cfg_dropout_prob = cfg_dropout_prob

        # --- Profiling (disabled by default; set policy.profiling = True to enable) ---
        self.profiling = False
        self._prof_log_every = 50   # log average timings every N forward calls
        self._prof_n   = 0
        self._prof_acc: dict[str, float] = {}

        # --- Image encoder (single shared ResNet18 for all three cameras) ---
        self.image_encoder = ImageEncoder(
            resnet_name=resnet_name,
            weights=resnet_weights,
        )
        # Output: (B, obs_horizon * num_cameras * IMG_TOKENS_PER_VIEW, TOKEN_DIM)

        # --- Diffusion timestep encoder ---
        self.timestep_encoder = DiffusionTimestepEncoder(timestep_embed_dim=d_model)

        # --- Task encoder → single task token ---
        self.task_encoder = TaskEncoder(task_embed_dim=d_model)

        # --- Robot state encoder → one state token per obs step ---
        # Input (ROBOT_STATE_DIM=20):
        #   tcp_pose.position (3), tcp_pose.orientation (4),
        #   joint_positions (7), wrench.force (3), wrench.torque (3)
        # Value ranges: position [m], orientation [unit quaternion], joints [rad],
        #   wrench [N / Nm] — all raw/unnormalised.
        # TODO: add input normalisation (mean/std from dataset stats)
        self.state_encoder = nn.Sequential(
            nn.Linear(robot_state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, d_model),    # output is TOKEN_DIM = 128
        )

        # --- Action input projection: action_dim → TOKEN_DIM ---
        # Kept outside DiffusionTransformer so the transformer has no projection layers.
        self.action_in_proj = nn.Linear(action_dim, d_model)

        # --- Diffusion Transformer (noise prediction network) ---
        self.noise_pred_net = DiffusionTransformer(
            action_dim=action_dim,
            d_model=d_model,
            n_heads=n_heads,
            n_layers=n_layers,
            ffn_dim=ffn_dim,
            dropout=dropout,
        )

    # ------------------------------------------------------------------
    # Profiling helpers
    # ------------------------------------------------------------------

    def _tock(self, t0: float, name: str, device: torch.device) -> float:
        """Sync CUDA (if needed), record elapsed ms since t0, return new t0."""
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        t1 = time.perf_counter()
        self._prof_acc[name] = self._prof_acc.get(name, 0.0) + (t1 - t0) * 1e3
        return t1

    def _prof_report(self) -> None:
        """Log average per-component timings and reset accumulators."""
        n = max(self._prof_n, 1)
        lines = ["DiffusionPolicy forward profile (avg over last "
                 f"{n} calls):"]
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

    def forward(
        self,
        images: dict,
        robot_state: torch.Tensor,
        target_module: torch.Tensor,
        port_name: torch.Tensor,
        noisy_action: torch.Tensor,
        timestep: torch.Tensor,
        use_uncond_task: bool = False,
        img_tokens_cache: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Predict noise given noisy action and all conditioning inputs.

        Args:
            images:           {camera_key: (B, obs_horizon, C, H, W)} — normalised to [0, 1]
            robot_state:      (B, obs_horizon, ROBOT_STATE_DIM)
            target_module:    (B,) int64
            port_name:        (B,) int64
            noisy_action:     (B, pred_horizon, action_dim)
            timestep:         (B,) int64 diffusion step
            use_uncond_task:  if True, replace task token with the learnable null token
                              (used for the unconditional pass during CFG inference)
            img_tokens_cache: pre-computed image tokens (B, img_seq_len, TOKEN_DIM).
                              When provided, the image encoder is skipped entirely.
                              Use sample() which pre-encodes images once per call.

        Returns:
            noise_pred: (B, pred_horizon, action_dim)
        """
        B   = noisy_action.shape[0]
        dev = noisy_action.device

        if self.profiling:
            if dev.type == "cuda":
                torch.cuda.synchronize(dev)
            _t = time.perf_counter()

        # --- Image tokens: encode once, or reuse pre-computed cache ---
        if img_tokens_cache is not None:
            img_tokens = img_tokens_cache
        else:
            img_tokens = self.image_encoder(images)

        if self.profiling and img_tokens_cache is None:
            _t = self._tock(_t, "image_encoder", dev)
        elif self.profiling:
            # cache hit: reset timer without recording (encoding happened outside)
            if dev.type == "cuda":
                torch.cuda.synchronize(dev)
            _t = time.perf_counter()

        # --- State tokens: one per obs step ---
        state_tokens = self.state_encoder(
            robot_state.flatten(end_dim=1)                    # (B*obs_horizon, ROBOT_STATE_DIM)
        ).reshape(B, self.obs_horizon, -1)                    # (B, obs_horizon, TOKEN_DIM)

        if self.profiling:
            _t = self._tock(_t, "state_encoder", dev)

        # --- Task token: conditional or unconditional ---
        if use_uncond_task:
            # CFG inference: unconditional pass uses the learnable null token
            task_token = self.task_encoder.uncond_token(B, dev)
        else:
            task_token = self.task_encoder(target_module, port_name)  # (B, 1, TOKEN_DIM)

            # CFG training dropout: randomly replace task token with null token
            if self.cfg_dropout_prob > 0.0 and self.training:
                null_token = self.task_encoder.uncond_token(B, dev)
                drop_mask = (
                    torch.rand(B, device=dev) < self.cfg_dropout_prob
                ).view(B, 1, 1)
                task_token = torch.where(drop_mask, null_token, task_token)

        # --- Diffusion timestep embedding ---
        timestep_emb = self.timestep_encoder(timestep)        # (B, TOKEN_DIM)

        # --- Project noisy action to token dim ---
        action_tokens = self.action_in_proj(noisy_action)     # (B, pred_horizon, TOKEN_DIM)

        if self.profiling:
            _t = self._tock(_t, "cond_encoders", dev)

        # --- Predict noise via Diffusion Transformer ---
        noise_pred = self.noise_pred_net(
            action_tokens=action_tokens,
            timestep_emb=timestep_emb,
            img_tokens=img_tokens,
            state_tokens=state_tokens,
            task_token=task_token,
        )                                                     # (B, pred_horizon, action_dim)

        if self.profiling:
            self._tock(_t, "noise_pred_net", dev)
            self._prof_n += 1
            if self._prof_n >= self._prof_log_every:
                self._prof_report()

        return noise_pred

    @torch.no_grad()
    def sample(
        self,
        images: dict,
        robot_state: torch.Tensor,
        target_module: torch.Tensor,
        port_name: torch.Tensor,
        num_timesteps: int = 100,
        beta_start: float = 1e-4,
        beta_end: float = 0.02,
        pred_horizon: int = 16,
        guidance_scale: float = 1.0,
        solver: str = "dpm_solver_pp_2m",
    ) -> torch.Tensor:
        """Run reverse diffusion to generate an action sequence.

        Supports two solvers:

        "dpm_solver_pp_2m" (default) — DPM-Solver++ 2M
            Deterministic 2nd-order multistep ODE solver in log-SNR space.
            Reference: Lu et al. "DPM-Solver++: Fast Solver for Guided Sampling
            of Diffusion Probabilistic Models" (NeurIPS 2022).
            Works well with as few as 10–20 steps; recommended for inference.
            Algorithm: maintains a rolling x̂₀ estimate and applies a 2nd-order
            linear multistep correction once the first step is complete.

        "ddpm" — stochastic DDPM ancestral sampling
            Original Ho et al. DDPM reverse process with posterior variance noise.
            Requires ~50–1000 steps; kept for training-time compatibility checks.

        CFG is supported for both solvers: when guidance_scale != 1.0, two
        forward passes are run per step and the noise estimates are combined as
            eps = eps_uncond + guidance_scale * (eps_cond - eps_uncond)

        Args:
            images:         {camera_key: (B, obs_horizon, C, H, W)} — normalised to [0, 1]
            robot_state:    (B, obs_horizon, ROBOT_STATE_DIM)
            target_module:  (B,) int64
            port_name:      (B,) int64
            num_timesteps:  number of denoising steps (should match training schedule)
            beta_start:     start of linear beta schedule (should match training)
            beta_end:       end of linear beta schedule (should match training)
            pred_horizon:   number of action steps to generate
            guidance_scale: CFG scale — 1.0 disables CFG, >1.0 strengthens task conditioning
            solver:         "dpm_solver_pp_2m" or "ddpm"

        Returns:
            action: (B, pred_horizon, action_dim) — denoised tcp_pose sequence
        """
        device = robot_state.device
        B = robot_state.shape[0]
        use_cfg = guidance_scale != 1.0

        # --- Noise schedule (must match training) ---
        betas     = torch.linspace(beta_start, beta_end, num_timesteps, device=device)
        alphas    = 1.0 - betas
        alpha_bar = torch.cumprod(alphas, dim=0)   # ᾱ_t,  shape (T,)

        # --- Pre-encode images once (they don't change across denoising steps) ---
        if self.profiling:
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            _t_img = time.perf_counter()
        img_tokens_cache = self.image_encoder(images)
        if self.profiling:
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            _img_enc_ms = (time.perf_counter() - _t_img) * 1e3
            print(f"[profile] image_encoder (once per sample): {_img_enc_ms:.2f} ms", flush=True)

        # --- Helper: predict noise with optional CFG ---
        def predict_eps(xt: torch.Tensor, t_tensor: torch.Tensor) -> torch.Tensor:
            eps_cond = self(
                images=images, robot_state=robot_state,
                target_module=target_module, port_name=port_name,
                noisy_action=xt, timestep=t_tensor, use_uncond_task=False,
                img_tokens_cache=img_tokens_cache,
            )
            if use_cfg:
                eps_uncond = self(
                    images=images, robot_state=robot_state,
                    target_module=target_module, port_name=port_name,
                    noisy_action=xt, timestep=t_tensor, use_uncond_task=True,
                    img_tokens_cache=img_tokens_cache,
                )
                return eps_uncond + guidance_scale * (eps_cond - eps_uncond)
            return eps_cond

        # x_T ~ N(0, I)
        x = torch.randn(B, pred_horizon, self.action_dim, device=device)

        # ------------------------------------------------------------------ #
        # Solver: DPM-Solver++ 2M                                             #
        # ------------------------------------------------------------------ #
        if solver == "dpm_solver_pp_2m":
            # α_t = sqrt(ᾱ_t),  σ_t = sqrt(1 - ᾱ_t)
            alpha_t_all = alpha_bar.sqrt()
            sigma_t_all = (1.0 - alpha_bar).sqrt()
            # Half log-SNR: λ_t = log(α_t / σ_t).
            # λ increases as t decreases (cleaner), so h = λ_{t-1} - λ_t > 0.
            lambda_t_all = torch.log(alpha_t_all / sigma_t_all)

            x0_pred_prev: torch.Tensor | None = None
            h_prev: torch.Tensor | None = None

            for t_int in reversed(range(num_timesteps)):
                t = torch.full((B,), t_int, dtype=torch.long, device=device)

                alpha_t = alpha_t_all[t_int]
                sigma_t = sigma_t_all[t_int]

                # Noise → x̂₀ (data prediction):  x̂₀ = (x_t - σ_t · ε) / α_t
                eps    = predict_eps(x, t)
                x0_pred = (x - sigma_t * eps) / alpha_t

                if t_int == 0:
                    # Final denoising step: return x̂₀ directly (no stochastic noise)
                    x = x0_pred
                    break

                # Move one step toward t_int - 1 (cleaner)
                t_next     = t_int - 1
                alpha_next = alpha_t_all[t_next]
                sigma_next = sigma_t_all[t_next]
                h = lambda_t_all[t_next] - lambda_t_all[t_int]   # > 0

                # Coefficient shared by both orders:
                #   (σ_next/σ_t) · x_t  −  α_next · (e^{-h} − 1) · D
                # Note: (e^{-h} − 1) < 0, so the D term is added (moves toward x̂₀).
                coeff = -alpha_next * (torch.exp(-h) - 1.0)

                if x0_pred_prev is None:
                    # 1st step → 1st-order update (DPM-Solver++ 1)
                    D = x0_pred
                else:
                    # Subsequent steps → 2nd-order multistep correction (Eq. 17, Algorithm 2)
                    #   r  = h_{i-1} / h_i   (ratio of consecutive λ-step sizes)
                    #   D  = (1 + 1/2r) · x̂₀^{(i)} − (1/2r) · x̂₀^{(i-1)}
                    r = h_prev / h
                    D = (1.0 + 0.5 / r) * x0_pred - (0.5 / r) * x0_pred_prev

                x = (sigma_next / sigma_t) * x + coeff * D

                x0_pred_prev = x0_pred
                h_prev = h

        # ------------------------------------------------------------------ #
        # Solver: DDPM (stochastic ancestral sampling)                        #
        # ------------------------------------------------------------------ #
        elif solver == "ddpm":
            for t_int in reversed(range(num_timesteps)):
                t = torch.full((B,), t_int, dtype=torch.long, device=device)
                eps = predict_eps(x, t)

                alpha_t     = alphas[t_int]
                alpha_bar_t = alpha_bar[t_int]
                beta_t      = betas[t_int]

                # x̂_{t-1} mean = 1/√α_t · (x_t − β_t/√(1−ᾱ_t) · ε)
                mean = (x - beta_t / (1.0 - alpha_bar_t).sqrt() * eps) / alpha_t.sqrt()

                if t_int > 0:
                    # Posterior variance: β_t · (1 − ᾱ_{t-1}) / (1 − ᾱ_t)
                    variance = beta_t * (1.0 - alpha_bar[t_int - 1]) / (1.0 - alpha_bar_t)
                    x = mean + variance.sqrt() * torch.randn_like(x)
                else:
                    x = mean

        else:
            raise ValueError(f"Unknown solver {solver!r}. Choose 'dpm_solver_pp_2m' or 'ddpm'.")

        return x  # (B, pred_horizon, action_dim)
