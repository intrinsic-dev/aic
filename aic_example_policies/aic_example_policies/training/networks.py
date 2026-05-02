"""Neural network architectures for the 2-phase SAC insertion policy."""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

# ── Observation dimensions ─────────────────────────────────────────────────
# Phase 1 (centering): center camera CNN (128) + xyz_rel (3) + ft_z (1)
#                      + port_type_onehot (2) + step_norm (1) = 135
P1_OBS_DIM = 135
P1_ACTION_DIM = 2   # (dx, dy)

# Phase 2 (insertion): 3-camera CNN (3×128=384) + xyz_rel (3) + ft (6)
#                      + port_type_onehot (2) + step_norm (1) + depth (1) = 397
P2_OBS_DIM = 397
P2_ACTION_DIM = 3   # (dx, dy, dz)

# ── Action scales (meters per normalized unit) ──────────────────────────────
XY_SCALE = 0.005       # Phase 1 lateral
XY_INS_SCALE = 0.001   # Phase 2 lateral fine-tune
Z_INS_SCALE = 0.002    # Phase 2 descent

LOG_STD_MIN = -5
LOG_STD_MAX = 2


# ── Building blocks ─────────────────────────────────────────────────────────

class SpatialSoftmax(nn.Module):
    """Learnable expected position pooling: (B, C, H, W) → (B, 2*C).

    For each channel, computes the softmax over spatial locations and returns
    the expected (x, y) coordinate — a soft argmax that is differentiable and
    forces the CNN to represent spatial positions explicitly.
    """

    def __init__(self, channels: int, height: int, width: int, temperature: float = 1.0):
        super().__init__()
        self.channels = channels
        self.height = height
        self.width = width
        self.temperature = nn.Parameter(
            torch.ones(1) * temperature, requires_grad=True
        )
        # Pre-compute normalized grid coordinates in [-1, 1]
        xs = torch.linspace(-1, 1, width).view(1, 1, 1, width).expand(1, channels, height, width)
        ys = torch.linspace(-1, 1, height).view(1, 1, height, 1).expand(1, channels, height, width)
        self.register_buffer("xs", xs)
        self.register_buffer("ys", ys)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        B, C, H, W = x.shape
        flat = x.view(B, C, -1) / self.temperature
        weights = F.softmax(flat, dim=2).view(B, C, H, W)
        ex = (weights * self.xs).sum(dim=(2, 3))  # (B, C)
        ey = (weights * self.ys).sum(dim=(2, 3))  # (B, C)
        return torch.cat([ex, ey], dim=1)          # (B, 2*C)


class CNNBackbone(nn.Module):
    """Small CNN: (B, 4, 84, 84) → (B, 128).

    Architecture:
        Conv(4→32, 8×8, s4) → ReLU   # 84 → 20
        Conv(32→64, 4×4, s2) → ReLU  # 20 → 9
        Conv(64→64, 3×3, s1) → ReLU  # 9 → 7
        SpatialSoftmax(64, 7, 7)      # → 128
    """

    FEATURE_DIM = 128  # 64 channels × 2 (x + y expected positions)

    def __init__(self, in_channels: int = 4):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, 32, kernel_size=8, stride=4),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(inplace=True),
        )
        self.spatial_softmax = SpatialSoftmax(64, 7, 7)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.spatial_softmax(self.conv(x))  # (B, 128)


class DetectionHead(nn.Module):
    """Auxiliary head: 128-d CNN features → (u, v) normalized in [-1, 1]."""

    def __init__(self, in_dim: int = CNNBackbone.FEATURE_DIM):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 2),
            nn.Tanh(),  # outputs in [-1, 1]
        )

    def forward(self, features: torch.Tensor) -> torch.Tensor:
        return self.net(features)  # (B, 2)


# ── SAC Actor ───────────────────────────────────────────────────────────────

class SACActor(nn.Module):
    """Actor for Phase 1 or Phase 2.

    phase=1: one center camera → 128-d features
    phase=2: three cameras (center, left, right) → 3×128=384-d features
    """

    def __init__(self, phase: int, hidden_dim: int = 256):
        super().__init__()
        assert phase in (1, 2)
        self.phase = phase
        self.n_cameras = 1 if phase == 1 else 3
        action_dim = P1_ACTION_DIM if phase == 1 else P2_ACTION_DIM
        proprio_dim = P1_OBS_DIM - CNNBackbone.FEATURE_DIM if phase == 1 else P2_OBS_DIM - 3 * CNNBackbone.FEATURE_DIM

        self.encoders = nn.ModuleList([CNNBackbone() for _ in range(self.n_cameras)])
        self.detection_head = DetectionHead()  # auxiliary; only used during training

        cnn_out = self.n_cameras * CNNBackbone.FEATURE_DIM
        self.mlp = nn.Sequential(
            nn.Linear(cnn_out + proprio_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
        )
        self.mean_head = nn.Linear(hidden_dim, action_dim)
        self.log_std_head = nn.Linear(hidden_dim, action_dim)

    def _encode(self, obs: dict) -> tuple[torch.Tensor, torch.Tensor]:
        """Encode images and return (cnn_features, det_head_output).

        The detection head always operates on center-camera features.
        For Phase 2, center features are computed once and reused.
        """
        if self.phase == 1:
            center_feat = self.encoders[0](obs["image"])  # (B, 128)
            feat = center_feat
        else:
            center_feat = self.encoders[0](obs["center"])  # (B, 128) — run once
            left_feat   = self.encoders[1](obs["left"])
            right_feat  = self.encoders[2](obs["right"])
            feat = torch.cat([center_feat, left_feat, right_feat], dim=1)  # (B, 384)
        det = self.detection_head(center_feat)
        return feat, det

    def forward(self, obs: dict) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Returns (action, log_prob, det_head_uv)."""
        feat, det = self._encode(obs)
        x = torch.cat([feat, obs["proprio"]], dim=1)
        h = self.mlp(x)
        mean = self.mean_head(h)
        log_std = self.log_std_head(h).clamp(LOG_STD_MIN, LOG_STD_MAX)
        std = log_std.exp()
        dist = Normal(mean, std)
        z = dist.rsample()
        action = torch.tanh(z)
        log_prob = dist.log_prob(z) - torch.log(1 - action.pow(2) + 1e-6)
        return action, log_prob.sum(dim=1, keepdim=True), det

    @torch.no_grad()
    def get_action(self, obs: dict) -> torch.Tensor:
        feat, _ = self._encode(obs)
        x = torch.cat([feat, obs["proprio"]], dim=1)
        h = self.mlp(x)
        mean = self.mean_head(h)
        log_std = self.log_std_head(h).clamp(LOG_STD_MIN, LOG_STD_MAX)
        action = torch.tanh(Normal(mean, log_std.exp()).rsample())
        return action


# ── SAC Critic ──────────────────────────────────────────────────────────────

class _QNet(nn.Module):
    """Single Q-network with its own CNN encoder."""

    def __init__(self, phase: int, hidden_dim: int = 256):
        super().__init__()
        self.phase = phase
        self.n_cameras = 1 if phase == 1 else 3
        action_dim = P1_ACTION_DIM if phase == 1 else P2_ACTION_DIM
        proprio_dim = P1_OBS_DIM - CNNBackbone.FEATURE_DIM if phase == 1 else P2_OBS_DIM - 3 * CNNBackbone.FEATURE_DIM
        cnn_out = self.n_cameras * CNNBackbone.FEATURE_DIM

        self.encoders = nn.ModuleList([CNNBackbone() for _ in range(self.n_cameras)])
        self.net = nn.Sequential(
            nn.Linear(cnn_out + proprio_dim + action_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, obs: dict, action: torch.Tensor) -> torch.Tensor:
        if self.phase == 1:
            feat = self.encoders[0](obs["image"])
        else:
            feats = [enc(obs[k]) for enc, k in zip(self.encoders, ("center", "left", "right"))]
            feat = torch.cat(feats, dim=1)
        x = torch.cat([feat, obs["proprio"], action], dim=1)
        return self.net(x)


class SACCritic(nn.Module):
    """Twin Q-networks (separate CNN encoders per Q-network)."""

    def __init__(self, phase: int, hidden_dim: int = 256):
        super().__init__()
        self.q1 = _QNet(phase, hidden_dim)
        self.q2 = _QNet(phase, hidden_dim)

    def forward(self, obs: dict, action: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        return self.q1(obs, action), self.q2(obs, action)
