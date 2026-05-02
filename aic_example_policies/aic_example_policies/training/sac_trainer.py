"""Custom SAC trainer with auxiliary port detection head.

Replay buffer stores images as uint8 for memory efficiency.
The detection auxiliary loss backpropagates through the actor's CNN encoder,
encouraging the CNN to learn spatially grounded port representations.
"""

from __future__ import annotations

import copy
import os
from collections import deque
from typing import Optional

import numpy as np
import torch
import torch.nn.functional as F
import torch.optim as optim

from .networks import SACActor, SACCritic, P1_ACTION_DIM, P2_ACTION_DIM

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# ── Replay Buffer ────────────────────────────────────────────────────────────

class ReplayBuffer:
    """Circular replay buffer that stores images as uint8 to save memory.

    Phase 1: images have key 'image' (1 camera).
    Phase 2: images have keys 'center', 'left', 'right' (3 cameras).
    """

    def __init__(self, capacity: int, phase: int, img_shape=(84, 84, 4), proprio_dim: int = 7):
        self.capacity = capacity
        self.phase = phase
        self.proprio_dim = proprio_dim
        self.action_dim = P1_ACTION_DIM if phase == 1 else P2_ACTION_DIM

        cam_keys = ["image"] if phase == 1 else ["center", "left", "right"]
        self._cam_keys = cam_keys

        # Pre-allocate numpy arrays (uint8 for images)
        self._images  = {k: np.zeros((capacity, *img_shape), dtype=np.uint8) for k in cam_keys}
        self._images_next = {k: np.zeros((capacity, *img_shape), dtype=np.uint8) for k in cam_keys}
        self._proprios      = np.zeros((capacity, proprio_dim), dtype=np.float32)
        self._proprios_next = np.zeros((capacity, proprio_dim), dtype=np.float32)
        self._actions  = np.zeros((capacity, self.action_dim), dtype=np.float32)
        self._rewards  = np.zeros((capacity, 1), dtype=np.float32)
        self._dones    = np.zeros((capacity, 1), dtype=np.float32)
        # Per-transition GT detection labels (u,v) in [-1,1]; NaN if unavailable
        self._gt_uvs   = np.full((capacity, 2), np.nan, dtype=np.float32)

        self._ptr = 0
        self._size = 0

    def add(
        self,
        obs: dict,
        action: np.ndarray,
        reward: float,
        next_obs: dict,
        done: bool,
        gt_uv: np.ndarray | None = None,
    ) -> None:
        i = self._ptr
        for k in self._cam_keys:
            self._images[k][i]      = obs[k]
            self._images_next[k][i] = next_obs[k]
        self._proprios[i]      = obs["proprio"]
        self._proprios_next[i] = next_obs["proprio"]
        self._actions[i]  = action
        self._rewards[i]  = reward
        self._dones[i]    = float(done)
        self._gt_uvs[i]   = gt_uv if gt_uv is not None else np.array([np.nan, np.nan])
        self._ptr = (i + 1) % self.capacity
        self._size = min(self._size + 1, self.capacity)

    def sample(self, batch_size: int, device: torch.device) -> tuple[dict, ...]:
        idx = np.random.randint(0, self._size, size=batch_size)

        def _imgs(store):
            return {k: _to_tensor(store[k][idx], device) for k in self._cam_keys}

        obs = {**_imgs(self._images), "proprio": _f(self._proprios[idx], device)}
        nxt = {**_imgs(self._images_next), "proprio": _f(self._proprios_next[idx], device)}
        actions = _f(self._actions[idx], device)
        rewards = _f(self._rewards[idx], device)
        dones   = _f(self._dones[idx], device)
        gt_uvs  = _f(self._gt_uvs[idx], device)   # (B, 2); rows may contain NaN
        return obs, actions, rewards, nxt, dones, gt_uvs

    def __len__(self) -> int:
        return self._size


def _to_tensor(arr: np.ndarray, device: torch.device) -> torch.Tensor:
    """Convert H×W×C uint8 numpy batch to B×C×H×W float [0,1] tensor."""
    t = torch.from_numpy(arr).permute(0, 3, 1, 2).float().to(device) / 255.0
    return t


def _f(arr: np.ndarray, device: torch.device) -> torch.Tensor:
    return torch.from_numpy(arr).to(device)


# ── SAC Trainer ─────────────────────────────────────────────────────────────

class SACTrainer:
    """Soft Actor-Critic with auxiliary port detection loss.

    Detection aux loss:
        L_det = MSE(actor.detection_head(features), gt_uv)
    where gt_uv is the projected port center in the center camera frame,
    provided alongside the environment transition.

    gt_uv should be a (2,) float32 array in [-1, 1] pixel coords,
    or None to skip the detection loss for that transition.
    """

    def __init__(
        self,
        phase: int,
        actor_lr: float = 3e-4,
        critic_lr: float = 3e-4,
        alpha_lr: float = 1e-4,
        gamma: float = 0.99,
        tau: float = 0.005,
        alpha_init: float = 0.1,
        det_loss_weight: float = 0.1,
        device: torch.device = DEVICE,
    ):
        self.phase = phase
        self.gamma = gamma
        self.tau = tau
        self.det_loss_weight = det_loss_weight
        self.device = device

        self.actor = SACActor(phase).to(device)
        self.critic = SACCritic(phase).to(device)
        self.critic_target = copy.deepcopy(self.critic)
        for p in self.critic_target.parameters():
            p.requires_grad_(False)

        self.actor_opt  = optim.Adam(self.actor.parameters(), lr=actor_lr)
        self.critic_opt = optim.Adam(self.critic.parameters(), lr=critic_lr)

        action_dim = P1_ACTION_DIM if phase == 1 else P2_ACTION_DIM
        target_entropy = -float(action_dim)
        self.log_alpha = torch.tensor(np.log(alpha_init), requires_grad=True, device=device)
        self.alpha_opt = optim.Adam([self.log_alpha], lr=alpha_lr)
        self.target_entropy = target_entropy

        self._train_steps = 0

    # ── Properties ─────────────────────────────────────────────────────────

    @property
    def alpha(self) -> torch.Tensor:
        return self.log_alpha.exp()

    # ── Core update ────────────────────────────────────────────────────────

    def update(
        self,
        buffer: ReplayBuffer,
        batch_size: int = 256,
    ) -> dict:
        """One gradient step.  Returns dict of scalar losses for logging.

        GT detection labels are pulled from the per-transition buffer entries.
        Transitions without GT (NaN) are masked out of the detection loss.
        """
        obs, actions, rewards, next_obs, dones, gt_uvs_batch = buffer.sample(batch_size, self.device)

        # ── Critic update ──────────────────────────────────────────────────
        with torch.no_grad():
            next_actions, next_log_pi, _ = self.actor(next_obs)
            q1_next, q2_next = self.critic_target(next_obs, next_actions)
            q_next = torch.min(q1_next, q2_next) - self.alpha * next_log_pi
            q_target = rewards + self.gamma * (1.0 - dones) * q_next

        q1, q2 = self.critic(obs, actions)
        critic_loss = F.mse_loss(q1, q_target) + F.mse_loss(q2, q_target)

        self.critic_opt.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_opt.step()

        # ── Actor + detection auxiliary update ─────────────────────────────
        new_actions, log_pi, det_uv = self.actor(obs)
        q1_pi, q2_pi = self.critic(obs, new_actions)
        q_pi = torch.min(q1_pi, q2_pi)

        actor_loss = (self.alpha.detach() * log_pi - q_pi).mean()

        # Detection auxiliary loss — only on transitions with valid GT
        valid_mask = ~torch.isnan(gt_uvs_batch).any(dim=1)
        if valid_mask.any():
            det_loss = F.mse_loss(det_uv[valid_mask], gt_uvs_batch[valid_mask])
            total_actor_loss = actor_loss + self.det_loss_weight * det_loss
        else:
            det_loss = torch.tensor(0.0)
            total_actor_loss = actor_loss

        self.actor_opt.zero_grad()
        total_actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_opt.step()

        # ── Temperature update ─────────────────────────────────────────────
        alpha_loss = -(self.log_alpha * (log_pi.detach() + self.target_entropy)).mean()
        self.alpha_opt.zero_grad()
        alpha_loss.backward()
        self.alpha_opt.step()

        # ── Soft update target critic ──────────────────────────────────────
        with torch.no_grad():
            for p, pt in zip(self.critic.parameters(), self.critic_target.parameters()):
                pt.data.copy_(self.tau * p.data + (1.0 - self.tau) * pt.data)

        self._train_steps += 1
        return {
            "critic_loss": float(critic_loss),
            "actor_loss":  float(actor_loss),
            "det_loss":    float(det_loss),
            "alpha":       float(self.alpha),
            "alpha_loss":  float(alpha_loss),
        }

    # ── Checkpointing ──────────────────────────────────────────────────────

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        torch.save({
            "actor":  self.actor.state_dict(),
            "critic": self.critic.state_dict(),
            "log_alpha": self.log_alpha,
            "train_steps": self._train_steps,
        }, path)

    def load(self, path: str) -> None:
        ckpt = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(ckpt["actor"])
        self.critic.load_state_dict(ckpt["critic"])
        self.critic_target = copy.deepcopy(self.critic)
        for p in self.critic_target.parameters():
            p.requires_grad_(False)
        self.log_alpha = ckpt["log_alpha"].to(self.device).requires_grad_(True)
        self.alpha_opt = optim.Adam([self.log_alpha], lr=1e-4)
        self._train_steps = ckpt.get("train_steps", 0)
