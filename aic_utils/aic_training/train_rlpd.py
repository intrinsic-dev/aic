#!/usr/bin/env python3
"""
RLPD (Reinforcement Learning with Prior Data) training for the AIC cable-insertion task.

Currently runs as online SAC only.  Offline data mixing is stubbed out — see
OfflineDataset and _merge() for the planned RLPD extension.

Actor:   RobotWithTaskPolicy — images + robot_state + prev_action + task → (means, log_stds)
         Actions sampled via TransformedDistribution(Normal, TanhTransform) over R^9.
Critic:  ImageConditionedQFunction — double Q-network with its own ImageConditioner.
         Input: images + robot_state + prev_action + action → scalar.
Temp:    Learnable log_alpha, auto-tuned to match target entropy = -action_dim.

References:
  Ball et al.    "RLPD: Efficient RL with Prior Data" (ICML 2023)
  Haarnoja et al. "Soft Actor-Critic" (ICML 2018)
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from collections import deque
from pathlib import Path
from threading import Lock

import cv2
import logging
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal, TransformedDistribution
from torch.distributions.transforms import TanhTransform

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------

_AIC_UTILS = Path(__file__).resolve().parent.parent
if str(_AIC_UTILS) not in sys.path:
    sys.path.insert(0, str(_AIC_UTILS))

import rclpy  # noqa: F401 — triggers rclpy context init

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Wrench
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from aic_training.aic_gym_env import AICGymEnv, AICGymEnvConfig
from aic_training.robot_vit_with_task import (
    ImageConditioner,
    ROBOT_STATE_DIM,
    TOKEN_DIM,
    RobotWithTaskPolicy,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CAMERA_TOPICS = {
    "left_camera":   "/left_camera/image",
    "center_camera": "/center_camera/image",
    "right_camera":  "/right_camera/image",
}
CAMERA_KEYS = tuple(CAMERA_TOPICS.keys())

IMG_H      = 256
IMG_W      = 288
ACTION_DIM = 9   # 3 position + 6 rot6d

TASK_TARGET_MODULE_ENCODING: dict[str, int] = {
    "nic_card_mount_0": 0, "nic_card_mount_1": 1, "nic_card_mount_2": 2,
    "nic_card_mount_3": 3, "nic_card_mount_4": 4,
    "sc_port_0": 5,        "sc_port_1": 6,
}
TASK_PORT_NAME_ENCODING: dict[str, int] = {
    "sfp_port_0": 0, "sfp_port_1": 1, "sc_port_base": 2,
}


def encode_task(task_info: dict) -> tuple[int, int]:
    tm = TASK_TARGET_MODULE_ENCODING.get(task_info.get("target_module_name", ""), -1)
    pn = TASK_PORT_NAME_ENCODING.get(task_info.get("port_name", ""), -1)
    return max(tm, 0), max(pn, 0)   # clamp to 0 so embedding never crashes

LOG_STD_MIN = -20.0
LOG_STD_MAX =   2.0

# RGB colors per entity — mirrors _ENTITY_COLORS in keypoint_model/train.py
_KEYPOINT_ENTITY_COLORS: dict[str, tuple[int, int, int]] = {
    "nic_card_mount_0": (255, 68,  68),
    "nic_card_mount_1": (255, 136,  0),
    "nic_card_mount_2": (255, 238,  0),
    "nic_card_mount_3": (  0, 255, 170),
    "nic_card_mount_4": (255, 68,  255),
    "sc_port_0":        ( 68, 255,  68),
    "sc_port_1":        ( 68, 170, 255),
}
_KEYPOINT_CAM_ORDER = ["left_camera", "center_camera", "right_camera"]

# ---------------------------------------------------------------------------
# Rotation helpers  (mirrors train_qsm_flow_matching.py)
# ---------------------------------------------------------------------------

def _rot6d_to_quat_xyzw(rot6d: np.ndarray) -> np.ndarray:
    r1 = rot6d[:3].astype(np.float64)
    r2 = rot6d[3:6].astype(np.float64)
    r1 = r1 / (np.linalg.norm(r1) + 1e-8)
    r2 = r2 - np.dot(r1, r2) * r1
    r2 = r2 / (np.linalg.norm(r2) + 1e-8)
    r3 = np.cross(r1, r2)
    R  = np.stack([r1, r2, r3], axis=0)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w, x = 0.25 / s, (R[2, 1] - R[1, 2]) * s
        y, z  = (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z  = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z  = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z  = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float32)
    return q / (np.linalg.norm(q) + 1e-8)


def _action9_to_pose7(action9: np.ndarray) -> np.ndarray:
    return np.concatenate([action9[:3], _rot6d_to_quat_xyzw(action9[3:9])]).astype(np.float32)


def _quat_xyzw_to_rot6d(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = quat.astype(np.float64)
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),      1 - 2*(x*x + z*z),   2*(y*z - w*x)],
    ], dtype=np.float32)
    return R.flatten()


# ---------------------------------------------------------------------------
# TanhNormal — reparameterised squashed Gaussian
# ---------------------------------------------------------------------------

def _tanh_normal(means: torch.Tensor, log_stds: torch.Tensor) -> TransformedDistribution:
    """Return a tanh-squashed Normal distribution.

    Uses TanhTransform(cache_size=1) so that log_prob(rsample()) reuses the
    cached pre-tanh value — no atanh call, no precision loss, no extra work.
    PyTorch's stable log|det J| replaces the ad-hoc (1-a²+ε) formula.
    """
    stds = log_stds.clamp(LOG_STD_MIN, LOG_STD_MAX).exp()
    return TransformedDistribution(Normal(means, stds), TanhTransform(cache_size=1))


# ---------------------------------------------------------------------------
# ImageConditionedQFunction — double Q-network
# ---------------------------------------------------------------------------

class ImageConditionedQFunction(nn.Module):
    """Double Q-function with image conditioning.

    Input:  images + robot_state + prev_action + action
    Output: (q1, q2) each (B,)

    The critic has its own ImageConditioner (independent from the actor's)
    so actor and critic gradients never interfere through the backbone.
    """

    def __init__(
        self,
        robot_state_dim:    int = ROBOT_STATE_DIM,
        action_dim:         int = ACTION_DIM,
        hidden_dim:         int = 512,
        camera_keys: tuple[str, ...] | None = None,
        resnet_name:        str = "resnet18",
        resnet_weights:     str = "IMAGENET1K_V1",
        local_ckpt_path:    str | None = None,
        features_per_group: int = 16,
    ):
        super().__init__()
        self.image_conditioner = ImageConditioner(
            resnet_name=resnet_name,
            weights=resnet_weights,
            features_per_group=features_per_group,
            camera_keys=camera_keys,
            local_weights_path=local_ckpt_path,
        )
        img_dim = self.image_conditioner.num_cameras * TOKEN_DIM
        in_dim  = img_dim + robot_state_dim + action_dim + action_dim  # +prev_action +action

        def _mlp() -> nn.Sequential:
            return nn.Sequential(
                nn.LayerNorm(in_dim),
                nn.Linear(in_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, 1),
            )

        self.q1 = _mlp()
        self.q2 = _mlp()

    def forward(
        self,
        images:      dict,
        robot_state: torch.Tensor,   # (B, robot_state_dim)
        prev_action: torch.Tensor,   # (B, action_dim)
        action:      torch.Tensor,   # (B, action_dim)
    ) -> tuple[torch.Tensor, torch.Tensor]:
        img_feat = self.image_conditioner(images)                     # (B, cams*TOKEN_DIM)
        x = torch.cat([img_feat, robot_state, prev_action, action], dim=-1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)        # (B,), (B,)


# ---------------------------------------------------------------------------
# AICRLEnv  (camera subscriptions + pose action step)
# ---------------------------------------------------------------------------

class AICRLEnv(AICGymEnv):
    """AICGymEnv extended with camera observations and pose action execution."""

    def __init__(
        self,
        config: AICGymEnvConfig | None = None,
        keypoint_checkpoint: str | None = None,
    ):
        super().__init__(config)
        self._img_lock = Lock()
        self._latest_images: dict[str, ROSImage | None] = {k: None for k in CAMERA_TOPICS}
        self._high_force_since: float | None = None

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        self._last_insertion_event_data: str = ""
        self._cached_port_pos:   np.ndarray | None = None
        self._cached_port_frame: str = ""
        self._prev_tcp_pos:      np.ndarray | None = None

        # Keypoint model (optional) — used by _spot_target
        self._keypoint_model = None
        if keypoint_checkpoint:
            from aic_training.keypoint_model import PortKeypointNet as _PortKeypointNet
            _kp_dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            _ckpt   = torch.load(keypoint_checkpoint, map_location="cpu")
            _kp     = _PortKeypointNet(pretrained=False)
            _kp.load_state_dict(_ckpt.get("model", _ckpt))
            _kp.eval().to(_kp_dev)
            self._keypoint_model = _kp
            logger.info(f"[RLPD] Keypoint model loaded from {keypoint_checkpoint}")

        for cam_key, topic in CAMERA_TOPICS.items():
            def _make_cb(key: str):
                def _cb(msg: ROSImage) -> None:
                    with self._img_lock:
                        self._latest_images[key] = msg
                return _cb
            self._node.create_subscription(ROSImage, topic, _make_cb(cam_key), 10)

    @staticmethod
    def _ros_image_to_tensor(msg: ROSImage) -> torch.Tensor:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ("bgr8", "bgra8"):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
        return torch.from_numpy(img).permute(2, 0, 1)   # (C, H, W) uint8

    def get_images(self) -> dict[str, torch.Tensor] | None:
        with self._img_lock:
            if any(v is None for v in self._latest_images.values()):
                return None
            return {k: self._ros_image_to_tensor(v) for k, v in self._latest_images.items()}

    def _insertion_event_cb(self, msg: String) -> None:
        self._last_insertion_event_data = msg.data
        self._last_insertion_event = 1.0 if msg.data else 0.0

    def reset(self, **kwargs):
        self._high_force_since = None
        self._last_insertion_event_data = ""
        self._cached_port_pos = None
        self._cached_port_frame = ""
        self._prev_tcp_pos = None
        return super().reset(**kwargs)

    def _compute_reward(
        self, obs: np.ndarray, action: np.ndarray, info: dict
    ) -> tuple[float, float]:
        """Return (reward, long_horizon_reward).

        long_horizon_reward contains only the insertion-event and force-penalty
        contributions so the caller can retroactively credit prior steps.
        """
        reward = -0.01   # step penalty
        long_horizon_reward = 0.0

        # Distance to target port (ground-truth TF)
        task = info.get("task", {})
        target_module_name = task.get("target_module_name", "")
        port_name          = task.get("port_name", "")
        if target_module_name and port_name:
            port_frame = f"task_board/{target_module_name}/{port_name}_link"
            if self._cached_port_frame != port_frame or self._cached_port_pos is None:
                try:
                    tf_stamped = self._tf_buffer.lookup_transform(
                        "base_link", port_frame, Time()
                    )
                    t = tf_stamped.transform.translation
                    self._cached_port_pos   = np.array([t.x, t.y, t.z], dtype=np.float32)
                    self._cached_port_frame = port_frame
                except TransformException:
                    pass
            if self._cached_port_pos is not None:
                tcp_pos  = obs[0:3].astype(np.float32)
                dist     = float(np.linalg.norm(tcp_pos - self._cached_port_pos))
                if self._prev_tcp_pos is not None:
                    prev_dist = float(np.linalg.norm(self._prev_tcp_pos - self._cached_port_pos))
                    delta = prev_dist - dist           # positive = getting closer
                    reward += 5.0 * delta
                    delta_z = float(tcp_pos[2] - self._prev_tcp_pos[2])
                    if delta_z > 0:
                        reward -= 4.0 * delta_z       # penalise moving upward
                    # While XY is still far, penalise large z changes (waste of motion)
                    xy_dist = float(np.linalg.norm(
                        (tcp_pos - self._cached_port_pos)[:2]
                    ))
                    if xy_dist > 0.05:
                        reward -= 2.0 * abs(delta_z)
                self._prev_tcp_pos = tcp_pos

        # Insertion event — reward only if it matches the target task
        event_data = self._last_insertion_event_data
        if event_data:
            if target_module_name and port_name \
                    and target_module_name in event_data and port_name in event_data:
                insertion_r = 3.0
                logger.info(
                    f"[RLPD][INSERTION] CORRECT  module={target_module_name}"
                    f"  port={port_name}  event='{event_data}'"
                )
            else:
                insertion_r = -3.0
                logger.info(
                    f"[RLPD][INSERTION] WRONG    target=({target_module_name}/{port_name})"
                    f"  event='{event_data}'"
                )
            reward              += insertion_r
            long_horizon_reward += insertion_r

        # Force penalty
        force = obs[20:23]
        if np.any(np.abs(force) > 20.0):
            if self._high_force_since is None:
                self._high_force_since = time.monotonic()
            elif time.monotonic() - self._high_force_since >= 1.0:
                reward              -= 2.5
                long_horizon_reward -= 2.5
        else:
            self._high_force_since = None
            if np.any(np.abs(force) > 15.0):
                reward              -= 1.0
                long_horizon_reward -= 1.0

        # Tilt penalty
        qx, qy  = float(obs[3]), float(obs[4])
        z_align = 1.0 - 2.0 * (qx * qx + qy * qy)
        tilt    = 1.0 - abs(z_align)
        if tilt > 0.3:
            reward -= 2.0 * (tilt - 0.3)

        return reward, long_horizon_reward

    def _spot_target(
        self,
        images: dict[str, torch.Tensor],   # {cam_key: (C, H, W) uint8}
        target_module: str,
        port_name: str,
    ) -> dict[str, torch.Tensor]:
        """Overlay confidence-weighted circles where the target port is detected.

        Uses the loaded PortKeypointNet to find the (target_module, port_name)
        keypoint in each camera frame and draws a filled circle whose alpha is
        proportional to conf_visible (only drawn when conf_visible > 0.5).

        Returns a new images dict with the same dtype/shape as the input.
        No-op if no keypoint model is loaded or task strings are empty.
        """
        if self._keypoint_model is None or not target_module or not port_name:
            return images

        from aic_training.keypoint_constants import OUTPUT_KEYS

        img_h = images[_KEYPOINT_CAM_ORDER[0]].shape[1]
        img_w = images[_KEYPOINT_CAM_ORDER[0]].shape[2]
        radius = max(2, img_h // 80)   # ~1.25 % of frame height

        # Build normalized input list for the keypoint model: (1, 3, H, W) float32
        kp_dev = next(self._keypoint_model.parameters()).device
        imgs_list = []
        for cam in _KEYPOINT_CAM_ORDER:
            img_f    = images[cam].float() / 255.0        # [0, 1]
            img_norm = (img_f - 0.5) / 0.5               # mean=0.5, std=0.5
            imgs_list.append(img_norm.unsqueeze(0).to(kp_dev))

        with torch.no_grad():
            pred = self._keypoint_model(imgs_list)

        conf_visible = pred["conf_visible"][0].cpu()   # (36,)
        xy_pred      = pred["xy"][0].cpu()             # (36, 2)

        # Copy images to numpy HWC for cv2 drawing
        numpy_imgs = {
            cam: images[cam].permute(1, 2, 0).numpy().copy()
            for cam in _KEYPOINT_CAM_ORDER
        }

        for out_idx, (entity, port, cam) in enumerate(OUTPUT_KEYS):
            if entity != target_module or port != port_name:
                continue
            cv_val = float(conf_visible[out_idx])
            if cv_val < 0.5:
                continue
            xn = float(xy_pred[out_idx, 0])
            yn = float(xy_pred[out_idx, 1])
            px = int(xn * img_w)
            py = int(yn * img_h)
            color = _KEYPOINT_ENTITY_COLORS.get(entity, (255, 255, 255))
            img_np  = numpy_imgs[cam]
            overlay = img_np.copy()
            cv2.circle(overlay, (px, py), radius=radius, color=color, thickness=-1)
            # logger.info("LXH debug circle added:", cv_val, px, py, ",radius:", radius)
            # alpha-blend: circle at opacity = conf_visible
            cv2.addWeighted(overlay, cv_val, img_np, 1.0 - cv_val, 0, img_np)

        return {
            cam: torch.from_numpy(numpy_imgs[cam]).permute(2, 0, 1)
            for cam in images
        }

    def send_pose_action(self, pos: np.ndarray, quat: np.ndarray) -> None:
        msg = MotionUpdate()
        msg.header.stamp     = self._node.get_clock().now().to_msg()
        msg.header.frame_id  = self.config.frame_id
        msg.pose = Pose(
            position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
            orientation=Quaternion(x=float(quat[0]), y=float(quat[1]),
                                   z=float(quat[2]), w=float(quat[3])),
        )
        msg.target_stiffness              = np.diag(self.config.cartesian_stiffness).flatten()
        msg.target_damping                = np.diag(self.config.cartesian_damping).flatten()
        msg.feedforward_wrench_at_tip     = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip  = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
        self._motion_update_pub.publish(msg)

    def _check_truncated(self, obs: np.ndarray, info: dict) -> bool:
        if super()._check_truncated(obs, info):
            return True
        # Truncate early when high force is sustained for > 1 seconds — prevents
        # long stuck episodes that flood the buffer with identical bad transitions.
        if self._high_force_since is not None:
            if time.monotonic() - self._high_force_since >= 1.0:
                logger.info("[RLPD] Truncating: sustained high force > 1 s")
                return True
        return False

    def step_pose(self, action_9d: np.ndarray):
        self._step_count += 1
        pose7 = _action9_to_pose7(action_9d)
        self.send_pose_action(pose7[:3], pose7[3:7])
        time.sleep(self._step_period)
        obs        = self._get_observation()
        info       = self._get_info()
        reward, lhr = self._compute_reward(obs, action_9d, info)
        terminated = self._check_terminated(obs, info)
        truncated  = self._check_truncated(obs, info)
        return obs, reward, lhr, terminated, truncated, info


# ---------------------------------------------------------------------------
# Debug visualisation for _spot_target
# ---------------------------------------------------------------------------

def _save_spot_debug(
    images: dict[str, torch.Tensor],
    target_module: str,
    port_name: str,
    save_path: Path,
) -> None:
    """Save a 3-panel figure showing the _spot_target overlay for each camera."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.info("[RLPD] matplotlib not available — skipping spot debug image")
        return

    cam_order = [k for k in _KEYPOINT_CAM_ORDER if k in images]
    fig, axes = plt.subplots(1, len(cam_order), figsize=(6 * len(cam_order), 5))
    if len(cam_order) == 1:
        axes = [axes]

    fig.suptitle(f"{target_module}  /  {port_name}", fontsize=12)
    for ax, cam in zip(axes, cam_order):
        ax.imshow(images[cam].permute(1, 2, 0).numpy())
        ax.set_title(cam, fontsize=9)
        ax.axis("off")

    plt.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    logger.info(f"[RLPD] Spot-target debug image: {save_path}")


# ---------------------------------------------------------------------------
# ImageStore
# ---------------------------------------------------------------------------

class ImageStore:
    """Circular buffer of single-frame image dicts (stored as uint8)."""

    def __init__(self, capacity: int):
        self._capacity = capacity
        self._frames: list[dict | None] = [None] * capacity
        self._write_idx = 0

    def push(self, frame: dict[str, torch.Tensor]) -> int:
        idx = self._write_idx
        self._frames[idx] = {k: v.clone() for k, v in frame.items()}
        self._write_idx = (self._write_idx + 1) % self._capacity
        return idx

    def get(self, idx: int) -> dict[str, torch.Tensor]:
        return {k: v.float() / 255.0 for k, v in self._frames[idx].items()}


# ---------------------------------------------------------------------------
# Online ReplayBuffer
# ---------------------------------------------------------------------------

class ReplayBuffer:
    """Circular buffer for online transitions.

    Each observation:
        image_idx:   int — index into ImageStore (single current frame)
        robot_state: (ROBOT_STATE_DIM,) float32
        prev_action: (ACTION_DIM,) float32 — action executed at t-1
    """

    def __init__(self, capacity: int, image_store: ImageStore):
        self._buf: deque[dict] = deque(maxlen=capacity)
        self._image_store = image_store
        self._trajectory_idx: int = 0

    def __len__(self) -> int:
        return len(self._buf)

    def new_trajectory(self) -> None:
        """Call at every episode reset to mark the boundary between trajectories."""
        self._trajectory_idx += 1

    def push(
        self,
        obs:        dict,
        action:     np.ndarray,
        reward:     float,
        next_obs:   dict,
        terminated: bool,
        truncated:  bool,
    ) -> None:
        self._buf.append({
            "obs":            obs,
            "action":         action.astype(np.float32),
            "reward":         float(reward),
            "next_obs":       next_obs,
            "terminated":     bool(terminated),
            "truncated":      bool(truncated),
            "trajectory_idx": self._trajectory_idx,
        })

    def update_recent_rewards(
        self, lhr: float, n_steps: int = 8, decay: float = 0.95
    ) -> None:
        """Add decayed lhr to the n_steps transitions before the latest push.

        Step k steps back from the most recent transition receives lhr * decay^k.
        Stops early if a transition from a different trajectory is encountered,
        so long_horizon_reward never bleeds across episode boundaries.
        The current (just-pushed) transition already contains lhr in its reward,
        so we start from k=1.
        """
        if lhr == 0.0:
            return
        current_traj = self._buf[-1]["trajectory_idx"]
        n_available  = len(self._buf) - 1   # exclude the just-pushed transition
        for k in range(1, min(n_steps, n_available) + 1):
            entry = self._buf[-1 - k]
            if entry["trajectory_idx"] != current_traj:
                break
            entry["reward"] += lhr * (decay ** k)

    def sample(self, n: int) -> list[dict]:
        return random.sample(self._buf, min(n, len(self._buf)))

    def sample_batch(self, n: int) -> dict:
        return _collate_online(self.sample(n), self._image_store)


def _collate_online(items: list[dict], store: ImageStore) -> dict:
    """Collate online transitions into batched tensors.

    Images are shaped (B, 1, C, H, W) — the time dimension (obs_horizon=1)
    expected by ImageEncoder inside RobotWithTaskPolicy.
    """
    cam_keys = list(store.get(items[0]["obs"]["image_idx"]).keys())

    def _imgs(key: str) -> dict[str, torch.Tensor]:
        # stack (C,H,W) frames → (B,C,H,W) then unsqueeze T → (B,1,C,H,W)
        return {k: torch.stack([store.get(it[key]["image_idx"])[k] for it in items]).unsqueeze(1)
                for k in cam_keys}

    return {
        "images":           _imgs("obs"),
        "robot_state":      torch.from_numpy(np.stack([it["obs"]["robot_state"]        for it in items])),
        "prev_action":      torch.from_numpy(np.stack([it["obs"]["prev_action"]        for it in items])),
        "target_module":    torch.tensor([it["obs"]["target_module"] for it in items], dtype=torch.long),
        "port_name":        torch.tensor([it["obs"]["port_name"]     for it in items], dtype=torch.long),
        "actions":          torch.from_numpy(np.stack([it["action"]                    for it in items])),
        "rewards":          torch.tensor([it["reward"]     for it in items], dtype=torch.float32),
        "next_images":      _imgs("next_obs"),
        "next_robot_state": torch.from_numpy(np.stack([it["next_obs"]["robot_state"]   for it in items])),
        "next_prev_action": torch.from_numpy(np.stack([it["next_obs"]["prev_action"]   for it in items])),
        "next_target_module": torch.tensor([it["next_obs"]["target_module"] for it in items], dtype=torch.long),
        "next_port_name":     torch.tensor([it["next_obs"]["port_name"]     for it in items], dtype=torch.long),
        "dones":            torch.tensor([it["terminated"] for it in items], dtype=torch.float32),
    }


# ---------------------------------------------------------------------------
# Offline dataset  (placeholder — not yet wired into training)
# ---------------------------------------------------------------------------

class OfflineDataset:
    """TODO: load demonstration .npz episodes and expose them as transitions.

    Expected npz keys per episode file:
        robot_states:     (T, ROBOT_STATE_DIM) float32
        actions:          (T, ACTION_DIM) float32
        images_{cam_key}: (T, C, H, W) uint8   for each camera in CAMERA_KEYS
        rewards:          (T,) float32          [optional; sparse +1 at end if absent]

    Transition at time t (for t ≥ 1):
        s_t   = (images[t-1], robot_states[t-1], actions[t-1])   ← prev_action = actions[t-1]
        a_t   = actions[t],  r_t = rewards[t],  done = (t == T-1)
        s_t+1 = (images[t],  robot_states[t],  actions[t])

    Not loaded during training until offline mixing is enabled.
    """

    def __init__(
        self,
        episode_dir:   str,
        camera_keys:   tuple[str, ...] = CAMERA_KEYS,
        sparse_reward: bool = False,
    ):
        raise NotImplementedError(
            "OfflineDataset is a placeholder.  "
            "Set --offline_dir only when offline mixing is implemented."
        )

    def __len__(self) -> int:
        return 0

    def sample(self, n: int) -> dict:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Batch mixing helper  (placeholder — used once OfflineDataset is active)
# ---------------------------------------------------------------------------

def _merge(offline: dict, online: dict, device: torch.device) -> dict:
    """Interleave offline and online samples (alternating rows, as in original RLPD).

    Alternating ensures LayerNorm sees both sources in every forward pass,
    consistent with the original paper's implementation.
    """
    # TODO: wire up once OfflineDataset is active
    merged: dict = {}
    for k in offline:
        ov, nv = offline[k], online[k]
        if isinstance(ov, dict):
            merged[k] = {}
            for ck in ov:
                a = ov[ck].to(device)
                b = nv[ck].to(device)
                out = torch.empty(a.shape[0] + b.shape[0], *a.shape[1:],
                                  dtype=a.dtype, device=device)
                out[0::2] = a
                out[1::2] = b
                merged[k][ck] = out
        else:
            a = ov.to(device)
            b = nv.to(device)
            out = torch.empty(a.shape[0] + b.shape[0], *a.shape[1:],
                              dtype=a.dtype, device=device)
            out[0::2] = a
            out[1::2] = b
            merged[k] = out
    return merged


# ---------------------------------------------------------------------------
# SAC update functions
# ---------------------------------------------------------------------------

@torch.no_grad()
def soft_update(target: nn.Module, source: nn.Module, tau: float) -> None:
    for tp, sp in zip(target.parameters(), source.parameters()):
        tp.data.copy_(tau * sp.data + (1.0 - tau) * tp.data)


def update_critic(
    critic:        ImageConditionedQFunction,
    target_critic: ImageConditionedQFunction,
    policy:        RobotWithTaskPolicy,
    log_alpha:     torch.Tensor,
    opt:           torch.optim.Optimizer,
    batch:         dict,
    device:        torch.device,
    gamma:         float = 0.99,
    tau:           float = 0.005,
) -> float:
    """SAC Bellman update for both Q-heads.

    TD target = r + γ(1-done)[min(Q1_tgt, Q2_tgt)(s', a') - α log π(a'|s')]
    where a' ~ π(s') is a fresh reparameterised sample from the current policy.
    """
    images               = {k: v.to(device) for k, v in batch["images"].items()}
    robot_state          = batch["robot_state"].to(device)
    prev_action          = batch["prev_action"].to(device)
    target_module        = batch["target_module"].to(device)
    port_name            = batch["port_name"].to(device)
    actions              = batch["actions"].to(device)
    rewards              = batch["rewards"].to(device)
    dones                = batch["dones"].to(device)
    next_images          = {k: v.to(device) for k, v in batch["next_images"].items()}
    next_robot_state     = batch["next_robot_state"].to(device)
    next_prev_action     = batch["next_prev_action"].to(device)
    next_target_module   = batch["next_target_module"].to(device)
    next_port_name       = batch["next_port_name"].to(device)

    with torch.no_grad():
        next_means, next_log_stds = policy(
            next_images, next_robot_state, next_prev_action,
            next_target_module, next_port_name,
        )
        next_dist = _tanh_normal(next_means, next_log_stds)
        next_a    = next_dist.rsample()
        next_lp   = next_dist.log_prob(next_a).sum(-1)                       # (B,)

        q1_t, q2_t = target_critic(next_images, next_robot_state, next_prev_action, next_a)
        next_v     = torch.min(q1_t, q2_t) - log_alpha.exp() * next_lp
        td_target  = (rewards + gamma * (1.0 - dones) * next_v).clamp(-300.0, 300.0)

    q1, q2 = critic(images, robot_state, prev_action, actions)
    loss   = F.mse_loss(q1, td_target) + F.mse_loss(q2, td_target)

    opt.zero_grad()
    loss.backward()
    nn.utils.clip_grad_norm_(critic.parameters(), 1.0)
    opt.step()
    soft_update(target_critic, critic, tau)
    return loss.item(), q1.mean().item(), q2.mean().item()


def update_actor(
    policy:      RobotWithTaskPolicy,
    critic:      ImageConditionedQFunction,
    log_alpha:   torch.Tensor,
    opt:         torch.optim.Optimizer,
    batch:       dict,
    device:      torch.device,
    normalize_q: bool = False,
) -> tuple[float, float]:
    """SAC actor update: maximise E[Q(s,a)] - α H(π).

    Returns:
        actor_loss:    scalar loss value
        mean_log_prob: mean log π(a|s) used for temperature update
    """
    images        = {k: v.to(device) for k, v in batch["images"].items()}
    robot_state   = batch["robot_state"].to(device)
    prev_action   = batch["prev_action"].to(device)
    target_module = batch["target_module"].to(device)
    port_name     = batch["port_name"].to(device)

    means, log_stds = policy(images, robot_state, prev_action, target_module, port_name)
    dist = _tanh_normal(means, log_stds)
    a    = dist.rsample()
    lp   = dist.log_prob(a).sum(-1)     # (B,)

    q1, q2 = critic(images, robot_state, prev_action, a)
    q_min  = torch.min(q1, q2)          # (B,)
    if normalize_q:
        q_min = (q_min - q_min.mean()) / (q_min.std() + 1e-8)
    actor_loss = (log_alpha.exp().detach() * lp - q_min).mean()

    opt.zero_grad()
    actor_loss.backward()
    nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
    opt.step()
    return actor_loss.item(), lp.mean().item()


def update_temperature(
    log_alpha:      torch.Tensor,
    alpha_opt:      torch.optim.Optimizer,
    mean_log_prob:  float,
    target_entropy: float,
) -> float:
    """Auto-tune α so that E[-log π] ≈ target_entropy.

    Loss: L(α) = -α (log π + H̄)  → increase α when entropy is below target.
    """
    alpha_loss = -(log_alpha * (mean_log_prob + target_entropy))
    alpha_opt.zero_grad()
    alpha_loss.backward()
    alpha_opt.step()
    return alpha_loss.item()


# ---------------------------------------------------------------------------
# Observation helpers
# ---------------------------------------------------------------------------

def build_robot_state(flat_obs: np.ndarray) -> np.ndarray:
    """Extract 20-D robot state: tcp_pose(7) + joint_positions(7) + wrench(6)."""
    return np.concatenate([flat_obs[0:7], flat_obs[13:20], flat_obs[20:26]]).astype(np.float32)


# ---------------------------------------------------------------------------
# Main training loop
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="RLPD training for AIC cable-insertion")

    # Model
    parser.add_argument("--checkpoint",          type=str, default="",
                        help="Pretrained RobotPolicy .pt to warm-start from.")
    parser.add_argument("--resume",              type=str, default="",
                        help="Previous RLPD .pt checkpoint to resume from.")
    parser.add_argument("--resnet_weights",      type=str, default="IMAGENET1K_V1")
    parser.add_argument("--local_ckpt_path",     type=str, default=None,
                        help="Local ResNet weights .pth (overrides --resnet_weights).")
    parser.add_argument("--keypoint_checkpoint", type=str, default=None,
                        help="PortKeypointNet checkpoint (.pt) for _spot_target overlay. "
                             "Leave empty to skip the keypoint preprocessing step.")

    # Offline data (placeholder — not active yet)
    parser.add_argument("--offline_dir",   type=str, default="",
                        help="[placeholder] Directory of demonstration .npz files.")
    parser.add_argument("--offline_ratio", type=float, default=0.5,
                        help="[placeholder] Offline fraction per batch (ignored until OfflineDataset is enabled).")
    parser.add_argument("--sparse_reward", action="store_true",
                        help="[placeholder] Use sparse +1 reward for offline demos.")

    # SAC hyperparameters
    parser.add_argument("--gamma",          type=float, default=0.99)
    parser.add_argument("--tau",            type=float, default=0.005,
                        help="Polyak coefficient for target critic EMA.")
    parser.add_argument("--init_alpha",     type=float, default=0.1,
                        help="Initial entropy temperature α.")
    parser.add_argument("--target_entropy", type=float, default=None,
                        help="Target entropy. Default: -action_dim = -9.")
    parser.add_argument("--actor_lr",       type=float, default=3e-4)
    parser.add_argument("--critic_lr",      type=float, default=3e-4)
    parser.add_argument("--alpha_lr",       type=float, default=3e-4)
    parser.add_argument("--hidden_dim",     type=int,   default=512)

    # Training loop
    parser.add_argument("--batch_size",         type=int, default=64,
                        help="Total batch size; split by offline_ratio between sources.")
    parser.add_argument("--buffer_capacity",    type=int, default=10_000)
    parser.add_argument("--start_training",     type=int, default=500,
                        help="Online transitions before training starts.")
    parser.add_argument("--updates_per_step",   type=int, default=1,
                        help="Gradient steps per environment step.")
    parser.add_argument("--actor_update_every", type=int, default=2,
                        help="Update actor every N critic steps.")
    parser.add_argument("--normalize_q", action="store_true",
                        help="Normalise Q values (zero-mean, unit-std) before the actor loss. "
                             "Stabilises actor gradients when Q-value scale drifts.")
    parser.add_argument("--max_steps",          type=int, default=200_000)

    # Environment
    parser.add_argument("--max_episode_steps", type=int,   default=300)
    parser.add_argument("--control_freq_hz",   type=float, default=10.0)
    parser.add_argument("--random_reset",      action="store_true")

    # Logging
    parser.add_argument("--output_dir", type=str, default="outputs/rlpd")
    parser.add_argument("--save_every", type=int, default=2_000)
    parser.add_argument("--log_every",  type=int, default=100)

    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    device         = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    target_entropy = args.target_entropy if args.target_entropy is not None else float(-ACTION_DIM)

    # ---- Log all config ----
    logger.info("[RLPD] =============================")
    logger.info("[RLPD] Configuration")
    logger.info("[RLPD] ======================")
    col_w = max(len(k) for k in vars(args)) + 2
    for k, v in sorted(vars(args).items()):
        logger.info(f"[RLPD]   {k:<{col_w}}{v}")
    logger.info(f"[RLPD]   {'device':<{col_w}}{device}")
    logger.info(f"[RLPD]   {'target_entropy':<{col_w}}{target_entropy}")
    logger.info("[RLPD] =============================")

    # ---- Actor ----
    policy = RobotWithTaskPolicy(
        robot_state_dim=ROBOT_STATE_DIM,
        action_dim=ACTION_DIM,
        obs_horizon=1,
        resnet_weights=args.resnet_weights,
        local_ckpt_path=args.local_ckpt_path,
    ).to(device)

    if args.checkpoint and not args.resume:
        ckpt = torch.load(args.checkpoint, map_location=device)
        policy.load_state_dict(ckpt.get("policy_state", ckpt), strict=False)
        logger.info(f"[RLPD] Loaded actor checkpoint: {args.checkpoint}")

    policy.train()

    # ---- Critic + target critic (separate ImageConditioner instances) ----
    def _make_critic() -> ImageConditionedQFunction:
        return ImageConditionedQFunction(
            robot_state_dim=ROBOT_STATE_DIM,
            action_dim=ACTION_DIM,
            hidden_dim=args.hidden_dim,
            resnet_weights=args.resnet_weights,
            local_ckpt_path=args.local_ckpt_path,
        ).to(device)

    critic        = _make_critic()
    target_critic = _make_critic()
    target_critic.load_state_dict(critic.state_dict())
    target_critic.eval()
    for p in target_critic.parameters():
        p.requires_grad_(False)

    # ---- Learnable temperature ----
    log_alpha = torch.tensor(
        np.log(args.init_alpha), dtype=torch.float32, device=device, requires_grad=True
    )

    # ---- Optimisers ----
    actor_opt  = torch.optim.AdamW(policy.parameters(),  lr=args.actor_lr)
    critic_opt = torch.optim.AdamW(critic.parameters(),  lr=args.critic_lr)
    alpha_opt  = torch.optim.Adam([log_alpha],            lr=args.alpha_lr)

    # ---- Offline dataset (placeholder — disabled) ----
    # TODO: instantiate OfflineDataset and enable _merge() once demos are ready.
    logger.info("[RLPD] Offline mixing disabled — running as online SAC.")

    # ---- Online buffer + image store ----
    image_store   = ImageStore(capacity=args.buffer_capacity + 10)
    replay_buffer = ReplayBuffer(capacity=args.buffer_capacity, image_store=image_store)

    # ---- Environment ----
    env = AICRLEnv(
        config=AICGymEnvConfig(
            control_mode="cartesian",
            control_freq_hz=args.control_freq_hz,
            max_episode_steps=args.max_episode_steps,
            random_reset=args.random_reset,
            frame_id="base_link",
        ),
        keypoint_checkpoint=args.keypoint_checkpoint or None,
    )
    logger.info("[RLPD] Environment ready.")

    # ---- Metrics ----
    episode_rewards: list[float] = []
    episode_lengths: list[int]   = []
    critic_losses:   list[float] = []
    actor_losses:    list[float] = []
    alpha_log:       list[float] = []
    q1_log:          list[float] = []
    q2_log:          list[float] = []
    recent_rewards: deque[float] = deque(maxlen=100)
    step_reward_log: list[float] = []
    total_steps  = 0
    update_count = 0

    # ---- Resume ----
    if args.resume:
        rl_ckpt = torch.load(args.resume, map_location=device)
        policy.load_state_dict(rl_ckpt["policy_state"])
        critic.load_state_dict(rl_ckpt["critic_state"])
        target_critic.load_state_dict(rl_ckpt["critic_state"])
        actor_opt.load_state_dict(rl_ckpt["actor_opt"])
        critic_opt.load_state_dict(rl_ckpt["critic_opt"])
        alpha_opt.load_state_dict(rl_ckpt["alpha_opt"])
        log_alpha.data.copy_(rl_ckpt["log_alpha"])
        total_steps  = rl_ckpt.get("step", 0)
        update_count = rl_ckpt.get("update_count", 0)
        logger.info(f"[RLPD] Resumed: step={total_steps}  updates={update_count}")

    # ---- Initial reset ----
    flat_obs, info = env.reset()
    _task_info = info.get("task", {})
    tm_name = _task_info.get("target_module_name", "")
    pn_name = _task_info.get("port_name", "")
    tm_idx, pn_idx = encode_task(_task_info)
    while True:
        imgs = env.get_images()
        if imgs is not None:
            imgs = env._spot_target(imgs, tm_name, pn_name)
            break
        time.sleep(0.05)

    #_save_spot_debug(imgs, tm_name, pn_name,
    #                 output_dir / "spot_debug" / f"step{total_steps:07d}_{tm_name}_{pn_name}.png")

    cam_keys    = list(imgs.keys())
    img_idx     = image_store.push(imgs)
    prev_action = np.concatenate([flat_obs[0:3], _quat_xyzw_to_rot6d(flat_obs[3:7])]).astype(np.float32)

    episode_reward = 0.0
    episode_len    = 0
    last_log_time  = time.monotonic()

    logger.info(f"[RLPD] start_training={args.start_training}  "
          f"offline_ratio={args.offline_ratio}  max_steps={args.max_steps}")

    # ====================================================================
    # Main loop
    # ====================================================================
    for _ in range(args.max_steps - total_steps):

        # ----------------------------------------------------------------
        # Select action
        # ----------------------------------------------------------------
        if total_steps < args.start_training:
            # Random warm-up: small noise around current TCP pose
            cur_rot6d = _quat_xyzw_to_rot6d(flat_obs[3:7])
            action_9d = np.concatenate([flat_obs[0:3], cur_rot6d]).astype(np.float32)
            action_9d[:3] += np.random.randn(3) * 0.01
        else:
            rs_t  = torch.from_numpy(build_robot_state(flat_obs)).unsqueeze(0).to(device)
            pa_t  = torch.from_numpy(prev_action).unsqueeze(0).to(device)
            tm_t  = torch.tensor([tm_idx], dtype=torch.long, device=device)
            pn_t  = torch.tensor([pn_idx], dtype=torch.long, device=device)
            # ImageEncoder expects (B, T, C, H, W); add T=1 dim
            imgs_t = {k: image_store.get(img_idx)[k].unsqueeze(0).unsqueeze(0).to(device)
                      for k in cam_keys}
            with torch.no_grad():
                means, log_stds = policy(imgs_t, rs_t, pa_t, tm_t, pn_t)
                action_9d = _tanh_normal(means, log_stds).rsample()[0].cpu().numpy()

        # ----------------------------------------------------------------
        # Step environment
        # ----------------------------------------------------------------
        obs_snap = {
            "image_idx":     img_idx,
            "robot_state":   build_robot_state(flat_obs),
            "prev_action":   prev_action.copy(),
            "target_module": tm_idx,
            "port_name":     pn_idx,
        }

        flat_obs_new, reward, lhr, terminated, truncated, _ = env.step_pose(action_9d)

        new_imgs = env.get_images()
        if new_imgs is not None:
            new_imgs = env._spot_target(new_imgs, tm_name, pn_name)
        new_img_idx = image_store.push(new_imgs) if new_imgs is not None else img_idx

        next_obs_snap = {
            "image_idx":     new_img_idx,
            "robot_state":   build_robot_state(flat_obs_new),
            "prev_action":   action_9d.copy(),   # action_9d becomes next step's prev_action
            "target_module": tm_idx,
            "port_name":     pn_idx,
        }

        replay_buffer.push(obs_snap, action_9d, reward, next_obs_snap, terminated, truncated)
        replay_buffer.update_recent_rewards(lhr)

        flat_obs    = flat_obs_new
        img_idx     = new_img_idx
        prev_action = action_9d.copy()

        episode_reward += reward
        recent_rewards.append(reward)
        episode_len    += 1
        total_steps    += 1
        done = terminated or truncated

        # ----------------------------------------------------------------
        # Episode reset
        # ----------------------------------------------------------------
        if done:
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_len)
            logger.info(
                f"[RLPD] Episode  len={episode_len}  return={episode_reward:.3f}  "
                f"buffer={len(replay_buffer)}  α={log_alpha.exp().item():.4f}"
            )
            episode_reward  = 0.0
            episode_len     = 0
            replay_buffer.new_trajectory()
            flat_obs, info  = env.reset()
            _task_info      = info.get("task", {})
            tm_name         = _task_info.get("target_module_name", "")
            pn_name         = _task_info.get("port_name", "")
            tm_idx, pn_idx  = encode_task(_task_info)
            prev_action     = np.concatenate([flat_obs[0:3], _quat_xyzw_to_rot6d(flat_obs[3:7])]).astype(np.float32)
            while True:
                imgs = env.get_images()
                if imgs is not None:
                    imgs = env._spot_target(imgs, tm_name, pn_name)
                    break
                time.sleep(0.05)
            #_save_spot_debug(imgs, tm_name, pn_name,
            #                 output_dir / "spot_debug" / f"step{total_steps:07d}_{tm_name}_{pn_name}.png")
            img_idx = image_store.push(imgs)

        # ----------------------------------------------------------------
        # Gradient updates
        # ----------------------------------------------------------------
        if total_steps >= args.start_training and len(replay_buffer) >= args.batch_size:
            for _ in range(args.updates_per_step):

                # Online-only batch (TODO: mix with offline_ds once enabled)
                raw   = replay_buffer.sample_batch(args.batch_size)
                batch = {k: (v.to(device) if isinstance(v, torch.Tensor)
                             else {ck: cv.to(device) for ck, cv in v.items()})
                         for k, v in raw.items()}

                # Critic step
                c_loss, q1_mean, q2_mean = update_critic(
                    critic, target_critic, policy, log_alpha, critic_opt,
                    batch, device, gamma=args.gamma, tau=args.tau,
                )
                critic_losses.append(c_loss)
                q1_log.append(q1_mean)
                q2_log.append(q2_mean)
                update_count += 1

                # Actor + temperature step
                if update_count % args.actor_update_every == 0:
                    a_loss, mean_lp = update_actor(
                        policy, critic, log_alpha, actor_opt, batch, device,
                        normalize_q=args.normalize_q,
                    )
                    actor_losses.append(a_loss)
                    update_temperature(log_alpha, alpha_opt, mean_lp, target_entropy)
                    alpha_log.append(log_alpha.exp().item())

        # ----------------------------------------------------------------
        # Logging
        # ----------------------------------------------------------------
        if total_steps % args.log_every == 0:
            now     = time.monotonic()
            elapsed = now - last_log_time
            last_log_time = now
            r_step = float(np.mean(recent_rewards)) if recent_rewards else float("nan")
            step_reward_log.append(r_step)
            logger.info(
                f"[RLPD] step={total_steps:6d}  "
                f"critic={np.mean(critic_losses[-50:]) if critic_losses else float('nan'):.4f}  "
                f"actor={np.mean(actor_losses[-50:]) if actor_losses else float('nan'):.4f}  "
                f"α={log_alpha.exp().item():.4f}  "
                f"q1={np.mean(q1_log[-50:]) if q1_log else float('nan'):.3f}  "
                f"q2={np.mean(q2_log[-50:]) if q2_log else float('nan'):.3f}  "
                f"ep_ret(10)={np.mean(episode_rewards[-10:]) if episode_rewards else float('nan'):.3f}  "
                f"step_r={r_step:.4f}  "
                f"buffer={len(replay_buffer)}  "
                f"steps/s={args.log_every / elapsed if elapsed > 0 else float('nan'):.2f}"
            )

        # ----------------------------------------------------------------
        # Checkpoint
        # ----------------------------------------------------------------
        if total_steps % args.save_every == 0:
            ckpt_path = output_dir / f"checkpoint_step{total_steps:07d}.pt"
            torch.save({
                "step":         total_steps,
                "update_count": update_count,
                "args":         vars(args),
                "policy_state": policy.state_dict(),
                "critic_state": critic.state_dict(),
                "actor_opt":    actor_opt.state_dict(),
                "critic_opt":   critic_opt.state_dict(),
                "alpha_opt":    alpha_opt.state_dict(),
                "log_alpha":    log_alpha.detach().cpu(),
            }, ckpt_path)
            logger.info(f"[RLPD] Saved: {ckpt_path}")

    # ---- Final checkpoint ----
    torch.save({
        "step":         total_steps,
        "args":         vars(args),
        "policy_state": policy.state_dict(),
        "critic_state": critic.state_dict(),
        "log_alpha":    log_alpha.detach().cpu(),
    }, output_dir / "checkpoint_final.pt")

    # ---- Metrics ----
    np.savez(
        output_dir / "metrics.npz",
        critic_losses   = np.array(critic_losses,   dtype=np.float32),
        actor_losses    = np.array(actor_losses,     dtype=np.float32),
        episode_rewards = np.array(episode_rewards,  dtype=np.float32),
        episode_lengths = np.array(episode_lengths,  dtype=np.float32),
        step_reward_log = np.array(step_reward_log,  dtype=np.float32),
        alpha_log       = np.array(alpha_log,         dtype=np.float32),
    )

    # ---- Plot ----
    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 3, figsize=(18, 8))
        fig.suptitle("RLPD Training Curves", fontsize=14)

        axes[0, 0].plot(critic_losses, alpha=0.7, linewidth=0.8)
        axes[0, 0].set_title("Critic loss (per update)"); axes[0, 0].set_xlabel("Update step")

        axes[0, 1].plot(actor_losses, alpha=0.7, linewidth=0.8, color="orange")
        axes[0, 1].set_title("Actor loss (per actor update)"); axes[0, 1].set_xlabel("Actor update")

        axes[0, 2].plot(alpha_log, alpha=0.7, linewidth=0.8, color="purple")
        axes[0, 2].set_title("Temperature α"); axes[0, 2].set_xlabel("Actor update")

        axes[1, 0].plot(episode_rewards, marker=".", markersize=3, linewidth=0.8, color="green")
        axes[1, 0].set_title("Episode return"); axes[1, 0].set_xlabel("Episode")

        log_steps = np.arange(len(step_reward_log)) * args.log_every
        axes[1, 1].plot(log_steps, step_reward_log, linewidth=0.8, color="red")
        axes[1, 1].set_title("Step reward (100-step avg)"); axes[1, 1].set_xlabel("Env step")

        axes[1, 2].plot(episode_lengths, marker=".", markersize=3, linewidth=0.8, color="blue")
        axes[1, 2].set_title("Episode length"); axes[1, 2].set_xlabel("Episode")

        for ax in axes.flat:
            ax.grid(True, alpha=0.3)
        plt.tight_layout()
        fig.savefig(output_dir / "training_curves.png", dpi=150)
        plt.close(fig)
        logger.info(f"[RLPD] Curves saved to {output_dir / 'training_curves.png'}")
    except ImportError:
        pass

    env.close()
    logger.info("[RLPD] Done.")


if __name__ == "__main__":
    main()
