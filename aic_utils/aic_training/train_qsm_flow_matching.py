#!/usr/bin/env python3
"""
Online RL training: Q-Score Matching with FlowMatchingPolicy actor.

The actor (FlowMatchingPolicy) generates absolute TCP-pose action sequences
from visual + robot-state observations.  A Q-function critic estimates the
expected return for each (state, action-sequence) pair.

Actor update — Q-Score Matching:
  The standard rectified-flow velocity target u_t = (x_1 - x_noise) is augmented
  with the gradient of the Q-function at the buffered action x_1:
      u_guided = α * ∇_{x_t} Q(s, x_t)
  The actor is trained to match this Q-guided velocity field, pushing the
  generative model toward regions of higher expected return.

Critic update — Monte-Carlo regression:
  Q(s, a) ← G_t  (discounted return computed from episode data)
  Optionally replaced with TD(0) once the buffer is large.

Reward:
  Placeholder (0.0 / 1.0 on insertion) until env_resetter.py is extended.

Prerequisites (inside distrobox):
    source /ws_aic/install/setup.bash
    python3 env_resetter.py --ros-args -p use_sim_time:=true \\
        -p delete_task_board:=false -p config_path:=/path/to/config.yaml

Usage:
    pixi run python3 aic_training/train_qsm_flow_matching.py \\
        --checkpoint /path/to/pretrained_flow.pt \\
        --output_dir outputs/qsm_rl
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
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

# ---------------------------------------------------------------------------
# Path setup — make aic_training importable
# ---------------------------------------------------------------------------

_AIC_UTILS = Path(__file__).resolve().parent.parent
if str(_AIC_UTILS) not in sys.path:
    sys.path.insert(0, str(_AIC_UTILS))

import rclpy  # noqa: F401 — triggers rclpy context init when imported

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from aic_control_interfaces.msg import (
    MotionUpdate,
    TrajectoryGenerationMode,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from aic_training.aic_gym_env import AICGymEnv, AICGymEnvConfig
from aic_training.flow_matching import FlowMatchingPolicy, ROBOT_STATE_DIM

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Camera topics (must match launch file / aic_robot.py)
CAMERA_TOPICS = {
    "left_camera":   "/left_camera/image",
    "center_camera": "/center_camera/image",
    "right_camera":  "/right_camera/image",
}

# Image size expected by the policy encoder (must match training)
IMG_H = 256
IMG_W = 288

# Task encoding (must match aic_robot_aic_controller.py)
# the TaskEncoder in aic_utils/aic_training/flow_matching.py might also be relevant
TASK_TARGET_MODULE_ENCODING: dict[str, int] = {
    "nic_card_mount_0": 0, "nic_card_mount_1": 1, "nic_card_mount_2": 2,
    "nic_card_mount_3": 3, "nic_card_mount_4": 4,
    "sc_port_0": 5,        "sc_port_1": 6,
}
TASK_PORT_NAME_ENCODING: dict[str, int] = {
    "sfp_port_0": 0, "sfp_port_1": 1, "sc_port_base": 2,
}
NUM_TASKS = 12  # must match FlowMatchingPolicy / TaskEncoder


# ---------------------------------------------------------------------------
# 6D rotation → quaternion  (mirrors RunFlowMatching.py)
# Training: quat(x,y,z,w) → R → rot6d = R[:2, :].  Invert at step time.
# ---------------------------------------------------------------------------

def _rot6d_to_quat_xyzw(rot6d: np.ndarray) -> np.ndarray:
    """Convert 6D rotation (first two rows of R, flattened) to quaternion (x,y,z,w)."""
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
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=np.float32)
    return q / (np.linalg.norm(q) + 1e-8)


def _action9_to_pose7(action9: np.ndarray) -> np.ndarray:
    """Convert (9,) action [pos(3) | rot6d(6)] → (7,) [pos(3) | quat(x,y,z,w)]."""
    return np.concatenate([action9[:3], _rot6d_to_quat_xyzw(action9[3:9])]).astype(np.float32)


def _quat_xyzw_to_rot6d(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion (x,y,z,w) → 6D rotation (first two rows of R, flattened)."""
    x, y, z, w = quat.astype(np.float64)
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),      1 - 2*(x*x + z*z),  2*(y*z - w*x)],
    ], dtype=np.float32)
    return R.flatten()  # (6,)


# ---------------------------------------------------------------------------
# AICRLEnv — extends AICGymEnv with camera observations + pose action mode
# ---------------------------------------------------------------------------

class AICRLEnv(AICGymEnv):
    """Gymnasium environment extended for RL training with FlowMatchingPolicy.

    Adds:
      1. Subscriptions to the three camera topics — access via ``get_images()``.
      2. ``send_pose_action(pos, quat)`` — sends an absolute TCP-pose target
         using TrajectoryGenerationMode.MODE_POSITION.
      3. ``step_pose(action_9d)`` — gym-compatible step for 9-D pose actions (pos+rot6d).
    """

    def __init__(self, config: AICGymEnvConfig | None = None):
        super().__init__(config)
        self._img_lock = Lock()
        self._latest_images: dict[str, ROSImage | None] = {k: None for k in CAMERA_TOPICS}

        # TF buffer for ground-truth port positions (same pattern as CheatCode.py)
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        # Raw insertion event string — base class only stores 0.0/1.0, we need the content
        self._last_insertion_event_data: str = ""

        # Timestamp (time.monotonic) when force first exceeded 20 N; None if below threshold
        self._high_force_since: float | None = None

        # ref code: aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_robot.py
        for cam_key, topic in CAMERA_TOPICS.items():
            # Capture cam_key in closure
            def _make_cb(key: str):
                def _cb(msg: ROSImage) -> None:
                    with self._img_lock:
                        self._latest_images[key] = msg
                return _cb
            self._node.create_subscription(ROSImage, topic, _make_cb(cam_key), 10)

    # ------------------------------------------------------------------
    # Camera helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _ros_image_to_tensor(msg: ROSImage) -> torch.Tensor:
        """Convert a ROS2 Image to a (C, H, W) float32 tensor in [0, 1]."""
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ("bgr8", "bgra8"):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
        return torch.from_numpy(img).permute(2, 0, 1).float().div(255.0)

    def get_images(self) -> dict[str, torch.Tensor] | None:
        """Return a dict of (C, H, W) image tensors, or None if cameras not ready."""
        with self._img_lock:
            if any(v is None for v in self._latest_images.values()):
                return None
            return {k: self._ros_image_to_tensor(v) for k, v in self._latest_images.items()}

    def reset(self, **kwargs):
        self._last_insertion_event_data = ""
        self._high_force_since = None
        return super().reset(**kwargs)

    # ------------------------------------------------------------------
    # Insertion event — keep raw string for task-match checking
    # ------------------------------------------------------------------

    def _insertion_event_cb(self, msg: String) -> None:
        self._last_insertion_event_data = msg.data
        self._last_insertion_event = 1.0 if msg.data else 0.0

    # ------------------------------------------------------------------
    # Reward
    # ------------------------------------------------------------------

    def _compute_reward(self, obs: np.ndarray, action: np.ndarray, info: dict) -> float:
        reward = 0.0

        # 1. Step penalty
        reward -= 0.02

        # 2. Distance to target port (ground-truth TF)
        task = info.get("task", {})
        target_module_name = task.get("target_module_name", "")
        port_name          = task.get("port_name", "")
        if target_module_name and port_name:
            port_frame = f"task_board/{target_module_name}/{port_name}_link"
            try:
                tf_stamped = self._tf_buffer.lookup_transform(
                    "base_link", port_frame, Time()
                )
                port_pos = tf_stamped.transform.translation
                tcp_pos  = obs[0:2]                               # x, y only
                dist = float(np.linalg.norm(
                    tcp_pos - np.array([port_pos.x, port_pos.y])
                ))
                if dist > 0.3:
                    reward -= 1.0
                else:
                    t = dist / 0.3                # 0 (close) → 1 (at boundary)
                    reward += 2.0 * (1.0 - t) + 0.05 * t
            except TransformException:
                pass  # TF not yet available — skip distance reward this step
        #print("LXH Debug reward:", info)

        # 3. Insertion event
        event_data = self._last_insertion_event_data
        if event_data:
            if target_module_name and port_name \
                    and target_module_name in event_data and port_name in event_data:
                reward += 3.0   # correct insertion
            else:
                reward -= 3.0   # wrong port

        # 4. Force penalties  (tare-corrected force at obs[20:23])
        force = obs[20:23]
        if np.any(np.abs(force) > 20.0):
            if self._high_force_since is None:
                self._high_force_since = time.monotonic()
            elif time.monotonic() - self._high_force_since >= 1.0:
                reward -= 2.5
        else:
            self._high_force_since = None
            if np.any(np.abs(force) > 15.0):
                reward -= 1.0

        return reward

    # ------------------------------------------------------------------
    # Pose action mode
    # ------------------------------------------------------------------

    def send_pose_action(self, pos: np.ndarray, quat: np.ndarray) -> None:
        """Send an absolute TCP-pose target (MODE_POSITION).
           No move_robot callback is needed here — AICGymEnv publishes 
           directly to the controller topic, which is the correct equivalent in the gym env context.  
        Args:
            pos:  (3,) xyz position in gripper/tcp frame
            quat: (4,) quaternion (x, y, z, w)
        """
        msg = MotionUpdate()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self.config.frame_id
        msg.pose = Pose(
            position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
            orientation=Quaternion(
                x=float(quat[0]), y=float(quat[1]),
                z=float(quat[2]), w=float(quat[3]),
            ),
        )
        msg.target_stiffness = np.diag(self.config.cartesian_stiffness).flatten()
        msg.target_damping   = np.diag(self.config.cartesian_damping).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
        self._motion_update_pub.publish(msg)

    def step_pose(self, action_9d: np.ndarray):
        """Execute one absolute-pose control step.

        Args:
            action_9d: (9,) array — [x, y, z, rot6d(6)]

        Returns:
            Same (obs, reward, terminated, truncated, info) as step().
        """
        self._step_count += 1
        pose7 = _action9_to_pose7(action_9d)
        self.send_pose_action(pose7[:3], pose7[3:7])
        time.sleep(self._step_period)

        obs        = self._get_observation()
        info       = self._get_info()
        reward     = self._compute_reward(obs, action_9d, info)
        terminated = self._check_terminated(obs, info)
        truncated  = self._check_truncated(obs, info)
        return obs, reward, terminated, truncated, info


# ---------------------------------------------------------------------------
# QFunction — state-only critic  Q(robot_state, task, action_seq) → scalar
# ---------------------------------------------------------------------------

class QFunction(nn.Module):
    """Double-Q critic: two independent MLPs, forward() returns (q1, q2).

    Input dimensions:
      robot_state:  (B, obs_horizon, ROBOT_STATE_DIM)
      target_module:(B,) int64
      port_name:    (B,) int64
      actions:      (B, pred_horizon, action_dim)

    Output:
      q1, q2: each (B,) — take min(q1, q2) for actor updates to reduce overestimation.

    Design notes:
      - LayerNorm on the concatenated input before the first linear layer stabilises
        training when input scales vary (positions in metres, angles in radians, etc.)
      - Two completely independent heads share no weights, so their errors are
        uncorrelated — the min prevents the actor from exploiting overestimated Q.
      - No activation on the output layer; Q-values are unbounded scalars.
      - the robot state + task embedding likely carries most of the value signal 
        (insertion success correlates strongly with TCP proximity to port). A state-only critic is a        
        reasonable starting point and is what the current QFunction in train_qsm_flow_matching.py uses.  
    """

    def __init__(
        self,
        obs_horizon:     int = 2,
        action_dim:      int = 9,
        robot_state_dim: int = ROBOT_STATE_DIM,
        hidden_dim:      int = 256,
        task_dim:        int = 64,
    ):
        super().__init__()
        self.obs_horizon = obs_horizon
        self.action_dim  = action_dim

        in_dim = obs_horizon * robot_state_dim + action_dim + task_dim

        self.task_embedding = nn.Embedding(NUM_TASKS, task_dim)

        def _make_mlp() -> nn.Sequential:
            return nn.Sequential(
                nn.LayerNorm(in_dim),
                nn.Linear(in_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, 1),
            )

        self.q1 = _make_mlp()
        self.q2 = _make_mlp()

    def forward(
        self,
        robot_state:   torch.Tensor,   # (B, obs_horizon, robot_state_dim)
        target_module: torch.Tensor,   # (B,) int64
        port_name:     torch.Tensor,   # (B,) int64
        action:        torch.Tensor,   # (B, action_dim)
    ) -> tuple[torch.Tensor, torch.Tensor]:   # (B,), (B,)
        task_idx = target_module.clamp(0, NUM_TASKS - 1)
        t = self.task_embedding(task_idx)                      # (B, task_dim)
        x = torch.cat([robot_state.flatten(1), action, t], dim=-1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)


# ---------------------------------------------------------------------------
# ImageStore — shared circular buffer for raw image frames
# ---------------------------------------------------------------------------

class ImageStore:
    """Circular buffer of single-frame image dicts {cam_key: (C, H, W) float32}.

    Frames are stored once and referenced by index from the ReplayBuffer.
    Consecutive transitions share obs_horizon-1 frames (step t's next_obs
    equals step t+1's obs), so this reduces image memory from
    2 * obs_horizon * capacity frames down to ~capacity + obs_horizon unique frames.

    Capacity should be at least replay_buffer_capacity + obs_horizon to ensure
    all frames referenced by live transitions are still present.
    """

    def __init__(self, capacity: int):
        self._capacity = capacity
        self._frames: list[dict | None] = [None] * capacity
        self._write_idx = 0

    def push(self, frame: dict[str, torch.Tensor]) -> int:
        """Store one frame dict and return its index."""
        idx = self._write_idx
        self._frames[idx] = {k: v.clone() for k, v in frame.items()}
        self._write_idx = (self._write_idx + 1) % self._capacity
        return idx

    def get(self, idx: int) -> dict[str, torch.Tensor]:
        """Retrieve a previously stored frame by index."""
        return self._frames[idx]


# ---------------------------------------------------------------------------
# ReplayBuffer
# ---------------------------------------------------------------------------

class ReplayBuffer:
    """Circular replay buffer storing one transition per environment step.

    Images are NOT copied into the buffer — each observation stores a list of
    obs_horizon integer indices into a shared ImageStore.  Full image tensors
    are reconstructed only at sample time.

    Each stored observation is a dict with:
      "image_indices": list[int] of length obs_horizon — indices into ImageStore
      "robot_state"  : (obs_horizon, ROBOT_STATE_DIM) float32 array
      "target_module": int
      "port_name"    : int

    "action" is the single (action_dim,) vector executed at that step.
    "reward" is the immediate per-step reward.
    "done"   is True when the episode ended after this step.
    """

    def __init__(self, capacity: int, image_store: ImageStore):
        self._buf: deque[dict] = deque(maxlen=capacity)
        self._image_store = image_store

    def __len__(self) -> int:
        return len(self._buf)

    def push(
        self,
        obs:      dict,
        action:   np.ndarray,   # (action_dim,)
        reward:   float,
        next_obs: dict,
        done:     bool,
    ) -> None:
        self._buf.append({
            "obs":      obs,
            "action":   action.astype(np.float32),
            "reward":   float(reward),
            "next_obs": next_obs,
            "done":     bool(done),
        })

    def sample(self, batch_size: int) -> dict:
        items = random.sample(self._buf, min(batch_size, len(self._buf)))
        store = self._image_store

        # Reconstruct stacked images from indices: {cam_key: (B, obs_horizon, C, H, W)}
        cam_keys = list(store.get(items[0]["obs"]["image_indices"][0]).keys())

        def _stack(image_indices: list[int]) -> dict[str, torch.Tensor]:
            return {k: torch.stack([store.get(i)[k] for i in image_indices]) for k in cam_keys}

        images = {
            k: torch.stack([_stack(it["obs"]["image_indices"])[k] for it in items])
            for k in cam_keys
        }
        robot_state = torch.stack([
            torch.as_tensor(it["obs"]["robot_state"]) for it in items
        ])
        target_module = torch.tensor(
            [it["obs"]["target_module"] for it in items], dtype=torch.long
        )
        port_name = torch.tensor(
            [it["obs"]["port_name"] for it in items], dtype=torch.long
        )
        actions = torch.tensor(
            np.stack([it["action"] for it in items]), dtype=torch.float32
        )  # (B, action_dim)
        rewards = torch.tensor(
            [it["reward"] for it in items], dtype=torch.float32
        )
        dones = torch.tensor(
            [it["done"] for it in items], dtype=torch.float32
        )
        next_images = {
            k: torch.stack([_stack(it["next_obs"]["image_indices"])[k] for it in items])
            for k in cam_keys
        }
        next_robot_state = torch.stack([
            torch.as_tensor(it["next_obs"]["robot_state"]) for it in items
        ])
        return {
            "images":           images,
            "robot_state":      robot_state,
            "target_module":    target_module,
            "port_name":        port_name,
            "actions":          actions,
            "rewards":          rewards,
            "next_images":      next_images,
            "next_robot_state": next_robot_state,
            "dones":            dones,
        }


# ---------------------------------------------------------------------------
# Actor / critic update functions
# ---------------------------------------------------------------------------

@torch.no_grad()
def soft_update(target: nn.Module, source: nn.Module, tau: float) -> None:
    """Polyak-average source parameters into target: θ_tgt ← τ·θ + (1-τ)·θ_tgt."""
    for tp, sp in zip(target.parameters(), source.parameters()):
        tp.data.copy_(tau * sp.data + (1.0 - tau) * tp.data)


def update_critic(
    q_func:               QFunction,
    target_q_func:        QFunction,
    policy:               "FlowMatchingPolicy",
    opt:                  torch.optim.Optimizer,
    batch:                dict,
    device:               torch.device,
    gamma:                float = 0.99,
    tau:                  float = 0.005,
    target_num_flow_steps: int  = 5,
) -> float:
    """TD critic update with target networks.

    TD target = r + γ · (1 − done) · min(Q1_tgt, Q2_tgt)(s', a')
    where a' ~ π(s') is sampled from the current policy with a fast ODE solve.
    Both critics are updated with MSE vs the shared (stop-gradient) TD target,
    then the target networks are soft-updated via Polyak averaging.

    Args:
      target_num_flow_steps - typically, we do 10 steps for flow matching generation, here we use 5 to save time
    """
    robot_state   = batch["robot_state"].to(device)
    target_module = batch["target_module"].to(device)
    port_name     = batch["port_name"].to(device)
    actions       = batch["actions"].to(device)          # (B, action_dim)
    rewards       = batch["rewards"].to(device)          # (B,)
    dones         = batch["dones"].to(device)            # (B,)
    next_images      = {k: v.to(device) for k, v in batch["next_images"].items()}
    next_robot_state = batch["next_robot_state"].to(device)

    # --- Sample next action a' ~ π(s') using target-quality fast ODE ---
    with torch.no_grad():
        next_actions_raw = policy.sample(
            images=next_images,
            robot_state=next_robot_state,
            target_module=target_module,
            port_name=port_name,
            pred_horizon=1,
            num_steps=target_num_flow_steps,
            guidance_scale=1.0,
            solver="euler",
        )                                                # (B, 1, action_dim)
        a_next = next_actions_raw[:, 0, :]              # (B, action_dim)

        # --- TD target using frozen target critics ---
        q1_tgt, q2_tgt = target_q_func(next_robot_state, target_module, port_name, a_next)
        next_v    = torch.min(q1_tgt, q2_tgt)           # (B,)
        td_target = rewards + gamma * (1.0 - dones) * next_v   # (B,)

    # --- Online critic loss ---
    q1, q2 = q_func(robot_state, target_module, port_name, actions)
    loss   = F.mse_loss(q1, td_target) + F.mse_loss(q2, td_target)

    opt.zero_grad()
    loss.backward()
    torch.nn.utils.clip_grad_norm_(q_func.parameters(), 1.0)
    opt.step()

    # --- Polyak update of target networks ---
    soft_update(target_q_func, q_func, tau)

    return loss.item()


def update_actor(
    policy:       FlowMatchingPolicy,
    q_func:       QFunction,
    opt:          torch.optim.Optimizer,
    batch:        dict,
    device:       torch.device,
    q_alpha:      float = 0.1,
    pred_horizon: int   = 16,
) -> float:
    """Q-Score Matching actor update.

    The buffer stores single per-step actions (action_dim,).  We expand them
    to a full sequence (pred_horizon, action_dim) for the flow-matching loss,
    then evaluate Q at each noisy interpolant x_t along the trajectory and
    average the gradients — matching the reference (score_matching_learner.py).

    Target velocity is purely the Q-gradient at x_t (mirrors reference):
        u_guided = α * ∇_{x_t} Q(s, x_t)

    Two design choices that mirror the reference:
      - Gradient w.r.t. x_t (noisy point), not x_1 (clean action): guides the
        velocity field at wherever we are on the trajectory, not just the endpoint.
      - Mean of the two critics (not min): min is for critic bootstrapping to
        prevent overestimation; for the actor gradient signal, mean is less
        pessimistic and gives a more balanced direction.

    Args:
        q_alpha:      Scale of the Q-gradient term.  Start small (0.01–0.1).
        pred_horizon: Length of the action sequence the policy generates.
    """
    B = batch["actions"].shape[0]

    images        = {k: v.to(device) for k, v in batch["images"].items()}
    robot_state   = batch["robot_state"].to(device)
    target_module = batch["target_module"].to(device)
    port_name     = batch["port_name"].to(device)
    # Expand single buffered action → sequence used as flow-matching target
    action_single = batch["actions"].to(device)                  # (B, action_dim)
    actions = action_single.unsqueeze(1).expand(-1, pred_horizon, -1).contiguous()
    #                                                              (B, T, action_dim)

    T, D = pred_horizon, actions.shape[-1]

    # --- Rectified-flow interpolation (x_t first — gradient is taken here) ---
    t_flow  = torch.rand(B, device=device)                       # (B,)
    t_bc    = t_flow[:, None, None]
    x_noise = torch.randn_like(actions)                          # (B, T, D)
    x_t     = (1.0 - t_bc) * x_noise + t_bc * actions.detach()

    # --- Q-gradient w.r.t. x_t, mean of two critics, averaged over horizon ---
    x_t_g = x_t.detach().requires_grad_(True)                   # (B, T, D)

    # Flatten B×T → per-step Q evaluations
    rs_flat = robot_state.unsqueeze(1).expand(-1, T, -1, -1).reshape(B * T, *robot_state.shape[1:])
    tm_flat = target_module.unsqueeze(1).expand(-1, T).reshape(B * T)
    pn_flat = port_name.unsqueeze(1).expand(-1, T).reshape(B * T)
    xt_flat = x_t_g.reshape(B * T, D)

    q1, q2  = q_func(rs_flat, tm_flat, pn_flat, xt_flat)        # (B*T,) each
    q_val   = ((q1 + q2) / 2).view(B, T).mean(dim=1)            # (B,) mean critics, avg horizon
    q_grad  = torch.autograd.grad(q_val.sum(), x_t_g)[0]        # (B, T, D)

    # Normalize so gradient scale is ~1 regardless of Q magnitude
    grad_norm = q_grad.norm(dim=[-1, -2], keepdim=True).clamp(min=1e-8)
    q_grad_n  = q_grad / grad_norm                               # unit-norm per sample

    # Target velocity is purely the Q-gradient direction (matches reference)
    u_guided = q_alpha * q_grad_n.detach()

    # --- Policy forward + actor loss ---
    v_pred = policy(
        images=images,
        robot_state=robot_state,
        target_module=target_module,
        port_name=port_name,
        x_t=x_t,
        t=t_flow,
    )
    loss = F.mse_loss(v_pred, u_guided)

    opt.zero_grad()
    loss.backward()
    torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
    opt.step()
    return loss.item()


# ---------------------------------------------------------------------------
# Observation helpers
# ---------------------------------------------------------------------------

def build_robot_state(flat_obs: np.ndarray) -> np.ndarray:
    """Extract ROBOT_STATE_DIM=20 state from the flat gym observation.

    AICGymEnv flat obs layout (26 values):
      [0:7]  tcp_pose (position xyz + quaternion xyzw)
      [7:13] tcp_velocity (linear xyz + angular xyz)
      [13:20] joint_positions (7)
      [20:26] wrench (force xyz + torque xyz)

    Policy robot_state layout (20 values = ROBOT_STATE_DIM):
      tcp_pose(7) + joint_positions(7) + wrench(6)
    """
    tcp_pose      = flat_obs[0:7]
    joint_pos     = flat_obs[13:20]
    wrench        = flat_obs[20:26]
    return np.concatenate([tcp_pose, joint_pos, wrench]).astype(np.float32)


def encode_task(task_info: dict) -> tuple[int, int]:
    """Map task_info dict from env.reset() to (target_module_idx, port_name_idx).

    Returns (-1, -1) if the task is unknown (e.g., env reset not yet called).
    """
    tm = TASK_TARGET_MODULE_ENCODING.get(
        task_info.get("target_module_name", ""), -1
    )
    pn = TASK_PORT_NAME_ENCODING.get(
        task_info.get("port_name", ""), -1
    )

    print("LXH Debug:", task_info)
    print("LXH Debug:", tm, pn)
    return max(tm, 0), max(pn, 0)  # clamp to 0 so embedding doesn't crash


# ---------------------------------------------------------------------------
# Main training loop
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Q-Score Matching RL training")

    # Policy
    parser.add_argument("--checkpoint",    type=str, default="",
                        help="Path to a pretrained FlowMatchingPolicy checkpoint (.pt). "
                             "Leave empty to train from scratch.")
    parser.add_argument("--action_dim",    type=int, default=9,
                        help="Action dimension: 7 for pos+quat, 9 for pos+rot6d.")
    parser.add_argument("--obs_horizon",   type=int, default=2)
    # make this consistent with the imitation training
    parser.add_argument("--pred_horizon",  type=int, default=8)
    parser.add_argument("--exec_steps",    type=int, default=4,
                        help="How many actions from the predicted sequence to execute "
                             "before re-querying the policy.")
    parser.add_argument("--num_flow_steps", type=int, default=10,
                        help="ODE integration steps for policy.sample().")
    parser.add_argument("--guidance_scale", type=float, default=1.0,
                        help="CFG guidance scale (1.0 = disabled).")

    # RL hyperparameters
    parser.add_argument("--gamma",           type=float, default=0.99)
    parser.add_argument("--tau",             type=float, default=0.005,
                        help="Polyak averaging coefficient for target critic EMA.")
    parser.add_argument("--target_num_flow_steps", type=int, default=3,
                        help="ODE steps used when sampling a' for the TD target "
                             "(fewer = faster, 3 Euler steps is usually sufficient).")
    parser.add_argument("--q_alpha",         type=float, default=0.05,
                        help="Q-gradient scale in actor loss.")
    parser.add_argument("--actor_lr",        type=float, default=1e-5)
    parser.add_argument("--critic_lr",       type=float, default=1e-4)
    parser.add_argument("--batch_size",      type=int,   default=32)
    parser.add_argument("--buffer_capacity", type=int,   default=10_000)
    parser.add_argument("--start_training",  type=int,   default=200,
                        help="Number of transitions to collect before training starts.")
    parser.add_argument("--actor_update_every",  type=int, default=4,
                        help="Update actor every N critic steps.")
    parser.add_argument("--updates_per_step", type=int, default=1,
                        help="Gradient steps per environment step.")

    # Environment
    parser.add_argument("--max_episode_steps", type=int,   default=300)
    parser.add_argument("--control_freq_hz",   type=float, default=10.0)
    parser.add_argument("--random_reset",      action="store_true",
                        help="Use random reset (requires env_resetter config_path).")
    parser.add_argument("--max_steps",         type=int,   default=100_000,
                        help="Total environment steps.")

    # Logging / checkpointing
    parser.add_argument("--output_dir", type=str, default="outputs/qsm_rl")
    parser.add_argument("--save_every", type=int, default=1_000,
                        help="Save checkpoint every N environment steps.")
    parser.add_argument("--log_every",  type=int, default=100)

    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[QSM] device={device}")

    # ---- Build policy ----
    policy = FlowMatchingPolicy(
        obs_horizon=args.obs_horizon,
        action_dim=args.action_dim,
        robot_state_dim=ROBOT_STATE_DIM,
        cfg_dropout_prob=0.0,          # no CFG dropout during RL fine-tuning
    ).to(device)

    if args.checkpoint:
        ckpt = torch.load(args.checkpoint, map_location=device)
        state = ckpt.get("model_state", ckpt)
        policy.load_state_dict(state, strict=False)
        print(f"[QSM] Loaded policy checkpoint: {args.checkpoint}")

    policy.train()

    # ---- Build Q-function ----
    q_func = QFunction(
        obs_horizon=args.obs_horizon,
        action_dim=args.action_dim,
        robot_state_dim=ROBOT_STATE_DIM,
    ).to(device)

    # Target critics — same architecture, initialized from q_func, never directly optimized
    target_q_func = QFunction(
        obs_horizon=args.obs_horizon,
        action_dim=args.action_dim,
        robot_state_dim=ROBOT_STATE_DIM,
    ).to(device)
    target_q_func.load_state_dict(q_func.state_dict())
    for p in target_q_func.parameters():
        p.requires_grad_(False)

    # ---- Optimizers ----
    actor_opt  = torch.optim.AdamW(policy.parameters(),  lr=args.actor_lr)
    critic_opt = torch.optim.AdamW(q_func.parameters(), lr=args.critic_lr)

    # ---- Image store + replay buffer ----
    # ImageStore holds one frame per env step; capacity = buffer + obs_horizon guards
    # against the oldest transitions referencing frames that have been overwritten.
    image_store   = ImageStore(capacity=args.buffer_capacity + args.obs_horizon + 10)
    replay_buffer = ReplayBuffer(capacity=args.buffer_capacity, image_store=image_store)

    # ---- Environment ----
    env_cfg = AICGymEnvConfig(
        control_mode="cartesian",
        control_freq_hz=args.control_freq_hz,
        max_episode_steps=args.max_episode_steps,
        random_reset=args.random_reset,
        frame_id="base_link",
    )
    env = AICRLEnv(config=env_cfg)
    print("[QSM] Environment ready.")

    # ---- Metrics ----
    episode_rewards: list[float] = []
    episode_lengths: list[int] = []
    critic_losses:   list[float] = []
    actor_losses:    list[float] = []
    recent_rewards:  deque[float] = deque(maxlen=100)  # per-step rewards, last 100 steps
    step_reward_log: list[float] = []   # mean step reward snapshot at each log interval
    total_steps = 0
    update_count = 0

    # ---- Initial reset ----
    flat_obs, info = env.reset()
    task_info    = info.get("task", {})
    tm_idx, pn_idx = encode_task(task_info)

    obs_buffer   = deque([flat_obs] * args.obs_horizon, maxlen=args.obs_horizon)

    # Seed the image buffer (deque of ImageStore indices, one per timestep slot)
    while True:
        imgs = env.get_images()
        if imgs is not None:
            break
        time.sleep(0.05)
    cam_keys = list(imgs.keys())
    init_idx = image_store.push(imgs)
    img_buffer = deque([init_idx] * args.obs_horizon, maxlen=args.obs_horizon)

    episode_reward = 0.0
    episode_len    = 0

    # ---- Action queue ----
    action_queue: list[np.ndarray] = []    # buffered policy actions to execute

    print(f"[QSM] Starting training.  start_training={args.start_training}  "
          f"max_steps={args.max_steps}")

    for step in range(1, args.max_steps + 1):
        # ----------------------------------------------------------------
        # Query policy when action queue is empty
        # ----------------------------------------------------------------
        if not action_queue:
            if total_steps < args.start_training:
                # Warm-up: current TCP pose (converted to rot6d) + small position noise
                cur_pos   = flat_obs[0:3]
                cur_rot6d = _quat_xyzw_to_rot6d(flat_obs[3:7])
                seq = np.tile(
                    np.concatenate([cur_pos, cur_rot6d]), (args.pred_horizon, 1)
                ).astype(np.float32)
                xy_noise = np.random.randn(args.pred_horizon, 2) * 0.01   # 1 cm on x, y
                z_noise  = np.random.randn(args.pred_horizon, 1) * 0.002  # 2 mm on z
                seq[:, :3] += np.concatenate([xy_noise, z_noise], axis=1)
                action_seq = seq  # (pred_horizon, 9)
            else:
                # Policy inference
                robot_state_t = torch.from_numpy(
                    np.stack([build_robot_state(o) for o in obs_buffer])
                ).unsqueeze(0).to(device)   # (1, obs_horizon, ROBOT_STATE_DIM)

                images_t = {
                    k: torch.stack([image_store.get(i)[k] for i in img_buffer]).unsqueeze(0).to(device)
                    for k in cam_keys
                }  # {cam: (1, obs_horizon, C, H, W)}

                tm_t  = torch.tensor([tm_idx], dtype=torch.long, device=device)
                pn_t  = torch.tensor([pn_idx], dtype=torch.long, device=device)

                with torch.no_grad():
                    actions_out = policy.sample(
                        images=images_t,
                        robot_state=robot_state_t,
                        target_module=tm_t,
                        port_name=pn_t,
                        pred_horizon=args.pred_horizon,
                        num_steps=args.num_flow_steps,
                        guidance_scale=args.guidance_scale,
                        solver="euler",
                    )  # (1, pred_horizon, action_dim)

                action_seq = actions_out[0].cpu().numpy()  # (pred_horizon, action_dim)

            action_queue = [action_seq[i] for i in range(args.exec_steps)]

        # ----------------------------------------------------------------
        # Snapshot obs BEFORE executing (for replay buffer)
        # ----------------------------------------------------------------
        step_obs_snap = {
            "image_indices": list(img_buffer),
            "robot_state":   np.stack([build_robot_state(o) for o in obs_buffer]),
            "target_module": tm_idx,
            "port_name":     pn_idx,
        }

        # ----------------------------------------------------------------
        # Execute one step from the action queue
        # ----------------------------------------------------------------
        action_9d = action_queue.pop(0)
        flat_obs_new, reward, terminated, truncated, info = env.step_pose(action_9d)

        # Update rolling buffers
        obs_buffer.append(flat_obs_new)
        new_imgs = env.get_images()
        if new_imgs is not None:
            img_buffer.append(image_store.push(new_imgs))

        flat_obs = flat_obs_new
        episode_reward += reward
        recent_rewards.append(reward)
        episode_len    += 1
        total_steps    += 1

        done = terminated or truncated

        # Snapshot next state (updated buffers after execution)
        next_obs_snap = {
            "image_indices": list(img_buffer),
            "robot_state":   np.stack([build_robot_state(o) for o in obs_buffer]),
            "target_module": tm_idx,
            "port_name":     pn_idx,
        }

        # ----------------------------------------------------------------
        # Push per-step transition immediately
        # ----------------------------------------------------------------
        replay_buffer.push(
            obs=step_obs_snap,
            action=action_9d,
            reward=reward,
            next_obs=next_obs_snap,
            done=done,
        )

        # ----------------------------------------------------------------
        # Episode end — log and reset
        # ----------------------------------------------------------------
        if done:
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_len)
            print(
                f"[QSM] Episode  steps={episode_len}  "
                f"return={episode_reward:.3f}  "
                f"buffer={len(replay_buffer)}"
            )

            action_queue.clear()
            episode_reward = 0.0
            episode_len    = 0

            flat_obs, info = env.reset()
            task_info = info.get("task", {})
            tm_idx, pn_idx = encode_task(task_info)

            obs_buffer.clear()
            obs_buffer.extend([flat_obs] * args.obs_horizon)

            while True:
                imgs = env.get_images()
                if imgs is not None:
                    break
                time.sleep(0.05)
            init_idx = image_store.push(imgs)
            img_buffer.clear()
            img_buffer.extend([init_idx] * args.obs_horizon)

        # ----------------------------------------------------------------
        # Gradient updates
        # ----------------------------------------------------------------
        if total_steps >= args.start_training and len(replay_buffer) >= args.batch_size:
            for _ in range(args.updates_per_step):
                batch = replay_buffer.sample(args.batch_size)

                # Critic update
                c_loss = update_critic(
                    q_func, target_q_func, policy, critic_opt, batch, device,
                    gamma=args.gamma,
                    tau=args.tau,
                    target_num_flow_steps=args.target_num_flow_steps,
                )
                critic_losses.append(c_loss)
                update_count += 1

                # Actor update (less frequent)
                if update_count % args.actor_update_every == 0:
                    a_loss = update_actor(
                        policy, q_func, actor_opt, batch, device, args.q_alpha,
                        pred_horizon=args.pred_horizon,
                    )
                    actor_losses.append(a_loss)

        # ----------------------------------------------------------------
        # Logging
        # ----------------------------------------------------------------
        if total_steps % args.log_every == 0:
            c_avg = np.mean(critic_losses[-50:]) if critic_losses else float("nan")
            a_avg = np.mean(actor_losses[-50:])  if actor_losses  else float("nan")
            r_avg = np.mean(episode_rewards[-10:]) if episode_rewards else float("nan")
            r_step_avg = np.mean(recent_rewards) if recent_rewards else float("nan")
            step_reward_log.append(r_step_avg)
            print(
                f"[QSM] step={total_steps:6d}  "
                f"critic_loss={c_avg:.4f}  "
                f"actor_loss={a_avg:.4f}  "
                f"ep_return(10)={r_avg:.3f}  "
                f"step_reward(100)={r_step_avg:.4f}  "
                f"buffer={len(replay_buffer)}"
            )

        # ----------------------------------------------------------------
        # Checkpointing
        # ----------------------------------------------------------------
        if total_steps % args.save_every == 0:
            ckpt_path = output_dir / f"checkpoint_step{total_steps:07d}.pt"
            torch.save(
                {
                    "step":         total_steps,
                    "args":         vars(args),
                    "policy_state": policy.state_dict(),
                    "q_func_state": q_func.state_dict(),
                    "actor_opt":    actor_opt.state_dict(),
                    "critic_opt":   critic_opt.state_dict(),
                },
                ckpt_path,
            )
            print(f"[QSM] Saved checkpoint: {ckpt_path}")

    # ---- Final checkpoint ----
    final_path = output_dir / "checkpoint_final.pt"
    torch.save(
        {
            "step":         total_steps,
            "args":         vars(args),
            "policy_state": policy.state_dict(),
            "q_func_state": q_func.state_dict(),
        },
        final_path,
    )
    print(f"[QSM] Training done.  Final checkpoint: {final_path}")

    # ---- Save metrics ----
    metrics_path = output_dir / "metrics.npz"
    np.savez(
        metrics_path,
        critic_losses   = np.array(critic_losses,   dtype=np.float32),
        actor_losses    = np.array(actor_losses,     dtype=np.float32),
        episode_rewards = np.array(episode_rewards,  dtype=np.float32),
        step_reward_log = np.array(step_reward_log,  dtype=np.float32),
    )
    print(f"[QSM] Metrics saved to {metrics_path}")

    # ---- Plot training curves ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(14, 8))
        fig.suptitle("QSM Training Curves", fontsize=14)

        axes[0, 0].plot(critic_losses, alpha=0.7, linewidth=0.8)
        axes[0, 0].set_title("Critic loss (per update)")
        axes[0, 0].set_xlabel("Update step")

        axes[0, 1].plot(actor_losses, alpha=0.7, linewidth=0.8, color="orange")
        axes[0, 1].set_title("Actor loss (per update)")
        axes[0, 1].set_xlabel("Update step")

        axes[1, 0].plot(episode_rewards, marker=".", markersize=3, linewidth=0.8, color="green")
        axes[1, 0].set_title("Episode return")
        axes[1, 0].set_xlabel("Episode")

        log_steps = np.arange(len(step_reward_log)) * args.log_every
        axes[1, 1].plot(log_steps, step_reward_log, linewidth=0.8, color="red")
        axes[1, 1].set_title("Step reward (100-step avg)")
        axes[1, 1].set_xlabel("Environment step")

        for ax in axes.flat:
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plot_path = output_dir / "training_curves.png"
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        print(f"[QSM] Training curves saved to {plot_path}")
    except ImportError:
        print("[QSM] matplotlib not available — skipping plot")

    env.close()


if __name__ == "__main__":
    main()
