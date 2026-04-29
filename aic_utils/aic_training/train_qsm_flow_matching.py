#!/usr/bin/env python3
"""
Online RL training: Q-Score Matching with FlowMatchingPolicy actor.

The actor (FlowMatchingPolicy) generates absolute TCP-pose action sequences
from visual + robot-state observations.  A Q-function critic estimates the
expected return for each (state, action-sequence) pair.

Actor update — Q-Score Matching:
  The standard rectified-flow velocity target u_t = (x_1 - x_noise) is augmented
  with the gradient of the Q-function at the buffered action x_1:
      u_guided = (x_1 - x_noise) + α * ∇_{x_1} Q(s, x_1)
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
from aic_control_interfaces.msg import (
    MotionUpdate,
    TrajectoryGenerationMode,
)
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
        pred_horizon:    int = 16,
        action_dim:      int = 9,
        robot_state_dim: int = ROBOT_STATE_DIM,
        hidden_dim:      int = 256,
        task_dim:        int = 64,
    ):
        super().__init__()
        self.obs_horizon  = obs_horizon
        self.pred_horizon = pred_horizon
        self.action_dim   = action_dim

        in_dim = obs_horizon * robot_state_dim + pred_horizon * action_dim + task_dim

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
        actions:       torch.Tensor,   # (B, pred_horizon, action_dim)
    ) -> tuple[torch.Tensor, torch.Tensor]:   # (B,), (B,)
        task_idx = target_module.clamp(0, NUM_TASKS - 1)
        t = self.task_embedding(task_idx)                      # (B, task_dim)
        x = torch.cat([robot_state.flatten(1), actions.flatten(1), t], dim=-1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)


# ---------------------------------------------------------------------------
# ReplayBuffer
# ---------------------------------------------------------------------------

class ReplayBuffer:
    """Circular replay buffer storing (obs, actions, returns) tuples.

    Each stored observation is a dict with:
      "images"       : {cam_key: (obs_horizon, C, H, W)} float32 tensors on CPU
      "robot_state"  : (obs_horizon, ROBOT_STATE_DIM) float32 tensor
      "target_module": int
      "port_name"    : int

    The "actions" are the full (pred_horizon, action_dim) sequence generated
    by the policy at the current step.  The "returns" are the discounted
    Monte-Carlo returns from this step.
    """

    def __init__(self, capacity: int = 10_000):
        self._buf: deque[dict] = deque(maxlen=capacity)

    def __len__(self) -> int:
        return len(self._buf)

    def push(
        self,
        obs:           dict,
        actions:       np.ndarray,
        returns:       float,
    ) -> None:
        self._buf.append({
            "obs":     obs,
            "actions": actions.astype(np.float32),
            "returns": float(returns),
        })

    def sample(self, batch_size: int) -> dict:
        items = random.sample(self._buf, min(batch_size, len(self._buf)))

        # Stack images: {cam_key: (B, obs_horizon, C, H, W)}
        cam_keys = list(items[0]["obs"]["images"].keys())
        images = {
            k: torch.stack([torch.as_tensor(it["obs"]["images"][k]) for it in items])
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
            np.stack([it["actions"] for it in items]), dtype=torch.float32
        )
        returns = torch.tensor(
            [it["returns"] for it in items], dtype=torch.float32
        )
        return {
            "images":        images,
            "robot_state":   robot_state,
            "target_module": target_module,
            "port_name":     port_name,
            "actions":       actions,
            "returns":       returns,
        }


# ---------------------------------------------------------------------------
# Actor / critic update functions
# ---------------------------------------------------------------------------

def update_critic(
    q_func:    QFunction,
    opt:       torch.optim.Optimizer,
    batch:     dict,
    device:    torch.device,
) -> float:
    """Minimize MSE between Q(s, a) and Monte-Carlo returns G_t."""
    robot_state   = batch["robot_state"].to(device)
    target_module = batch["target_module"].to(device)
    port_name     = batch["port_name"].to(device)
    actions       = batch["actions"].to(device)
    returns       = batch["returns"].to(device)

    q1, q2 = q_func(robot_state, target_module, port_name, actions)
    loss   = F.mse_loss(q1, returns) + F.mse_loss(q2, returns)

    opt.zero_grad()
    loss.backward()
    torch.nn.utils.clip_grad_norm_(q_func.parameters(), 1.0)
    opt.step()
    return loss.item()


def update_actor(
    policy:    FlowMatchingPolicy,
    q_func:    QFunction,
    opt:       torch.optim.Optimizer,
    batch:     dict,
    device:    torch.device,
    q_alpha:   float = 0.1,
) -> float:
    """Q-Score Matching actor update.

    Augments the rectified-flow velocity target with the Q-gradient:
        u_guided = (x_1 - x_noise) + α * ∇_{x_1} Q(s, x_1)

    Args:
        q_alpha: Scale of the Q-gradient term.  Start small (0.01–0.1).
    """
    B = batch["actions"].shape[0]

    images        = {k: v.to(device) for k, v in batch["images"].items()}
    robot_state   = batch["robot_state"].to(device)
    target_module = batch["target_module"].to(device)
    port_name     = batch["port_name"].to(device)
    actions       = batch["actions"].to(device)  # (B, pred_horizon, action_dim)

    # --- Compute Q-gradient w.r.t. buffered actions (min of double-Q) ---
    actions_g  = actions.detach().requires_grad_(True)
    q1, q2     = q_func(robot_state, target_module, port_name, actions_g)
    q_val      = torch.min(q1, q2)
    q_grad     = torch.autograd.grad(q_val.sum(), actions_g)[0]  # (B, pred_horizon, action_dim)

    # Normalize Q-gradient so its scale is ~1 (prevents instability if Q is large)
    grad_norm = q_grad.norm(dim=[-1, -2], keepdim=True).clamp(min=1e-8)
    q_grad_n  = q_grad / grad_norm                              # unit-norm per sample

    # --- Rectified-flow interpolation ---
    t      = torch.rand(B, device=device)                       # (B,)
    t_bc   = t[:, None, None]
    x_noise = torch.randn_like(actions)                         # (B, pred_horizon, action_dim)
    x_t     = (1.0 - t_bc) * x_noise + t_bc * actions.detach()

    # Guided target velocity: standard flow target + Q-gradient push
    u_t      = actions.detach() - x_noise                       # unguided velocity
    u_guided = u_t + q_alpha * q_grad_n.detach()                # Q-guided velocity

    # --- Policy forward + actor loss ---
    v_pred = policy(
        images=images,
        robot_state=robot_state,
        target_module=target_module,
        port_name=port_name,
        x_t=x_t,
        t=t,
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
    parser.add_argument("--pred_horizon",  type=int, default=16)
    parser.add_argument("--exec_steps",    type=int, default=8,
                        help="How many actions from the predicted sequence to execute "
                             "before re-querying the policy.")
    parser.add_argument("--num_flow_steps", type=int, default=10,
                        help="ODE integration steps for policy.sample().")
    parser.add_argument("--guidance_scale", type=float, default=1.0,
                        help="CFG guidance scale (1.0 = disabled).")

    # RL hyperparameters
    parser.add_argument("--gamma",           type=float, default=0.99)
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
        pred_horizon=args.pred_horizon,
        action_dim=args.action_dim,
        robot_state_dim=ROBOT_STATE_DIM,
    ).to(device)

    # ---- Optimizers ----
    actor_opt  = torch.optim.AdamW(policy.parameters(),  lr=args.actor_lr)
    critic_opt = torch.optim.AdamW(q_func.parameters(), lr=args.critic_lr)

    # ---- Replay buffer ----
    replay_buffer = ReplayBuffer(capacity=args.buffer_capacity)

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
    total_steps = 0
    update_count = 0

    # ---- Initial reset ----
    flat_obs, info = env.reset()
    task_info    = info.get("task", {})
    tm_idx, pn_idx = encode_task(task_info)

    obs_buffer   = deque([flat_obs] * args.obs_horizon, maxlen=args.obs_horizon)

    # Seed the image buffer
    while True:
        imgs = env.get_images()
        if imgs is not None:
            break
        time.sleep(0.05)
    img_buffer = deque([imgs] * args.obs_horizon, maxlen=args.obs_horizon)

    episode_reward = 0.0
    episode_len    = 0

    # Per-episode storage for MC return computation
    episode_transitions: list[dict] = []   # {obs, action_seq, reward}

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
                seq[:, :3] += np.random.randn(args.pred_horizon, 3) * 0.01  # 1 cm noise
                action_seq = seq  # (pred_horizon, 9)
            else:
                # Policy inference
                robot_state_t = torch.from_numpy(
                    np.stack([build_robot_state(o) for o in obs_buffer])
                ).unsqueeze(0).to(device)   # (1, obs_horizon, ROBOT_STATE_DIM)

                images_t = {
                    k: torch.stack([f[k] for f in img_buffer]).unsqueeze(0).to(device)
                    for k in imgs.keys()
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
                        solver="midpoint",
                    )  # (1, pred_horizon, action_dim)

                action_seq = actions_out[0].cpu().numpy()  # (pred_horizon, action_dim)

            # Snapshot the current observation for replay buffer storage
            current_obs_snap = {
                "images": {
                    k: torch.stack([f[k] for f in img_buffer]).clone()
                    for k in img_buffer[0].keys()
                },
                "robot_state":   np.stack([build_robot_state(o) for o in obs_buffer]),
                "target_module": tm_idx,
                "port_name":     pn_idx,
            }

            action_queue = [action_seq[i] for i in range(args.pred_horizon)]

            # Store the (obs, action_seq) — reward is filled later from episode
            episode_transitions.append({
                "obs":        current_obs_snap,
                "action_seq": action_seq.copy(),
                "reward":     0.0,
            })

        # ----------------------------------------------------------------
        # Execute one step from the action queue
        # ----------------------------------------------------------------
        action_9d = action_queue.pop(0)
        flat_obs_new, reward, terminated, truncated, info = env.step_pose(action_9d)

        # Update last transition's reward
        if episode_transitions:
            episode_transitions[-1]["reward"] += reward

        # Update rolling buffers
        obs_buffer.append(flat_obs_new)
        new_imgs = env.get_images()
        if new_imgs is not None:
            img_buffer.append(new_imgs)
            imgs = new_imgs

        flat_obs = flat_obs_new
        episode_reward += reward
        episode_len    += 1
        total_steps    += 1

        done = terminated or truncated

        # ----------------------------------------------------------------
        # Episode end — compute returns and push to buffer
        # ----------------------------------------------------------------
        if done:
            # Compute discounted Monte-Carlo returns
            G = 0.0
            for trans in reversed(episode_transitions):
                G = trans["reward"] + args.gamma * G
                replay_buffer.push(
                    obs=trans["obs"],
                    actions=trans["action_seq"],
                    returns=G,
                )

            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_len)
            print(
                f"[QSM] Episode  steps={episode_len}  "
                f"return={episode_reward:.3f}  "
                f"buffer={len(replay_buffer)}"
            )

            # Reset episode state
            episode_transitions.clear()
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
            img_buffer.clear()
            img_buffer.extend([imgs] * args.obs_horizon)

        # ----------------------------------------------------------------
        # Gradient updates
        # ----------------------------------------------------------------
        if total_steps >= args.start_training and len(replay_buffer) >= args.batch_size:
            for _ in range(args.updates_per_step):
                batch = replay_buffer.sample(args.batch_size)

                # Critic update
                c_loss = update_critic(q_func, critic_opt, batch, device)
                critic_losses.append(c_loss)
                update_count += 1

                # Actor update (less frequent)
                if update_count % args.actor_update_every == 0:
                    a_loss = update_actor(
                        policy, q_func, actor_opt, batch, device, args.q_alpha
                    )
                    actor_losses.append(a_loss)

        # ----------------------------------------------------------------
        # Logging
        # ----------------------------------------------------------------
        if total_steps % args.log_every == 0:
            c_avg = np.mean(critic_losses[-50:]) if critic_losses else float("nan")
            a_avg = np.mean(actor_losses[-50:])  if actor_losses  else float("nan")
            r_avg = np.mean(episode_rewards[-10:]) if episode_rewards else float("nan")
            print(
                f"[QSM] step={total_steps:6d}  "
                f"critic_loss={c_avg:.4f}  "
                f"actor_loss={a_avg:.4f}  "
                f"ep_return(10)={r_avg:.3f}  "
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

    # ---- Save loss curve ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
        ax1.plot(critic_losses, alpha=0.7)
        ax1.set_title("Critic loss")
        ax1.set_xlabel("Update step")
        ax2.plot(episode_rewards)
        ax2.set_title("Episode returns")
        ax2.set_xlabel("Episode")
        plt.tight_layout()
        fig.savefig(output_dir / "training_curves.png", dpi=150)
        print(f"[QSM] Training curves saved to {output_dir}/training_curves.png")
    except ImportError:
        pass

    env.close()


if __name__ == "__main__":
    main()
