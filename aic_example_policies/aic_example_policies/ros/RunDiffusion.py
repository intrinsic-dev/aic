#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import torch

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion

# Make aic_utils importable.
# aic_training is not an installed package, so we resolve the workspace path
# explicitly.  The relative-path trick breaks when this file is loaded from
# the pixi site-packages install rather than the source tree.
_AIC_UTILS = Path("/home/lhphanto/ws_aic/src/aic/aic_utils")
if not _AIC_UTILS.exists():
    # Fallback for development: file is two package-dirs deep inside aic_example_policies
    _AIC_UTILS = Path(__file__).resolve().parents[3] / "aic_utils"
if str(_AIC_UTILS) not in sys.path:
    sys.path.insert(0, str(_AIC_UTILS))

from aic_training.diffusion_policy import DiffusionPolicy, ROBOT_STATE_DIM

# Task encoding tables (must match aic_robot_aic_controller.py)
TASK_TARGET_MODULE_ENCODING: dict[str, int] = {
    "nic_card_mount_0": 0, "nic_card_mount_1": 1, "nic_card_mount_2": 2,
    "nic_card_mount_3": 3, "nic_card_mount_4": 4,
    "sc_port_0": 5, "sc_port_1": 6, "sc_port_2": 7, "sc_port_3": 8, "sc_port_4": 9,
}
TASK_PORT_NAME_ENCODING: dict[str, int] = {
    "sfp_port_0": 0, "sfp_port_1": 1, "sc_port_base": 2,
}

CHECKPOINT_PATH = Path(
    "/home/lhphanto/ws_aic/src/aic/outputs/apri_22/checkpoint_epoch0020.pt"
)

# Image size expected by the policy (must match training dataset resolution)
IMG_H = 256
IMG_W = 288

# Diffusion sampling hyperparameters (must match training)
NUM_DIFFUSION_STEPS = 50   # DPM-Solver++ 2M needs far fewer steps than DDPM
BETA_START = 1e-4
BETA_END   = 0.02

# How many obs frames to buffer before running the policy
OBS_HORIZON = 2
# How many action steps to generate and execute per inference call
PRED_HORIZON = 4
# How many generated actions to execute before re-running inference
# (temporal ensemble: execute only the first N steps of the predicted sequence)
ACTION_EXEC_STEPS = 2


class RunDiffusion(Policy):
    """Diffusion policy inference node for the AIC cable-insertion task.

    Loads a trained DiffusionPolicy checkpoint and runs DDPM reverse diffusion
    at each inference step to generate a sequence of tcp_pose targets.

    The policy maintains a rolling buffer of the last OBS_HORIZON observations
    and re-runs inference every ACTION_EXEC_STEPS executed actions.
    """

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"RunDiffusion: using device {self.device}")

        self._policy   = self._load_policy()
        self._obs_buf  = deque(maxlen=OBS_HORIZON)  # rolling obs buffer
        self._action_queue: list[np.ndarray] = []   # pre-computed actions to execute

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_policy(self) -> DiffusionPolicy:
        ckpt = torch.load(CHECKPOINT_PATH, map_location=self.device)
        args = ckpt.get("args", {})

        policy = DiffusionPolicy(
            obs_horizon=args.get("obs_horizon", OBS_HORIZON),
            action_dim=args.get("action_dim", 7),
            robot_state_dim=ROBOT_STATE_DIM,
            n_heads=args.get("n_heads", 8),
            n_layers=args.get("n_layers", 4),
            ffn_dim=args.get("ffn_dim", 1024),
        ).to(self.device)

        policy.load_state_dict(ckpt["model_state"])
        policy.eval()
        policy.profiling = True

        # Compile the transformer (the repeated inner loop of sample()).
        # "reduce-overhead" enables CUDA graph capture, which eliminates kernel
        # launch overhead — the biggest win for the small fixed-shape batches
        # used at inference (B=1).  Compilation happens on the first call;
        # subsequent calls use the cached graph.
        # NOTE: profiling's cuda.synchronize() calls are still valid inside a
        # compiled graph, but disable profiling if you want the cleanest numbers.
        policy.noise_pred_net = torch.compile(
            policy.noise_pred_net, mode="reduce-overhead"
        )
        self.get_logger().info("RunDiffusion: noise_pred_net compiled with torch.compile")

        self.get_logger().info(f"RunDiffusion: loaded checkpoint {CHECKPOINT_PATH}")
        return policy

    # ------------------------------------------------------------------
    # Observation preprocessing
    # ------------------------------------------------------------------

    @staticmethod
    def _ros_image_to_tensor(ros_img) -> torch.Tensor:
        """Convert a sensor_msgs/Image to a (C, H, W) float32 tensor in [0, 1]."""
        img_np = np.frombuffer(ros_img.data, dtype=np.uint8).reshape(
            ros_img.height, ros_img.width, -1
        )
        if ros_img.encoding in ("bgr8", "bgra8"):
            img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        img_np = cv2.resize(img_np, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
        return torch.from_numpy(img_np).permute(2, 0, 1).float().div(255.0)

    @staticmethod
    def _build_robot_state(obs_msg: Observation) -> np.ndarray:
        """Extract ROBOT_STATE_DIM=20 state vector from an Observation message.

        Layout: tcp_pose(7) + joint_positions(7) + wrench.force(3) + wrench.torque(3)
        """
        cs = obs_msg.controller_state
        p  = cs.tcp_pose.position
        q  = cs.tcp_pose.orientation
        w  = obs_msg.wrist_wrench.wrench
        return np.array([
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            *obs_msg.joint_states.position[:7],
            w.force.x,  w.force.y,  w.force.z,
            w.torque.x, w.torque.y, w.torque.z,
        ], dtype=np.float32)

    def _obs_to_frame(self, obs_msg: Observation) -> dict:
        """Convert a single Observation message into a frame dict."""
        return {
            "left_camera":   self._ros_image_to_tensor(obs_msg.left_image),
            "center_camera": self._ros_image_to_tensor(obs_msg.center_image),
            "right_camera":  self._ros_image_to_tensor(obs_msg.right_image),
            "robot_state":   self._build_robot_state(obs_msg),
        }

    def _build_policy_inputs(
        self,
        task: Task,
    ) -> tuple[dict, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Stack the obs buffer into batched policy inputs (batch size = 1)."""
        frames = list(self._obs_buf)

        # Images: {camera_key: (1, obs_horizon, C, H, W)}
        images = {
            k: torch.stack([f[k] for f in frames])   # (obs_horizon, C, H, W)
              .unsqueeze(0)                            # (1, obs_horizon, C, H, W)
              .to(self.device)
            for k in ("left_camera", "center_camera", "right_camera")
        }

        # Robot state: (1, obs_horizon, ROBOT_STATE_DIM)
        robot_state = torch.from_numpy(
            np.stack([f["robot_state"] for f in frames])
        ).unsqueeze(0).to(self.device)

        # Task identity
        target_module = torch.tensor(
            [TASK_TARGET_MODULE_ENCODING[task.target_module_name]],
            dtype=torch.int64, device=self.device,
        )
        port_name = torch.tensor(
            [TASK_PORT_NAME_ENCODING[task.port_name]],
            dtype=torch.int64, device=self.device,
        )

        return images, robot_state, target_module, port_name

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def _run_inference(self, task: Task) -> list[np.ndarray]:
        """Run diffusion sampling and return a list of tcp_pose numpy arrays."""
        images, robot_state, target_module, port_name = self._build_policy_inputs(task)

        t0 = time.perf_counter()
        actions = self._policy.sample(
            images=images,
            robot_state=robot_state,
            target_module=target_module,
            port_name=port_name,
            num_timesteps=NUM_DIFFUSION_STEPS,
            beta_start=BETA_START,
            beta_end=BETA_END,
            pred_horizon=PRED_HORIZON,
            solver="ddpm",
        )  # (1, pred_horizon, 7)
        elapsed_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(
            f"RunDiffusion: sample() {elapsed_ms:.1f} ms  "
            f"({NUM_DIFFUSION_STEPS} steps, {elapsed_ms / NUM_DIFFUSION_STEPS:.2f} ms/step)"
        )

        actions_np = actions[0].cpu().numpy()  # (pred_horizon, 7)
        return [actions_np[i] for i in range(PRED_HORIZON)]

    # ------------------------------------------------------------------
    # Action validation
    # ------------------------------------------------------------------

    def _log_action(self, action: np.ndarray, obs_msg) -> None:
        """Log the commanded action and warn if values look suspicious.

        Checks:
          - Quaternion norm ≈ 1 (bad norm → the model is outputting garbage)
          - Step delta from current TCP pose (large jump → policy is erratic)
        """
        x, y, z, qx, qy, qz, qw = action.tolist()
        quat_norm = float(np.sqrt(qx**2 + qy**2 + qz**2 + qw**2))

        cs  = obs_msg.controller_state
        cur = cs.tcp_pose.position
        dx  = x - cur.x
        dy  = y - cur.y
        dz  = z - cur.z
        step_dist = float(np.sqrt(dx**2 + dy**2 + dz**2))

        self.get_logger().info(
            f"action  pos=({x:.4f}, {y:.4f}, {z:.4f})  "
            f"quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})  "
            f"|q|={quat_norm:.4f}  delta={step_dist*1000:.1f} mm"
        )

        if abs(quat_norm - 1.0) > 0.05:
            self.get_logger().warn(
                f"Quaternion norm {quat_norm:.4f} deviates from 1 — "
                "policy may be producing invalid orientations"
            )
        if step_dist > 0.05:
            self.get_logger().warn(
                f"Large step distance {step_dist*1000:.1f} mm — "
                "policy may be commanding an unsafe jump"
            )

    # ------------------------------------------------------------------
    # Policy entry point
    # ------------------------------------------------------------------

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(f"RunDiffusion.insert_cable() task: {task}")

        self._obs_buf.clear()
        self._action_queue.clear()

        # Pre-fill the observation buffer
        while len(self._obs_buf) < OBS_HORIZON:
            self._obs_buf.append(self._obs_to_frame(get_observation()))
            self.sleep_for(0.1)

        step = 0
        while True:
            obs_msg = get_observation()
            self._obs_buf.append(self._obs_to_frame(obs_msg))

            # Re-run inference when the action queue is empty
            if not self._action_queue:
                self._action_queue = self._run_inference(task)

            # Execute the next pre-computed action
            action = self._action_queue.pop(0)   # (7,)  [x,y,z,qx,qy,qz,qw]
            self._log_action(action, obs_msg)
            pose = Pose(
                position=Point(x=float(action[0]), y=float(action[1]), z=float(action[2])),
                orientation=Quaternion(
                    x=float(action[3]), y=float(action[4]),
                    z=float(action[5]), w=float(action[6]),
                ),
            )
            self.set_pose_target(move_robot=move_robot, pose=pose)

            # Check for successful insertion
            cs = obs_msg.controller_state
            if hasattr(cs, "insertion_event") and cs.insertion_event >= 2:
                self.get_logger().info("RunDiffusion: insertion success!")
                return True

            step += 1
            if step % 20 == 0:
                self.get_logger().info(
                    f"RunDiffusion: step {step}, "
                    f"tcp=({cs.tcp_pose.position.x:.3f}, "
                    f"{cs.tcp_pose.position.y:.3f}, "
                    f"{cs.tcp_pose.position.z:.3f})"
                )

            self.sleep_for(0.1)  # 10 Hz
