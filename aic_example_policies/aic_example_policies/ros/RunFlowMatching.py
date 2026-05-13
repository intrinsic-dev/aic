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

# Make aic_training importable.
_AIC_UTILS = Path("/home/lhphanto/ws_aic/src/aic/aic_utils")
if not _AIC_UTILS.exists():
    _AIC_UTILS = Path(__file__).resolve().parents[3] / "aic_utils"
if str(_AIC_UTILS) not in sys.path:
    sys.path.insert(0, str(_AIC_UTILS))

# flow_matching.py imports both timm (EfficientFormer) and transformers (DINOv2)
# at module level, which adds ~20 s of cold-start cost even when only one encoder
# is used.  Once the encoder choice is finalised, the unused import can be removed
# from flow_matching.py to cut startup time further.
# For now, defer the entire import to _load_policy() so this module loads in
# milliseconds and the lifecycle node can respond to the configure transition
# before the external manager times out.
FlowMatchingPolicy = None
ROBOT_STATE_DIM = None

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
    "/home/lhphanto/ws_aic/src/aic/outputs/05_09_cfg_fm_6d1/checkpoint_epoch0025.pt"
)

# Image size expected by the policy (must match training dataset resolution)
IMG_H = 256
IMG_W = 288

# Flow matching sampling hyperparameters
NUM_FLOW_STEPS = 10   # midpoint solver needs far fewer steps than DDPM
FLOW_SOLVER     = "euler"   # "euler" or "midpoint"
GUIDANCE_SCALE  = 1.0          # > 1.0 enables CFG (requires cfg_dropout_prob > 0 at training)

# Observation / prediction horizon (must match training)
OBS_HORIZON      = 2
PRED_HORIZON     = 8
# How many generated actions to execute before re-running inference
ACTION_EXEC_STEPS = 2


# ---------------------------------------------------------------------------
# 6D rotation → quaternion conversion  (inverse of training-time conversion)
# ---------------------------------------------------------------------------
# Training stores: quat(x,y,z,w) → R = quaternion_to_matrix(w,x,y,z) → rot6d = R[:2, :]
# At inference we invert: rot6d → R (Gram-Schmidt) → quat(x,y,z,w)

def _rot6d_to_quat_xyzw(rot6d: np.ndarray) -> np.ndarray:
    """Convert 6D rotation representation to quaternion (x, y, z, w).

    The 6D representation is the first two rows of the rotation matrix,
    flattened.  Gram-Schmidt orthogonalisation recovers the full matrix;
    Shepperd's method then yields the unit quaternion.

    Args:
        rot6d: (6,) float32

    Returns:
        quat: (4,) float32 — (x, y, z, w), unit norm
    """
    # Reconstruct orthonormal rows
    r1 = rot6d[:3].astype(np.float64)
    r2 = rot6d[3:6].astype(np.float64)

    r1 = r1 / (np.linalg.norm(r1) + 1e-8)
    r2 = r2 - np.dot(r1, r2) * r1
    r2 = r2 / (np.linalg.norm(r2) + 1e-8)
    r3 = np.cross(r1, r2)

    R = np.stack([r1, r2, r3], axis=0)  # (3, 3) — rows are r1, r2, r3

    # Shepperd's method: R → quaternion (w, x, y, z), then reorder to (x, y, z, w)
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
    q /= np.linalg.norm(q) + 1e-8
    return q  # (x, y, z, w)


def _action9_to_pose7(action9: np.ndarray) -> np.ndarray:
    """Convert a (9,) flow-matching action [pos(3) | rot6d(6)] to (7,) [pos(3) | quat(4)]."""
    pos  = action9[:3]
    quat = _rot6d_to_quat_xyzw(action9[3:9])  # (x, y, z, w)
    return np.concatenate([pos, quat]).astype(np.float32)  # (7,)


class RunFlowMatching(Policy):
    """Flow Matching policy inference node for the AIC cable-insertion task.

    Supports action_dim=9 (pos+rot6d) or action_dim=10 (pos+rot6d+dist_to_target).
    When action_dim=10, dim[9] is the predicted distance-to-target and is logged
    but not sent to the robot.

    The action representation uses the 6D continuous rotation (Zhou et al. 2019)
    internally; actions are converted back to quaternions before being sent to
    the robot controller.

    The policy maintains a rolling buffer of the last OBS_HORIZON observations
    and re-runs inference every ACTION_EXEC_STEPS executed actions.
    """

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"RunFlowMatching: using device {self.device}")

        self._policy     = self._load_policy()
        self._obs_buf    = deque(maxlen=OBS_HORIZON)
        self._action_queue: list[np.ndarray] = []   # pre-computed pose actions (7-D)

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_policy(self):
        global FlowMatchingPolicy, ROBOT_STATE_DIM
        if FlowMatchingPolicy is None:
            from aic_training.flow_matching import (
                FlowMatchingPolicy as _FMP,
                ROBOT_STATE_DIM as _RSD,
            )
            FlowMatchingPolicy = _FMP
            ROBOT_STATE_DIM = _RSD

        ckpt = torch.load(CHECKPOINT_PATH, map_location=self.device)
        args = ckpt.get("args", {})

        policy = FlowMatchingPolicy(
            obs_horizon=args.get("obs_horizon", OBS_HORIZON),
            pred_horizon=args.get("pred_horizon", PRED_HORIZON),
            action_dim=args.get("action_dim", 9),  # 9 = pos+rot6d, 10 = +dist_to_target
            robot_state_dim=ROBOT_STATE_DIM,
            n_heads=args.get("n_heads", 8),
            n_layers=args.get("n_layers", 4),
            ffn_dim=args.get("ffn_dim", 1024),
            image_encoder_type=args.get("image_encoder_type", "resnet"),
            resnet_name=args.get("resnet_name", "resnet18"),
            resnet_weights=args.get("resnet_weights", "IMAGENET1K_V1"),
            local_ckpt_path=args.get("dino_checkpoint", args.get("local_ckpt_path", None)),
            camera_keys=("center_camera", "right_camera") if args.get("skip_left_camera", False) else None,
            pos_enc=args.get("pos_enc", "rope"),
        ).to(self.device)

        # torch.compile() renames state-dict keys by inserting "._orig_mod." into
        # every submodule path (e.g. image_encoder._orig_mod.backbone.*).  Strip
        # that infix so checkpoints saved after compilation load cleanly here.
        raw_sd = ckpt["model_state"]
        sd = {k.replace("._orig_mod.", "."): v for k, v in raw_sd.items()}
        # strict=False: tolerates keys added after the checkpoint was saved
        # (e.g. state_time_emb introduced after this checkpoint was trained).
        missing, unexpected = policy.load_state_dict(sd, strict=False)
        if missing:
            self.get_logger().warn(
                f"load_state_dict: {len(missing)} missing key(s) — "
                "will use random init: " + str(missing[:5])
            )
        if unexpected:
            self.get_logger().warn(
                f"load_state_dict: {len(unexpected)} unexpected key(s) ignored: "
                + str(unexpected[:5])
            )
        policy.eval()
        policy.profiling = True

        # Compile the transformer (the repeated inner loop of sample()).
        policy.vector_field_net = torch.compile(policy.vector_field_net)
        self.get_logger().info(
            "RunFlowMatching: vector_field_net compiled with torch.compile"
        )
        self.get_logger().info(f"RunFlowMatching: loaded checkpoint {CHECKPOINT_PATH}")
        return policy

    # ------------------------------------------------------------------
    # Observation preprocessing  (identical to RunDiffusion)
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
        """Run flow matching ODE sampling and return a list of tcp_pose arrays.

        Returns:
            List of (7,) arrays — [x, y, z, qx, qy, qz, qw] — one per step.
        """
        images, robot_state, target_module, port_name = self._build_policy_inputs(task)

        t0 = time.perf_counter()
        actions = self._policy.sample(
            images=images,
            robot_state=robot_state,
            target_module=target_module,
            port_name=port_name,
            num_steps=NUM_FLOW_STEPS,
            guidance_scale=GUIDANCE_SCALE,
            solver=FLOW_SOLVER,
        )  # (1, pred_horizon, action_dim)
        elapsed_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(
            f"RunFlowMatching: sample() {elapsed_ms:.1f} ms  "
            f"({NUM_FLOW_STEPS} steps, solver={FLOW_SOLVER!r}, "
            f"{elapsed_ms / NUM_FLOW_STEPS:.2f} ms/step)"
        )

        actions_np = actions[0].cpu().numpy()  # (pred_horizon, action_dim)
        action_dim = actions_np.shape[-1]

        # If model predicts dist_to_target as dim 10, log it
        if action_dim >= 10:
            dist_steps = actions_np[:, 9]  # (pred_horizon,)
            self.get_logger().info(
                f"RunFlowMatching: dist_to_target  "
                f"mean={dist_steps.mean():.4f}  "
                f"min={dist_steps.min():.4f}  "
                f"max={dist_steps.max():.4f}  "
                f"steps={dist_steps.tolist()}"
            )

        # Convert rot6d → quaternion for each action step (first 9 dims only)
        pose_actions = [_action9_to_pose7(actions_np[i, :9]) for i in range(PRED_HORIZON)]
        return pose_actions

    # ------------------------------------------------------------------
    # Action validation / logging
    # ------------------------------------------------------------------

    def _log_action(self, action: np.ndarray, obs_msg: Observation) -> None:
        """Log the commanded action and warn if values look suspicious.

        Checks:
          - Quaternion norm ≈ 1 (bad norm → rot6d → quat conversion failed)
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
                "rot6d → quat conversion may have failed"
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
        self.get_logger().info(f"RunFlowMatching.insert_cable() task: {task}")

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
            action = self._action_queue.pop(0)   # (7,) [x, y, z, qx, qy, qz, qw]
            self._log_action(action, obs_msg)
            pose = Pose(
                position=Point(
                    x=float(action[0]), y=float(action[1]), z=float(action[2])
                ),
                orientation=Quaternion(
                    x=float(action[3]), y=float(action[4]),
                    z=float(action[5]), w=float(action[6]),
                ),
            )
            self.set_pose_target(move_robot=move_robot, pose=pose)

            ## Check for successful insertion
            cs = obs_msg.controller_state
            #if hasattr(cs, "insertion_event") and cs.insertion_event >= 2:
            #    self.get_logger().info("RunFlowMatching: insertion success!")
            #    return True

            step += 1
            if step % 20 == 0:
                self.get_logger().info(
                    f"RunFlowMatching: step {step}, "
                    f"tcp=({cs.tcp_pose.position.x:.3f}, "
                    f"{cs.tcp_pose.position.y:.3f}, "
                    f"{cs.tcp_pose.position.z:.3f})"
                )

            self.sleep_for(0.1)  # 10 Hz
