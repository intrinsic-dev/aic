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

from aic_training.robot_vit_with_task import RobotWithTaskPolicy, ROBOT_STATE_DIM

# ---------------------------------------------------------------------------
# Checkpoint paths  — update before running
# ---------------------------------------------------------------------------

RLPD_CHECKPOINT_PATH = Path(
    "/home/lhphanto/ws_aic/src/aic/outputs/0513_rlpd1/checkpoint_final.pt"
)

# Optional: path to PortKeypointNet checkpoint for keypoint overlay.
# Set to None to skip the keypoint preprocessing step.
KEYPOINT_CHECKPOINT_PATH: Path | None = Path(
    "/home/lhphanto/ws_aic/src/aic/outputs/0512_keypoint1/best.pt"
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Task encoding tables (must match training)
TASK_TARGET_MODULE_ENCODING: dict[str, int] = {
    "nic_card_mount_0": 0, "nic_card_mount_1": 1, "nic_card_mount_2": 2,
    "nic_card_mount_3": 3, "nic_card_mount_4": 4,
    "sc_port_0": 5, "sc_port_1": 6,
}
TASK_PORT_NAME_ENCODING: dict[str, int] = {
    "sfp_port_0": 0, "sfp_port_1": 1, "sc_port_base": 2,
}

# Image size expected by the policy (must match training)
IMG_H = 256
IMG_W = 288

# Keypoint overlay — RGB colors per entity (mirrors _ENTITY_COLORS in keypoint training)
_KEYPOINT_ENTITY_COLORS: dict[str, tuple[int, int, int]] = {
    "nic_card_mount_0": (255,  68,  68),
    "nic_card_mount_1": (255, 136,   0),
    "nic_card_mount_2": (255, 238,   0),
    "nic_card_mount_3": (  0, 255, 170),
    "nic_card_mount_4": (255,  68, 255),
    "sc_port_0":        ( 68, 255,  68),
    "sc_port_1":        ( 68, 170, 255),
}
_KEYPOINT_CAM_ORDER = ["left_camera", "center_camera", "right_camera"]


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------

def _quat_xyzw_to_rot6d(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to 6D rotation representation (first two rows of R)."""
    x, y, z, w = quat.astype(np.float64)
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),      1 - 2*(x*x + z*z),   2*(y*z - w*x)],
    ], dtype=np.float32)
    return R.flatten()


def _rot6d_to_quat_xyzw(rot6d: np.ndarray) -> np.ndarray:
    """Convert 6D rotation representation to quaternion (x, y, z, w)."""
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
    """Convert (9,) action [pos(3) | rot6d(6)] to (7,) [pos(3) | quat_xyzw(4)]."""
    return np.concatenate([action9[:3], _rot6d_to_quat_xyzw(action9[3:9])]).astype(np.float32)


# ---------------------------------------------------------------------------
# RunRlpd policy
# ---------------------------------------------------------------------------

class RunRlpd(Policy):
    """RLPD (SAC) policy inference node for the AIC cable-insertion task.

    Loads a RobotWithTaskPolicy checkpoint and runs deterministic inference
    (tanh of policy means — no exploration noise).  Optionally applies a
    PortKeypointNet overlay on the camera images before policy inference,
    matching the preprocessing used during RLPD training.

    Action representation: 9-D = pos(3) + rot6d(6).  Converted to
    pos(3) + quat(4) before being sent to the robot controller.
    """

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"RunRlpd: using device {self.device}")

        self._policy         = self._load_policy()
        self._keypoint_model = self._load_keypoint_model()
        self._prev_action: np.ndarray | None = None   # (9,) float32, updated each step

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_policy(self) -> RobotWithTaskPolicy:
        ckpt = torch.load(RLPD_CHECKPOINT_PATH, map_location=self.device)
        args = ckpt.get("args", {})

        policy = RobotWithTaskPolicy(
            robot_state_dim=ROBOT_STATE_DIM,
            action_dim=args.get("action_dim", 9),
            obs_horizon=args.get("obs_horizon", 1),
            resnet_weights=args.get("resnet_weights", "IMAGENET1K_V1"),
            local_ckpt_path=args.get("local_ckpt_path", None),
        ).to(self.device)

        sd = ckpt.get("policy_state", ckpt)
        missing, unexpected = policy.load_state_dict(sd, strict=False)
        if missing:
            self.get_logger().warn(
                f"RunRlpd: {len(missing)} missing key(s) in checkpoint: {missing[:5]}"
            )
        if unexpected:
            self.get_logger().warn(
                f"RunRlpd: {len(unexpected)} unexpected key(s) ignored: {unexpected[:5]}"
            )
        policy.eval()
        self.get_logger().info(f"RunRlpd: loaded policy from {RLPD_CHECKPOINT_PATH}")
        return policy

    def _load_keypoint_model(self):
        if KEYPOINT_CHECKPOINT_PATH is None or not KEYPOINT_CHECKPOINT_PATH.exists():
            self.get_logger().info("RunRlpd: no keypoint model — skipping overlay")
            return None

        from aic_training.keypoint_model import PortKeypointNet
        kp_dev = self.device
        ckpt   = torch.load(KEYPOINT_CHECKPOINT_PATH, map_location="cpu")
        model  = PortKeypointNet(pretrained=False)
        model.load_state_dict(ckpt.get("model", ckpt))
        model.eval().to(kp_dev)
        self.get_logger().info(
            f"RunRlpd: loaded keypoint model from {KEYPOINT_CHECKPOINT_PATH}"
        )
        return model

    # ------------------------------------------------------------------
    # Observation preprocessing
    # ------------------------------------------------------------------

    @staticmethod
    def _ros_image_to_tensor(ros_img) -> torch.Tensor:
        """Convert a sensor_msgs/Image to a (C, H, W) uint8 tensor."""
        img_np = np.frombuffer(ros_img.data, dtype=np.uint8).reshape(
            ros_img.height, ros_img.width, -1
        )
        if ros_img.encoding in ("bgr8", "bgra8"):
            img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        img_np = cv2.resize(img_np, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
        return torch.from_numpy(img_np).permute(2, 0, 1)   # (C, H, W) uint8

    @staticmethod
    def _build_robot_state(obs_msg: Observation) -> np.ndarray:
        """Extract 20-D robot state: tcp_pose(7) + joint_positions(7) + wrench(6)."""
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

    def _get_images(self, obs_msg: Observation) -> dict[str, torch.Tensor]:
        """Extract all three camera images as (C, H, W) uint8 tensors."""
        return {
            "left_camera":   self._ros_image_to_tensor(obs_msg.left_image),
            "center_camera": self._ros_image_to_tensor(obs_msg.center_image),
            "right_camera":  self._ros_image_to_tensor(obs_msg.right_image),
        }

    def _spot_target(
        self,
        images: dict[str, torch.Tensor],   # {cam_key: (C, H, W) uint8}
        target_module: str,
        port_name: str,
    ) -> dict[str, torch.Tensor]:
        """Overlay confidence-weighted circles where the target port is detected.

        No-op if no keypoint model is loaded or task strings are empty.
        Returns a new images dict with the same dtype/shape as input.
        """
        if self._keypoint_model is None or not target_module or not port_name:
            return images

        from aic_training.keypoint_constants import OUTPUT_KEYS

        img_h  = images[_KEYPOINT_CAM_ORDER[0]].shape[1]
        img_w  = images[_KEYPOINT_CAM_ORDER[0]].shape[2]
        radius = max(2, img_h // 80)
        kp_dev = next(self._keypoint_model.parameters()).device

        # Build normalized input list for keypoint model: [(1, 3, H, W) float32 ...]
        imgs_list = []
        for cam in _KEYPOINT_CAM_ORDER:
            img_f    = images[cam].float() / 255.0
            img_norm = (img_f - 0.5) / 0.5
            imgs_list.append(img_norm.unsqueeze(0).to(kp_dev))

        with torch.no_grad():
            pred = self._keypoint_model(imgs_list)

        conf_visible = pred["conf_visible"][0].cpu()   # (N,)
        xy_pred      = pred["xy"][0].cpu()             # (N, 2)

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
            px = int(float(xy_pred[out_idx, 0]) * img_w)
            py = int(float(xy_pred[out_idx, 1]) * img_h)
            color   = _KEYPOINT_ENTITY_COLORS.get(entity, (255, 255, 255))
            img_np  = numpy_imgs[cam]
            overlay = img_np.copy()
            cv2.circle(overlay, (px, py), radius=radius, color=color, thickness=-1)
            cv2.addWeighted(overlay, cv_val, img_np, 1.0 - cv_val, 0, img_np)

        return {
            cam: torch.from_numpy(numpy_imgs[cam]).permute(2, 0, 1)
            for cam in images
        }

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def _run_inference(self, obs_msg: Observation, task: Task) -> np.ndarray:
        """Run RLPD policy inference.  Returns a (9,) action in rot6d space."""
        robot_state   = self._build_robot_state(obs_msg)
        images        = self._get_images(obs_msg)
        t0 = time.perf_counter()
        images        = self._spot_target(images, task.target_module_name, task.port_name)
        elapsed_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(f"RunRlpd: spot target inference {elapsed_ms:.1f} ms")


        # Policy expects (B=1, T=1, C, H, W) float32 in [0, 1]
        imgs_t = {
            k: v.float().div(255.0).unsqueeze(0).unsqueeze(0).to(self.device)
            for k, v in images.items()
        }
        rs_t = torch.from_numpy(robot_state).unsqueeze(0).to(self.device)
        pa_t = torch.from_numpy(self._prev_action).unsqueeze(0).to(self.device)
        tm_t = torch.tensor(
            [TASK_TARGET_MODULE_ENCODING.get(task.target_module_name, 0)],
            dtype=torch.long, device=self.device,
        )
        pn_t = torch.tensor(
            [TASK_PORT_NAME_ENCODING.get(task.port_name, 0)],
            dtype=torch.long, device=self.device,
        )

        t0 = time.perf_counter()
        with torch.no_grad():
            means, _log_stds = self._policy(imgs_t, rs_t, pa_t, tm_t, pn_t)
            # Deterministic inference: apply tanh to means (no exploration noise)
            action_9d = torch.tanh(means)[0].cpu().numpy()
        elapsed_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(f"RunRlpd: policy inference {elapsed_ms:.1f} ms")

        return action_9d.astype(np.float32)

    # ------------------------------------------------------------------
    # Action logging
    # ------------------------------------------------------------------

    def _log_action(self, pose7: np.ndarray, obs_msg: Observation) -> None:
        x, y, z, qx, qy, qz, qw = pose7.tolist()
        quat_norm  = float(np.sqrt(qx**2 + qy**2 + qz**2 + qw**2))
        cur        = obs_msg.controller_state.tcp_pose.position
        step_dist  = float(np.sqrt((x - cur.x)**2 + (y - cur.y)**2 + (z - cur.z)**2))

        self.get_logger().info(
            f"action  pos=({x:.4f}, {y:.4f}, {z:.4f})  "
            f"quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})  "
            f"|q|={quat_norm:.4f}  delta={step_dist * 1000:.1f} mm"
        )
        if abs(quat_norm - 1.0) > 0.05:
            self.get_logger().warn(
                f"Quaternion norm {quat_norm:.4f} deviates from 1"
            )
        if step_dist > 0.05:
            self.get_logger().warn(
                f"Large step distance {step_dist * 1000:.1f} mm — policy may be unsafe"
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
        self.get_logger().info(f"RunRlpd.insert_cable() task: {task}")

        # Initialize prev_action from the current TCP pose so the policy has
        # a valid conditioning input on the very first inference step.
        obs_msg = get_observation()
        cs  = obs_msg.controller_state
        p, q = cs.tcp_pose.position, cs.tcp_pose.orientation
        tcp_pos  = np.array([p.x, p.y, p.z], dtype=np.float32)
        tcp_quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float32)
        self._prev_action = np.concatenate(
            [tcp_pos, _quat_xyzw_to_rot6d(tcp_quat)]
        ).astype(np.float32)

        step = 0
        while True:
            obs_msg   = get_observation()
            action_9d = self._run_inference(obs_msg, task)

            pose7 = _action9_to_pose7(action_9d)
            self._log_action(pose7, obs_msg)

            pose = Pose(
                position=Point(
                    x=float(pose7[0]), y=float(pose7[1]), z=float(pose7[2])
                ),
                orientation=Quaternion(
                    x=float(pose7[3]), y=float(pose7[4]),
                    z=float(pose7[5]), w=float(pose7[6]),
                ),
            )
            self.set_pose_target(move_robot=move_robot, pose=pose)

            # Update prev_action for the next inference step
            self._prev_action = action_9d.copy()

            step += 1
            if step % 20 == 0:
                cs = obs_msg.controller_state
                self.get_logger().info(
                    f"RunRlpd: step {step}  "
                    f"tcp=({cs.tcp_pose.position.x:.3f}, "
                    f"{cs.tcp_pose.position.y:.3f}, "
                    f"{cs.tcp_pose.position.z:.3f})"
                )

            self.sleep_for(0.1)  # 10 Hz
