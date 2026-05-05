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

import os

os.environ["HF_HUB_ENABLE_HF_TRANSFER"] = "1"

import time
import json
import torch
import numpy as np
import cv2
import draccus
from pathlib import Path
from typing import Callable, Dict, Any, List
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task

from aic_control_interfaces.msg import (
    MotionUpdate,
    TrajectoryGenerationMode,
)
from geometry_msgs.msg import Wrench

# LeRobot & Safetensors
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.configuration_act import ACTConfig
from safetensors.torch import load_file
from huggingface_hub import snapshot_download


class RunACT(Policy):
    # Scale velocity-action before integrating into a position target.
    # 1.0 for our locally-trained model (actions are in real m/s and rad/s).
    # The 6.0 hack was for the OOD published grkw/aic_act_policy.
    ACTION_SCALE = 1.0

    # When True, integrate the velocity action into a position target and publish
    # via MODE_POSITION (set_pose_target). When False, publish raw scaled velocity.
    USE_POSITION_MODE = True

    # Loop period (sim-time seconds). Matches camera/observation rate (~20 Hz).
    LOOP_DT = 0.05

    # Cap |target − measured TCP| to bound impedance spring force.
    # With stiffness 90 N/m and clamp 2 cm: max steady-state force ~1.8 N.
    MAX_TARGET_OFFSET_M = 0.02
    MAX_TARGET_OFFSET_RAD = 0.10

    # Don't let target z drift more than this far below the TCP's initial z.
    WORKSPACE_Z_BELOW_START = 0.30

    # Force-aware backoff: threshold is calibrated at trial start as
    # (baseline_force + BACKOFF_FORCE_DELTA_N). The wrist FT sensor reads ~20 N
    # at trial start due to gripper + cable + plug static weight (the controller's
    # gravity_compensation covers the robot but not the payload). An absolute
    # threshold false-fires on tick 0; calibrating to baseline+delta only
    # triggers on actual contact above static load.
    BACKOFF_FORCE_DELTA_N = 15.0
    BACKOFF_TRIGGER_DURATION_S = 0.1
    BACKOFF_RECOVERY_TICKS = 5
    BACKOFF_TOTAL_BUDGET_S = 2.0
    BACKOFF_RECOVERY_VZ = 0.10  # m/s upward in base frame
    FORCE_BASELINE_SAMPLES = 6  # 6 ticks * 0.05 s = 0.3 s of baseline averaging

    def __init__(self, parent_node: Node):
        super().__init__(parent_node)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # -------------------------------------------------------------------------
        # 1. Configuration & Weights Loading
        # -------------------------------------------------------------------------
        # Local checkpoint trained on saivemu/aic_act_v1 (100 CheatCode rollouts).
        # Baked into the docker image via Dockerfile COPY at /opt/policy.
        # Falls back to a workspace-relative path for non-docker testing.
        policy_path = Path("/opt/policy")
        if not policy_path.exists():
            policy_path = Path(
                "/home/saivemu/code/aic-train/outputs/train/act_aic_v1/checkpoints/last/pretrained_model"
            )

        # Load Config Manually (Fixes 'Draccus' error by removing unknown 'type' field)
        with open(policy_path / "config.json", "r") as f:
            config_dict = json.load(f)
            if "type" in config_dict:
                del config_dict["type"]

        config = draccus.decode(ACTConfig, config_dict)

        # Load Policy Architecture & Weights
        self.policy = ACTPolicy(config)
        model_weights_path = policy_path / "model.safetensors"
        self.policy.load_state_dict(load_file(model_weights_path))
        self.policy.eval()
        self.policy.to(self.device)

        self.get_logger().info(f"ACT Policy loaded on {self.device} from {policy_path}")

        # -------------------------------------------------------------------------
        # 2. Normalization Stats Loading
        # -------------------------------------------------------------------------
        stats_path = (
            policy_path / "policy_preprocessor_step_3_normalizer_processor.safetensors"
        )
        stats = load_file(stats_path)

        # Helper to extract and shape stats for broadcasting
        def get_stat(key, shape):
            return stats[key].to(self.device).view(*shape)

        # Image Stats (1, 3, 1, 1) for broadcasting against (Batch, Channel, Height, Width)
        self.img_stats = {
            "left": {
                "mean": get_stat("observation.images.left_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.left_camera.std", (1, 3, 1, 1)),
            },
            "center": {
                "mean": get_stat("observation.images.center_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.center_camera.std", (1, 3, 1, 1)),
            },
            "right": {
                "mean": get_stat("observation.images.right_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.right_camera.std", (1, 3, 1, 1)),
            },
        }
        print(f"Image stats: {self.img_stats}")

        # Robot State Stats (1, 26)
        self.state_mean = get_stat("observation.state.mean", (1, -1))
        self.state_std = get_stat("observation.state.std", (1, -1))
        print(f"Robot state mean: {self.state_mean}")
        print(f"Robot state std: {self.state_std}")

        # Action Stats (1, 7) - Used for Un-normalization
        self.action_mean = get_stat("action.mean", (1, -1))
        self.action_std = get_stat("action.std", (1, -1))
        print(f"Action mean: {self.action_mean}")
        print(f"Action std: {self.action_std}")

        # Config
        self.image_scaling = 0.25  # Must match AICRobotAICControllerConfig

        self.get_logger().info("Normalization statistics loaded successfully.")

    @staticmethod
    def _img_to_tensor(
        raw_img,
        device: torch.device,
        scale: float,
        mean: torch.Tensor,
        std: torch.Tensor,
    ) -> torch.Tensor:
        """Converts ROS Image -> Resized -> Permuted -> Normalized Tensor."""
        # 1. Bytes to Numpy (H, W, C)
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )

        # 2. Resize
        if scale != 1.0:
            img_np = cv2.resize(
                img_np, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )

        # 3. To Tensor -> Permute (HWC -> CHW) -> Float -> Div(255) -> Batch Dim
        tensor = (
            torch.from_numpy(img_np)
            .permute(2, 0, 1)
            .float()
            .div(255.0)
            .unsqueeze(0)
            .to(device)
        )

        # 4. Normalize (Apply Mean/Std)
        # Formula: (x - mean) / std
        return (tensor - mean) / std

    def prepare_observations(self, obs_msg: Observation) -> Dict[str, torch.Tensor]:
        """Convert ROS Observation message into dictionary of normalized tensors."""

        # --- Process Cameras ---
        obs = {
            "observation.images.left_camera": self._img_to_tensor(
                obs_msg.left_image,
                self.device,
                self.image_scaling,
                self.img_stats["left"]["mean"],
                self.img_stats["left"]["std"],
            ),
            "observation.images.center_camera": self._img_to_tensor(
                obs_msg.center_image,
                self.device,
                self.image_scaling,
                self.img_stats["center"]["mean"],
                self.img_stats["center"]["std"],
            ),
            "observation.images.right_camera": self._img_to_tensor(
                obs_msg.right_image,
                self.device,
                self.image_scaling,
                self.img_stats["right"]["mean"],
                self.img_stats["right"]["std"],
            ),
        }

        # --- Process Robot State ---
        # Construct flat state vector (26 dims) matching training order
        tcp_pose = obs_msg.controller_state.tcp_pose
        tcp_vel = obs_msg.controller_state.tcp_velocity

        state_np = np.array(
            [
                # TCP Position (3)
                tcp_pose.position.x,
                tcp_pose.position.y,
                tcp_pose.position.z,
                # TCP Orientation (4)
                tcp_pose.orientation.x,
                tcp_pose.orientation.y,
                tcp_pose.orientation.z,
                tcp_pose.orientation.w,
                # TCP Linear Vel (3)
                tcp_vel.linear.x,
                tcp_vel.linear.y,
                tcp_vel.linear.z,
                # TCP Angular Vel (3)
                tcp_vel.angular.x,
                tcp_vel.angular.y,
                tcp_vel.angular.z,
                # TCP Error (6)
                *obs_msg.controller_state.tcp_error,
                # Joint Positions (7)
                *obs_msg.joint_states.position[:7],
            ],
            dtype=np.float32,
        )

        # Normalize State
        raw_state_tensor = (
            torch.from_numpy(state_np).float().unsqueeze(0).to(self.device)
        )
        obs["observation.state"] = (raw_state_tensor - self.state_mean) / self.state_std

        return obs

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        """Sim-time control loop with bounded position-mode integration.

        Architecture:
        - All time math uses sim time via Policy.time_now() / Policy.sleep_for(),
          so behavior is consistent across RTF (GUI 1.0× vs headless faster).
        - Position targets are integrated from the LAST COMMANDED TARGET (not from
          observed TCP), then clamped so |target - obs.tcp_pose| <= MAX_TARGET_OFFSET_M.
          This prevents the controller's reference from drifting below the actual
          TCP under sustained "go down" commands in contact.
        - Workspace z floor prevents target z from going below start_z - 0.30 m.
        - Force-aware backoff overrides the policy with upward motion when wrist
          force exceeds 10 N for >100 ms, then resets the anchor.
        - Gripper gravity is already compensated by aic_controller's
          gravity_compensation_action; no feedforward needed in MotionUpdate.
        """
        self.policy.reset()
        self.get_logger().info(f"RunACT.insert_cable() enter. Task: {task}")

        # Wait up to 5 sim-seconds for the first observation.
        start_t = self.time_now()
        observation_msg = get_observation()
        while observation_msg is None:
            if (self.time_now() - start_t).nanoseconds * 1e-9 > 5.0:
                self.get_logger().error("No observation in 5s; aborting trial")
                return False
            self.sleep_for(self.LOOP_DT)
            observation_msg = get_observation()

        # Anchor integration at the first observed TCP pose.
        last_target_pose = observation_msg.controller_state.tcp_pose
        start_z = last_target_pose.position.z
        z_floor = start_z - self.WORKSPACE_Z_BELOW_START

        # Calibrate backoff force threshold to (baseline + delta). The wrist FT
        # sensor reads ~20 N at trial start due to gripper + cable + plug static
        # weight; an absolute threshold below that false-fires every tick.
        baseline_samples = []
        for _ in range(self.FORCE_BASELINE_SAMPLES):
            obs = get_observation()
            if obs is not None:
                w = obs.wrist_wrench.wrench
                fmag = float(
                    np.sqrt(
                        w.force.x * w.force.x
                        + w.force.y * w.force.y
                        + w.force.z * w.force.z
                    )
                )
                baseline_samples.append(fmag)
            self.sleep_for(self.LOOP_DT)
        baseline_force = (
            float(np.mean(baseline_samples)) if baseline_samples else 0.0
        )
        backoff_threshold_n = baseline_force + self.BACKOFF_FORCE_DELTA_N

        # Backoff state.
        force_above_since = None  # Time.now value or None
        backoff_remaining_ticks = 0
        backoff_total_used_s = 0.0
        backoff_started_at = None

        self.get_logger().info(
            f"start_z={start_z:.4f} z_floor={z_floor:.4f} "
            f"loop_dt={self.LOOP_DT} action_scale={self.ACTION_SCALE} "
            f"baseline_force={baseline_force:.1f}N "
            f"backoff_threshold={backoff_threshold_n:.1f}N"
        )

        trial_start = self.time_now()
        loop_count = 0
        peak_force = 0.0

        while (self.time_now() - trial_start).nanoseconds * 1e-9 < 30.0:
            loop_start = self.time_now()
            observation_msg = get_observation()
            if observation_msg is None:
                self.sleep_for(self.LOOP_DT)
                continue

            # Inference.
            obs_tensors = self.prepare_observations(observation_msg)
            with torch.inference_mode():
                normalized_action = self.policy.select_action(obs_tensors)
            raw_action_tensor = (normalized_action * self.action_std) + self.action_mean
            scaled_action_tensor = raw_action_tensor * self.ACTION_SCALE
            raw_action = raw_action_tensor[0].cpu().numpy()
            action = scaled_action_tensor[0].cpu().numpy()

            # Force-aware backoff state machine.
            wrench = observation_msg.wrist_wrench.wrench
            force_mag = float(
                np.sqrt(
                    wrench.force.x * wrench.force.x
                    + wrench.force.y * wrench.force.y
                    + wrench.force.z * wrench.force.z
                )
            )
            peak_force = max(peak_force, force_mag)
            sim_now = self.time_now()
            mode_label = "POS"

            if backoff_remaining_ticks > 0:
                action_used = np.array(
                    [0.0, 0.0, self.BACKOFF_RECOVERY_VZ, 0.0, 0.0, 0.0],
                    dtype=np.float64,
                )
                backoff_remaining_ticks -= 1
                mode_label = "BACKOFF"
                if backoff_remaining_ticks == 0:
                    # End of recovery: re-anchor and account for time used.
                    last_target_pose = observation_msg.controller_state.tcp_pose
                    if backoff_started_at is not None:
                        backoff_total_used_s += (
                            sim_now - backoff_started_at
                        ).nanoseconds * 1e-9
                        backoff_started_at = None
                    force_above_since = None
                    self.get_logger().info(
                        f"backoff ended; total_used_s={backoff_total_used_s:.2f}"
                    )
            else:
                # Default: use the policy action. Override below if backoff fires.
                action_used = action[:6].copy()
                if force_mag > backoff_threshold_n:
                    if force_above_since is None:
                        force_above_since = sim_now
                    else:
                        dur = (sim_now - force_above_since).nanoseconds * 1e-9
                        if (
                            dur > self.BACKOFF_TRIGGER_DURATION_S
                            and backoff_total_used_s < self.BACKOFF_TOTAL_BUDGET_S
                        ):
                            backoff_remaining_ticks = self.BACKOFF_RECOVERY_TICKS
                            backoff_started_at = sim_now
                            self.get_logger().warn(
                                f"BACKOFF triggered: force={force_mag:.1f}N for {dur:.2f}s"
                            )
                            action_used = np.array(
                                [0.0, 0.0, self.BACKOFF_RECOVERY_VZ, 0.0, 0.0, 0.0],
                                dtype=np.float64,
                            )
                            mode_label = "BACKOFF"
                else:
                    force_above_since = None

            # Build & publish target.
            if self.USE_POSITION_MODE:
                # Integrate from LAST COMMANDED TARGET (not from observed TCP) so
                # the controller's reference doesn't drift below actual TCP in contact.
                proposed_target = self._integrate_action_to_pose(
                    last_target_pose, action_used, self.LOOP_DT
                )
                # Clamp |target − measured TCP| to bound impedance spring force.
                clamped_target = self._clamp_pose_offset(
                    proposed_target,
                    observation_msg.controller_state.tcp_pose,
                    self.MAX_TARGET_OFFSET_M,
                    self.MAX_TARGET_OFFSET_RAD,
                )
                # Workspace z floor.
                if clamped_target.position.z < z_floor:
                    clamped_target = Pose(
                        position=Point(
                            x=clamped_target.position.x,
                            y=clamped_target.position.y,
                            z=z_floor,
                        ),
                        orientation=clamped_target.orientation,
                    )
                last_target_pose = clamped_target
                self.set_pose_target(move_robot=move_robot, pose=clamped_target)

                tcp = observation_msg.controller_state.tcp_pose.position
                tgt = clamped_target.position
                dist_t2a = float(
                    np.sqrt(
                        (tcp.x - tgt.x) ** 2
                        + (tcp.y - tgt.y) ** 2
                        + (tcp.z - tgt.z) ** 2
                    )
                )
            else:
                twist = Twist(
                    linear=Vector3(
                        x=float(action_used[0]),
                        y=float(action_used[1]),
                        z=float(action_used[2]),
                    ),
                    angular=Vector3(
                        x=float(action_used[3]),
                        y=float(action_used[4]),
                        z=float(action_used[5]),
                    ),
                )
                motion_update = self.set_cartesian_twist_target(twist)
                move_robot(motion_update=motion_update)
                mode_label = "VEL"
                dist_t2a = -1.0

            send_feedback("in progress...")

            if loop_count % 10 == 0:
                self.get_logger().info(
                    f"t={loop_count} raw={raw_action[:6].round(4).tolist()} "
                    f"act={action_used.round(4).tolist()} "
                    f"F={force_mag:.1f}N peak={peak_force:.1f}N "
                    f"d_t2a={dist_t2a*1000:.1f}mm mode={mode_label}"
                )
            loop_count += 1

            # Sim-time pacing.
            elapsed_s = (self.time_now() - loop_start).nanoseconds * 1e-9
            sleep_remaining = self.LOOP_DT - elapsed_s
            if sleep_remaining > 0:
                self.sleep_for(sleep_remaining)

        self.get_logger().info(
            f"RunACT.insert_cable() exiting; loops={loop_count} peak_force={peak_force:.1f}N "
            f"backoff_used={backoff_total_used_s:.2f}s"
        )
        return True

    @staticmethod
    def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Hamilton product, both inputs (w, x, y, z)."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ]
        )

    @staticmethod
    def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
        """Convert a unit quaternion (w, x, y, z) to a 3x3 rotation matrix R
        such that for a vector v in TCP frame, R @ v gives v in base_link."""
        w, x, y, z = q
        n = w * w + x * x + y * y + z * z
        if n < 1e-12:
            return np.eye(3)
        s = 2.0 / n
        return np.array(
            [
                [1.0 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
                [s * (x * y + w * z), 1.0 - s * (x * x + z * z), s * (y * z - w * x)],
                [s * (x * z - w * y), s * (y * z + w * x), 1.0 - s * (x * x + y * y)],
            ]
        )

    def _clamp_pose_offset(
        self,
        target: Pose,
        reference: Pose,
        max_pos_offset: float,
        max_rot_rad: float,
    ) -> Pose:
        """Clamp the offset between target and reference so |target - reference|
        is bounded. Caps impedance spring force at stiffness * max_pos_offset.

        Linear: scale offset vector to length max_pos_offset if it exceeds.
        Angular: if the rotation between reference and target exceeds max_rot_rad,
        slerp from reference toward target by max_rot_rad / full_angle.
        """
        # Linear clamp.
        dx = target.position.x - reference.position.x
        dy = target.position.y - reference.position.y
        dz = target.position.z - reference.position.z
        dist = float(np.sqrt(dx * dx + dy * dy + dz * dz))
        if dist > max_pos_offset and dist > 1e-9:
            scale = max_pos_offset / dist
            out_x = reference.position.x + dx * scale
            out_y = reference.position.y + dy * scale
            out_z = reference.position.z + dz * scale
        else:
            out_x = target.position.x
            out_y = target.position.y
            out_z = target.position.z

        # Angular clamp via quaternion delta angle.
        q_t = np.array(
            [
                target.orientation.w,
                target.orientation.x,
                target.orientation.y,
                target.orientation.z,
            ],
            dtype=np.float64,
        )
        q_r = np.array(
            [
                reference.orientation.w,
                reference.orientation.x,
                reference.orientation.y,
                reference.orientation.z,
            ],
            dtype=np.float64,
        )
        # cos(half_angle) = |q_r · q_t|, robust to sign.
        dot = float(np.clip(abs(np.dot(q_r, q_t)), 0.0, 1.0))
        full_angle = 2.0 * float(np.arccos(dot))
        if full_angle > max_rot_rad and full_angle > 1e-9:
            t = max_rot_rad / full_angle
            # Linear-interp slerp approximation (exact small-angle).
            # Ensure shortest-path by flipping q_t if dot product was negative.
            if np.dot(q_r, q_t) < 0.0:
                q_t = -q_t
            q_out = (1.0 - t) * q_r + t * q_t
            q_out = q_out / max(float(np.linalg.norm(q_out)), 1e-9)
        else:
            q_out = q_t

        return Pose(
            position=Point(x=float(out_x), y=float(out_y), z=float(out_z)),
            orientation=Quaternion(
                w=float(q_out[0]),
                x=float(q_out[1]),
                y=float(q_out[2]),
                z=float(q_out[3]),
            ),
        )

    def _integrate_action_to_pose(
        self, current_pose: Pose, action6: np.ndarray, dt: float
    ) -> Pose:
        """Treat action[:3] as linear velocity (m/s) and action[3:6] as angular
        velocity (rad/s) in base frame. Integrate over dt, compose onto current
        TCP pose, and return the resulting target Pose."""
        # Linear: target = current + v * dt
        target_x = current_pose.position.x + float(action6[0]) * dt
        target_y = current_pose.position.y + float(action6[1]) * dt
        target_z = current_pose.position.z + float(action6[2]) * dt

        # Angular: rotation vector = omega * dt -> quaternion -> compose
        omega_dt = np.asarray(action6[3:6], dtype=np.float64) * dt
        angle = float(np.linalg.norm(omega_dt))
        if angle < 1e-9:
            q_delta = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            axis = omega_dt / angle
            half = angle / 2.0
            q_delta = np.array(
                [np.cos(half), *(axis * np.sin(half))], dtype=np.float64
            )
        q_curr = np.array(
            [
                current_pose.orientation.w,
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
            ],
            dtype=np.float64,
        )
        # World-frame angular velocity → left-multiply
        q_new = self._quat_mul(q_delta, q_curr)
        # Normalize defensively
        q_new = q_new / max(np.linalg.norm(q_new), 1e-9)

        return Pose(
            position=Point(x=float(target_x), y=float(target_y), z=float(target_z)),
            orientation=Quaternion(
                w=float(q_new[0]),
                x=float(q_new[1]),
                y=float(q_new[2]),
                z=float(q_new[3]),
            ),
        )

    def set_cartesian_twist_target(self, twist: Twist, frame_id: str = "base_link"):
        motion_update_msg = MotionUpdate()
        motion_update_msg.velocity = twist
        motion_update_msg.header.frame_id = frame_id
        motion_update_msg.header.stamp = self.get_clock().now().to_msg()

        motion_update_msg.target_stiffness = np.diag(
            [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        ).flatten()
        motion_update_msg.target_damping = np.diag(
            [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        ).flatten()

        motion_update_msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.0)
        )

        motion_update_msg.wrench_feedback_gains_at_tip = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]

        motion_update_msg.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_VELOCITY
        )

        return motion_update_msg
