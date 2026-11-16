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

"""ACT policy runner. Heavy imports run in __init__ to keep module discovery fast."""

from pathlib import Path
from typing import Any, Dict

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from rclpy.node import Node

class RunACT(Policy):
    """Loads ACT weights in ``__init__``; avoids top-level torch/lerobot imports."""

    _SIM_TIMEOUT_SEC = 5.0

    @staticmethod
    def _is_act_policy_checkpoint_dir(d: Path) -> bool:
        return (d / "config.json").is_file() and (d / "model.safetensors").is_file()

    def _resolve_local_act_policy_dir(self) -> Path:
        """Resolve ACT checkpoint in dev/install/container layouts."""
        here = Path(__file__).resolve()
        for d in here.parents:
            for candidate in (
                d / "resource" / "aic_act_policy",
                d / "aic_example_policies" / "resource" / "aic_act_policy",
            ):
                if self._is_act_policy_checkpoint_dir(candidate):
                    self.get_logger().info(f"Using local ACT policy: {candidate}")
                    return candidate

        raise FileNotFoundError(
            "No local ACT policy checkpoint under any ancestor of "
            f"{here} (expected .../resource/aic_act_policy or "
            ".../aic_example_policies/resource/aic_act_policy with config.json and model.safetensors)"
        )

    def _load_policy_processors(self, policy_path: Path) -> None:
        """Load LeRobot pre/post processors from checkpoint."""
        import torch

        pre_cfg = policy_path / "policy_preprocessor.json"
        post_cfg = policy_path / "policy_postprocessor.json"
        processor_overrides = {}
        if not torch.cuda.is_available():
            processor_overrides = {"device_processor": {"device": "cpu"}}

        from lerobot.processor.pipeline import PolicyProcessorPipeline

        self.preprocessor = PolicyProcessorPipeline.from_pretrained(
            pretrained_model_name_or_path=policy_path,
            config_filename=pre_cfg.name,
            overrides=processor_overrides,
        )
        self.postprocessor = PolicyProcessorPipeline.from_pretrained(
            pretrained_model_name_or_path=policy_path,
            config_filename=post_cfg.name,
            overrides=processor_overrides,
        )
        self.get_logger().info(
            "Loaded policy pre/post processors from checkpoint configs."
        )

    def __init__(self, parent_node: Node):
        super().__init__(parent_node)

        import json

        import draccus
        import torch
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from safetensors.torch import load_file

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # -------------------------------------------------------------------------
        # 1. Configuration & Weights Loading
        # -------------------------------------------------------------------------
        policy_path = self._resolve_local_act_policy_dir()

        # Torchvision ResNet etc. load via torch.hub; use vendored weights under resource/hub/.
        hub_dir = policy_path.parent / "hub"
        if hub_dir.is_dir():
            torch.hub.set_dir(str(hub_dir.resolve()))
            self.get_logger().info(f"torch.hub.set_dir({hub_dir}) (offline vision backbone)")

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

        # Config
        self.image_scaling = 0.25  # Must match AICRobotAICControllerConfig

        self._load_policy_processors(policy_path)
        self.get_logger().info("Using policy processors for normalization.")

    def _img_to_tensor(
        self,
        raw_img,
    ):
        """Converts ROS Image -> Resized -> Permuted -> Float Tensor."""
        import cv2
        import numpy as np
        import torch

        device = self.device
        scale = self.image_scaling

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

        return tensor

    def prepare_observations(self, obs_msg: Observation) -> Dict[str, Any]:
        """Convert ROS Observation message into dictionary of normalized tensors."""
        import torch

        # --- Process Cameras ---
        obs = {
            "observation.images.left_camera": self._img_to_tensor(
                obs_msg.left_image,
            ),
            "observation.images.center_camera": self._img_to_tensor(
                obs_msg.center_image,
            ),
            "observation.images.right_camera": self._img_to_tensor(
                obs_msg.right_image,
            ),
        }

        # --- Process Robot State ---
        # Construct flat state vector (26 dims) matching training order
        tcp_pose = obs_msg.controller_state.tcp_pose
        tcp_vel = obs_msg.controller_state.tcp_velocity

        state_values = [
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
        ]

        obs["observation.state"] = (
            torch.tensor(
                state_values,
                dtype=torch.float32,
                device=self.device,
            )
            .unsqueeze(0)
        )

        obs = self.preprocessor.process_observation(obs)

        return obs

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        import time

        import torch
        from geometry_msgs.msg import Twist, Vector3
        from rclpy.duration import Duration

        self.policy.reset()
        self.get_logger().info(f"RunACT.insert_cable() enter. Task: {task}")

        clock = self.get_clock()
        start = clock.now()
        deadline = start + Duration(seconds=self._SIM_TIMEOUT_SEC)

        while clock.now() < deadline:
            loop_start = time.time()

            # 1. Get & Process Observation
            observation_msg = get_observation()

            if observation_msg is None:
                self.get_logger().info("No observation received.")
                continue

            obs_tensors = self.prepare_observations(observation_msg)

            # 2. Model Inference
            with torch.inference_mode():
                # returns shape [1, 7] (first action of chunk)
                normalized_action = self.policy.select_action(obs_tensors)

            # 3. Un-normalize Action
            raw_action_tensor = self.postprocessor.process_action(normalized_action)

            # 4. Extract and Command
            # raw_action_tensor is [1, 7], taking [0] gives vector of 7
            if isinstance(raw_action_tensor, dict):
                if "action" in raw_action_tensor:
                    raw_action_tensor = raw_action_tensor["action"]
                else:
                    raw_action_tensor = next(iter(raw_action_tensor.values()))
            action = raw_action_tensor[0].cpu().numpy()

            self.get_logger().info(f"Action: {action}")

            twist = Twist(
                linear=Vector3(
                    x=float(action[0]), y=float(action[1]), z=float(action[2])
                ),
                angular=Vector3(
                    x=float(action[3]), y=float(action[4]), z=float(action[5])
                ),
            )
            motion_update = self.set_cartesian_twist_target(twist)
            move_robot(motion_update=motion_update)
            send_feedback("in progress...")

            # Maintain control rate (approx 4Hz loop = 0.25s sleep)
            elapsed = time.time() - loop_start
            time.sleep(max(0, 0.25 - elapsed))

        self.get_logger().info(
            f"RunACT.insert_cable(): sim timeout after {self._SIM_TIMEOUT_SEC}s"
        )
        return True

    def set_cartesian_twist_target(self, twist, frame_id: str = "base_link"):
        import numpy as np

        from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
        from geometry_msgs.msg import Vector3, Wrench

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
