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

import time
import torch
import numpy as np
from typing import Callable, Dict, Any
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

from aic_model.policy_ros import PolicyRos
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from lerobot.policies.act.modeling_act import ACTPolicy


class RunACT(PolicyRos):
    def __init__(self, parent_node: Node):
        super().__init__(parent_node)

        policy_path = "/home/aic/ws_aic/outputs/grkw/random_start_poses_10_eps"  # TODO (@grkw): change this to public HF repo once it's released

        # Load Policy
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy = ACTPolicy.from_pretrained(policy_path).to(self.device)
        self.policy.eval()

        self.get_logger().info(f"ACT Policy loaded on {self.device} from {policy_path}")

    @staticmethod
    def _img_to_tensor(raw_img, device: torch.device) -> torch.Tensor:
        """Helper to convert ROS Image msg to normalized PyTorch tensor."""
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        # Permute to (C, H, W) and normalize
        return (
            torch.from_numpy(img_np)
            .permute(2, 0, 1)
            .float()
            .div(255.0)
            .unsqueeze(0)
            .to(device)
        )

    def prepare_observations(self, obs_msg: Observation) -> Dict[str, torch.Tensor]:
        """Process camera and controller state for inference, i.e. prepare an observation to send to the policy. The `obs` dict format should match that of `AICRobotAICController` `get_observation()` used for recording episodes."""
        # Process Cameras
        obs = {
            "observation.images.left_camera": self._img_to_tensor(
                obs_msg.left_image, self.device
            ),
            "observation.images.center_camera": self._img_to_tensor(
                obs_msg.center_image, self.device
            ),
            "observation.images.right_camera": self._img_to_tensor(
                obs_msg.right_image, self.device
            ),
        }

        # Process Controller States (The example policy uses TCP pose, TCP velocity, TCP error, and joint positions. See `AICRobotAICController` `get_observation()`.)
        state_np = np.zeros(26, dtype=np.float32)

        tcp_pose = obs_msg.controller_state.tcp_pose
        tcp_velocity = obs_msg.controller_state.tcp_velocity
        state_np[0:7] = [tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z, tcp_pose.orientation.x, tcp_pose.orientation.y, tcp_pose.orientation.z, tcp_pose.orientation.w]
        state_np[7:13] = [tcp_velocity.linear.x, tcp_velocity.linear.y, tcp_velocity.linear.z, tcp_velocity.angular.x, tcp_velocity.angular.y, tcp_velocity.angular.z]
        state_np[13:19] = obs_msg.controller_state.tcp_error
        state_np[19:26] = obs_msg.joint_states.position[
            0:7
        ]
        obs["observation.state"] = (
            torch.from_numpy(state_np).float().unsqueeze(0).to(self.device)
        )

        return obs

    def insert_cable(
        self,
        task: Task,
        get_observation: Callable[[], Observation],
        set_cartesian_twist_target: Callable[[Twist], None],
        send_feedback: Callable[[str], None],
        **kwargs,  # Capture unused callbacks
    ):
        self.policy.reset()  # Clear ACT temporal aggregation
        self.get_logger().info(f"RunACT.insert_cable() enter. Task: {task}")
        start_time = time.clock_gettime(0)

        while time.clock_gettime(0) - start_time < 10.0:
            time.sleep(0.25)
            observation_msg = get_observation()

            obs_tensors = self.prepare_observations(observation_msg)
            with torch.inference_mode():
                actions_tensor = self.policy.select_action(obs_tensors)

            # Action conversion (take the first action in the list)
            actions = actions_tensor[0].to("cpu").numpy()
            self.get_logger().info(f"Actions: {actions}")

            # Command robot
            twist = Twist(
                linear=Vector3(
                    x=float(actions[0]), y=float(actions[1]), z=float(actions[2])
                ),
                angular=Vector3(
                    x=float(actions[3]), y=float(actions[4]), z=float(actions[5])
                ),
            )
            set_cartesian_twist_target(twist)

            send_feedback("in progress...")

        self.get_logger().info("RunACT.insert_cable() exiting...")
        return True
