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
import torchvision.transforms as T
from lerobot.policies.factory import PreTrainedPolicy
from lerobot.policies.act.modeling_act import ACTPolicy

from aic_model.policy_ros import PolicyRos
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration
from typing import Callable, Dict, Any, TypedDict
import numpy as np

ControllerObservationState = TypedDict(
    "ControllerObservationState",
    {
        "tcp_pose.position.x": float,
        "tcp_pose.position.y": float,
        "tcp_pose.position.z": float,
        "tcp_pose.orientation.x": float,
        "tcp_pose.orientation.y": float,
        "tcp_pose.orientation.z": float,
        "tcp_pose.orientation.w": float,
        "tcp_velocity.linear.x": float,
        "tcp_velocity.linear.y": float,
        "tcp_velocity.linear.z": float,
        "tcp_velocity.angular.x": float,
        "tcp_velocity.angular.y": float,
        "tcp_velocity.angular.z": float,
        "tcp_error.x": float,
        "tcp_error.y": float,
        "tcp_error.z": float,
        "tcp_error.rx": float,
        "tcp_error.ry": float,
        "tcp_error.rz": float,
        "joint_positions.0": float,
        "joint_positions.1": float,
        "joint_positions.2": float,
        "joint_positions.3": float,
        "joint_positions.4": float,
        "joint_positions.5": float,
    },
)

CameraObservationState = TypedDict(
    "CameraObservationState",
    {
        "left_image": np.ndarray,
        "center_image": np.ndarray,
        "right_image": np.ndarray,
    },
)

# from aic_utils.lerobot_robot_aic.lerobot_robot_aic.types import ControllerObservationState, CameraObservationState

class RunACT(PolicyRos):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("RunACT.__init__()")

        policy_path = "/home/aic/ws_aic/outputs/grkw/random_start_poses_10_eps"

        # Load the policy (this automatically detects it's an ACT policy)
        self.policy = ACTPolicy.from_pretrained(policy_path)
        self.get_logger().info(f"Policy config: {self.policy.config}")
        # Move to GPU/CPU and set to evaluation mode
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy.to(self.device)
        self.policy.eval()
        self.policy.reset() # clear temporal aggregation
        self.get_logger().info(f"Loaded ACT policy from {policy_path}")

        # 1. Define the transformation (Match your training config!)
        # Usually ACT uses 224x224 or 480x640
        self.inference_tf = T.Compose([
            T.ToTensor(),  # Converts to [0, 1] and (C, H, W)
        ])

    
    def run_inference(self, observation: Dict[str, Any]):
        """
        Example function to run inference with the loaded policy.
        The exact input format (observation) depends on your dataset structure.
        """
        with torch.no_grad():
            # Pre-process the observation if necessary (policy handles this internally)
            # Assuming observation is a dictionary of tensors/numpy arrays on the correct device
            
            # Forward pass to get actions
            actions = self.policy.select_action(observation)
            
            # Post-process actions if needed (policy handles this internally)
            # e.g., denormalize actions
            return actions

    def prepare_camera_observation(self, raw_cam_image):
        # Apply resizing and tensor conversion

        # Manual conversion if encoding is already rgb8
        img_np = np.frombuffer(raw_cam_image.data, dtype=np.uint8).reshape(raw_cam_image.height, raw_cam_image.width, 3)
        img_tensor = torch.from_numpy(img_np).permute(2, 0, 1).float() / 255.0
        return img_tensor.unsqueeze(0).to(self.device)
        # State usually just needs tensor conversion
        # state_tensor = torch.from_numpy(raw_robot_state).float().unsqueeze(0).to(device)
        
        return img_tensor

    def insert_cable(
        self,
        task: Task,
        get_observation: Callable[[], Observation],
        set_pose_target: Callable[[Pose, str], []],
        set_joint_target: Callable[[list[float]], []],
        send_feedback: Callable[[str], []],
    ):
        self.get_logger().info(f"RunACT.insert_cable() enter. Task: {task}")
        start_time = time.clock_gettime(0)
        send_feedback("waving the arm around")
        while time.clock_gettime(0) - start_time < 10.0:
            time.sleep(0.25)
            observation_msg = get_observation()
            # controller_observation_state: ControllerObservationState = {
            # "tcp_pose.position.x": 0.0,
            # "tcp_pose.position.y": 0.0,
            # "tcp_pose.position.z": 0.0,
            # "tcp_pose.orientation.x": 0.0,
            # "tcp_pose.orientation.y": 0.0,
            # "tcp_pose.orientation.z": 0.0,
            # "tcp_pose.orientation.w": 0.0,
            # "tcp_velocity.linear.x": 0.0,
            # "tcp_velocity.linear.y": 0.0,
            # "tcp_velocity.linear.z": 0.0,
            # "tcp_velocity.angular.x": 0.0,
            # "tcp_velocity.angular.y": 0.0,
            # "tcp_velocity.angular.z": 0.0,
            # "tcp_error.x": 0.0,
            # "tcp_error.y": 0.0,
            # "tcp_error.z": 0.0,
            # "tcp_error.rx": 0.0,
            # "tcp_error.ry": 0.0,
            # "tcp_error.rz": 0.0,
            # "joint_positions.0": observation_msg.joint_states.position[0],
            # "joint_positions.1": observation_msg.joint_states.position[1],
            # "joint_positions.2": observation_msg.joint_states.position[2],
            # "joint_positions.3": observation_msg.joint_states.position[3],
            # "joint_positions.4": observation_msg.joint_states.position[4],
            # "joint_positions.5": observation_msg.joint_states.position[5],
            # "joint_positions.6": observation_msg.joint_states.position[6],
            # }
            controller_observation_state = torch.from_numpy(np.zeros(26))
            controller_observation_state[19] = observation_msg.joint_states.position[0]
            controller_observation_state[20] = observation_msg.joint_states.position[1]
            controller_observation_state[21] = observation_msg.joint_states.position[2]
            controller_observation_state[22] = observation_msg.joint_states.position[3]
            controller_observation_state[23] = observation_msg.joint_states.position[4]
            controller_observation_state[24] = observation_msg.joint_states.position[5]
            controller_observation_state[25] = observation_msg.joint_states.position[6]
            controller_observation_state = {"observation.state": controller_observation_state.float().unsqueeze(0).to(self.device)}

            camera_observation_state: CameraObservationState = {
                "observation.images.left_camera": self.prepare_camera_observation(observation_msg.left_image),
                "observation.images.center_camera": self.prepare_camera_observation(observation_msg.center_image),
                "observation.images.right_camera": self.prepare_camera_observation(observation_msg.right_image),
            }

            observation_state = {**camera_observation_state, **controller_observation_state}

            for key in observation_state:
                if isinstance(observation_state[key], torch.Tensor):
                    observation_state[key] = observation_state[key].to(self.device)
            self.get_logger().info(f"Observation state keys: {observation_state.keys()}")
            actions = self.run_inference(observation_state)
            self.get_logger().info(f"Actions: {actions}")

            set_joint_target(actions)

        self.get_logger().info("RunACT.insert_cable() exiting...")
        return True
