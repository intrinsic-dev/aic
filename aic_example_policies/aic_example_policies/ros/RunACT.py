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
from lerobot.policies.act.modeling_act import ACTPolicy

from aic_model.policy_ros import PolicyRos
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rclpy.duration import Duration
from typing import Callable, Dict, Any, TypedDict
import numpy as np

class RunACT(PolicyRos):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("RunACT.__init__()")

        policy_path = "/home/aic/ws_aic/outputs/grkw/random_start_poses_10_eps"

        # Load the policy
        self.policy = ACTPolicy.from_pretrained(policy_path)
        self.get_logger().info(f"Policy config: {self.policy.config}")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy.to(self.device)
        self.policy.eval()
        self.get_logger().info(f"Loaded ACT policy from {policy_path}")

        self.inference_tf = T.Compose([
            T.ToTensor(),  # Converts to [0, 1] and (C, H, W)
        ])

    
    def run_inference(self, observation: Dict[str, Any]):

        with torch.no_grad():
            actions = self.policy.select_action(observation)
            return actions

    def prepare_camera_observation(self, left_image, center_image, right_image):
        # Apply resizing and tensor conversion
        def prepare_image(raw_cam_image):
            img_np = np.frombuffer(raw_cam_image.data, dtype=np.uint8).reshape(raw_cam_image.height, raw_cam_image.width, 3)
            img_tensor = torch.from_numpy(img_np).permute(2, 0, 1).float() / 255.0
            img_obs = img_tensor.unsqueeze(0).to(self.device)
            return img_obs
        
        camera_observation_state = {
                "observation.images.left_camera": prepare_image(left_image),
                "observation.images.center_camera": prepare_image(center_image),
                "observation.images.right_camera": prepare_image(right_image),
            }
        
        return camera_observation_state
    
    def prepare_controller_observation(self, joint_states):
        controller_observation_state = np.zeros(26, dtype=np.float32)
        controller_observation_state[19:26] = joint_states
        state_tensor = torch.from_numpy(controller_observation_state).float().to(self.device)
        state_tensor = state_tensor.unsqueeze(0) 
        controller_observation_state = {"observation.state": state_tensor}
        return controller_observation_state
    
    def insert_cable(
        self,
        task: Task,
        get_observation: Callable[[], Observation],
        set_cartesian_target: Callable[[Pose, str], []],
        set_cartesian_twist_target: Callable[[Twist, str], []],
        set_joint_target: Callable[[list[float]], []],
        send_feedback: Callable[[str], []],
    ):
        self.policy.reset() # clear ACT temporal aggregation
        self.get_logger().info(f"RunACT.insert_cable() enter. Task: {task}")
        start_time = time.clock_gettime(0)
        
        send_feedback("some feedback")
        
        while time.clock_gettime(0) - start_time < 10.0:
            time.sleep(0.25)
            observation_msg = get_observation()

            controller_observation_state = self.prepare_controller_observation(observation_msg.joint_states.position[0:7])
            camera_observation_state = self.prepare_camera_observation(observation_msg.left_image, observation_msg.center_image, observation_msg.right_image)

            observation_state = {**camera_observation_state, **controller_observation_state}

            self.get_logger().info(f"Observation state keys: {observation_state.keys()}")

            actions = self.run_inference(observation_state)
            actions = actions.to('cpu').tolist()[0]
            self.get_logger().info(f"Actions: {actions}")
            
            twist = Twist(
                linear=Vector3(x=actions[0], y=actions[1], z=actions[2]),
                angular=Vector3(x=actions[3], y=actions[4], z=actions[5]),
            )
            set_cartesian_twist_target(twist)

        self.get_logger().info("RunACT.insert_cable() exiting...")
        return True
