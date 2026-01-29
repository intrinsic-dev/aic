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


from abc import ABC, abstractmethod
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Wrench, Vector3
import numpy as np


class PolicyRos(ABC):
    def __init__(self, parent_node):
        self._parent_node = parent_node
        self.get_logger().info("PolicyRos.__init__()")

    def set_pose_target(self, pose: Pose, frame_id: str = "base_link"):
        """Set a pose target for the robot arm.

        The robot can be controlled in several different ways. This function
        is intended to be the simplest way to move the arm around, by sending
        a desired pose (position and orientation) for the gripper's
        "tool control point" (TCP), which is the "pinch point" between the very
        end of the gripper fingers. The rest of the control stack will take care
        of moving all the arm's joints to so that the gripper TCP ends up in
        the desired position and orientation.

        The constants defined in this function are intended to provide
        reasonable default behavior if the arm is unable to achieve the
        requested pose. Different values for stiffness, damping, wrenches, and
        so on can be used for different types of arm behavior. These values
        are only intended to provide a starting point, and can be adjusted as
        desired.
        """

        motion_update_msg = MotionUpdate()
        motion_update_msg.pose = pose
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

        motion_update_msg.wrench_feedback_gains_at_tip = Wrench(
            force=Vector3(x=0.5, y=0.5, z=0.5), torque=Vector3(x=0.0, y=0.0, z=0.0)
        )

        motion_update_msg.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_POSITION
        )

        self._parent_node.motion_update_pub.publish(motion_update_msg)

    @abstractmethod
    def get_feedback_string(self) -> str:
        """Returns a string with the insert cable action's feedback"""
        pass

    @abstractmethod
    def start_callback(self, task: Task):
        """Called when the insert cable action is started"""
        pass

    @abstractmethod
    def stop_callback(self):
        """Called when the insert cable action is stopped"""
        pass

    @abstractmethod
    def goal_completed(self) -> bool:
        """Returns whether the goal is considered completed by the policy"""
        pass

    @abstractmethod
    def observation_callback(self, observation: Observation):
        """Called whenever a new observation is received"""
        pass

    def get_logger(self):
        return self._parent_node.get_logger()

    def get_clock(self):
        return self._parent_node.get_clock()
