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

"""LeRobot driver for AIC robot"""

import logging
from dataclasses import dataclass, field
from math import pi
from typing import cast

from control_msgs.action import ParallelGripperCommand
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot_robot_ros import ROS2Config, ROS2Robot
from lerobot_robot_ros.config import ActionType, ROS2InterfaceConfig
from lerobot_robot_ros.robot import ActionType, ROS2Interface
from rclpy.action.client import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from .aic_robot import arm_joint_names, gripper_joint_name, aic_cameras

logger = logging.getLogger(__name__)


class AICROS2ControlInterface(ROS2Interface):
    def __init__(self, config: ROS2InterfaceConfig, action_type: ActionType):
        super().__init__(config, action_type)
        self.parallel_gripper_action_client: (
            ActionClient[
                ParallelGripperCommand.Goal,
                ParallelGripperCommand.Result,
                ParallelGripperCommand.Feedback,
            ]
            | None
        ) = None

    def connect(self) -> None:
        super().connect()
        # robot_node is guaranteed to be created in super().connect()
        robot_node = cast(Node, self.robot_node)
        self.parallel_gripper_action_client = ActionClient(
            robot_node,
            ParallelGripperCommand,
            self.config.gripper_action_name,
            callback_group=ReentrantCallbackGroup(),
        )

    def send_gripper_command(self, position: float, unnormalize: bool = True) -> bool:
        """
        Override to send gripper command using ParallelGripperCommand action.
        """
        if not self.robot_node:
            raise DeviceNotConnectedError(
                "ROS2Interface is not connected. You need to call `connect()`."
            )

        if unnormalize:
            # Map normalized position (0=open, 1=closed) to actual gripper joint position
            open_pos = self.config.gripper_open_position
            closed_pos = self.config.gripper_close_position
            gripper_goal = open_pos + position * (closed_pos - open_pos)
        else:
            gripper_goal = position

        if not self.parallel_gripper_action_client:
            raise DeviceNotConnectedError(
                "Parallel gripper action client is not initialized."
            )
        if not self.parallel_gripper_action_client.wait_for_server(timeout_sec=1.0):
            logger.error("Parallel gripper action server not available")
            return False

        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self.config.gripper_joint_name]
        goal.command.position = [gripper_goal]
        goal.command.header.stamp = self.robot_node.get_clock().now().to_msg()

        resp = self.parallel_gripper_action_client.send_goal(
            goal, feedback_callback=None, goal_uuid=None
        )
        if not resp:
            logger.error("Failed to send parallel gripper command")
            return False
        result = resp.result
        if result.reached_goal:
            return True
        logger.error(f"Parallel gripper did not reach goal. stalled: {result.stalled}")
        return False


@RobotConfig.register_subclass("aic_ros2_control")
@dataclass
class AICRobotROS2ControlConfig(ROS2Config):
    action_type: ActionType = ActionType.JOINT_TRAJECTORY

    cameras: dict[str, CameraConfig] = field(default_factory=lambda: aic_cameras.copy())

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            base_link="base_link",
            arm_joint_names=arm_joint_names.copy(),
            gripper_joint_name=gripper_joint_name,
            gripper_open_position=0.025,
            gripper_close_position=0.012,
            gripper_action_name="/gripper_action_controller/gripper_cmd",
            min_joint_positions=[-2 * pi for _ in arm_joint_names],
            max_joint_positions=[2 * pi for _ in arm_joint_names],
            joint_trajectory_topic="/joint_trajectory_controller/joint_trajectory",
        )
    )


class AICRobotROS2Control(ROS2Robot):
    name = "ur5e_ros"

    def __init__(self, config: ROS2Config):
        super().__init__(config)
        self.ros2_interface: AICROS2ControlInterface = AICROS2ControlInterface(
            config.ros2_interface, config.action_type
        )
