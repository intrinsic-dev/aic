import logging
from typing import cast

from control_msgs.action import ParallelGripperCommand
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot_robot_ros.config import ROS2InterfaceConfig
from lerobot_robot_ros.robot import ActionType, ROS2Interface
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

logger = logging.getLogger(__name__)


class AICInterface(ROS2Interface):
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
