import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from control_msgs.action import ParallelGripperCommand
from lerobot.utils.errors import DeviceNotConnectedError
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

logger = logging.getLogger(__name__)


@dataclass(kw_only=True)
class AICInterfaceConfig:
    arm_joint_names: list[str]
    min_joint_positions: list[float]
    max_joint_positions: list[float]
    joint_trajectory_topic: str
    gripper_joint_name: str
    gripper_action_name: str
    gripper_open_position: float
    gripper_close_position: float


class AICInterfaceNode(Node, ABC):
    @abstractmethod
    def get_observation(self) -> dict[str, Any]: ...

    @abstractmethod
    def send_joint_position_command(self, joint_positions: list[float]) -> None: ...

    @abstractmethod
    def send_gripper_command(self, position: float) -> bool: ...


class AICROS2ControlInterfaceNode(AICInterfaceNode):
    def _joint_state_callback(self, msg: JointState) -> None:
        self._last_joint_state = self._last_joint_state or {}
        positions = {}
        velocities = {}
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        for joint_name in self.config.arm_joint_names:
            idx = name_to_index.get(joint_name)
            if idx is None:
                raise ValueError(f"Joint '{joint_name}' not found in joint state.")
            positions[joint_name] = msg.position[idx]
            velocities[joint_name] = msg.velocity[idx]

        if self.config.gripper_joint_name:
            idx = name_to_index.get(self.config.gripper_joint_name)
            if idx is None:
                raise ValueError(
                    f"Gripper joint '{self.config.gripper_joint_name}' not found in joint state."
                )
            positions[self.config.gripper_joint_name] = msg.position[idx]
            velocities[self.config.gripper_joint_name] = msg.velocity[idx]

        self._last_joint_state["position"] = positions
        self._last_joint_state["velocity"] = velocities

    def __init__(self, config: AICInterfaceConfig):
        super().__init__("aic_ros2_control_interface")
        self.config = config
        self.node = Node("aic_ros2_control_interface")
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )
        self.traj_cmd_pub = self.create_publisher(
            JointTrajectory, self.config.joint_trajectory_topic, 10
        )
        self.parallel_gripper_action_client: ActionClient[
            ParallelGripperCommand.Goal,
            ParallelGripperCommand.Result,
            ParallelGripperCommand.Feedback,
        ] = ActionClient(
            self.node,
            ParallelGripperCommand,
            self.config.gripper_action_name,
            callback_group=ReentrantCallbackGroup(),
        )
        self._last_joint_state: dict[str, dict[str, float]] | None = None

    def get_observation(self) -> dict[str, Any]:
        if self._last_joint_state is None:
            logger.warning("Joint state is not available yet.")
            return {}
        return {
            f"{joint}.pos": pos
            for joint, pos in self._last_joint_state["position"].items()
        }

    def send_joint_position_command(
        self, joint_positions: list[float], unnormalize: bool = True
    ) -> None:
        joint_positions = [
            min(max(pos, min_pos), max_pos)
            for pos, min_pos, max_pos in zip(
                joint_positions,
                self.config.min_joint_positions,
                self.config.max_joint_positions,
                strict=True,
            )
        ]

        if len(joint_positions) != len(self.config.arm_joint_names):
            raise ValueError(
                f"Expected {len(self.config.arm_joint_names)} joint positions, but got {len(joint_positions)}."
            )

        if self.traj_cmd_pub is None:
            raise DeviceNotConnectedError(
                "Trajectory command publisher is not initialized."
            )
        msg = JointTrajectory()
        msg.joint_names = self.config.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        msg.points = [point]
        self.traj_cmd_pub.publish(msg)

    def send_gripper_command(self, position: float, unnormalize: bool = True) -> bool:
        # Map normalized position (0=open, 1=closed) to actual gripper joint position
        open_pos = self.config.gripper_open_position
        closed_pos = self.config.gripper_close_position
        gripper_goal = open_pos + position * (closed_pos - open_pos)

        if not self.parallel_gripper_action_client.wait_for_server(timeout_sec=1.0):
            logger.error("Parallel gripper action server not available")
            return False

        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self.config.gripper_joint_name]
        goal.command.position = [gripper_goal]
        goal.command.header.stamp = self.get_clock().now().to_msg()

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
