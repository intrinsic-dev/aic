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
import time
from dataclasses import dataclass, field
from functools import cached_property
from threading import Thread
from typing import Any, TypedDict, cast

import numpy as np
import rclpy
from aic_control_interfaces.msg import (
    ControllerState,
    MotionUpdate,
    TrajectoryGenerationMode,
)
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Twist, Vector3, Wrench
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from numpy.typing import NDArray
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.task import Future as RclFuture

from .aic_robot import aic_cameras, arm_joint_names, gripper_joint_name
from .types import MotionUpdateActionDict

logger = logging.getLogger(__name__)


ControllerStateDict = TypedDict(
    "ControllerStateDict",
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
        "joint_state.positions": NDArray[np.float64],
        "joint_state.velocities": NDArray[np.float64],
        "joint_state.accelerations": NDArray[np.float64],
        "joint_state.efforts": NDArray[np.float64],
    },
)


@RobotConfig.register_subclass("aic_controller")
@dataclass(kw_only=True)
class AICRobotAICControllerConfig(RobotConfig):
    arm_joint_names: list[str] = field(default_factory=arm_joint_names.copy)
    gripper_joint_name: str = gripper_joint_name
    gripper_action_name: str = "/gripper_action_controller/gripper_cmd"
    cameras: dict[str, CameraConfig] = field(default_factory=aic_cameras.copy)


class AICRobotAICController(Robot):
    name = "ur5e_aic"

    def __init__(self, config: AICRobotAICControllerConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.node: Node | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.executor_thread: Thread | None = None
        self.motion_update_pub: Publisher[MotionUpdate] | None = None
        self.controller_state_sub: Subscription[ControllerState] | None = None
        self.last_controller_state: ControllerState | None = None
        self.parallel_gripper_action_client: (
            ActionClient[
                ParallelGripperCommand.Goal,
                ParallelGripperCommand.Result,
                ParallelGripperCommand.Feedback,
            ]
            | None
        ) = None
        self.last_gripper_target: float | None = None
        self.gripper_result: (
            RclFuture[
                ClientGoalHandle[
                    ParallelGripperCommand.Goal,
                    ParallelGripperCommand.Result,
                    ParallelGripperCommand.Feedback,
                ]
            ]
            | None
        ) = None
        self._is_connected = False

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict:
        all_joint_names = [
            *self.config.arm_joint_names,
            self.config.gripper_joint_name,
        ]
        motor_state_ft = {f"{motor}.pos": float for motor in all_joint_names}
        return {
            **motor_state_ft,
            **self._cameras_ft,
            **ControllerStateDict.__annotations__,
        }

    @cached_property
    def action_features(self) -> dict[str, type]:
        return MotionUpdateActionDict.__annotations__

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        if not rclpy.ok():
            rclpy.init()

        if calibrate is True:
            print(
                "Warning: Calibration is not supported, ensure the robot is already calibrated before running lerobot."
            )

        self.node = Node("aic_robot_node")
        self.node.get_logger().set_level(logging.DEBUG)

        self.motion_update_pub = self.node.create_publisher(
            MotionUpdate, "/aic_controller/pose_commands", 10
        )

        def controller_state_cb(msg: ControllerState):
            self.last_controller_state = msg

        self.node.create_subscription(
            ControllerState, "/aic_controller/controller_state", controller_state_cb, 10
        )

        self.parallel_gripper_action_client = ActionClient(
            self.node,
            ParallelGripperCommand,
            self.config.gripper_action_name,
            callback_group=ReentrantCallbackGroup(),
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(3)  # Give some time to connect to services and receive messages

        for cam in self.cameras.values():
            cam.connect()

        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass  # robot must be calibrated before running LeRobot

    def configure(self) -> None:
        pass  # robot must be configured before running LeRobot

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if not self.last_controller_state:
            return {}

        tcp_pose = self.last_controller_state.tcp_pose
        tcp_velocity = self.last_controller_state.tcp_velocity
        tcp_error = self.last_controller_state.tcp_error
        joint_positions = self.last_controller_state.joint_state.positions
        joint_velocities = self.last_controller_state.joint_state.velocities
        joint_accelerations = self.last_controller_state.joint_state.accelerations
        joint_efforts = self.last_controller_state.joint_state.effort
        controller_state_obs: ControllerStateDict = {
            "tcp_pose.position.x": tcp_pose.position.x,
            "tcp_pose.position.y": tcp_pose.position.y,
            "tcp_pose.position.z": tcp_pose.position.z,
            "tcp_pose.orientation.x": tcp_pose.orientation.x,
            "tcp_pose.orientation.y": tcp_pose.orientation.y,
            "tcp_pose.orientation.z": tcp_pose.orientation.z,
            "tcp_pose.orientation.w": tcp_pose.orientation.w,
            "tcp_velocity.linear.x": tcp_velocity.linear.x,
            "tcp_velocity.linear.y": tcp_velocity.linear.y,
            "tcp_velocity.linear.z": tcp_velocity.linear.z,
            "tcp_velocity.angular.x": tcp_velocity.angular.x,
            "tcp_velocity.angular.y": tcp_velocity.angular.y,
            "tcp_velocity.angular.z": tcp_velocity.angular.z,
            "tcp_error.x": tcp_error[0],
            "tcp_error.y": tcp_error[1],
            "tcp_error.z": tcp_error[2],
            "tcp_error.rx": tcp_error[3],
            "tcp_error.ry": tcp_error[4],
            "tcp_error.rz": tcp_error[5],
            "joint_state.positions": np.array(joint_positions),
            "joint_state.velocities": np.array(joint_velocities),
            "joint_state.accelerations": np.array(joint_accelerations),
            "joint_state.efforts": np.array(joint_efforts),
        }

        # Capture images from cameras
        cam_obs = {}
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            try:
                cam_obs[cam_key] = cam.async_read(timeout_ms=300)
            except Exception as e:
                logger.error(f"Failed to read camera {cam_key}: {e}")
                cam_obs[cam_key] = None
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return {**cam_obs, **controller_state_obs}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        motion_update_action = cast(MotionUpdateActionDict, action)

        twist_msg = Twist()
        twist_msg.linear.x = float(motion_update_action["linear.x"])
        twist_msg.linear.y = float(motion_update_action["linear.y"])
        twist_msg.linear.z = float(motion_update_action["linear.z"])
        twist_msg.angular.x = float(motion_update_action["angular.x"])
        twist_msg.angular.y = float(motion_update_action["angular.y"])
        twist_msg.angular.z = float(motion_update_action["angular.z"])

        msg = MotionUpdate()
        msg.velocity = twist_msg
        msg.target_stiffness = np.diag(
            [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        ).flatten()
        msg.target_damping = np.diag([40.0, 40.0, 40.0, 15.0, 15.0, 15.0]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=1.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = Wrench(
            force=Vector3(x=0.5, y=0.5, z=0.5),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
        msg.time_to_target_seconds = 0.0
        if self.motion_update_pub is not None:
            self.motion_update_pub.publish(msg)

        if not self.node or not self.parallel_gripper_action_client:
            raise RuntimeError("unexpected error")
        logger = self.node.get_logger()

        if self.last_gripper_target != motion_update_action["gripper_target"]:
            if self.gripper_result:
                logger.debug("cancelling existing gripper goal")
                self.gripper_result.cancel()
                logger.debug("waiting for gripper goal to finish")
                self.gripper_result.result()
                logger.debug("gripper goal has finished")
                self.gripper_result = None

            goal = ParallelGripperCommand.Goal()
            goal.command.name = [self.config.gripper_joint_name]
            goal.command.position = [motion_update_action["gripper_target"]]
            goal.command.header.stamp = self.node.get_clock().now().to_msg()
            logger.debug(f"sending new gripper goal {goal.command.position}")
            self.gripper_result = self.parallel_gripper_action_client.send_goal_async(
                goal
            )
            self.last_gripper_target = motion_update_action["gripper_target"]

        return action

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for cam in self.cameras.values():
            cam.disconnect()

        if self.node is not None:
            self.node.destroy_node()

        if self.executor_thread is not None:
            self.executor_thread.join()

        self._is_connected = False
