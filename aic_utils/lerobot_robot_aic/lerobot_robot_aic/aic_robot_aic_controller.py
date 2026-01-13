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
from threading import Thread
from typing import Any

import rclpy
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

from .aic_robot import aic_cameras, arm_joint_names, gripper_joint_name

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("aic_controller")
@dataclass(kw_only=True)
class AICRobotAICControllerConfig(RobotConfig):
    arm_joint_names: list[str] = field(default_factory=lambda: arm_joint_names.copy())
    gripper_joint_name: str = gripper_joint_name
    cameras: dict[str, CameraConfig] = field(default_factory=lambda: aic_cameras.copy())


class AICRobotAICController(Robot):
    name = "ur5e_aic"

    def __init__(self, config: AICRobotAICControllerConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.robot_node: Node | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.executor_thread: Thread | None = None
        self._is_connected = False
        self._last_joint_state: dict[str, dict[str, float]] | None = None

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        all_joint_names = [
            *self.config.arm_joint_names,
            self.config.gripper_joint_name,
        ]
        motor_state_ft = {f"{motor}.pos": float for motor in all_joint_names}
        return {**motor_state_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in self.config.arm_joint_names} | {
            "gripper.pos": float
        }

    @property
    def is_connected(self) -> bool:
        return self._is_connected

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

    def connect(self, calibrate: bool = True) -> None:
        if self.robot_node is not None:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        if not rclpy.ok():
            rclpy.init()

        if calibrate is True:
            print(
                "Warning: Calibration is not supported, ensure the robot is already calibrated before running lerobot."
            )

        self.robot_node = Node("aic_robot_node")

        self.robot_node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.robot_node)
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

        obs_dict: dict[str, Any] = {}
        # TODO: get observation from aic controller topics

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            try:
                obs_dict[cam_key] = cam.async_read(timeout_ms=300)
            except Exception as e:
                logger.error(f"Failed to read camera {cam_key}: {e}")
                obs_dict[cam_key] = None
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        # TODO:
        return {}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for cam in self.cameras.values():
            cam.disconnect()

        if self.robot_node is not None:
            self.robot_node.destroy_node()
            self.robot_node = None

        if self.executor_thread is not None:
            self.executor_thread.join()
            self.executor_thread = None

        self._is_connected = False
