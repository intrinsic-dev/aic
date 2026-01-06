"""LeRobot driver for AIC robot"""

import logging
import time
from dataclasses import dataclass, field
from threading import Thread
from typing import Any

from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from rclpy.node import Node

from .aic_robot import aic_cameras, arm_joint_names, gripper_joint_name

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("aic_aic_controller")
@dataclass(kw_only=True)
class AICRobotAICConfig(RobotConfig):
    arm_joint_names: list[str] = field(default_factory=lambda: arm_joint_names.copy())
    gripper_joint_name: str = gripper_joint_name
    cameras: dict[str, CameraConfig] = field(default_factory=lambda: aic_cameras.copy())


class AICRobot(Robot):
    name = "ur5e_aic"

    def __init__(self, config: AICRobotAICConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.robot_node: Node | None = None
        self.spin_thread: Thread | None = None

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
        return (
            self.robot_node is not None
            and self.spin_thread is not None
            and all(cam.is_connected for cam in self.cameras.values())
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        if calibrate is True:
            print(
                "Warning: Calibration is not supported, ensure the robot is already calibrated before running lerobot."
            )

        for cam in self.cameras.values():
            cam.connect()

        # TODO: Subscribe to aic controller topics.
        raise NotImplementedError("aic_controller interface not implemented yet.")

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

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for cam in self.cameras.values():
            cam.disconnect()

        if self.robot_node is not None:
            self.robot_node.destroy_node()
            self.robot_node = None

        if self.spin_thread is not None:
            self.spin_thread.join()
            self.spin_thread = None
