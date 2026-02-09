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

import cv2
import numpy as np
import rclpy
from aic_control_interfaces.msg import (
    ControllerState,
    MotionUpdate,
    JointMotionUpdate,
    TrajectoryGenerationMode,
)
from aic_control_interfaces.srv import (
    ChangeTargetMode,
)
from geometry_msgs.msg import Twist, Vector3, Wrench
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from rclpy.task import Future as RclFuture
from sensor_msgs.msg import JointState

from .aic_robot import aic_cameras, arm_joint_names
from .types import MotionUpdateActionDict, JointMotionUpdateActionDict

logger = logging.getLogger(__name__)


ObservationState = TypedDict(
    "ObservationState",
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
        "joint_positions.6": float,
    },
)


class CameraImageScaling(TypedDict):
    left_camera: float
    center_camera: float
    right_camera: float


@RobotConfig.register_subclass("aic_controller")
@dataclass(kw_only=True)
class AICRobotAICControllerConfig(RobotConfig):
    teleop_target_mode: str = "cartesian"  # "cartesian" or "joint"
    teleop_frame_id: str = "gripper/tcp"  # "gripper/tcp" or "base_link"

    arm_joint_names: list[str] = field(default_factory=arm_joint_names.copy)

    cameras: dict[str, CameraConfig] = field(default_factory=aic_cameras.copy)
    camera_image_scaling: CameraImageScaling = field(
        default_factory=lambda: {
            "left_camera": 0.25,
            "center_camera": 0.25,
            "right_camera": 0.25,
        }
    )


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
        self.joint_states_sub: Subscription[JointState] | None = None
        self.last_joint_states: JointState | None = None

        self._is_connected = False

        if config.teleop_frame_id not in ["gripper/tcp", "base_link"]:
            raise ValueError(
                f"Invalid teleop_frame_id: '{config.teleop_frame_id}'. "
                "Supported frames are 'gripper/tcp' or 'base_link'."
            )
        self.frame_id = config.teleop_frame_id

        if config.teleop_target_mode not in ["cartesian", "joint"]:
            raise ValueError(
                f"Invalid teleop_target_mode: '{config.teleop_target_mode}'. "
                "Supported modes are 'cartesian' or 'joint'."
            )
        self.teleop_target_mode = config.teleop_target_mode

        print(f"Teleop frame id: {self.frame_id}")
        print(f"Teleop target mode: {self.teleop_target_mode}")

    def send_change_control_mode_req(self, mode: ChangeTargetMode.Request):

        req = ChangeTargetMode.Request()
        req.target_mode = mode

        self.node.get_logger().info(f"Sending request to change control mode to {mode}")

        future = self.node.client.call_async(req)

        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()

        if response.success:
            self.node.get_logger().info(f"Successfully changed control mode to {mode}")
        else:
            self.node.get_logger().info(f"Failed to change control mode to {mode}")

        time.sleep(0.5)

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (
                # assuming that opencv2 rounds down when being asked to scale without perfect ratio
                int(
                    self.config.cameras[cam].height
                    * self.config.camera_image_scaling[cam]
                ),
                int(
                    self.config.cameras[cam].width
                    * self.config.camera_image_scaling[cam]
                ),
                3,
            )
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict:
        return {**ObservationState.__annotations__, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return (
            MotionUpdateActionDict.__annotations__
            if self.teleop_target_mode == "cartesian"
            else JointMotionUpdateActionDict.__annotations__
        )

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

        self.node.client = self.node.create_client(
            ChangeTargetMode, f"/aic_controller/change_target_mode"
        )

        while not self.node.client.wait_for_service():
            self.node.get_logger().info(
                f"Waiting for service 'aic_controller/change_target_mode'..."
            )
            time.sleep(1.0)

        change_mode_req = (
            ChangeTargetMode.Request.TARGET_MODE_JOINT
            if self.teleop_target_mode == "joint"
            else ChangeTargetMode.Request.TARGET_MODE_CARTESIAN
        )
        self.send_change_control_mode_req(change_mode_req)

        self.motion_update_pub = self.node.create_publisher(
            MotionUpdate, "/aic_controller/pose_commands", 10
        )

        self.joint_motion_update_pub = self.node.create_publisher(
            JointMotionUpdate, "/aic_controller/joint_commands", 10
        )

        def controller_state_cb(msg: ControllerState):
            self.last_controller_state = msg

        self.node.create_subscription(
            ControllerState, "/aic_controller/controller_state", controller_state_cb, 10
        )

        def joint_states_cb(msg: JointState):
            self.last_joint_states = msg

        self.node.create_subscription(
            JointState, "/joint_states", joint_states_cb, qos_profile_sensor_data
        )

        for cam in self.cameras.values():
            cam.connect()

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(3)  # Give some time to connect to services and receive messages

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

        if not self.last_controller_state or not self.last_joint_states:
            return {}

        tcp_pose = self.last_controller_state.tcp_pose
        tcp_velocity = self.last_controller_state.tcp_velocity
        tcp_error = self.last_controller_state.tcp_error
        joint_positions = self.last_joint_states.position
        controller_state_obs: ObservationState = {
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
            "joint_positions.0": joint_positions[0],
            "joint_positions.1": joint_positions[1],
            "joint_positions.2": joint_positions[2],
            "joint_positions.3": joint_positions[3],
            "joint_positions.4": joint_positions[4],
            "joint_positions.5": joint_positions[5],
            "joint_positions.6": joint_positions[6],
        }

        # Capture images from cameras
        cam_obs = {}
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            try:
                cam_obs[cam_key] = cam.async_read(timeout_ms=2000)
                image_scale = self.config.camera_image_scaling[cam_key]
                if image_scale != 1:
                    cam_obs[cam_key] = cv2.resize(
                        cam_obs[cam_key],
                        None,
                        fx=image_scale,
                        fy=image_scale,
                        interpolation=cv2.INTER_AREA,
                    )
            except Exception as e:
                logger.error(f"Failed to read camera {cam_key}: {e}")
                cam_obs[cam_key] = None
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        obs = {**cam_obs, **controller_state_obs}
        return obs

    def send_action_cartesian(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self._is_connected or not self.node:
            raise DeviceNotConnectedError()

        motion_update_action = cast(MotionUpdateActionDict, action)

        twist_msg = Twist()

        try:
            twist_msg.linear.x = float(motion_update_action["linear.x"])
        except KeyError:
            raise KeyError(
                "Missing key 'linear.x'. If using `--teleop.type=aic_keyboard_joint`, have you set `--robot.teleop_target_mode=joint`?"
            ) from None
        twist_msg.linear.y = float(motion_update_action["linear.y"])
        twist_msg.linear.z = float(motion_update_action["linear.z"])
        twist_msg.angular.x = float(motion_update_action["angular.x"])
        twist_msg.angular.y = float(motion_update_action["angular.y"])
        twist_msg.angular.z = float(motion_update_action["angular.z"])

        msg = MotionUpdate()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.velocity = twist_msg
        msg.target_stiffness = np.diag([85.0, 85.0, 85.0, 85.0, 85.0, 85.0]).flatten()
        msg.target_damping = np.diag([75.0, 75.0, 75.0, 75.0, 75.0, 75.0]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
        if self.motion_update_pub is not None:
            self.motion_update_pub.publish(msg)

    def send_action_joint(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self._is_connected or not self.node:
            raise DeviceNotConnectedError()

        joint_motion_update_action = cast(JointMotionUpdateActionDict, action)
        msg = JointMotionUpdate()

        if "shoulder_pan_joint" not in joint_motion_update_action:
            raise KeyError(
                "Missing key 'shoulder_pan_joint'. If using `--teleop.type=aic_keyboard_ee` or `--teleop.type=aic_spacemouse`, have you set `--robot.teleop_target_mode=cartesian`?"
            )

        msg.target_state.velocities = list(action.values())

        msg.target_stiffness = [85.0, 85.0, 85.0, 85.0, 85.0, 85.0]
        msg.target_damping = [75.0, 75.0, 75.0, 75.0, 75.0, 75.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY

        if self.joint_motion_update_pub is not None:
            self.joint_motion_update_pub.publish(msg)

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if self.teleop_target_mode == "cartesian":
            return self.send_action_cartesian(action)
        elif self.teleop_target_mode == "joint":
            return self.send_action_joint(action)
        else:
            raise ValueError("Invalid teleop_target_mode")

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for cam in self.cameras.values():
            cam.disconnect()

        if self.node is not None:
            self.node.destroy_node()

        if self.executor_thread is not None:
            if self.executor is not None:
                self.executor.shutdown()
            self.executor_thread.join()

        self._is_connected = False
