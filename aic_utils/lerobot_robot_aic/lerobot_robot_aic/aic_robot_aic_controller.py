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

"""LeRobot driver for AIC robot: derived from https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/robot.py"""

import json
import logging
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from functools import cached_property
from threading import Event, Thread

from typing import Any, Callable, TypedDict, cast

import math

import cv2
import numpy as np
import rclpy
from aic_control_interfaces.msg import (
    ControllerState,
    JointMotionUpdate,
    MotionUpdate,
    TargetMode,
    TrajectoryGenerationMode,
)
from aic_control_interfaces.srv import ChangeTargetMode
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist, Vector3, Wrench, WrenchStamped
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from numpy.typing import NDArray
from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .aic_robot import aic_cameras, arm_joint_names
from .aic_teleop import AICCheatCodeTeleopConfig, approach_state, reset_in_progress
from .types import JointMotionUpdateActionDict, MotionUpdateActionDict

logger = logging.getLogger(__name__)


# Encoding maps for task observation integers
TASK_TARGET_MODULE_ENCODING: dict[str, int] = {
    "nic_card_mount_0": 0,
    "nic_card_mount_1": 1,
    "nic_card_mount_2": 2,
    "nic_card_mount_3": 3,
    "nic_card_mount_4": 4,
    "sc_port_0": 5,
    "sc_port_1": 6,
    "sc_port_2": 7,
    "sc_port_3": 8,
    "sc_port_4": 9,
}

TASK_PORT_NAME_ENCODING: dict[str, int] = {
    "sfp_port_0": 0,
    "sfp_port_1": 1,
    "sc_port_base": 2,
}

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
        "wrench.force.x": float,
        "wrench.force.y": float,
        "wrench.force.z": float,
        "wrench.torque.x": float,
        "wrench.torque.y": float,
        "wrench.torque.z": float,
        "fts_tare_offset.force.x": float,
        "fts_tare_offset.force.y": float,
        "fts_tare_offset.force.z": float,
        "fts_tare_offset.torque.x": float,
        "fts_tare_offset.torque.y": float,
        "fts_tare_offset.torque.z": float,
        "max_force_magnitude": float,
        "insertion_event": float,
        "teleop.approach_done": float,  # 1.0 when AICCheatCodeApproachOnlyTeleop reaches DONE
        "teleop.approach_dist_x": float,
        "teleop.approach_dist_y": float,
        "teleop.approach_dist_z": float,
        "task.target_module": float,  # see TASK_TARGET_MODULE_ENCODING
        "task.port_name": float,      # see TASK_PORT_NAME_ENCODING
        "episode_number": float,
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
    auto_reset: bool = False      # if True, trigger env reset 2s after each insertion event
    episode_timeout_s: float = 60.0  # force reset after this many seconds with no insertion

    arm_joint_names: list[str] = field(default_factory=arm_joint_names.copy)

    cameras: dict[str, CameraConfig] = field(default_factory=aic_cameras.copy)
    camera_image_scaling: CameraImageScaling = field(
        default_factory=lambda: {
            "left_camera": 0.25,
            "center_camera": 0.25,
            "right_camera": 0.25,
        }
    )


@dataclass(kw_only=True)
class AICRos2Interface:
    node: Node
    executor: SingleThreadedExecutor
    executor_thread: Thread
    change_target_mode_client: Client[
        ChangeTargetMode.Request, ChangeTargetMode.Response
    ]
    env_reset_client: Client[SetBool.Request, SetBool.Response]
    motion_update_pub: Publisher[MotionUpdate]
    joint_motion_update_pub: Publisher[JointMotionUpdate]
    task_info_pub: Publisher[String]
    controller_state_sub: Subscription[ControllerState]
    joint_states_sub: Subscription[JointState]
    wrench_sub: Subscription[WrenchStamped]
    insertion_event_sub: Subscription[String]
    logger: RcutilsLogger

    @staticmethod
    def connect(
        controller_state_cb: Callable[[ControllerState], None],
        joint_states_cb: Callable[[JointState], None],
        wrench_cb: Callable[[WrenchStamped], None],
        insertion_event_cb: Callable[[String], None],
    ) -> "AICRos2Interface":
        if not rclpy.ok():
            rclpy.init()

        node = Node("aic_robot_node")
        logger = node.get_logger()
        logger.set_level(logging.DEBUG)

        change_target_mode_client = node.create_client(
            ChangeTargetMode, f"/aic_controller/change_target_mode"
        )

        while not change_target_mode_client.wait_for_service():
            node.get_logger().info(
                f"Waiting for service 'aic_controller/change_target_mode'..."
            )
            time.sleep(1.0)

        motion_update_pub = node.create_publisher(
            MotionUpdate, "/aic_controller/pose_commands", 10
        )

        joint_motion_update_pub = node.create_publisher(
            JointMotionUpdate, "/aic_controller/joint_commands", 10
        )

        controller_state_sub = node.create_subscription(
            ControllerState, "/aic_controller/controller_state", controller_state_cb, 10
        )

        joint_states_sub = node.create_subscription(
            JointState, "/joint_states", joint_states_cb, qos_profile_sensor_data
        )

        wrench_sub = node.create_subscription(
            WrenchStamped, "/fts_broadcaster/wrench", wrench_cb, 10
        )

        insertion_event_sub = node.create_subscription(
            String, "/scoring/insertion_event", insertion_event_cb, 10
        )

        # Client for /env/reset (env_resetter.py). Not waited on here —
        # the service lives in a separate process and may start later.
        env_reset_client = node.create_client(SetBool, "/env/reset")

        # Publisher for task info after each reset. Teleop nodes subscribe to
        # this to update their targets without requiring a direct object reference.
        task_info_pub = node.create_publisher(String, "/env/task_info", 10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        time.sleep(3)  # Give some time to connect to services and receive messages

        return AICRos2Interface(
            node=node,
            executor=executor,
            executor_thread=executor_thread,
            change_target_mode_client=change_target_mode_client,
            env_reset_client=env_reset_client,
            motion_update_pub=motion_update_pub,
            joint_motion_update_pub=joint_motion_update_pub,
            task_info_pub=task_info_pub,
            controller_state_sub=controller_state_sub,
            joint_states_sub=joint_states_sub,
            wrench_sub=wrench_sub,
            insertion_event_sub=insertion_event_sub,
            logger=logger,
        )


class AICRobotAICController(Robot):
    name = "ur5e_aic"

    def __init__(self, config: AICRobotAICControllerConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.ros2_interface: AICRos2Interface | None = None
        self.last_controller_state: ControllerState | None = None
        self.last_joint_states: JointState | None = None
        self.last_wrench: WrenchStamped | None = None
        self.max_force_magnitude: float = 0.0
        self.last_insertion_event: float = 0.0
        self.fts_tare_offset: WrenchStamped = WrenchStamped()

        self._is_connected = False
        self.current_task_info: dict = {
            "target_module_name": AICCheatCodeTeleopConfig.task_module_name,
            "port_name": AICCheatCodeTeleopConfig.task_port_name,
        }
        self.episode_number: int = 0

        # Auto-reset on insertion event, approach done, or episode timeout
        self._auto_reset_enabled: bool = False
        self._auto_reset_pending: bool = False
        self._last_reset_time: float = 0.0
        self._watchdog_stop: Event = Event()
        self._watchdog_thread: Thread | None = None
        self._last_approach_done: bool = False

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

    def send_change_control_mode_req(self, mode: int):
        if not self.ros2_interface:
            raise DeviceNotConnectedError()

        req = ChangeTargetMode.Request()
        req.target_mode.mode = mode

        self.ros2_interface.logger.info(
            f"Sending request to change control mode to {mode}"
        )

        response = self.ros2_interface.change_target_mode_client.call(req)

        if not response or not response.success:
            self.ros2_interface.logger.info(f"Failed to change control mode to {mode}")
        else:
            self.ros2_interface.logger.info(
                f"Successfully changed control mode to {mode}"
            )

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

        if calibrate is True:
            print(
                "Warning: Calibration is not supported, ensure the robot is already calibrated before running lerobot."
            )

        def controller_state_cb(msg: ControllerState):
            self.last_controller_state = msg
            self.fts_tare_offset = msg.fts_tare_offset

        def joint_states_cb(msg: JointState):
            self.last_joint_states = msg

        def wrench_cb(msg: WrenchStamped):
            self.last_wrench = msg
            tare = self.fts_tare_offset.wrench
            magnitude = math.sqrt(
                (msg.wrench.force.x - tare.force.x) ** 2
                + (msg.wrench.force.y - tare.force.y) ** 2
                + (msg.wrench.force.z - tare.force.z) ** 2
            )
            if magnitude > self.max_force_magnitude:
                self.max_force_magnitude = magnitude

        def insertion_event_cb(msg: String):
            prev = self.last_insertion_event
            if not msg.data:
                # 0: no insertion
                self.last_insertion_event = 0.0
            else:
                target = self.current_task_info.get("target_module_name", "")
                port = self.current_task_info.get("port_name", "")
                if target and port and target in msg.data and port in msg.data:
                    # 2: insertion matches current task
                    self.last_insertion_event = 2.0
                    logger.info(f"Insertion event matched task target. msg={msg.data}")
                else:
                    # 1: insertion into wrong target
                    self.last_insertion_event = 1.0
                    logger.info(
                        f"Insertion event did NOT match task target. msg={msg.data} "
                        f"expected target={target!r} port={port!r}"
                    )
            # Rising edge: 0 → non-zero. Spawn reset thread if auto-reset is enabled.
            if self.last_insertion_event > 0.0 and prev == 0.0:
                logger.info("LXH insertion event detected")
                if self._auto_reset_enabled and not self._auto_reset_pending:
                    logger.info("LXH reset trigger start")
                    self._auto_reset_pending = True
                    Thread(target=self._auto_reset_after_delay, daemon=True).start()

        self.ros2_interface = AICRos2Interface.connect(
            controller_state_cb, joint_states_cb, wrench_cb, insertion_event_cb
        )

        change_mode_req = (
            TargetMode.MODE_JOINT
            if self.teleop_target_mode == "joint"
            else TargetMode.MODE_CARTESIAN
        )
        self.send_change_control_mode_req(change_mode_req)

        for cam in self.cameras.values():
            cam.connect()

        self._is_connected = True

        if self.config.auto_reset:
            self.enable_auto_reset()

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
            "wrench.force.x": (self.last_wrench.wrench.force.x - self.fts_tare_offset.wrench.force.x) if self.last_wrench else 0.0,
            "wrench.force.y": (self.last_wrench.wrench.force.y - self.fts_tare_offset.wrench.force.y) if self.last_wrench else 0.0,
            "wrench.force.z": (self.last_wrench.wrench.force.z - self.fts_tare_offset.wrench.force.z) if self.last_wrench else 0.0,
            "wrench.torque.x": (self.last_wrench.wrench.torque.x - self.fts_tare_offset.wrench.torque.x) if self.last_wrench else 0.0,
            "wrench.torque.y": (self.last_wrench.wrench.torque.y - self.fts_tare_offset.wrench.torque.y) if self.last_wrench else 0.0,
            "wrench.torque.z": (self.last_wrench.wrench.torque.z - self.fts_tare_offset.wrench.torque.z) if self.last_wrench else 0.0,
            "fts_tare_offset.force.x": self.fts_tare_offset.wrench.force.x,
            "fts_tare_offset.force.y": self.fts_tare_offset.wrench.force.y,
            "fts_tare_offset.force.z": self.fts_tare_offset.wrench.force.z,
            "fts_tare_offset.torque.x": self.fts_tare_offset.wrench.torque.x,
            "fts_tare_offset.torque.y": self.fts_tare_offset.wrench.torque.y,
            "fts_tare_offset.torque.z": self.fts_tare_offset.wrench.torque.z,
            "max_force_magnitude": self.max_force_magnitude,
            "insertion_event": self.last_insertion_event,
            "teleop.approach_done": float(approach_state.done),
            "teleop.approach_dist_x": approach_state.dist_x,
            "teleop.approach_dist_y": approach_state.dist_y,
            "teleop.approach_dist_z": approach_state.dist_z,
            "episode_number": float(self.episode_number),
            "task.target_module": float(
                TASK_TARGET_MODULE_ENCODING.get(
                    self.current_task_info.get("target_module_name", ""), -1
                )
            ),
            "task.port_name": float(
                TASK_PORT_NAME_ENCODING.get(
                    self.current_task_info.get("port_name", ""), -1
                )
            ),
        }

        # Capture images from cameras in parallel to reduce latency
        def _read_camera(cam_key: str) -> tuple[str, NDArray[Any]]:
            cam = self.cameras[cam_key]
            try:
                data = cam.async_read(timeout_ms=2000)
                if data is not None and data.size > 0:
                    image_scale = self.config.camera_image_scaling[cam_key]
                    if image_scale != 1:
                        return cam_key, cv2.resize(
                            data,
                            None,
                            fx=image_scale,
                            fy=image_scale,
                            interpolation=cv2.INTER_AREA,
                        )
                    return cam_key, data
            except Exception as e:
                logger.error(f"Failed to read camera {cam_key}: {e}")
            logger.debug(f"Camera {cam_key} data is empty (camera not ready yet?), using all-black placeholder.")
            return cam_key, np.zeros(self._cameras_ft[cam_key], dtype=np.uint8)

        cam_obs: dict[str, NDArray[Any]] = {}
        with ThreadPoolExecutor(max_workers=len(self.cameras)) as pool:
            futures = {pool.submit(_read_camera, k): k for k in self.cameras}
            for future in as_completed(futures):
                key, frame = future.result()
                cam_obs[key] = frame

        # Rising edge: approach_state.done just became True → trigger reset
        approach_done_now = approach_state.done
        if approach_done_now and not self._last_approach_done:
            logger.info("Approach phase done detected in get_observation — triggering reset.")
            if not self._auto_reset_pending:
                self._auto_reset_pending = True
                Thread(target=self._auto_reset_after_delay, daemon=True).start()
        self._last_approach_done = approach_done_now

        obs = {**cam_obs, **controller_state_obs}
        return obs

    def send_action_cartesian(self, action: dict[str, Any]) -> None:
        if not self._is_connected or not self.ros2_interface:
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
        msg.header.stamp = self.ros2_interface.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.velocity = twist_msg
        msg.target_stiffness = np.diag([85.0, 85.0, 85.0, 85.0, 85.0, 85.0]).flatten()
        msg.target_damping = np.diag([75.0, 75.0, 75.0, 75.0, 75.0, 75.0]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
        self.ros2_interface.motion_update_pub.publish(msg)

    def send_action_joint(self, action: dict[str, Any]) -> None:
        if not self._is_connected or not self.ros2_interface:
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

        self.ros2_interface.joint_motion_update_pub.publish(msg)

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if self.teleop_target_mode == "cartesian":
            self.send_action_cartesian(action)
            return action
        elif self.teleop_target_mode == "joint":
            self.send_action_joint(action)
            return action
        else:
            raise ValueError("Invalid teleop_target_mode")

    def enable_auto_reset(self) -> None:
        """Enable automatic env reset on insertion event or episode timeout."""
        self._auto_reset_enabled = True
        self._last_reset_time = time.monotonic()
        self._watchdog_stop.clear()
        self._watchdog_thread = Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()
        logger.info(
            f"Auto-reset enabled — insertion event + {self.config.episode_timeout_s}s timeout."
        )

    def disable_auto_reset(self) -> None:
        """Disable automatic env reset and stop the watchdog thread."""
        self._auto_reset_enabled = False
        self._watchdog_stop.set()
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=5.0)
            self._watchdog_thread = None
        logger.info("Auto-reset disabled.")

    def _watchdog_loop(self) -> None:
        """Background thread: force reset if episode_timeout_s elapses with no insertion."""
        timeout = self.config.episode_timeout_s
        while not self._watchdog_stop.wait(timeout=1.0):
            if not self._auto_reset_enabled or self._auto_reset_pending:
                continue
            elapsed = time.monotonic() - self._last_reset_time
            if elapsed >= timeout:
                logger.warning(
                    f"Episode timeout ({elapsed:.1f}s >= {timeout}s) — triggering forced reset."
                )
                self._auto_reset_pending = True
                Thread(target=self._auto_reset_after_delay, args=(0.0,), daemon=True).start()

    def _auto_reset_after_delay(self, delay: float = 2.0) -> None:
        """Background thread: wait `delay` seconds then call reset()."""
        logger.info("auto reset thread start")
        time.sleep(delay)
        try:
            self.reset()
        finally:
            self._auto_reset_pending = False

    def reset(self) -> dict:
        """Trigger a random environment reset via the /env/reset service.

        Calls env_resetter.py with data=True (random reset). Parses the JSON
        task_info from the response and publishes it on /env/task_info so
        any subscriber (e.g. AICCheatCodeTeleop) can update its targets.

        Returns:
            task_info dict (may be empty if reset failed or resetter is unavailable).
        """
        if not self.ros2_interface:
            raise DeviceNotConnectedError()

        client = self.ros2_interface.env_reset_client
        if not client.wait_for_service(timeout_sec=10.0):
            logger.warning("/env/reset service not available — is env_resetter.py running?")
            return {}

        req = SetBool.Request()
        req.data = True  # random reset

        reset_in_progress.set()
        logger.info("Reset in progress — teleop actions paused.")
        try:
            future = client.call_async(req)
            t0 = time.monotonic()
            while not future.done():
                time.sleep(0.05)
                if time.monotonic() - t0 > 120.0:
                    logger.error("/env/reset timed out after 120 s")
                    return {}

            result = future.result()
            if result is None or not result.success:
                logger.error(f"Environment reset failed: {getattr(result, 'message', 'no response')}")
                return {}

            try:
                task_info = json.loads(result.message)
            except (json.JSONDecodeError, TypeError):
                task_info = {}
        finally:
            reset_in_progress.clear()
            logger.info("Reset complete — teleop actions resumed.")

        self.current_task_info = task_info
        self.last_insertion_event = 0.0
        self.episode_number += 1
        logger.info(f"Environment reset complete. episode_number={self.episode_number} task_info={task_info}")

        # Stamp reset time so the watchdog timeout starts from now.
        self._last_reset_time = time.monotonic()

        # Publish task_info so any subscriber (e.g. AICCheatCodeTeleop) can update
        # its targets without needing a direct reference to this robot object.
        if task_info and self.ros2_interface:
            msg = String()
            msg.data = json.dumps(task_info)
            self.ros2_interface.task_info_pub.publish(msg)

        return task_info

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.disable_auto_reset()

        for cam in self.cameras.values():
            cam.disconnect()

        if self.ros2_interface:
            self.ros2_interface.node.destroy_node()
            self.ros2_interface.executor.shutdown()
            self.ros2_interface.executor_thread.join()
            self.ros2_interface = None

        self._is_connected = False
