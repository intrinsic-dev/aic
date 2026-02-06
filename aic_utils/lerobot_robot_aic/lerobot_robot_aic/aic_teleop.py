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

from dataclasses import dataclass, field
from typing import Any, cast
import pyspacemouse, rclpy
from threading import Thread

from geometry_msgs.msg import Twist

from lerobot.teleoperators import TeleoperatorConfig, Teleoperator
from lerobot.teleoperators.keyboard import (
    KeyboardEndEffectorTeleop,
    KeyboardEndEffectorTeleopConfig,
)
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig
from rclpy.executors import SingleThreadedExecutor

from .aic_robot import arm_joint_names
from .types import MotionUpdateActionDict, JointMotionUpdateActionDict


@TeleoperatorConfig.register_subclass("aic_keyboard_joint")
@dataclass
class AICKeyboardJointTeleopConfig(KeyboardJointTeleopConfig):
    arm_action_keys: list[str] = field(
        default_factory=lambda: [f"{x}" for x in arm_joint_names]
    )
    high_command_scaling: float = 0.05
    low_command_scaling: float = 0.02


class AICKeyboardJointTeleop(KeyboardJointTeleop):
    def __init__(self, config: KeyboardJointTeleopConfig):
        super().__init__(config)

        self.config = config
        self._low_scaling = config.low_command_scaling
        self._high_scaling = config.high_command_scaling
        self._current_scaling = self._high_scaling

        self.curr_joint_actions: JointMotionUpdateActionDict = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_joint": 0.0,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        }

    @property
    def action_features(self) -> dict:
        return {"names": JointMotionUpdateActionDict.__annotations__}

    def _get_action_value(self, is_pressed: bool) -> float:
        return self._current_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "u" and is_pressed:
                is_low_scaling = self._current_scaling == self._low_scaling
                self._current_scaling = (
                    self._high_scaling if is_low_scaling else self._low_scaling
                )
                print(f"Command scaling toggled to: {self._current_scaling}")
                continue

            val = self._get_action_value(is_pressed)

            if key == "q":
                self.curr_joint_actions["shoulder_pan_joint"] = val
            elif key == "a":
                self.curr_joint_actions["shoulder_pan_joint"] = -val
            elif key == "w":
                self.curr_joint_actions["shoulder_lift_joint"] = val
            elif key == "s":
                self.curr_joint_actions["shoulder_lift_joint"] = -val
            elif key == "e":
                self.curr_joint_actions["elbow_joint"] = val
            elif key == "d":
                self.curr_joint_actions["elbow_joint"] = -val
            elif key == "r":
                self.curr_joint_actions["wrist_1_joint"] = val
            elif key == "f":
                self.curr_joint_actions["wrist_1_joint"] = -val
            elif key == "t":
                self.curr_joint_actions["wrist_2_joint"] = val
            elif key == "g":
                self.curr_joint_actions["wrist_2_joint"] = -val
            elif key == "y":
                self.curr_joint_actions["wrist_3_joint"] = val
            elif key == "h":
                self.curr_joint_actions["wrist_3_joint"] = -val
            elif is_pressed:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, self.curr_joint_actions)


@TeleoperatorConfig.register_subclass("aic_keyboard_ee")
@dataclass(kw_only=True)
class AICKeyboardEETeleopConfig(KeyboardEndEffectorTeleopConfig):
    high_command_scaling: float = 0.1
    low_command_scaling: float = 0.02


class AICKeyboardEETeleop(KeyboardEndEffectorTeleop):
    def __init__(self, config: AICKeyboardEETeleopConfig):
        super().__init__(config)
        self.config = config

        self._high_scaling = config.high_command_scaling
        self._low_scaling = config.low_command_scaling
        self._current_scaling = self._high_scaling

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    def _get_action_value(self, is_pressed: bool) -> float:
        return self._current_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "t" and is_pressed:
                is_low_speed = self._current_scaling == self._low_scaling
                self._current_scaling = (
                    self._high_scaling if is_low_speed else self._low_scaling
                )
                print(f"Command scaling toggled to: {self._current_scaling}")
                continue

            val = self._get_action_value(is_pressed)

            if key == "w":
                self._current_actions["linear.y"] = -val
            elif key == "s":
                self._current_actions["linear.y"] = val
            elif key == "a":
                self._current_actions["linear.x"] = -val
            elif key == "d":
                self._current_actions["linear.x"] = val
            elif key == "r":
                self._current_actions["linear.z"] = -val
            elif key == "f":
                self._current_actions["linear.z"] = val
            elif key == "W":
                self._current_actions["angular.x"] = val
            elif key == "S":
                self._current_actions["angular.x"] = -val
            elif key == "A":
                self._current_actions["angular.y"] = -val
            elif key == "D":
                self._current_actions["angular.y"] = val
            elif key == "q":
                self._current_actions["angular.z"] = -val
            elif key == "e":
                self._current_actions["angular.z"] = val
            elif is_pressed:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, self._current_actions)


@TeleoperatorConfig.register_subclass("aic_spacemouse")
@dataclass(kw_only=True)
class AICSpaceMouseTeleopConfig(TeleoperatorConfig):
    operator_position_front: bool = True
    device_path: str = None  # only needed for multiple space mice
    command_scaling: float = 0.1


class AICSpaceMouseTeleop(Teleoperator):
    def __init__(self, config: AICSpaceMouseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def name(self) -> str:
        return "aic_spacemouse"

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    @property
    def feedback_features(self) -> dict:
        # TODO
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError()

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("spacemouse_teleop")
        if calibrate:
            self._node.get_logger().warn(
                "Calibration not supported, ensure the robot is calibrated before running teleop."
            )

        self._device_open_success = pyspacemouse.open(
            dof_callback=None,
            # button_callback_arr=[
            #     pyspacemouse.ButtonCallback([0], self._button_callback),  # Button 1
            #     pyspacemouse.ButtonCallback([1], self._button_callback),  # Button 2
            # ],
            path=self.config.device_path,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._executor.spin)
        self._executor_thread.start()
        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        # Calibration not supported
        return True

    def calibrate(self) -> None:
        # Calibration not supported
        pass

    def configure(self) -> None:
        pass

    def apply_deadband(self, value, threshold=0.02):
        return value if abs(value) > threshold else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        state = pyspacemouse.read()

        clean_x = self.apply_deadband(float(state.x))
        clean_y = self.apply_deadband(float(state.y))
        clean_z = self.apply_deadband(float(state.z))
        clean_roll = self.apply_deadband(float(state.roll))
        clean_pitch = self.apply_deadband(float(state.pitch))
        clean_yaw = self.apply_deadband(float(state.yaw))

        twist_msg = Twist()
        twist_msg.linear.x = clean_x**1 * self.config.command_scaling
        twist_msg.linear.y = -(clean_y**1) * self.config.command_scaling
        twist_msg.linear.z = -(clean_z**1) * self.config.command_scaling
        twist_msg.angular.x = -(clean_pitch**1) * self.config.command_scaling
        twist_msg.angular.y = clean_roll**1 * self.config.command_scaling  #
        twist_msg.angular.z = clean_yaw**1 * self.config.command_scaling

        if not self.config.operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        self._current_actions = {
            "linear.x": twist_msg.linear.x,
            "linear.y": twist_msg.linear.y,
            "linear.z": twist_msg.linear.z,
            "angular.x": twist_msg.angular.x,
            "angular.y": twist_msg.angular.y,
            "angular.z": twist_msg.angular.z,
        }

        return cast(dict, self._current_actions)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        pyspacemouse.close()
        self._is_connected = False
        pass
