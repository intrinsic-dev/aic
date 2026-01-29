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
from .types import MotionUpdateActionDict

GRIPPER_CLOSED_POS = 0.012
GRIPPER_OPEN_POS = 0.025


@TeleoperatorConfig.register_subclass("aic_keyboard")
@dataclass
class AICKeyboardTeleopConfig(KeyboardJointTeleopConfig):
    arm_action_keys: list[str] = field(
        default_factory=lambda: [f"{x}.pos" for x in arm_joint_names]
    )
    action_increment: float = 0.02


class AICKeyboardTeleop(KeyboardJointTeleop):
    def __init__(self, config: KeyboardJointTeleopConfig):
        super().__init__(config)
        # Set initial goals.
        # Not sure if it is a bug or intended, lerobot-ros does not normalize arm joints,
        # it clamps them instead. But it does normalize for gripper.
        self.curr_joint_actions = {
            "shoulder_pan_joint.pos": -0.546,
            "shoulder_lift_joint.pos": -1.703,
            "elbow_joint.pos": -1.291,
        "wrist_1_joint.pos": -1.719,
            "wrist_2_joint.pos": 1.571,
            "wrist_3_joint.pos": -2.116,
            "gripper.pos": GRIPPER_CLOSED_POS,  # closed
        }


@TeleoperatorConfig.register_subclass("aic_keyboard_ee")
@dataclass(kw_only=True)
class AICKeyboardEETeleopConfig(KeyboardEndEffectorTeleopConfig):
    command_scaling: float = 0.1


class AICKeyboardEETeleop(KeyboardEndEffectorTeleop):
    def __init__(self, config: AICKeyboardEETeleopConfig):
        super().__init__(config)
        self.config = config

        self._high_scaling = config.command_scaling
        self._low_scaling = 0.02
        self._is_low_speed = False

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
            "gripper_target": GRIPPER_CLOSED_POS,
        }

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    def _get_action_value(self, is_pressed: bool) -> float:
        return self.config.command_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "t" and is_pressed:
                self._is_low_speed = not self._is_low_speed
                self.config.command_scaling = self._low_scaling if self._is_low_speed else self._high_scaling
                print(f"Command scaling toggled to: {self.config.command_scaling}")
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
                self._current_actions["linear.z"] = val
            elif key == "f":
                self._current_actions["linear.z"] = -val
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
            elif key == "j":
                self._current_actions["gripper_target"] = GRIPPER_CLOSED_POS
            elif key == "k":
                self._current_actions["gripper_target"] = GRIPPER_OPEN_POS
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
    device_path: str = None # only needed for multiple space mice
    command_scaling: float = 0.1


class AICSpaceMouseTeleop(Teleoperator):
    def __init__(self, config: AICSpaceMouseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._target_gripper_target: float = 1.0

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
            "gripper_target": GRIPPER_CLOSED_POS,
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

    def _button_callback(self, state, buttons, pressed_buttons):
        if 0 in pressed_buttons:
            print("Button 1 pressed")
            self._target_gripper_target = 0.0

        elif 1 in pressed_buttons:
            print("Button 2 pressed")
            self._target_gripper_target = 1.0

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
            button_callback_arr=[
                pyspacemouse.ButtonCallback([0], self._button_callback),  # Button 1
                pyspacemouse.ButtonCallback([1], self._button_callback),  # Button 2
            ],
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

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        state = pyspacemouse.read()

        twist_msg = Twist()
        twist_msg.linear.x = -float(state.y) ** 3 * self.config.command_scaling
        twist_msg.linear.y = float(state.x) ** 3 * self.config.command_scaling
        twist_msg.linear.z = float(state.z) ** 3 * self.config.command_scaling
        twist_msg.angular.x = -float(state.roll) ** 3 * self.config.command_scaling
        twist_msg.angular.y = -float(state.pitch) ** 3 * self.config.command_scaling
        twist_msg.angular.z = -float(state.yaw) ** 3 * self.config.command_scaling

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
            "gripper_target": self._target_gripper_target,
        }

        return cast(dict, self._current_actions)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        pyspacemouse.close()
        self._is_connected = False
        pass