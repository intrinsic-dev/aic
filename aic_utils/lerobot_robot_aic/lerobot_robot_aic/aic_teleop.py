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

from lerobot.teleoperators import TeleoperatorConfig
from lerobot.teleoperators.keyboard import (
    KeyboardEndEffectorTeleop,
    KeyboardEndEffectorTeleopConfig,
)
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig

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
            if key == "w":
                self._current_actions["linear.y"] = -self._get_action_value(is_pressed)
            elif key == "s":
                self._current_actions["linear.y"] = self._get_action_value(is_pressed)
            elif key == "a":
                self._current_actions["linear.x"] = -self._get_action_value(is_pressed)
            elif key == "d":
                self._current_actions["linear.x"] = self._get_action_value(is_pressed)
            elif key == "r":
                self._current_actions["linear.z"] = -self._get_action_value(is_pressed)
            elif key == "f":
                self._current_actions["linear.z"] = self._get_action_value(is_pressed)
            elif key == "W":
                self._current_actions["angular.x"] = self._get_action_value(is_pressed)
            elif key == "S":
                self._current_actions["angular.x"] = -self._get_action_value(is_pressed)
            elif key == "A":
                self._current_actions["angular.y"] = -self._get_action_value(is_pressed)
            elif key == "D":
                self._current_actions["angular.y"] = self._get_action_value(is_pressed)
            elif key == "q":
                self._current_actions["angular.z"] = -self._get_action_value(is_pressed)
            elif key == "e":
                self._current_actions["angular.z"] = self._get_action_value(is_pressed)
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
