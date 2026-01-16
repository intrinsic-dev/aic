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
from lerobot.teleoperators.keyboard.teleop_keyboard import keyboard
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig

from .aic_robot import arm_joint_names
from .types import MotionUpdateActionDict, motion_update_action_features


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
            "shoulder_pan_joint.pos": 0.6,
            "shoulder_lift_joint.pos": -1.30,
            "elbow_joint.pos": -1.91,
            "wrist_1_joint.pos": -1.57,
            "wrist_2_joint.pos": 1.57,
            "wrist_3_joint.pos": 0.6,
            "gripper.pos": 1.0,  # open
        }


@TeleoperatorConfig.register_subclass("aic_keyboard_ee")
@dataclass(kw_only=True)
class AICKeyboardEETeleopConfig(KeyboardEndEffectorTeleopConfig):
    pass


class AICKeyboardEETeleop(KeyboardEndEffectorTeleop):
    def __init__(self, config: AICKeyboardEETeleopConfig):
        super().__init__(config)

    @property
    def action_features(self) -> dict:
        return motion_update_action_features()

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()
        actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
            "gripper_width_percent": 0.0,
        }

        for key, val in self.current_pressed.items():
            if key == "w":
                actions["linear.y"] = -1.0
            elif key == "s":
                actions["linear.y"] = 1.0
            elif key == "a":
                actions["linear.x"] = 1.0
            elif key == "d":
                actions["linear.x"] = -1.0
            elif key == "r":
                actions["linear.z"] = -1.0
            elif key == "f":
                actions["linear.z"] = 1.0
            elif key == "j":
                actions["gripper_width_percent"] = 0.0
            elif key == "k":
                actions["gripper_width_percent"] = 1.0
            elif val:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, actions)
