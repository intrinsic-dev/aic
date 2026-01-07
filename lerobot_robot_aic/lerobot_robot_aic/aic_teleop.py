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

from lerobot.teleoperators import TeleoperatorConfig
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig

from .aic_robot import arm_joint_names


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
