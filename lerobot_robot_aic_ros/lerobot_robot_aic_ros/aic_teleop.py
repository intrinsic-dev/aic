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
