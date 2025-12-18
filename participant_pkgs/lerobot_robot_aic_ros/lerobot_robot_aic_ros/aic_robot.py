"""LeRobot driver for AIC robot"""

from dataclasses import dataclass, field
from math import pi

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig
from lerobot_robot_ros import ROS2CameraConfig, ROS2Config, ROS2Robot
from lerobot_robot_ros.config import ActionType, ROS2InterfaceConfig
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig

from .aic_interface import AICInterface

arm_joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

gripper_joint_name = "gripper/left_finger_joint"


class AICRobot(ROS2Robot):
    name = "ur5e_ros"

    def __init__(self, config: ROS2Config):
        super().__init__(config)
        self.ros2_interface: AICInterface = AICInterface(
            config.ros2_interface, config.action_type
        )


@RobotConfig.register_subclass("aic_ros")
@dataclass
class AICRobotConfig(ROS2Config):
    action_type: ActionType = ActionType.JOINT_TRAJECTORY

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "wrist_camera_1": ROS2CameraConfig(
                fps=30,
                width=1420,
                height=1420,
                topic="/wrist_camera_1/image",
            ),
            "wrist_camera_2": ROS2CameraConfig(
                fps=30,
                width=1420,
                height=1420,
                topic="/wrist_camera_2/image",
            ),
            "wrist_camera_3": ROS2CameraConfig(
                fps=30,
                width=1420,
                height=1420,
                topic="/wrist_camera_3/image",
            ),
        }
    )

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            base_link="base_link",
            arm_joint_names=arm_joint_names,
            gripper_joint_name=gripper_joint_name,
            gripper_open_position=0.0,
            gripper_close_position=0.024,
            gripper_action_name="/gripper_action_controller/gripper_cmd",
            min_joint_positions=[-2 * pi for _ in arm_joint_names],
            max_joint_positions=[2 * pi for _ in arm_joint_names],
            joint_trajectory_topic="/joint_trajectory_controller/joint_trajectory",
        )
    )


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
            "wrist_3_joint.pos": 0.0,
            "gripper.pos": 1.0,  # open
        }
