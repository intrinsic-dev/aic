"""LeRobot driver for AIC robot"""

from dataclasses import dataclass, field

from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig
from lerobot_robot_ros import ROS2Config, ROS2Robot
from lerobot_robot_ros.config import ActionType, ROS2InterfaceConfig
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig


class AICRobot(ROS2Robot):
    name = "ur5e_ros"


arm_joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

gripper_joint_name = "gripper/left_finger_joint"


@RobotConfig.register_subclass("aic_ros")
@dataclass
class AICRobotConfig(ROS2Config):
    action_type: ActionType = ActionType.JOINT_TRAJECTORY

    # TODO: lerobot-ros does not support ROS cameras yet
    # camera: dict[str, CameraConfig] = field(
    #     default_factory=lambda: {
    #         "wrist_1_camera": CameraConfig(fps=30, width=512, height=512),
    #         "wrist_2_camera": CameraConfig(fps=30, width=512, height=512),
    #         "wrist_3_camera": CameraConfig(fps=30, width=512, height=512),
    #     }
    # )

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            base_link="base_link",
            arm_joint_names=arm_joint_names,
            gripper_joint_name=gripper_joint_name,
            # TODO: parallel_gripper_action_controller not supported yet
            # gripper_action_type=GripperActionType.TRAJECTORY,
            gripper_open_position=0.0,
            gripper_close_position=0.024,
            min_joint_positions=[-3.14 for _ in arm_joint_names],
            max_joint_positions=[3.14 for _ in arm_joint_names],
            joint_trajectory_topic="/joint_trajectory_controller/joint_trajectory",
        )
    )


@TeleoperatorConfig.register_subclass("aic_keyboard")
@dataclass
class AICKeyboardTeleopConfig(KeyboardJointTeleopConfig):
    arm_action_keys: list[str] = field(
        default_factory=lambda: [f"{x}.pos" for x in arm_joint_names]
    )


class AICKeyboardTeleop(KeyboardJointTeleop):
    def __init__(self, config: KeyboardJointTeleopConfig):
        super().__init__(config)
        # set initial goals
        self.curr_joint_actions = {
            "shoulder_pan_joint.pos": 0.0,
            "shoulder_lift_joint.pos": -1.30,
            "elbow_joint.pos": -1.91,
            "wrist_1_joint.pos": -1.55,
            "wrist_2_joint.pos": 1.57,
            "wrist_3_joint.pos": 0.0,
            "gripper.pos": 0.02,
        }
