from lerobot.cameras import CameraConfig
from lerobot_robot_ros import ROS2CameraConfig

arm_joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

gripper_joint_name = "gripper/left_finger_joint"

aic_cameras: dict[str, CameraConfig] = {
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
