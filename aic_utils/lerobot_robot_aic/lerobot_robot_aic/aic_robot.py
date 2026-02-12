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

aic_cameras: dict[str, CameraConfig] = {
    "left_camera": ROS2CameraConfig(
        name="left_camera",
        fps=20,
        width=1152,
        height=1024,
        topic="/left_camera/image",
    ),
    "center_camera": ROS2CameraConfig(
        name="center_camera",
        fps=20,
        width=1152,
        height=1024,
        topic="/center_camera/image",
    ),
    "right_camera": ROS2CameraConfig(
        name="right_camera",
        fps=20,
        width=1152,
        height=1024,
        topic="/right_camera/image",
    ),
}
