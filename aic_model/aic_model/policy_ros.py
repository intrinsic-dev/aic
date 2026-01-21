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


from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from geometry_msgs.msg import Point, Pose, Quaternion, Wrench, Vector3
import numpy as np


class PolicyRos:
    def __init__(self, parent_node):
        self._parent_node = parent_node
        print("PolicyRos.__init__()")

    def start_callback(self, task):
        print("PolicyRos.start_callback()")

    def stop_callback(self):
        print("PolicyRos.stop_callback()")

    def observation_callback(self, observation):
        print("PolicyRos.observation_callback()")

    def set_pose_target(self, pose):
        motion_update_msg = MotionUpdate()
        motion_update_msg.pose = pose

        motion_update_msg.target_stiffness = np.diag(
            [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        ).flatten()
        motion_update_msg.target_damping = np.diag(
            [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        ).flatten()

        motion_update_msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.0)
        )

        motion_update_msg.wrench_feedback_gains_at_tip = Wrench(
            force=Vector3(x=0.5, y=0.5, z=0.5), torque=Vector3(x=0.0, y=0.0, z=0.0)
        )

        motion_update_msg.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_POSITION
        )

        motion_update_msg.time_to_target_seconds = 0.05

        self._parent_node.motion_update_pub.publish(motion_update_msg)

    def get_logger(self):
        return self._parent_node.get_logger()
