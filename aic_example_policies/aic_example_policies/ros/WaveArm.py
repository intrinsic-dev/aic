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


from aic_model.policy_ros import PolicyRos
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration


class WaveArm(PolicyRos):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("WaveArm.__init__()")

    def start_callback(self, task: Task):
        self.get_logger().info("WaveArm.start_callback()")
        self.goal_start_time = self.get_clock().now()

    def stop_callback(self):
        self.get_logger().info("WaveArm.stop_callback()")
        self.goal_start_time = None

    def goal_completed(self) -> bool:
        return self.get_clock().now() - self.goal_start_time >= Duration(seconds=5)

    def observation_callback(self, observation: Observation):
        #
        # Move the arm along a line, while looking down at the task board.
        #
        t = self.get_seconds(observation.left_image.header)
        tcp = observation.tcp_transform.transform.translation

        loop_duration = 5.0  # seconds

        # loop_fraction smoothly interpolates from 0..1 during the loop time
        loop_fraction = (t % loop_duration) / loop_duration

        # y_fraction smoothly interpolates from -1..1..-1 during the loop time
        y_fraction = 2 * loop_fraction
        if y_fraction > 1.0:
            y_fraction = 2.0 - y_fraction
        y_fraction -= 1.0

        # create a smooth series of target points that flies over the task board
        target_x = -0.4
        target_y = 0.45 + 0.3 * y_fraction
        target_z = 0.25

        self.set_pose_target(
            Pose(
                position=Point(x=target_x, y=target_y, z=target_z),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            )
        )

        self.get_logger().info(
            (
                f"tcp: ({tcp.x:+0.3f} {tcp.y:+0.3f}, {tcp.z:+0.3f}) "
                f"target: ({target_x:+0.3f} {target_y:0.3f} {target_z:0.3f})"
            )
        )

    def get_seconds(self, header):
        return header.stamp.sec + header.stamp.nanosec / 1e9

    def get_feedback_string(self) -> str:
        return "Hello, world!"
