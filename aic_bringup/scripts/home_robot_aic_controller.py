#!/usr/bin/env python3

#
#  Copyright (C) 2025 Intrinsic Innovation LLC
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

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Accel, Wrench
from aic_control_interfaces.msg import MotionUpdate, JointMotionUpdate, TrajectoryGenerationMode

JOINT_POSITIONS_HOME = [0.6, -1.3, -1.9, -1.57, 1.57, 0]
class HomeTrajectoryNode(Node):
    def __init__(self):
        super().__init__('home_trajectory_node')

        self.declare_parameter('controller_namespace', '')
        self.declare_parameter('tool_frame', '')

        self.controller_namespace = self.get_parameter('controller_namespace').get_parameter_value().string_value
        self.tool_frame = self.get_parameter('tool_frame').get_parameter_value().string_value

        self.joint_motion_update_publisher = self.create_publisher(
            JointMotionUpdate, self.controller_namespace+'/joint_motion_update', 10)

        while self.joint_motion_update_publisher.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for subscriber to '{self.controller_namespace+'/joint_motion_update'}'...")
            time.sleep(1.0)

    def get_result_callback(self, future):
        rclpy.shutdown()

    def generate_motion_update(self):
        msg = MotionUpdate()

        msg.header.frame_id = self.tool_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose = Pose()
        msg.velocity = Twist()
        msg.acceleration = Accel()

        msg.target_stiffness = []
        msg.target_damping = []
        msg.target_mass = []

        msg.feedforward_wrench_at_tip = Wrench()

        msg.trajectory_generation_mode = TrajectoryGenerationMode.MODE_POSITION

        msg.time_to_target_seconds = 1.0

        return msg

    def generate_joint_motion_update(self, joint_positions, time_to_target):
        msg = JointMotionUpdate()

        msg.target_state.positions = joint_positions

        msg.target_stiffness = []
        msg.target_damping = []

        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION

        msg.time_to_target_seconds = time_to_target

        return msg

    def send_joint_motion_update(self, joint_positions, time_to_target):
        print("Sending joint motion update")
        joint_motion_update_msg = self.generate_joint_motion_update(joint_positions, time_to_target)
        self.joint_motion_update_publisher.publish(joint_motion_update_msg)


def main(args=None):
    rclpy.init(args=args)

    node = HomeTrajectoryNode(JOINT_POSITIONS_HOME, 2.0)

    node.send_joint_motion_update()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
