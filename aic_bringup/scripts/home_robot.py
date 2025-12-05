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
from rclpy.executors import ExternalShutdownException

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from aic_control_interfaces.msg import JointMotionUpdate, TrajectoryGenerationMode

class HomeTrajectoryNode(Node):
    def __init__(self):
        super().__init__('home_trajectory_node')
        self.get_logger().info('HomeTrajectoryNode started')

        # Declare parameters.
        self.use_aic_control = self.declare_parameter('use_aic_controller', False).value
        self.controller_namespace = self.declare_parameter('controller_namespace', 'aic_controller').value
        self.home_joint_positions = [0.0, -1.3, -1.9, -1.57, 1.57, 0.0]
        # Create publisher if needed.
        if self.use_aic_control:
            self.publisher = self.create_publisher(
                JointMotionUpdate, f'/{self.controller_namespace}/joint_motion_update', 10)

            while self.publisher.get_subscription_count() == 0:
                self.get_logger().info(
                    f"Waiting for subscriber to '{self.controller_namespace}/joint_motion_update'..."
                )
                time.sleep(1.0)

        else:
            # todo(Yadunund): We could also directly publish a JouintTrajectory message
            # to /joint_trajectory_controller/joint_trajectory.
            self.action_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/joint_trajectory_controller/follow_joint_trajectory')
            while not self.action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(
                    f'Waiting for {self.action_client._action_name}')

        # A timer that will send the trajectory only once.
        self.timer = self.create_timer(1.0, self.send_trajectory)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Home trajectory goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        rclpy.shutdown()

    def send_trajectory(self):
        if self.use_aic_control:
            msg = JointMotionUpdate()
            # Home joints configuration
            msg.target_state.positions = self.home_joint_positions
            msg.target_state.time_from_start.sec = 2
            msg.target_stiffness = []
            msg.target_damping = []
            msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
            msg.time_to_target_seconds = 2.0
            self.publisher.publish(msg)
            self.get_logger().info('Published home joint motion update to aic_controller')
            # Shutdown after a short delay to ensure message is sent.
            time.sleep(1.0)
        else:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint',
            ]
            home_point = JointTrajectoryPoint()
            home_point.positions = self.home_joint_positions
            home_point.time_from_start.sec = 1
            goal.trajectory.points.append(home_point)
            self.send_goal_future = self.action_client.send_goal_async(goal)
            self.send_goal_future.add_done_callback(self.goal_response_callback)

        self.timer.cancel()  # Send only once.


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = HomeTrajectoryNode()
            node.send_trajectory()
            if node.use_aic_control:
                # Keep alive for a short duration to ensure message delivery.
                rclpy.spin_once(node, timeout_sec=2.0)
                rclpy.shutdown()
            else:
                rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main(sys.argv)
