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
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from control_msgs.action import ParallelGripperCommand


class GripperActionClient(Node):
    def __init__(self):
        super().__init__("gripper_action_client")

        self.declare_parameter("gripper_name", "")
        self.declare_parameter("gripper_action_name", "")
        self.declare_parameter("use_position", False)
        self.declare_parameter("position", 0.0)
        self.declare_parameter("use_effort", False)
        self.declare_parameter("effort", 0.0)

        self.gripper_name = (
            self.get_parameter("gripper_name").get_parameter_value().string_value
        )
        self.gripper_action_name = (
            self.get_parameter("gripper_action_name").get_parameter_value().string_value
        )
        self.use_position = (
            self.get_parameter("use_position").get_parameter_value().bool_value
        )
        self.position = (
            self.get_parameter("position").get_parameter_value().double_value
        )
        self.use_effort = (
            self.get_parameter("use_effort").get_parameter_value().bool_value
        )
        self.effort = self.get_parameter("effort").get_parameter_value().double_value

        if self.use_position == self.use_effort:
            self.get_logger().info(
                f"Either the 'use_position' or 'use_effort' parameter must be True. Currently, 'use_position' is {self.use_position} and 'use_effort' is {self.use_effort} "
            )
            rclpy.shutdown()

        self.gripper_action_client = ActionClient(
            self, ParallelGripperCommand, self.gripper_action_name
        )

    def send_goal(self):
        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command.name = self.gripper_name
        if self.use_position:
            goal_msg.command.position = [self.position]
        if self.use_effort:
            goal_msg.command.position = [self.position]
            goal_msg.command.effort = [self.effort]

        self.get_logger().info(
            f"Waiting for gripper action controller server '{self.gripper_action_name}'"
        )

        self.gripper_action_client.wait_for_server()

        self.get_logger().info(f"Sending action goal")
        self.send_goal_future = self.gripper_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        return goal_msg

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f"Goal rejected by gripper action controller {self.gripper_action_name}"
            )
            return

        self.get_logger().info(
            f"Goal accepted by gripper action controller {self.gripper_action_name}"
        )

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"[Feedback] Gripper position: {result.state.position[0]}"
        )
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[Feedback] Gripper position: {feedback.state.position[0]}"
        )


def main(args=None):
    try:
        with rclpy.init(args=args):
            action_client = GripperActionClient()

            action_client.send_goal()

            rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main(sys.argv)
