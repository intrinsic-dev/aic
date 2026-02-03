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

import rclpy

from action_msgs.msg import GoalStatus
from aic_task_interfaces.action import InsertCable
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CreateAndCancelTaskNode(Node):
    def __init__(self):
        super().__init__("test_create_and_cancel_task")
        self.action_client = ActionClient(self, InsertCable, "insert_cable")
        self.get_state_client = self.create_client(GetState, "aic_model/get_state")
        self.change_state_client = self.create_client(
            ChangeState, "aic_model/change_state"
        )

    def get_model_state(self):
        future = self.get_state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        return future.result()

    def change_model_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result and result.success:
            self.get_logger().info(f"transitioned to {transition_id} successfully")
            return True
        else:
            self.get_logger().error(f"unable to transition to {transition_id}")
            return False

    def activate_model_node(self):
        model_state = self.get_model_state().current_state
        print(f"model_state: {model_state}")

        if model_state.id == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info("Requesting aic_model to configure...")
            self.change_model_state(Transition.TRANSITION_CONFIGURE)
            model_state = self.get_model_state().current_state

        if model_state.id == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info("Requesting aic_model to activate...")
            self.change_model_state(Transition.TRANSITION_ACTIVATE)
            model_state = self.get_model_state().current_state

        return model_state.id == State.PRIMARY_STATE_ACTIVE

    def send_goal(self):
        self.get_logger().info("Waiting for insert_cable action server...")
        self.action_client.wait_for_server()
        goal_msg = InsertCable.Goal()
        goal_msg.task.id = "test_task"
        goal_msg.task.cable_type = "sfp_sc"
        goal_msg.task.cable_name = "cable_0"
        goal_msg.task.plug_type = "sfp"
        goal_msg.task.plug_name = "sfp_plug"
        goal_msg.task.port_type = "sfp"
        goal_msg.task.port_name = "sfp_port_0"
        goal_msg.task.target_module_name = "nic_card_0"
        goal_msg.task.time_limit = 300
        self.get_logger().info("Sending goal request...")
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            rclpy.shutdown()
            return
        self.goal_handle = goal_handle
        self.get_logger().info("Waiting 30 seconds before canceling goal....")
        self.timer = self.create_timer(30.0, self.timer_callback)
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Received feedback: {feedback}")

    def timer_callback(self):
        self.get_logger().info("Canceling goal")
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)
        self.timer.cancel()

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().info("Goal failed to cancel")

        self.change_model_state(Transition.TRANSITION_DEACTIVATE)

        rclpy.shutdown()

    def get_result_callback(self, future):
        self.get_logger().info("Received result")
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal succeeded. Result: {result}")
        else:
            self.get_logger().info(f"Goal failed. Status: {status} result: {result}")
        rclpy.shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = CreateAndCancelTaskNode()
            node.activate_model_node()
            node.send_goal()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
