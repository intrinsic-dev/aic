#!/usr/bin/env python3

import rclpy

from action_msgs.msg import GoalStatus
from aic_task_interfaces.action import InsertCable
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CreateAndCancelTaskNode(Node):
    def __init__(self):
        super().__init__("test_create_and_cancel_task")
        self.client = ActionClient(self, InsertCable, "insert_cable")

    def send_goal(self):
        self.get_logger().info("Waiting for insert_cable action server...")
        self.client.wait_for_server()
        goal_msg = InsertCable.Goal()
        self.get_logger().info("Sending goal request...")
        self.send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.goal_handle = goal_handle
        self.get_logger().info("Waiting 5 seconds before canceling goal....")
        self.timer = self.create_timer(5.0, self.timer_callback)
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
            node.send_goal()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
