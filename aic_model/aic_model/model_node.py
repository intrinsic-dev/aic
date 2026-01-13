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
import textwrap

from aic_model_interfaces.msg import Observation
from aic_task_interfaces.action import InsertCable
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import (
    LifecycleNode,
    LifecycleState,
    LifecyclePublisher,
    TransitionCallbackReturn,
)
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Empty


class AicModel(LifecycleNode):
    def __init__(self):
        super().__init__("aic_model")
        self.get_logger().info("Hello, world!")
        self.cancel_service = self.create_service(
            Empty, "cancel_task", self.cancel_task_callback
        )
        self.goal_handle = None
        self.goal_completed = False
        self.is_active = False
        self.observation_sub = self.create_subscription(
            Observation, "observations", self.observation_callback, 10
        )
        self.action_server = ActionServer(
            self,
            InsertCable,
            "insert_cable",
            execute_callback=self.insert_cable_execute_callback,
            goal_callback=self.insert_cable_goal_callback,
            handle_accepted_callback=self.insert_cable_accepted_goal_callback,
            cancel_callback=self.insert_cable_cancel_callback,
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_configure({state})")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_activate({state})")
        self.is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_deactivate({state})")
        self.is_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_cleanup({state})")
        self.is_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_shutdown({state})")
        self.is_active = False
        self.destroy_subscription(self.observation_sub)
        self.observation_sub = None
        self.action_server = None
        return TransitionCallbackReturn.SUCCESS

    def cancel_task_callback(self, request, response):
        self.get_logger().info("cancel_task_callback()")
        if self.goal_handle and self.goal_handle.is_active:
            self.goal_handle.abort()
        return Empty.Response()

    def get_seconds(self, header):
        return header.stamp.sec + header.stamp.nanosec / 1e9

    def observation_callback(self, msg):
        if not self.is_active:
            return
        #
        # YOUR CODE HERE.
        #
        # The following sample just prints the timestamps of the incoming data.
        #
        t_cam_0 = self.get_seconds(msg.images[0].header)
        t_cam_1 = self.get_seconds(msg.images[1].header)
        t_cam_2 = self.get_seconds(msg.images[2].header)
        t_joints = self.get_seconds(msg.joint_states.header)
        t_wrench = self.get_seconds(msg.wrist_wrench.header)
        tcp_x = msg.tcp_to_world.transform.translation.x
        tcp_y = msg.tcp_to_world.transform.translation.y
        tcp_z = msg.tcp_to_world.transform.translation.z
        self.get_logger().info(
            f"times: images [{t_cam_0:.2f}, {t_cam_1:.2f}, {t_cam_2:.2f}] joints {t_joints:.2f} wrench {t_wrench:.2f} tcp: ({tcp_x:+0.4f} {tcp_y:+0.4f}, {tcp_z:+0.4f})"
        )

    def insert_cable_goal_callback(self, goal_request):
        if not self.is_active:
            self.get_logger().error("aic_model lifecycle is not in the active state")
            return GoalResponse.REJECT

        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().error(
                "A goal is active and must be canceled before a new insert_cable goal can begin"
            )
            return GoalResponse.REJECT
        else:
            self.get_logger().info("Goal accepted")
            return GoalResponse.ACCEPT

    def insert_cable_accepted_goal_callback(self, goal_handle):
        self.get_logger().info(
            f"Accepted insert_cable goal: {goal_handle.request.task}"
        )
        self.goal_completed = False
        self.goal_handle = goal_handle
        self.goal_handle.execute()

    def insert_cable_cancel_callback(self, goal_handle):
        self.get_logger().info("Received insert_cable cancel request")
        return CancelResponse.ACCEPT

    async def insert_cable_execute_callback(self, goal_handle):
        self.get_logger().info("Entering insert_cable_execute_callback()")
        while rclpy.ok():
            self.get_logger().info("insert_cable execute loop")

            # First, wait a bit so this loop doesn't consume much CPU time.
            # This must be an async wait in order for other callbacks to run.
            wait_future = Future()

            def done_waiting():
                wait_future.set_result(None)

            wait_timer = self.create_timer(1.0, done_waiting, clock=self.get_clock())
            await wait_future
            wait_timer.cancel()
            self.destroy_timer(wait_timer)

            # Check if a cancellation request has arrived.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = InsertCable.Result()
                result.success = False
                result.message = "Canceled via action client"
                self.get_logger().info(
                    "Exiting insert_cable execute loop due to cancellation request."
                )
                self.goal_handle = None
                return result

            # Check if the goal was aborted via the cancel_task service,
            # or if this aic_model node is deactivating or shutting down.
            if not goal_handle.is_active or not self.is_active:
                result = InsertCable.Result()
                result.success = False
                result.message = "Canceled via cancel_task service"
                self.get_logger().info(
                    "Exiting insert_cable execute loop due to cancel_task request."
                )
                self.goal_handle = None
                return result

            # Check if the task has been completed.
            if self.goal_completed:
                self.get_logger().info(
                    "Exiting insert_cable execute loop after success."
                )
                goal_handle.succeed()
                result = InsertCable.Result()
                result.success = True
                self.goal_handle = None
                return result

            # Send a feedback message.
            feedback = InsertCable.Feedback()
            feedback.message = "Here is a feedback message"
            goal_handle.publish_feedback(feedback)

        self.get_logger().info("Exiting insert_cable execute loop")


def main(args=None):
    try:
        with rclpy.init(args=args):
            aic_model = AicModel()
            rclpy.spin(aic_model)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
