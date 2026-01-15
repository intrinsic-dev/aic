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


import numpy as np
import rclpy
import textwrap

from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_control_interfaces.srv import ChangeTargetMode
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.action import InsertCable
from geometry_msgs.msg import Point, Pose, Quaternion, Wrench, Vector3
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
from trajectory_msgs.msg import JointTrajectoryPoint


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
        self.motion_update_pub = self.create_lifecycle_publisher(
            MotionUpdate, "/aic_controller/pose_commands", 2
        )
        self.change_target_mode_client = self.create_client(
            ChangeTargetMode, "/aic_controller/change_target_mode"
        )
        # while not self.change_target_mode_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Waiting for aic_controller/change_target_mode service....")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_configure({state})")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"activating...")
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
        self.destroy_publisher(self.motion_update_pub)
        self.motion_update_pub = None
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
        # The following sample just prints the timestamps of the incoming data
        # and moves the arm to a specific pose.
        #
        t_cam_0 = self.get_seconds(msg.images[0].header)
        t_cam_1 = self.get_seconds(msg.images[1].header)
        t_cam_2 = self.get_seconds(msg.images[2].header)
        t_joints = self.get_seconds(msg.joint_states.header)
        t_wrench = self.get_seconds(msg.wrist_wrench.header)
        tcp_x = msg.tcp_transform.transform.translation.x
        tcp_y = msg.tcp_transform.transform.translation.y
        tcp_z = msg.tcp_transform.transform.translation.z
        # move the camera back and forth parallel to the Y axis of base_link
        t = self.get_seconds(msg.images[0].header)

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
        target_y = 0.35 + 0.3 * y_fraction
        target_z = 0.3

        self.set_pose_target(
            Pose(
                position=Point(x=target_x, y=target_y, z=target_z),
                orientation=Quaternion(x=0.7071, y=0.7071, z=0.0, w=0.0),
            )
        )

        self.get_logger().info(
            f"tcp: ({tcp_x:+0.3f} {tcp_y:+0.3f}, {tcp_z:+0.3f}) target: ({target_x:+0.3f} {target_y:0.3f} {target_z:0.3f}"
        )

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

        self.motion_update_pub.publish(motion_update_msg)

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
        await self.set_cartesian_mode()

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

    async def set_target_mode(self, target_mode):
        target_mode_request = ChangeTargetMode.Request()
        target_mode_request.target_mode = target_mode
        future = self.change_target_mode_client.call_async(target_mode_request)
        await future
        # rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.success:
            self.get_logger().error("Unable to set target mode")

    async def set_joint_mode(self):
        await self.set_target_mode(ChangeTargetMode.Request.TARGET_MODE_JOINT)

    async def set_cartesian_mode(self):
        await self.set_target_mode(ChangeTargetMode.Request.TARGET_MODE_CARTESIAN)


def main(args=None):
    try:
        with rclpy.init(args=args):
            aic_model = AicModel()
            rclpy.spin(aic_model)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
