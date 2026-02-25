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


import importlib
import inspect
import numpy as np
import rclpy
import threading

from aic_control_interfaces.msg import (
    JointMotionUpdate,
    MotionUpdate,
    TrajectoryGenerationMode,
    TargetMode,
)
from aic_control_interfaces.srv import ChangeTargetMode
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.action import InsertCable
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Wrench, Vector3
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import (
    LifecycleNode,
    LifecycleState,
    LifecyclePublisher,
    TransitionCallbackReturn,
)
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Empty
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class AicModel(LifecycleNode):
    def __init__(self):
        super().__init__("aic_model")
        self.declare_parameter("policy", "WaveArm")
        policy_module_name = (
            self.get_parameter("policy").get_parameter_value().string_value
        )
        self.get_logger().info(f"Loading policy module: {policy_module_name}")
        try:
            policy_module = importlib.import_module(policy_module_name)
        except Exception as e:
            self.get_logger().fatal(f"Unable to load policy {policy_module_name}: {e}")
            raise
        self.get_logger().info(f"Loaded policy module {policy_module_name}")
        policy_module_classes = inspect.getmembers(policy_module, inspect.isclass)
        self._policy_class = None
        self._observation_msg = None
        expected_policy_class_name = policy_module_name.split(".")[-1]
        for policy_class_name, policy_class in policy_module_classes:
            if policy_class_name == expected_policy_class_name:
                self.get_logger().info(f"Using policy: {policy_class_name}")
                self._policy_class = policy_class
        if not self._policy_class:
            self.get_logger().fatal(
                f"Class {expected_policy_class_name} not in module {policy_module_name}"
            )
            raise LookupError(expected_policy_class_name)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            buffer=self._tf_buffer, node=self, spin_thread=True
        )

        self.cancel_service = self.create_service(
            Empty, "cancel_task", self.cancel_task_callback
        )
        self.goal_handle = None
        self.is_active = False
        self.observation_sub = self.create_subscription(
            Observation, "observations", self.observation_callback, 10
        )
        self._action_callback_group = ReentrantCallbackGroup()
        self._action_thread = None
        self._action_thread_result = None
        self.action_server = ActionServer(
            self,
            InsertCable,
            "insert_cable",
            execute_callback=self.insert_cable_execute_callback,
            goal_callback=self.insert_cable_goal_callback,
            handle_accepted_callback=self.insert_cable_accepted_goal_callback,
            cancel_callback=self.insert_cable_cancel_callback,
            callback_group=self._action_callback_group,
        )
        self.motion_update_pub = self.create_lifecycle_publisher(
            MotionUpdate, "/aic_controller/pose_commands", 2
        )
        self.joint_motion_update_pub = self.create_lifecycle_publisher(
            JointMotionUpdate, "/aic_controller/joint_commands", 2
        )
        self._target_mode = TargetMode.MODE_UNSPECIFIED
        self._change_target_mode_client = self.create_client(
            ChangeTargetMode, "/aic_controller/change_target_mode"
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_configure({state})")
        self.get_logger().info(f"Instantiating policy...")
        try:
            self._policy = self._policy_class(self)
        except Exception as e:
            self.get_logger().error(f"Error instantiating policy: {e}")
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_activate()")
        self.is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_deactivate({state})")
        self.is_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_cleanup({state})")
        self.is_active = False
        self._policy = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"on_shutdown({state})")
        self.is_active = False
        self.destroy_publisher(self.joint_motion_update_pub)
        self.joint_motion_update_pub = None
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

    def observation_callback(self, msg):
        self._observation_msg = msg

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
        self.goal_handle = goal_handle
        self.goal_handle.execute()

    def insert_cable_cancel_callback(self, goal_handle):
        self.get_logger().info("Received insert_cable cancel request")
        return CancelResponse.ACCEPT

    def observation_callable(self):
        return self._observation_msg

    def handle_motion_update(self, motion_update: MotionUpdate):
        if self._target_mode != TargetMode.MODE_CARTESIAN:
            self.get_logger().info("Setting cartesian mode...")
            self.set_target_mode(TargetMode.MODE_CARTESIAN)
        self.motion_update_pub.publish(motion_update)
        return True

    def handle_joint_motion_update(self, joint_motion_update: JointMotionUpdate):
        if self._target_mode != TargetMode.MODE_JOINT:
            self.get_logger().info("Setting joint mode...")
            self.set_target_mode(TargetMode.MODE_JOINT)
        self.joint_motion_update_pub.publish(joint_motion_update)
        return True

    def move_robot(
        self,
        motion_update: MotionUpdate = None,
        joint_motion_update: JointMotionUpdate = None,
    ) -> bool:
        """Set a motion target for the robot.

        There are two ways to move the robot: via a cartesian commands or via
        joint-space commands. Within each of those spaces, it is possible to
        provide either position targets or velocity targets.
        """
        if motion_update is not None and joint_motion_update is not None:
            self.get_logger().error(
                "motion_update and joint_motion_update cannot both be provided simultaneously to move_robot()."
            )
            return False

        if motion_update is not None:
            return self.handle_motion_update(motion_update)
        elif joint_motion_update is not None:
            return self.handle_joint_motion_update(joint_motion_update)
        else:
            self.get_logger().error(
                "Either motion_update or joint_motion_update must be provided."
            )
            return False

    def send_feedback(self, goal_handle, feedback):
        feedback_msg = InsertCable.Feedback()
        feedback_msg.message = feedback
        goal_handle.publish_feedback(feedback_msg)

    def action_thread_func(self, goal_handle: ServerGoalHandle):
        self._action_thread_result = self._policy.insert_cable(
            task=goal_handle.request.task,
            get_observation=lambda: self.observation_callable(),
            move_robot=lambda motion_update=None, joint_motion_update=None: self.move_robot(
                motion_update, joint_motion_update
            ),
            send_feedback=lambda feedback: self.send_feedback(goal_handle, feedback),
        )
        if self._action_thread_result is None:
            self.get_logger().warn("insert_cable() returned None. Assuming False...")
            self._action_thread_result = False

    async def insert_cable_execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Entering insert_cable_execute_callback()")
        self._action_thread_result = None
        self._action_thread = threading.Thread(
            target=self.action_thread_func,
            kwargs={
                "goal_handle": goal_handle,
            },
        )
        self._action_thread.start()

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
            if not self._action_thread.is_alive():
                self.get_logger().info(
                    f"insert_cable() returned {self._action_thread_result}"
                )
                goal_handle.succeed()
                result = InsertCable.Result()
                result.success = self._action_thread_result
                self.goal_handle = None
                return result

        self.get_logger().info("Exiting insert_cable execute loop")

    def set_target_mode(self, target_mode):
        target_mode_request = ChangeTargetMode.Request()
        target_mode_request.target_mode.mode = target_mode
        response = self._change_target_mode_client.call(target_mode_request)
        if not response.success:
            self.get_logger().error("Unable to set target mode")
        else:
            self._target_mode = target_mode
            self.get_logger().info("Successfully set target mode")


def main(args=None):
    try:
        with rclpy.init(args=args):
            aic_model_node = AicModel()
            executor = MultiThreadedExecutor()
            executor.add_node(aic_model_node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
