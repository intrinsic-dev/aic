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

import time

import numpy as np
from aic_control_interfaces.msg import MotionUpdate
from aic_control_interfaces.msg import TrajectoryGenerationMode
from aic_model.policy_ros import (
    PolicyRos,
    GetObservationCallback,
    SetPoseTargetCallback,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Wrench


class Jackhammer(PolicyRos):
    """Policy that hammers the robot gripper up and down against the task
    board to trigger excessive force on the F/T sensor."""

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("Jackhammer.__init__()")

    def _publish_pose(self, x: float, y: float, z: float, stiffness: float = 1500.0):
        """Publish a Cartesian pose command with specified stiffness."""
        msg = MotionUpdate()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self._parent_node.get_clock().now().to_msg()
        msg.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        )
        msg.target_stiffness = np.diag(
            [stiffness, stiffness, stiffness, 50.0, 50.0, 50.0]
        ).flatten()
        msg.target_damping = np.diag([80.0, 80.0, 80.0, 10.0, 10.0, 10.0]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = Wrench(
            force=Vector3(x=0.5, y=0.5, z=0.5),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
        self._parent_node.motion_update_pub.publish(msg)

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        set_pose_target: SetPoseTargetCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info("Jackhammer.insert_cable() enter")
        send_feedback("jackhammering the task board")

        # Gripper points down. The task board surface is near base_link z=0.0.
        # Push the gripper into the board so robot finger links collide with it.
        for cycle in range(3):
            # Move above the board first
            self.get_logger().info(f"Cycle {cycle + 1}: positioning above board")
            self._publish_pose(-0.3, 0.4, 0.15, stiffness=1500.0)
            time.sleep(3.0)

            # Push down into the board with continuous commands
            self.get_logger().info(f"Cycle {cycle + 1}: pushing into board")
            for _ in range(50):
                self._publish_pose(-0.3, 0.4, -0.10, stiffness=1500.0)
                time.sleep(0.1)

        # Return to safe position before exiting so engine reset succeeds
        self.get_logger().info("Returning to safe position")
        self._publish_pose(-0.3, 0.4, 0.20, stiffness=1500.0)
        time.sleep(3.0)

        self.get_logger().info("Jackhammer.insert_cable() exiting...")
        return True
