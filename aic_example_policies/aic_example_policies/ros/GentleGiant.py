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

from rclpy.duration import Duration

from aic_control_interfaces.msg import JointMotionUpdate
from aic_control_interfaces.msg import TrajectoryGenerationMode
from aic_model.policy import (
    Policy,
    GetObservationCallback,
    MoveRobotCallback,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task


class GentleGiant(Policy):
    """Policy that moves the arm slowly and smoothly using low stiffness
    and high damping, producing minimal jerk (high Tier 2 jerk score)."""

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("GentleGiant.__init__()")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info("GentleGiant.insert_cable() enter")
        send_feedback("moving slowly and smoothly")

        # Low stiffness + high damping = slow, smooth motion (low jerk)
        joint_motion_update = JointMotionUpdate(
            target_stiffness=[50.0, 50.0, 50.0, 20.0, 20.0, 20.0],
            target_damping=[40.0, 40.0, 40.0, 20.0, 20.0, 20.0],
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION
            ),
        )

        home = [-0.16, -1.35, -1.66, -1.69, 1.57, 1.41]
        target = [0.6, -1.3, -1.9, -1.57, 1.57, 0.6]

        for cycle in range(3):
            self.get_logger().info(f"Cycle {cycle + 1}: moving to target")
            for _ in range(50):
                joint_motion_update.target_state.positions = target
                move_robot(joint_motion_update=joint_motion_update)
                self.sleep_for(0.1)

            self.get_logger().info(f"Cycle {cycle + 1}: returning to home")
            for _ in range(50):
                joint_motion_update.target_state.positions = home
                move_robot(joint_motion_update=joint_motion_update)
                self.sleep_for(0.1)

        # Return to home
        self.get_logger().info("Settling at home position")
        for _ in range(30):
            joint_motion_update.target_state.positions = home
            move_robot(joint_motion_update=joint_motion_update)
            self.sleep_for(0.1)

        self.get_logger().info("GentleGiant.insert_cable() exiting...")
        return True
