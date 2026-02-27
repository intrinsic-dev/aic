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


class WallPresser(Policy):
    """Policy that presses the arm into an enclosure wall using
    joint-space control to trigger the Tier 2 insertion force penalty.

    The arm is rotated to one side and alternated between a retracted
    and an extended position with high stiffness, pressing the forearm
    into the wall panel.  The sustained contact force exceeds the F/T
    sensor threshold for long enough to trigger the penalty.
    """

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("WallPresser.__init__()")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info("WallPresser.insert_cable() enter")
        send_feedback("pressing the enclosure wall to trigger force penalty")

        # Joints: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        # Home:   [-0.16,        -1.35,         -1.66, -1.69,   1.57,    1.41]
        #
        # Strategy: rotate shoulder_pan to face the opposite side from
        # WallToucher (-1.57 instead of +1.57), then extend the arm to
        # press the forearm into the enclosure wall.  The enclosure
        # walls are ~0.71 m from center; the UR5e reach is ~0.85 m.
        # This reliably generates sustained force at the F/T sensor
        # without destabilizing the physics engine.

        joint_motion_update = JointMotionUpdate(
            target_stiffness=[300.0, 300.0, 300.0, 50.0, 50.0, 50.0],
            target_damping=[40.0, 40.0, 40.0, 15.0, 15.0, 15.0],
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION
            ),
        )

        # Retracted: arm rotated to the left side, folded
        retracted = [-1.57, -1.35, -1.66, -1.69, 1.57, 1.41]
        # Extended: arm stretched toward the left wall
        extended = [-1.57, -0.5, 0.0, -1.69, 1.57, 1.41]

        for cycle in range(3):
            # Retract
            self.get_logger().info(f"Cycle {cycle + 1}: retracting")
            joint_motion_update.target_state.positions = retracted
            for _ in range(30):
                move_robot(joint_motion_update=joint_motion_update)
                self.get_logger().info("about to sleep")
                self.sleep_for(0.1)
                self.get_logger().info("slept")

            # Push into the wall and hold
            self.get_logger().info(f"Cycle {cycle + 1}: pushing into wall")
            for _ in range(50):
                joint_motion_update.target_state.positions = extended
                move_robot(joint_motion_update=joint_motion_update)
                self.sleep_for(0.1)

        # Return to home position
        self.get_logger().info("Returning to home position")
        home = [-0.16, -1.35, -1.66, -1.69, 1.57, 1.41]
        for _ in range(50):
            joint_motion_update.target_state.positions = home
            move_robot(joint_motion_update=joint_motion_update)
            self.sleep_for(0.1)

        self.get_logger().info("WallPresser.insert_cable() exiting...")
        return True
