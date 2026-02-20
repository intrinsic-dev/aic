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
from aic_control_interfaces.msg import TargetMode
from aic_control_interfaces.srv import ChangeTargetMode
from aic_model.policy import (
    Policy,
    GetObservationCallback,
    SetPoseTargetCallback,
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

    def _switch_target_mode(self, mode):
        """Switch controller between Cartesian (0) and Joint (1) mode."""
        req = ChangeTargetMode.Request()
        req.target_mode.mode = mode
        future = self._parent_node.change_target_mode_client.call_async(req)
        start = self.time_now()
        timeout = Duration(seconds=5.0)
        while not future.done() and (self.time_now() - start) < timeout:
            time.sleep(0.01)
        if future.done():
            result = future.result()
            self.get_logger().info(
                f"Target mode set to {mode}: success={result.success}"
            )
            return result.success
        self.get_logger().warn("ChangeTargetMode service call timed out")
        return False

    def _publish_joint_command(self, positions, stiffness=None, damping=None):
        """Publish a joint-space motion command."""
        msg = JointMotionUpdate()
        msg.target_state.positions = list(positions)
        msg.target_stiffness = list(
            stiffness or [300.0, 300.0, 300.0, 50.0, 50.0, 50.0]
        )
        msg.target_damping = list(damping or [40.0, 40.0, 40.0, 15.0, 15.0, 15.0])
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
        self._parent_node.joint_motion_update_pub.publish(msg)

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        set_pose_target: SetPoseTargetCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info("WallPresser.insert_cable() enter")
        send_feedback("pressing the enclosure wall to trigger force penalty")

        # Switch to joint target mode
        self.get_logger().info("Switching to joint target mode")
        if not self._switch_target_mode(TargetMode.MODE_JOINT):
            self.get_logger().error("Failed to switch to joint mode")
            return True

        # Joints: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        # Home:   [-0.16,        -1.35,         -1.66, -1.69,   1.57,    1.41]
        #
        # Strategy: rotate shoulder_pan to face the opposite side from
        # WallToucher (-1.57 instead of +1.57), then extend the arm to
        # press the forearm into the enclosure wall.  The enclosure
        # walls are ~0.71 m from center; the UR5e reach is ~0.85 m.
        # This reliably generates sustained force at the F/T sensor
        # without destabilizing the physics engine.

        high_stiffness = [300.0, 300.0, 300.0, 50.0, 50.0, 50.0]

        # Retracted: arm rotated to the left side, folded
        retracted = [-1.57, -1.35, -1.66, -1.69, 1.57, 1.41]
        # Extended: arm stretched toward the left wall
        extended = [-1.57, -0.5, 0.0, -1.69, 1.57, 1.41]

        for cycle in range(3):
            # Retract
            self.get_logger().info(f"Cycle {cycle + 1}: retracting")
            for _ in range(30):
                self._publish_joint_command(retracted)
                self.sleep_for(0.1)

            # Push into the wall and hold
            self.get_logger().info(f"Cycle {cycle + 1}: pushing into wall")
            for _ in range(50):
                self._publish_joint_command(extended, stiffness=high_stiffness)
                self.sleep_for(0.1)

        # Return to home position
        self.get_logger().info("Returning to home position")
        home = [-0.16, -1.35, -1.66, -1.69, 1.57, 1.41]
        for _ in range(50):
            self._publish_joint_command(home)
            self.sleep_for(0.1)

        # Switch back to Cartesian mode for engine reset
        self.get_logger().info("Switching back to Cartesian mode")
        self._switch_target_mode(TargetMode.MODE_CARTESIAN)

        self.get_logger().info("WallPresser.insert_cable() exiting...")
        return True
