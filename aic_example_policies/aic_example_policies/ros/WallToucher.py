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

from aic_control_interfaces.msg import JointMotionUpdate
from aic_control_interfaces.msg import TrajectoryGenerationMode
from aic_control_interfaces.srv import ChangeTargetMode
from aic_model.policy import (
    Policy,
    GetObservationCallback,
    SetPoseTargetCallback,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task


class WallToucher(Policy):
    """Policy that uses joint-space control to extend the robot arm
    into an enclosure wall, triggering off-limit contact detection.

    The arm is rotated to one side (shoulder_pan ≈ 1.57) and then
    extended outward (shoulder_lift ≈ -1.0, elbow ≈ -0.3) to touch
    the forearm against the enclosure wall panel.
    """

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("WallToucher.__init__()")

    def _switch_target_mode(self, mode):
        """Switch controller between Cartesian (0) and Joint (1) mode."""
        req = ChangeTargetMode.Request()
        req.target_mode = mode
        future = self._parent_node.change_target_mode_client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
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
            stiffness or [200.0, 200.0, 200.0, 50.0, 50.0, 50.0]
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
        self.get_logger().info("WallToucher.insert_cable() enter")
        send_feedback("triggering off-limit contacts via joint control")

        # Switch to joint target mode
        self.get_logger().info("Switching to joint target mode")
        if not self._switch_target_mode(ChangeTargetMode.Request.TARGET_MODE_JOINT):
            self.get_logger().error("Failed to switch to joint mode")
            return True

        # Home: [-0.16, -1.35, -1.66, -1.69, 1.57, 1.41]
        # Joints: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        #
        # Strategy: rotate shoulder_pan to face a side wall (1.57),
        # then extend the arm outward (straighten shoulder_lift and
        # elbow) to press the forearm into the enclosure wall panel.
        # The enclosure walls are ~0.71 m from center; the UR5e reach
        # is ~0.85 m, so near-full extension is needed.

        high_stiffness = [300.0, 300.0, 300.0, 50.0, 50.0, 50.0]

        # Arm pointing sideways, retracted
        retracted = [1.57, -1.35, -1.66, -1.69, 1.57, 1.41]
        # Arm pointing sideways, fully extended toward the wall
        extended = [1.57, -0.5, 0.0, -1.69, 1.57, 1.41]

        for cycle in range(3):
            # Retract the arm
            self.get_logger().info(f"Cycle {cycle + 1}: retracting arm")
            for _ in range(30):
                self._publish_joint_command(retracted)
                time.sleep(0.1)

            # Extend arm toward the wall
            self.get_logger().info(f"Cycle {cycle + 1}: extending toward wall")
            for _ in range(50):
                self._publish_joint_command(extended, stiffness=high_stiffness)
                time.sleep(0.1)

        # Return to home position
        self.get_logger().info("Returning to home position")
        home = [-0.16, -1.35, -1.66, -1.69, 1.57, 1.41]
        for _ in range(50):
            self._publish_joint_command(home)
            time.sleep(0.1)

        # Switch back to Cartesian mode for engine reset
        self.get_logger().info("Switching back to Cartesian mode")
        self._switch_target_mode(ChangeTargetMode.Request.TARGET_MODE_CARTESIAN)

        self.get_logger().info("WallToucher.insert_cable() exiting...")
        return True
