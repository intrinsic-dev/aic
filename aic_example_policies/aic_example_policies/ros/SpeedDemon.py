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


class SpeedDemon(Policy):
    """Policy that moves the arm rapidly with high stiffness and low
    damping, producing high jerk (low Tier 2 jerk score)."""

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("SpeedDemon.__init__()")

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
            return future.result().success
        return False

    def _publish_joint_command(self, positions, stiffness, damping):
        """Publish a joint-space motion command."""
        msg = JointMotionUpdate()
        msg.target_state.positions = list(positions)
        msg.target_stiffness = list(stiffness)
        msg.target_damping = list(damping)
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
        self._parent_node.joint_motion_update_pub.publish(msg)

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        set_pose_target: SetPoseTargetCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info("SpeedDemon.insert_cable() enter")
        send_feedback("moving fast and aggressively")

        self._switch_target_mode(TargetMode.MODE_JOINT)

        # High stiffness + low damping = fast, jerky motion (high jerk)
        stiffness = [500.0, 500.0, 500.0, 200.0, 200.0, 200.0]
        damping = [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]

        home = [-0.16, -1.35, -1.66, -1.69, 1.57, 1.41]
        target = [0.6, -1.3, -1.9, -1.57, 1.57, 0.6]

        for cycle in range(3):
            self.get_logger().info(f"Cycle {cycle + 1}: snapping to target")
            for _ in range(50):
                self._publish_joint_command(target, stiffness, damping)
                self.sleep_for(0.1)

            self.get_logger().info(f"Cycle {cycle + 1}: snapping back to home")
            for _ in range(50):
                self._publish_joint_command(home, stiffness, damping)
                self.sleep_for(0.1)

        # Settle at home with moderate parameters
        self.get_logger().info("Settling at home position")
        settle_stiffness = [200.0, 200.0, 200.0, 50.0, 50.0, 50.0]
        settle_damping = [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        for _ in range(30):
            self._publish_joint_command(home, settle_stiffness, settle_damping)
            self.sleep_for(0.1)

        self._switch_target_mode(TargetMode.MODE_CARTESIAN)

        self.get_logger().info("SpeedDemon.insert_cable() exiting...")
        return True
