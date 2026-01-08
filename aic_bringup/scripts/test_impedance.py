#!/usr/bin/env python3

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

import sys
import time
import rclpy
import numpy as np
from rclpy.executors import ExternalShutdownException

from rclpy.node import Node
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from geometry_msgs.msg import Pose, Point, Quaternion, Wrench, Vector3


class TestImpedanceNode(Node):
    def __init__(self):
        super().__init__("test_impedance_node")
        self.get_logger().info("TestImpedanceNode started")

        # Declare parameters.
        self.controller_namespace = self.declare_parameter(
            "controller_namespace", "aic_controller"
        ).value
        # Create publisher if needed.
        self.publisher = self.create_publisher(
            MotionUpdate, f"/{self.controller_namespace}/motion_update", 10
        )

        while self.publisher.get_subscription_count() == 0:
            self.get_logger().info(
                f"Waiting for subscriber to '{self.controller_namespace}/motion_update'..."
            )
            time.sleep(1.0)

        # A timer that will send the trajectory only once.
        self.timer = self.create_timer(1.0, self.send_motion_update)

    def generate_motion_update(self, pos, quat, time_to_target):

        msg = MotionUpdate()
        msg.pose = Pose(
            position=Point(x=pos[0], y=pos[1], z=pos[2]),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )
        msg.target_stiffness = np.diag(
            [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        ).flatten()
        msg.target_damping = np.diag([40.0, 40.0, 40.0, 15.0, 15.0, 15.0]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=1.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = Wrench(
            force=Vector3(x=0.5, y=0.5, z=0.5),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
        msg.time_to_target_seconds = time_to_target

        return msg

    def send_motion_update(self):
        pos_tool_up = [-0.501, -0.175, 0.2]
        pos_tool_down = [-0.501, -0.175, 0.0]

        quat_upright = [
            0.7071068,
            0.7071068,
            0.0,
            0.0,
        ]  # ZYX = (180, 0, 90), z axis normal to plane and (x,y) axes are aligned with base_link axes

        self.publisher.publish(
            self.generate_motion_update(pos_tool_up, quat_upright, time_to_target=2.0)
        )
        self.get_logger().info(
            "Published MotionUpdate for tool up configuration to aic_controller"
        )

        time.sleep(5.0)

        self.publisher.publish(
            self.generate_motion_update(pos_tool_down, quat_upright, time_to_target=2.0)
        )
        self.get_logger().info(
            "Published MotionUpdate for tool down configuration to aic_controller"
        )

        # Shutdown after a short delay to ensure message is sent.
        time.sleep(1.0)

        self.timer.cancel()  # Send only once.


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = TestImpedanceNode()
            node.send_motion_update()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main(sys.argv)
