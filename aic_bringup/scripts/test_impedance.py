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

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from geometry_msgs.msg import Pose, Point, Quaternion

# TODO(johntgz) refactor script for just sending MotionUpdate commands for AIC Controller

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

    def send_motion_update(self):

        quat = [ 0.7071068, 0.7071068, 0.0, 0.0] # ZYX = (180, 0, 90), z axis normal to plane and (x,y) axes are aligned with base_link axes (but not necessarily in the same direction)
        # quat = [0.3826834, 0.9238795, 0., 0.] # ZYX = (180, 0, 135),
        # quat = [0.3535534, 0.8535534, 0.3535534, 0.1464466 ] # ZYX = (135, 0, 135),
        # quat = [ 0.681, 0.681, 0.266, 0.051]

        POSE_HOME = Pose(
            # position=Point(x=-0.3, y=-0.3, z=0.2),
            position=Point(x=-0.35, y=-0.25, z=0.5),
            # position=Point(x=-0.55, y=-0.55, z=0.4),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )

        msg = MotionUpdate()
        msg.pose = POSE_HOME
        msg.target_stiffness = np.diag([
            250.0, 200.0, 550.0, 250.0, 250.0, 100.0]).flatten()
        msg.target_damping = np.diag([
            75.0, 50.0, 150.0, 50.0, 50.0, 50.0]).flatten()
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION
        msg.time_to_target_seconds = 2.0
        self.publisher.publish(msg)
        self.get_logger().info(
            "Published home joint motion update to aic_controller"
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
