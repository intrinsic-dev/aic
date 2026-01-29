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
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
from aic_control_interfaces.msg import (
    JointMotionUpdate,
    TrajectoryGenerationMode,
)
from aic_control_interfaces.srv import (
    ChangeTargetMode,
)
from geometry_msgs.msg import Pose, Point, Quaternion, Wrench, Vector3, Twist
from sensor_msgs.msg import JointState

STEP = 0.05  # Step size for linear movement (meters)

KEY_MAPPINGS = {
    "q": (1, 0, 0, 0, 0, 0),  # +j1
    "a": (-1, 0, 0, 0, 0, 0),  # -j1
    "w": (0, 1, 0, 0, 0, 0),  # +j2
    "s": (0, -1, 0, 0, 0, 0),  # -j3
    "e": (0, 0, 1, 0, 0, 0),  # +j3
    "d": (0, 0, -1, 0, 0, 0),  # -j3
    "r": (0, 0, 0, 1, 0, 0),  # +j4
    "f": (0, 0, 0, -1, 0, 0),  # -j4
    "t": (0, 0, 0, 0, 1, 0),  # +j5
    "g": (0, 0, 0, 0, -1, 0),  # -j5
    "y": (0, 0, 0, 0, 0, 1),  # +j6
    "h": (0, 0, 0, 0, 0, -1),  # -j6
}


class AICTeleoperatorNode(Node):
    def __init__(self):
        super().__init__("aic_teleoperator_node")
        self.get_logger().info("AICTeleoperatorNode started")

        # Declare parameters.
        self.controller_namespace = self.declare_parameter(
            "controller_namespace", "aic_controller"
        ).value

        # Create the subscription
        self.joint_state_subscription = self.create_subscription(
            JointState,  # Message type
            "joint_states",  # Topic name
            self.joint_state_callback,
            10,
        )  # QoS Profile (History depth)

        self.joint_motion_update_publisher = self.create_publisher(
            JointMotionUpdate, f"/{self.controller_namespace}/joint_commands", 10
        )

        while self.joint_motion_update_publisher.get_subscription_count() == 0:
            self.get_logger().info(
                f"Waiting for subscriber to '{self.controller_namespace}/joint_commands'..."
            )
            time.sleep(1.0)

        self.client = self.create_client(
            ChangeTargetMode, f"/{self.controller_namespace}/change_target_mode"
        )

        # Wait for service
        while not self.client.wait_for_service():
            self.get_logger().info(
                f"Waiting for service '{self.controller_namespace}/change_target_mode'..."
            )
            time.sleep(1.0)

        # Save terminal settings to restore them later
        self.settings = termios.tcgetattr(sys.stdin)

        # Poll keyboard and send commands at 50Hz
        self.timer = self.create_timer(0.02, self.send_references)

        self.joint_map = None

        self.get_logger().info("Initialized AICTeleoperatorNode!")

    def joint_state_callback(self, msg):
        if not msg.position:
            return

        self.joint_map = dict(zip(msg.name, msg.position))

    def get_key(self):
        """Captures a single keypress from stdin without waiting for return."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def generate_joint_motion_update(self, joint_pos):
        msg = JointMotionUpdate()

        msg.target_state.positions = joint_pos
        msg.target_stiffness = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        msg.target_damping = [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION

        return msg

    def send_references(self):
        joint_pos = [
            self.joint_map["shoulder_pan_joint"],
            self.joint_map["shoulder_lift_joint"],
            self.joint_map["elbow_joint"],
            self.joint_map["wrist_1_joint"],
            self.joint_map["wrist_2_joint"],
            self.joint_map["wrist_3_joint"],
        ]

        key = self.get_key()

        if key == "\x03":  # CTRL-C
            self.timer.cancel()

        # Check if key is in our mapping (convert to lowercase to be safe)
        if key.lower() in KEY_MAPPINGS:
            j1, j2, j3, j4, j5, j6 = KEY_MAPPINGS[key.lower()]

            joint_pos[0] += j1 * STEP
            joint_pos[1] += j2 * STEP
            joint_pos[2] += j3 * STEP
            joint_pos[3] += j4 * STEP
            joint_pos[4] += j5 * STEP
            joint_pos[5] += j6 * STEP

        self.joint_motion_update_publisher.publish(
            self.generate_joint_motion_update(joint_pos)
        )

        self.get_logger().info(f"Published joint position: {joint_pos}")

    def send_change_control_mode_req(self, mode):
        ChangeTargetMode

        req = ChangeTargetMode.Request()
        req.target_mode = mode

        self.get_logger().info(f"Sending request to change control mode to {mode}")

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response.success:
            self.get_logger().info(f"Successfully changed control mode to {mode}")
        else:
            self.get_logger().info(f"Failed to change control mode to {mode}")

        time.sleep(0.5)


def main(args=None):

    try:
        with rclpy.init(args=args):
            node = AICTeleoperatorNode()
            node.send_change_control_mode_req(
                ChangeTargetMode.Request().TARGET_MODE_JOINT
            )
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
