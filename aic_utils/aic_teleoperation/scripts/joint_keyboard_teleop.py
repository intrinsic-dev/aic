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
from pynput import keyboard
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

ANGULAR_STEP = 0.025  # Step size for incrementing/decrementing angular velocity (rad/s)

MIN_ANGULAR_VEL = 0.0  # rad/s
MAX_ANGULAR_VEL = 2.0  # rad/s

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

        # Track currently pressed keys
        self.active_keys = set()

        # Start the keyboard listener in a non-blocking way
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_key_press, on_release=self.on_key_release
        )
        self.keyboard_listener.start()

        # Poll keyboard and send commands at 50Hz
        self.timer = self.create_timer(0.02, self.send_references)

        # Variable parameters for teleoperation
        self.angular_vel = 0.2  # Angular velocity (rad/s)

    def on_key_press(self, key):
        """Callback for keyboard listener when a key is pressed."""
        try:
            # We use char.lower() to handle standard keys
            if hasattr(key, "char") and key.char is not None:
                k = key.char
                self.active_keys.add(k)
        except AttributeError:
            pass

    def on_key_release(self, key):
        """Callback for keyboard listener when a key is released."""
        try:
            if hasattr(key, "char") and key.char is not None:
                k = key.char
                if k in self.active_keys:
                    self.active_keys.remove(k)
        except AttributeError:
            pass

        if key == keyboard.Key.esc:
            rclpy.shutdown()

    def generate_joint_motion_update(self, velocities):
        msg = JointMotionUpdate()

        msg.target_state.velocities = velocities
        msg.target_stiffness = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        msg.target_damping = [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY

        return msg

    def send_references(self):
        velocities = np.zeros(6)

        teleop_keys_active = False
        scale_angular_velocity = False

        for key in self.active_keys:
            if key in KEY_MAPPINGS:
                teleop_keys_active = True
                vals = KEY_MAPPINGS[key]
                velocities += np.array(vals, dtype=float) * self.angular_vel
            if key == "l":
                scale_angular_velocity = True
                self.angular_vel -= ANGULAR_STEP
            if key == "o":
                scale_angular_velocity = True
                self.angular_vel += ANGULAR_STEP

        self.joint_motion_update_publisher.publish(
            self.generate_joint_motion_update(velocities)
        )

        if not (MIN_ANGULAR_VEL < self.angular_vel < MAX_ANGULAR_VEL):
            self.get_logger().info(
                f"Angular velocity is scaled to {self.angular_vel} which is beyond the range of [{MIN_ANGULAR_VEL:.2f}, {MAX_ANGULAR_VEL:.2f}]. Clamping to minimum and maximum values."
            )
            self.angular_vel = np.clip(
                self.angular_vel,
                MIN_ANGULAR_VEL + ANGULAR_STEP,
                MAX_ANGULAR_VEL - ANGULAR_STEP,
            )

        # Only print logs if relevant keys are pressed
        if teleop_keys_active:
            self.get_logger().info(
                f"Published joint velocities: [{velocities[0]:.2f}, {velocities[1]:.2f}, {velocities[2]:.2f}, {velocities[3]:.2f}, {velocities[4]:.2f}, {velocities[5]:.2f}]"
            )
        if scale_angular_velocity:
            self.get_logger().info(f"Scaled angular velocity to {self.angular_vel:.2f}")

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
