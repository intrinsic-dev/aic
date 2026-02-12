#!/usr/bin/env python3

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

"""
This script is used for teleoperation of the robot end-effector Cartesian pose 
using the keyboard.
Note that this script uses pynput to monitor keyboard input which might have issues working 
on the Wayland display server, but has been tested successfully with the X11 display server.
This script can also be run within the pixi environment.  
"""

import sys
import time
import rclpy
from pynput import keyboard
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
from aic_control_interfaces.msg import (
    MotionUpdate,
    TrajectoryGenerationMode,
)
from aic_control_interfaces.srv import (
    ChangeTargetMode,
)
from geometry_msgs.msg import Wrench, Vector3, Twist

SLOW_LINEAR_VEL = 0.02
SLOW_ANGULAR_VEL = 0.02
FAST_LINEAR_VEL = 0.1
FAST_ANGULAR_VEL = 0.1

KEY_MAPPINGS = {
    "d": (1, 0, 0, 0, 0, 0),  # +linear.x
    "a": (-1, 0, 0, 0, 0, 0),  # -linear.x
    "w": (0, -1, 0, 0, 0, 0),  # -linear.y
    "s": (0, 1, 0, 0, 0, 0),  # +linear.y
    "r": (0, 0, -1, 0, 0, 0),  # -linear.z
    "f": (0, 0, 1, 0, 0, 0),  # +linear.z
    "W": (0, 0, 0, 1, 0, 0),  # angular.x
    "S": (0, 0, 0, -1, 0, 0),  # -angular.x
    "A": (0, 0, 0, 0, -1, 0),  # -angular.y
    "D": (0, 0, 0, 0, 1, 0),  # +angular.y
    "e": (0, 0, 0, 0, 0, 1),  # +angular.z
    "q": (0, 0, 0, 0, 0, -1),  # -angular.z
}


class AICCartesianTeleoperatorNode(Node):
    def __init__(self):
        super().__init__("aic_teleoperator_node")

        # Declare parameters.
        self.controller_namespace = self.declare_parameter(
            "controller_namespace", "aic_controller"
        ).value

        self.motion_update_publisher = self.create_publisher(
            MotionUpdate, f"/{self.controller_namespace}/pose_commands", 10
        )

        while self.motion_update_publisher.get_subscription_count() == 0:
            self.get_logger().info(
                f"Waiting for subscriber to '{self.controller_namespace}/pose_commands'..."
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

        # Poll keyboard and send commands at 25Hz
        self.timer = self.create_timer(0.04, self.send_references)

        # Variable parameters for teleoperation
        self.linear_vel = FAST_LINEAR_VEL  # Linear velocity (m/s)
        self.angular_vel = FAST_ANGULAR_VEL  # Angular velocity (rad/s)
        self.frame_id = "gripper/tcp"

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

    def generate_velocity_motion_update(self, twist, frame_id):

        msg = MotionUpdate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.velocity = twist
        msg.target_stiffness = np.diag([85.0, 85.0, 85.0, 85.0, 85.0, 85.0]).flatten()
        msg.target_damping = np.diag([75.0, 75.0, 75.0, 75.0, 75.0, 75.0]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY

        return msg

    def send_references(self):
        # Accumulate inputs from all currently pressed keys to allow for diagonal movement
        input_twist = np.zeros(6)

        teleop_keys_active = False
        activate_slow_mode = False
        activate_fast_mode = False
        toggle_frame_id = False

        for key in self.active_keys:
            if key in KEY_MAPPINGS:
                teleop_keys_active = True
                vals = KEY_MAPPINGS[key]
                input_twist[0:3] += np.array(vals[0:3], dtype=float) * self.linear_vel
                input_twist[3:6] += np.array(vals[3:6], dtype=float) * self.angular_vel
            if key == "n":
                toggle_frame_id = True
                self.frame_id = "gripper/tcp"
            if key == "m":
                toggle_frame_id = True
                self.frame_id = "base_link"
            if key == "k":
                activate_slow_mode = True
                self.linear_vel = SLOW_LINEAR_VEL
                self.angular_vel = SLOW_ANGULAR_VEL
            if key == "l":
                activate_fast_mode = True
                self.linear_vel = FAST_LINEAR_VEL
                self.angular_vel = FAST_ANGULAR_VEL

        twist = Twist()
        twist.linear.x = input_twist[0]
        twist.linear.y = input_twist[1]
        twist.linear.z = input_twist[2]
        twist.angular.x = input_twist[3]
        twist.angular.y = input_twist[4]
        twist.angular.z = input_twist[5]

        self.motion_update_publisher.publish(
            self.generate_velocity_motion_update(twist=twist, frame_id=self.frame_id)
        )

        # Only print logs if relevant keys are pressed
        if teleop_keys_active:
            self.get_logger().info(
                f"Published twist: Translation [{twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f}], Angular [{twist.angular.x:.2f}, {twist.angular.y:.2f}, {twist.angular.z:.2f}]"
            )
        if toggle_frame_id:
            self.get_logger().info(f"Toggled target frame_id to '{self.frame_id}'")
        if activate_slow_mode:
            self.get_logger().info(
                f"Activated slow mode: Linear velocity = {self.linear_vel} m/s, angular velocity = {self.angular_vel} rad/s"
            )
        if activate_fast_mode:
            self.get_logger().info(
                f"Activated fast mode: Linear velocity = {self.linear_vel}, angular velocity = {self.angular_vel} rad/s"
            )

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

    def stop_keyboard_listener(self):
        if self.keyboard_listener:
            self.keyboard_listener.stop()


def main(args=None):

    print(
        f"""
        Keyboard teleoperation for Cartesian control
        ---------------------------
        Linear translation:
            a/d : -/+ Linear X
            w/s : -/+ Linear Y
            r/f : -/+ Linear Z

        Angular rotation:
            Shift + s/w : -/+ Angular X
            Shift + a/d : -/+ Angular Y
            q/e : -/+ Angular Z

        Toggle between SLOW and FAST teleoperation:
            k : Activate SLOW mode ({SLOW_LINEAR_VEL} m/s and {SLOW_ANGULAR_VEL} rad/s)
            l : Activate FAST mode ({FAST_LINEAR_VEL} m/s and {FAST_ANGULAR_VEL} rad/s)

        Toggle target between global and TCP frames:
            n : Use TCP ('gripper/tcp') frame
            m : Use global ('base_link') frame

        Press ESC to quit
        """
    )

    try:
        with rclpy.init(args=args):
            node = AICCartesianTeleoperatorNode()
            node.send_change_control_mode_req(
                ChangeTargetMode.Request().TARGET_MODE_CARTESIAN
            )
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_keyboard_listener()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
