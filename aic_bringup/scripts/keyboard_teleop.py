# Publish Cartesian targets to the AICController via messages of type MotionUpdate

import sys
import termios
import tty
import time
import rclpy
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
from geometry_msgs.msg import Pose, Point, Quaternion, Wrench, Vector3, Twist


# --- Configuration ---
LINEAR_STEP = 0.1  # Step size for linear movement (meters)
ANGULAR_STEP = 0.5  # Step size for angular movement (radians)

# Key mappings based on your instructions
KEY_MAPPINGS = {
    "w": (1, 0, 0, 0, 0, 0),  # +x
    "s": (-1, 0, 0, 0, 0, 0),  # -x
    "a": (0, 1, 0, 0, 0, 0),  # +y
    "d": (0, -1, 0, 0, 0, 0),  # -y
    "r": (0, 0, 1, 0, 0, 0),  # +z
    "f": (0, 0, -1, 0, 0, 0),  # -z
    "y": (0, 0, 0, 1, 0, 0),  # +roll (around X)
    "h": (0, 0, 0, -1, 0, 0),  # -roll (around X)
    "u": (0, 0, 0, 0, 1, 0),  # +pitch (around Y)
    "j": (0, 0, 0, 0, -1, 0),  # -pitch (around Y)
    "i": (0, 0, 0, 0, 0, 1),  # +yaw (around Z)
    "k": (0, 0, 0, 0, 0, -1),  # -yaw (around Z)
}

INSTRUCTION_MSG = """
Control Your Robot End-Effector!
---------------------------
Moving around:
    W/S : +/- X axis
    A/D : +/- Y axis
    R/F : +/- Z axis

Rotating:
    Y/H : +/- Roll (around X)
    U/J : +/- Pitch (around Y)
    I/K : +/- Yaw (around Z)

Press any other key to stop the robot.

CTRL-C to quit
"""


class AICTeleoperatorNode(Node):
    def __init__(self):
        super().__init__("aic_teleoperator_node")
        self.get_logger().info("AICTeleoperatorNode started")

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

        # Save terminal settings to restore them later
        self.settings = termios.tcgetattr(sys.stdin)

        # Poll keyboard and send commands at 50Hz
        self.timer = self.create_timer(0.02, self.send_references)

        self.get_logger().info("Initialized AICTeleoperatorNode!")

    def get_key(self):
        """Captures a single keypress from stdin without waiting for return."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def generate_motion_update(
        self,
        pos,
        quat,
        time_to_target,
        mode=TrajectoryGenerationMode.MODE_POSITION,
        twist=None,
    ):

        msg = MotionUpdate()
        if mode == TrajectoryGenerationMode.MODE_POSITION:
            msg.header.frame_id = "base_link"
            msg.pose = Pose(
                position=Point(x=pos[0], y=pos[1], z=pos[2]),
                orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
            )
        elif mode == TrajectoryGenerationMode.MODE_VELOCITY:
            msg.header.frame_id = "gripper/tcp"
            msg.velocity = twist
        msg.target_stiffness = np.diag([175.0, 175.0, 175.0, 175.0, 175.0, 175.0]).flatten()
        msg.target_damping = np.diag([35.0, 35.0, 35.0, 35.0, 35.0, 35.0]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.trajectory_generation_mode.mode = mode

        return msg

    def send_references(self):
        twist = Twist()

        key = self.get_key()

        if key == "\x03":  # CTRL-C
            self.timer.cancel()

        # Check if key is in our mapping (convert to lowercase to be safe)
        if key.lower() in KEY_MAPPINGS:
            x, y, z, roll, pitch, yaw = KEY_MAPPINGS[key.lower()]

            twist.linear.x = x * LINEAR_STEP
            twist.linear.y = y * LINEAR_STEP
            twist.linear.z = z * LINEAR_STEP
            twist.angular.x = roll * ANGULAR_STEP
            twist.angular.y = pitch * ANGULAR_STEP
            twist.angular.z = yaw * ANGULAR_STEP

        else:
            twist = Twist()

        self.motion_update_publisher.publish(
            self.generate_motion_update(
                None,
                None,
                0.0,
                mode=TrajectoryGenerationMode.MODE_VELOCITY,
                twist=twist,
            )
        )

        self.get_logger().info(
            f"Published twist: {twist.linear.x}, {twist.linear.y}, {twist.linear.z}, {twist.angular.x}, {twist.angular.y}, {twist.angular.z}"
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


def main(args=None):

    print(INSTRUCTION_MSG)

    try:
        with rclpy.init(args=args):
            node = AICTeleoperatorNode()
            node.send_change_control_mode_req(
                ChangeTargetMode.Request().TARGET_MODE_CARTESIAN
            )
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
