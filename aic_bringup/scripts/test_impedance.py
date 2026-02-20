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
from aic_control_interfaces.msg import (
    MotionUpdate,
    JointMotionUpdate,
    TrajectoryGenerationMode,
    TargetMode,
)
from aic_control_interfaces.srv import (
    ChangeTargetMode,
)
from geometry_msgs.msg import Pose, Point, Quaternion, Wrench, Vector3, Twist


class TestImpedanceNode(Node):
    def __init__(self):
        super().__init__("test_impedance_node")
        self.get_logger().info("TestImpedanceNode started")

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

        self.joint_motion_update_publisher = self.create_publisher(
            JointMotionUpdate,
            f"/{self.controller_namespace}/joint_commands",
            10,
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

    def generate_motion_update(
        self,
        pos,
        quat,
        frame_id,
        mode=TrajectoryGenerationMode.MODE_POSITION,
        twist=None,
    ):

        msg = MotionUpdate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        if mode == TrajectoryGenerationMode.MODE_POSITION:
            msg.pose = Pose(
                position=Point(x=pos[0], y=pos[1], z=pos[2]),
                orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
            )
        elif mode == TrajectoryGenerationMode.MODE_VELOCITY:
            msg.velocity = Twist(
                linear=Vector3(x=twist[0], y=twist[1], z=twist[2]),
                angular=Vector3(x=twist[3], y=twist[4], z=twist[5]),
            )
        msg.target_stiffness = np.diag([75.0, 75.0, 75.0, 75.0, 75.0, 75.0]).flatten()
        msg.target_damping = np.diag([35.0, 35.0, 35.0, 35.0, 35.0, 35.0]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = mode

        return msg

    def generate_joint_motion_update(self, joint_pos):
        msg = JointMotionUpdate()

        msg.target_state.positions = joint_pos
        msg.target_stiffness = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        msg.target_damping = [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_POSITION

        return msg

    def send_cartesian_pose_target(self, pos, quat, frame_id):

        self.motion_update_publisher.publish(
            self.generate_motion_update(
                pos, quat, frame_id, TrajectoryGenerationMode.MODE_POSITION
            )
        )
        self.get_logger().info(
            f"Published MotionUpdate with POSITION trajectory mode to aic_controller"
        )

    def send_cartesian_twist_target(self, twist, frame_id):

        self.motion_update_publisher.publish(
            self.generate_motion_update(
                None, None, frame_id, TrajectoryGenerationMode.MODE_VELOCITY, twist
            )
        )
        self.get_logger().info(
            f"Published MotionUpdate with VELOCITY trajectory mode to aic_controller"
        )

    def send_joint_target(self, joint_pos):

        self.joint_motion_update_publisher.publish(
            self.generate_joint_motion_update(joint_pos)
        )

        self.get_logger().info("Published JointMotionUpdate to aic_controller")

    def send_change_target_mode_req(self, mode):
        ChangeTargetMode

        req = ChangeTargetMode.Request()
        req.target_mode.mode = mode

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
            node = TestImpedanceNode()

            # Send service request to switch to Cartesian target mode
            node.send_change_target_mode_req(TargetMode.MODE_CARTESIAN)

            quat_tool_down = [
                0.7071068,
                0.7071068,
                0.0,
                0.0,
            ]  # ZYX = (180, 0, 90), z axis normal to plane and (x,y) axes are aligned with base_link axes

            # Send Cartesian pose targets in both "base_link" and "gripper/tcp" frames
            node.send_cartesian_pose_target(
                [-0.501, -0.175, 0.2],
                quat_tool_down,
                "base_link",
            )
            time.sleep(3)

            node.send_cartesian_pose_target(
                [0.2, 0.0, 0.0],
                [-0.2126311, 0.2126311, 0.6743797, 0.6743797],
                "gripper/tcp",
            )
            time.sleep(3)

            node.send_cartesian_pose_target(
                [0.0, 0.2, 0.0],
                [0.2126311, -0.2126311, -0.6743797, 0.6743797],
                "gripper/tcp",
            )
            time.sleep(3)

            # Send Cartesian twist targets in both "base_link" and "gripper/tcp" frames
            node.send_cartesian_twist_target(
                [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
                "base_link",
            )
            time.sleep(3)
            node.send_cartesian_twist_target(
                [0.0, -0.05, 0.0, 0.0, 0.0, 0.0],
                "gripper/tcp",
            )
            time.sleep(3)

            # Send service request to switch to joint target mode
            node.send_change_target_mode_req(TargetMode.MODE_JOINT)

            node.send_joint_target([0.0, -1.57, -1.57, -1.57, 1.57, 0.0])
            time.sleep(3)

            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
