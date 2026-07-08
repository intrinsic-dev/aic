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
import json
from datetime import datetime
import rclpy
import numpy as np
from rclpy.executors import ExternalShutdownException

from rclpy.node import Node
from aic_control_interfaces.msg import (
    MotionUpdate,
    ControllerState,
    TrajectoryGenerationMode,
    TargetMode,
)
from aic_control_interfaces.srv import (
    ChangeTargetMode,
)
from geometry_msgs.msg import Pose, Point, Quaternion, Wrench, Vector3, Twist, WrenchStamped


class TestRobotControlBridgeNode(Node):
    def __init__(self):
        super().__init__("test_impedance_node")
        self.get_logger().info("TestRobotControlBridgeNode started")

        # Declare parameters.
        self.controller_namespace = self.declare_parameter(
            "controller_namespace", "aic_controller"
        ).value

        self.test_pose_targets_str = self.declare_parameter(
            "test_pose_targets", "[]"
        ).value


        self.log_filepath = self.declare_parameter(
            "log_filepath", ""
        ).value

        stiffness_param = self.declare_parameter(
            "target_stiffness", [10.0, 10.0]
        ).value
        self.target_stiffness = [stiffness_param[0]] * 3 + [stiffness_param[1]] * 3

        damping_param = self.declare_parameter(
            "target_damping", [5.0, 5.0]
        ).value
        self.target_damping = [damping_param[0]] * 3 + [damping_param[1]] * 3

        self.publish_duration = self.declare_parameter(
            "publish_duration", 5.0
        ).value

        self.stop_duration = self.declare_parameter(
            "stop_duration", 5.0
        ).value

        parsed_targets = json.loads(self.test_pose_targets_str)
        if parsed_targets and not isinstance(parsed_targets[0], list):
            # Reshape 1D array into 2D array of poses (7 elements each)
            self.test_pose_targets = [
                parsed_targets[i : i + 7]
                for i in range(0, len(parsed_targets), 7)
            ]
        else:
            self.test_pose_targets = parsed_targets

        self.tcp_error = None
        self.latest_wrench = None

        self.wrench_subscriber = self.create_subscription(
            WrenchStamped,
            "/fts_broadcaster/wrench",
            self.wrench_callback,
            10
        )

        self.motion_update_publisher = self.create_publisher(
            MotionUpdate, f"/{self.controller_namespace}/pose_commands", 10
        )

        while self.motion_update_publisher.get_subscription_count() == 0:
            self.get_logger().info(
                f"Waiting for subscriber to '{self.controller_namespace}/pose_commands'..."
            )
            time.sleep(1.0)

        self.state_subscriber = self.create_subscription(
            ControllerState,
            f"/{self.controller_namespace}/controller_state",
            self.state_callback,
            10
        )

        self.client = self.create_client(
            ChangeTargetMode, f"/{self.controller_namespace}/change_target_mode"
        )

        # Wait for service
        while not self.client.wait_for_service():
            self.get_logger().info(
                f"Waiting for service '{self.controller_namespace}/change_target_mode'..."
            )
            time.sleep(1.0)

    def state_callback(self, msg: ControllerState):
        self.tcp_error = list(msg.tcp_error)

    def wrench_callback(self, msg: WrenchStamped):
        self.latest_wrench = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ]

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
        msg.target_stiffness = np.diag(self.target_stiffness).flatten()
        msg.target_damping = np.diag(self.target_damping).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = mode

        return msg

    def send_cartesian_pose_target(self, pos, quat, frame_id):

        self.motion_update_publisher.publish(
            self.generate_motion_update(
                pos, quat, frame_id, TrajectoryGenerationMode.MODE_POSITION
            )
        )

    def send_change_target_mode_req(self, mode):

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
            raise RuntimeError(f"Failed to change control mode to {mode}")

        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)

    node = TestRobotControlBridgeNode()

    try:
        # Send service request to switch to Cartesian target mode
        node.send_change_target_mode_req(TargetMode.MODE_CARTESIAN)

        output_log = {
            "target_stiffness": node.target_stiffness,
            "target_damping": node.target_damping,
            "results": {}
        }

        for pose in node.test_pose_targets:
            # pose is expected to be [x, y, z, qx, qy, qz, qw]
            pos = pose[0:3]
            quat = pose[3:7]

            node.get_logger().info(f"Testing pose target: {pose}")

            node.get_logger().info(f"Publishing motion update for {node.publish_duration} seconds.")
            start_time = time.time()
            while time.time() - start_time < node.publish_duration and rclpy.ok():
                node.send_cartesian_pose_target(pos, quat, "base_link")
                rclpy.spin_once(node, timeout_sec=0.04) # 25 Hz

            node.get_logger().info(f"Stopping motion updates for {node.stop_duration} seconds.")
            start_time = time.time()
            while time.time() - start_time < node.stop_duration and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

            if node.tcp_error is not None:
                node.get_logger().info(f"Recording tcp_error: {node.tcp_error}")
                recorded_data = {"tcp_error": node.tcp_error}
                if node.latest_wrench is not None:
                    node.get_logger().info(f"Recording wrench: {node.latest_wrench}")
                    recorded_data["wrench"] = node.latest_wrench
                output_log["results"][str(pose)] = recorded_data

        if output_log["results"]:
            if node.log_filepath:
                filename = node.log_filepath
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"tcp_accuracy_{timestamp}.json"

            with open(filename, "w") as f:
                json.dump(output_log, f, indent=4)
            node.get_logger().info(f"Saved TCP accuracy to {filename}")

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        node.get_logger().error(f"Test script crashed with RuntimeError: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
