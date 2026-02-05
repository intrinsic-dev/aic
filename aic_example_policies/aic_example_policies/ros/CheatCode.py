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

from aic_model.policy_ros import PolicyRos
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from typing import Callable
from tf_transformations import quaternion_multiply, quaternion_slerp


class CheatCode(PolicyRos):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("CheatCode.__init__()")

    def go_to_pose(self, pose: Pose, timeout_sec: float) -> bool:
        self._set_pose_target(pose)
        self.get_logger().info("Waiting to reach pose...")
        # todo: smart stuff here to wait for the robot to reach the pose
        time.sleep(timeout_sec)
        self.get_logger().info("Reached pose, hopefully...")
        return True

    def insert_cable(
        self,
        task: Task,
        get_observation: Callable[[], Observation],
        set_pose_target: Callable[[Pose, str], []],
        send_feedback: Callable[[str], []],
    ):
        self.get_logger().info(f"CheatCode.insert_cable() enter. Task: {task}")
        self._get_observation = get_observation
        self._set_pose_target = set_pose_target

        start_time = time.clock_gettime(0)
        # send_feedback("waving the arm around")
        try:
            port_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link",
                "task_board/nic_card_mount_0/sfp_port_0_link",
                Time(),
            )
        except TransformException as ex:
            self.get_logger().error(f"Could not find transform to sfp_port_0_link: {ex}")
            return False

        self.get_logger().info(f"base to port transform: {port_tf_stamped}")

        approach_pose = Pose(
            position=Point(
                x=port_tf_stamped.transform.translation.x,
                y=port_tf_stamped.transform.translation.y,
                z=port_tf_stamped.transform.translation.z + 0.1,
            ),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
            # orientation=Quaternion(
            #     x=port_tf_stamped.transform.rotation.x,
            #     y=port_tf_stamped.transform.rotation.y,
            #     z=port_tf_stamped.transform.rotation.z,
            #     w=port_tf_stamped.transform.rotation.w,
            # ),
        )
        self.go_to_pose(approach_pose, 2.0)

        #approach_pose.orientation = sfp_tf_stamped.transform.rotation
        #self.go_to_pose(approach_pose)

        for approach_count in range(0, 20):
            sfp_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link",
                "cable_0/sfp_tip_link",
                Time(),
            )
    
            self.get_logger().info(f"sfp transform: {sfp_tf_stamped}")
    
            q_receptacle = (1.0, 0.0, 0.0, 0.0)  # todo: query tf for this...
            q_module = (
                sfp_tf_stamped.transform.rotation.x,
                sfp_tf_stamped.transform.rotation.y,
                sfp_tf_stamped.transform.rotation.z,
                sfp_tf_stamped.transform.rotation.w,
            )
            q_module_inv = (
                -q_module[0],
                -q_module[1],
                -q_module[2],
                q_module[3],
            )
            q_diff = quaternion_multiply(q_receptacle, q_module_inv)
            self.get_logger().info(f"q_diff: {q_diff}")
    
            gripper_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link",
                "gripper/tcp",
                Time(),
            )
            q_gripper = (
                gripper_tf_stamped.transform.rotation.x,
                gripper_tf_stamped.transform.rotation.y,
                gripper_tf_stamped.transform.rotation.z,
                gripper_tf_stamped.transform.rotation.w,
            )
            # 1.0, 0.0, 0.0, 0.0)  # todo: query TF for this
            q_gripper_next = quaternion_multiply(q_gripper, q_diff)
            q_gripper_slerp = quaternion_slerp(q_gripper, q_gripper_next, 0.2)

            # todo: slerp this rather than "snap" to the target orientation
    
            if approach_count == 0:
                approach_pose.orientation = Quaternion(
                    x=q_gripper_next[0],
                    y=q_gripper_next[1],
                    z=q_gripper_next[2],
                    w=q_gripper_next[3],
                )
            # self.go_to_pose(approach_pose, 2.0)

            translation_diff = (
                port_tf_stamped.transform.translation.x - sfp_tf_stamped.transform.translation.x,
                port_tf_stamped.transform.translation.y - sfp_tf_stamped.transform.translation.y,
                port_tf_stamped.transform.translation.z - sfp_tf_stamped.transform.translation.z,
            )

            self.get_logger().info(f"sfp: {sfp_tf_stamped.transform.translation} diff: {translation_diff}")
            approach_pose.position.x += translation_diff[0] * 0.2
            approach_pose.position.y += translation_diff[1] * 0.2
            if translation_diff[2] < 0:
                approach_pose.position.z -= 0.01
            else:
                break
            self.go_to_pose(approach_pose, 2.0)

        # set_pose_target(sfp_pose)

        # while time.clock_gettime(0) - start_time < 10.0:
        #     time.sleep(0.25)
        #     observation = get_observation()
        #     t = (
        #         observation.center_image.header.stamp.sec
        #         + observation.center_image.header.stamp.nanosec / 1e9
        #     )
        #     self.get_logger().info(f"observation time: {t}")

        #     # Move the arm along a line, while looking down at the task board.
        #     tcp = observation.tcp_transform.transform.translation
        #     loop_duration = 5.0  # seconds
        #     loop_fraction = (t % loop_duration) / loop_duration
        #     y_scale = 2 * loop_fraction
        #     if y_scale > 1.0:
        #         y_scale = 2.0 - y_scale
        #     y_scale -= 1.0

        #     # create a smooth series of target points that flies over the task board
        #     set_pose_target(
        #         Pose(
        #             position=Point(x=-0.4, y=0.45 + 0.3 * y_scale, z=0.25),
        #             orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
        #         )
        #     )

        self.get_logger().info("CheatCode.insert_cable() exiting...")
        return True
