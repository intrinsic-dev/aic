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

from aic_model.policy_ros import (
    PolicyRos,
    GetObservationCallback,
    SetPoseTargetCallback,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from tf_transformations import quaternion_multiply, quaternion_slerp


class CheatCode(PolicyRos):
    def __init__(self, parent_node):
        super().__init__(parent_node)

    def go_to_pose(self, pose: Pose, timeout_sec: float) -> bool:
        self._set_pose_target(pose)
        # todo: smart stuff here to wait for the robot to reach the pose
        time.sleep(timeout_sec)
        return True

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        set_pose_target: SetPoseTargetCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info(f"CheatCode.insert_cable() enter. Task: {task}")
        self._set_pose_target = set_pose_target

        start_time = time.clock_gettime(0)
        try:
            port_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link",
                "task_board/nic_card_mount_0/sfp_port_0_link",
                Time(),
            )
        except TransformException as ex:
            self.get_logger().error(
                f"Could not find transform to sfp_port_0_link: {ex}"
            )
            return False
        q_port = (
            port_tf_stamped.transform.rotation.x,
            port_tf_stamped.transform.rotation.y,
            port_tf_stamped.transform.rotation.z,
            port_tf_stamped.transform.rotation.w,
        )

        self.get_logger().info(f"base to port transform: {port_tf_stamped}")

        approach_pose = Pose(
            position=Point(
                x=port_tf_stamped.transform.translation.x,
                y=port_tf_stamped.transform.translation.y,
                z=port_tf_stamped.transform.translation.z + 0.1,
            ),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        )
        self.go_to_pose(approach_pose, 5.0)

        while True:
            sfp_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link",
                "cable_0/sfp_tip_link",
                Time(),
            )

            q_module = (
                sfp_tf_stamped.transform.rotation.x,
                sfp_tf_stamped.transform.rotation.y,
                sfp_tf_stamped.transform.rotation.z,
                sfp_tf_stamped.transform.rotation.w,
            )
            q_module_inv = (
                q_module[0],
                q_module[1],
                q_module[2],
                -q_module[3],
            )
            q_diff = quaternion_multiply(q_port, q_module_inv)

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
            q_gripper_target = quaternion_multiply(q_diff, q_gripper)
            q_gripper_slerp = quaternion_slerp(q_gripper, q_gripper_target, 0.3)

            approach_pose.orientation = Quaternion(
                x=q_gripper_slerp[0],
                y=q_gripper_slerp[1],
                z=q_gripper_slerp[2],
                w=q_gripper_slerp[3],
            )

            translation_diff = (
                port_tf_stamped.transform.translation.x
                - sfp_tf_stamped.transform.translation.x,
                port_tf_stamped.transform.translation.y
                - sfp_tf_stamped.transform.translation.y,
                port_tf_stamped.transform.translation.z
                - sfp_tf_stamped.transform.translation.z,
            )

            approach_pose.position.x += translation_diff[0] * 0.1
            approach_pose.position.y += translation_diff[1] * 0.1
            self.get_logger().info(f"z diff: {translation_diff[2]:0.5}")
            if translation_diff[2] < 0.0:
                approach_pose.position.z -= 0.0003
            else:
                break
            self.go_to_pose(approach_pose, 0.05)

        self.get_logger().info("CheatCode.insert_cable() exiting...")
        return True
