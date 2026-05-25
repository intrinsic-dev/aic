# Copyright 2025 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "service_tunnel",
                default_value="localhost:17080",
                description="Service tunnel address",
            ),
            DeclareLaunchArgument(
                "flowstate_zenoh_router_address",
                default_value="tcp/localhost:17447",
                description="Flowstate Zenoh router address",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically start the bridge",
            ),
            DeclareLaunchArgument(
                "server_address",
                default_value="localhost:17080",
                description="Address of the robot controller application layer server",
            ),
            DeclareLaunchArgument(
                "instance",
                default_value="robot_controller",
                description="Name of the robot controller service/resource instance",
            ),
            DeclareLaunchArgument(
                "part_name", default_value="arm", description="Part to control"
            ),
            DeclareLaunchArgument(
                "robot_joint_state_topic",
                default_value="/joint_states",
                description="Topic to publish joint state data on",
            ),
            DeclareLaunchArgument(
                "force_torque_topic",
                default_value="/fts_broadcaster/wrench",
                description="Topic to publish F/T sensor data on",
            ),
            DeclareLaunchArgument(
                "robot_base_frame_id",
                default_value="robot/robot/base_link",
                description="Frame ID of robot base to be used in robot_joint_state_topic",
            ),
            DeclareLaunchArgument(
                "force_torque_sensor_frame_id",
                default_value="force_torque_sensor/force_torque_sensor/AtiForceTorqueSensor",
                description="Frame ID of F/T sensor to be used in force_torque_topic",
            ),
            # DeclareLaunchArgument(
            #     "override_joint_names",
            #     default_value="",
            #     description="",
            # ),
            LifecycleNode(
                package="flowstate_ros_bridge",
                executable="flowstate_ros_bridge",
                name="flowstate_ros_bridge",
                namespace="",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "service_tunnel": LaunchConfiguration("service_tunnel"),
                        "flowstate_zenoh_router_address": LaunchConfiguration(
                            "flowstate_zenoh_router_address"
                        ),
                        "bridge_plugins": [
                            "flowstate_ros_bridge::ExecutiveBridge",
                            "flowstate_ros_bridge::WorldBridge",
                            "flowstate_ros_bridge::RobotControlBridge",
                            "flowstate_ros_bridge::AicCameraBridge",
                        ],
                        "server_address": LaunchConfiguration("server_address"),
                        "instance": LaunchConfiguration("instance"),
                        "part_name": LaunchConfiguration("part_name"),
                        "robot_joint_state_topic": LaunchConfiguration(
                            "robot_joint_state_topic"
                        ),
                        "force_torque_topic": LaunchConfiguration("force_torque_topic"),
                        "robot_base_frame_id": LaunchConfiguration(
                            "robot_base_frame_id"
                        ),
                        "force_torque_sensor_frame_id": LaunchConfiguration(
                            "force_torque_sensor_frame_id"
                        ),
                        "task_settings_file": os.path.join(
                            "config",
                            "default_task_settings.pbtxt",
                        ),
                        "joint_task_settings_file": os.path.join(
                            "config",
                            "default_joint_task_settings.pbtxt",
                        ),
                        "override_joint_names": [
                            "shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint",
                        ],
                    }
                ],
            ),
        ]
    )
