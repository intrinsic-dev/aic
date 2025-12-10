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

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    controller_namespace = LaunchConfiguration("controller_namespace")
    tool_frame = LaunchConfiguration("tool_frame")
    contact_force_z = LaunchConfiguration("contact_force_z")

    move_to_contact = Node(
        package="aic_bringup",
        executable="move_to_contact.py",
        name="move_to_contact",
        namespace="",
        output="both",
        parameters=[
            {
                "controller_namespace": controller_namespace,
                "tool_frame": tool_frame,
                "contact_force_z": contact_force_z,
            }
        ],
    )

    nodes_to_start = [
        move_to_contact,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_namespace",
            default_value="admittance_controller",
            description="Namespace for the admittance_controller node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_frame",
            default_value="tool0",
            description="Tool frame for the move_to_contact node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "contact_force_z",
            default_value="10.0",
            description="Contact force along the z-axis for the move_to_contact node.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
