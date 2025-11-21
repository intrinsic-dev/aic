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
    gripper_name = LaunchConfiguration("gripper_name")
    gripper_action_name = LaunchConfiguration("gripper_action_name")
    position = LaunchConfiguration("position")
    use_position = LaunchConfiguration("use_position")
    effort = LaunchConfiguration("effort")
    use_effort = LaunchConfiguration("use_effort")

    gripper_action = Node(
        package="aic_bringup",
        executable="gripper_action.py",
        name="gripper_action",
        namespace="",
        output="both",
        parameters=[
            {
                "gripper_name": gripper_name,
                "gripper_action_name": gripper_action_name,
                "use_position": use_position,
                "position": position,
                "use_effort": use_effort,
                "effort": effort,
            }
        ],
    )

    nodes_to_start = [
        gripper_action,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_name",
            default_value="gripper",
            description="Name of gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_action_name",
            default_value="/gripper_action_controller/gripper_cmd",
            description="Name of gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_position",
            default_value="false",
            description="If enabled, send a position command",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "position",
            default_value="0.0",
            description="Gripper position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_effort",
            default_value="false",
            description="If enabled, send an effort command",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "effort",
            default_value="0.0",
            description="Gripper effort",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
