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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Cable arguments
    cable_description_file = LaunchConfiguration("cable_description_file")
    cable_x = LaunchConfiguration("cable_x")
    cable_y = LaunchConfiguration("cable_y")
    cable_z = LaunchConfiguration("cable_z")
    cable_roll = LaunchConfiguration("cable_roll")
    cable_pitch = LaunchConfiguration("cable_pitch")
    cable_yaw = LaunchConfiguration("cable_yaw")
    attach_cable_to_gripper = LaunchConfiguration("attach_cable_to_gripper")
    cable_type = LaunchConfiguration("cable_type")

    # Process cable description
    cable_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            cable_description_file,
            " ",
            "attach_cable_to_gripper:=",
            attach_cable_to_gripper,
            " ",
            "cable_type:=",
            cable_type,
        ]
    )

    # Spawn cable in Gazebo
    gz_spawn_cable = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            cable_description_content,
            "-name",
            "cable",
            "-allow_renaming",
            "true",
            "-x",
            cable_x,
            "-y",
            cable_y,
            "-z",
            cable_z,
            "-R",
            cable_roll,
            "-P",
            cable_pitch,
            "-Y",
            cable_yaw,
        ],
    )

    return [gz_spawn_cable]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_description"), "urdf", "cable.sdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the cable.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_x",
            default_value="-0.35",
            description="Cable spawn X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_y",
            default_value="0.4",
            description="Cable spawn Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_z",
            default_value="1.15",
            description="Cable spawn Z position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_roll",
            default_value="0.0",
            description="Cable spawn roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_pitch",
            default_value="0.0",
            description="Cable spawn pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_yaw",
            default_value="0.0",
            description="Cable spawn yaw orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "attach_cable_to_gripper",
            default_value="false",
            description="Whether to attach cable to gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_type",
            default_value="sfp_sc_cable",
            description="Type of cable model to spawn. Available options: 'sfp_sc_cable', and 'sfp_sc_cable_reversed'",
            choices=["sfp_sc_cable", "sfp_sc_cable_reversed"],
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
