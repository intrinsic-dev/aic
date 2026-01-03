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
    # Task board arguments
    task_board_description_file = LaunchConfiguration("task_board_description_file")
    task_board_x = LaunchConfiguration("task_board_x")
    task_board_y = LaunchConfiguration("task_board_y")
    task_board_z = LaunchConfiguration("task_board_z")
    task_board_roll = LaunchConfiguration("task_board_roll")
    task_board_pitch = LaunchConfiguration("task_board_pitch")
    task_board_yaw = LaunchConfiguration("task_board_yaw")

    # Component delta arguments
    lc_mount_rail_0_present = LaunchConfiguration("lc_mount_rail_0_present")
    lc_mount_rail_0_translation = LaunchConfiguration("lc_mount_rail_0_translation")
    lc_mount_rail_0_roll = LaunchConfiguration("lc_mount_rail_0_roll")
    lc_mount_rail_0_pitch = LaunchConfiguration("lc_mount_rail_0_pitch")
    lc_mount_rail_0_yaw = LaunchConfiguration("lc_mount_rail_0_yaw")

    sfp_mount_rail_0_present = LaunchConfiguration("sfp_mount_rail_0_present")
    sfp_mount_rail_0_translation = LaunchConfiguration("sfp_mount_rail_0_translation")
    sfp_mount_rail_0_roll = LaunchConfiguration("sfp_mount_rail_0_roll")
    sfp_mount_rail_0_pitch = LaunchConfiguration("sfp_mount_rail_0_pitch")
    sfp_mount_rail_0_yaw = LaunchConfiguration("sfp_mount_rail_0_yaw")

    sc_mount_rail_0_present = LaunchConfiguration("sc_mount_rail_0_present")
    sc_mount_rail_0_translation = LaunchConfiguration("sc_mount_rail_0_translation")
    sc_mount_rail_0_roll = LaunchConfiguration("sc_mount_rail_0_roll")
    sc_mount_rail_0_pitch = LaunchConfiguration("sc_mount_rail_0_pitch")
    sc_mount_rail_0_yaw = LaunchConfiguration("sc_mount_rail_0_yaw")

    lc_mount_rail_1_present = LaunchConfiguration("lc_mount_rail_1_present")
    lc_mount_rail_1_translation = LaunchConfiguration("lc_mount_rail_1_translation")
    lc_mount_rail_1_roll = LaunchConfiguration("lc_mount_rail_1_roll")
    lc_mount_rail_1_pitch = LaunchConfiguration("lc_mount_rail_1_pitch")
    lc_mount_rail_1_yaw = LaunchConfiguration("lc_mount_rail_1_yaw")

    sfp_mount_rail_1_present = LaunchConfiguration("sfp_mount_rail_1_present")
    sfp_mount_rail_1_translation = LaunchConfiguration("sfp_mount_rail_1_translation")
    sfp_mount_rail_1_roll = LaunchConfiguration("sfp_mount_rail_1_roll")
    sfp_mount_rail_1_pitch = LaunchConfiguration("sfp_mount_rail_1_pitch")
    sfp_mount_rail_1_yaw = LaunchConfiguration("sfp_mount_rail_1_yaw")

    sc_mount_rail_1_present = LaunchConfiguration("sc_mount_rail_1_present")
    sc_mount_rail_1_translation = LaunchConfiguration("sc_mount_rail_1_translation")
    sc_mount_rail_1_roll = LaunchConfiguration("sc_mount_rail_1_roll")
    sc_mount_rail_1_pitch = LaunchConfiguration("sc_mount_rail_1_pitch")
    sc_mount_rail_1_yaw = LaunchConfiguration("sc_mount_rail_1_yaw")

    # SC Port parameters
    sc_port_0_present = LaunchConfiguration("sc_port_0_present")
    sc_port_0_translation = LaunchConfiguration("sc_port_0_translation")
    sc_port_0_roll = LaunchConfiguration("sc_port_0_roll")
    sc_port_0_pitch = LaunchConfiguration("sc_port_0_pitch")
    sc_port_0_yaw = LaunchConfiguration("sc_port_0_yaw")

    sc_port_1_present = LaunchConfiguration("sc_port_1_present")
    sc_port_1_translation = LaunchConfiguration("sc_port_1_translation")
    sc_port_1_roll = LaunchConfiguration("sc_port_1_roll")
    sc_port_1_pitch = LaunchConfiguration("sc_port_1_pitch")
    sc_port_1_yaw = LaunchConfiguration("sc_port_1_yaw")

    # NIC Card Mount parameters
    nic_card_mount_0_present = LaunchConfiguration("nic_card_mount_0_present")
    nic_card_mount_0_translation = LaunchConfiguration("nic_card_mount_0_translation")
    nic_card_mount_0_roll = LaunchConfiguration("nic_card_mount_0_roll")
    nic_card_mount_0_pitch = LaunchConfiguration("nic_card_mount_0_pitch")
    nic_card_mount_0_yaw = LaunchConfiguration("nic_card_mount_0_yaw")

    nic_card_mount_1_present = LaunchConfiguration("nic_card_mount_1_present")
    nic_card_mount_1_translation = LaunchConfiguration("nic_card_mount_1_translation")
    nic_card_mount_1_roll = LaunchConfiguration("nic_card_mount_1_roll")
    nic_card_mount_1_pitch = LaunchConfiguration("nic_card_mount_1_pitch")
    nic_card_mount_1_yaw = LaunchConfiguration("nic_card_mount_1_yaw")

    nic_card_mount_2_present = LaunchConfiguration("nic_card_mount_2_present")
    nic_card_mount_2_translation = LaunchConfiguration("nic_card_mount_2_translation")
    nic_card_mount_2_roll = LaunchConfiguration("nic_card_mount_2_roll")
    nic_card_mount_2_pitch = LaunchConfiguration("nic_card_mount_2_pitch")
    nic_card_mount_2_yaw = LaunchConfiguration("nic_card_mount_2_yaw")

    nic_card_mount_3_present = LaunchConfiguration("nic_card_mount_3_present")
    nic_card_mount_3_translation = LaunchConfiguration("nic_card_mount_3_translation")
    nic_card_mount_3_roll = LaunchConfiguration("nic_card_mount_3_roll")
    nic_card_mount_3_pitch = LaunchConfiguration("nic_card_mount_3_pitch")
    nic_card_mount_3_yaw = LaunchConfiguration("nic_card_mount_3_yaw")

    nic_card_mount_4_present = LaunchConfiguration("nic_card_mount_4_present")
    nic_card_mount_4_translation = LaunchConfiguration("nic_card_mount_4_translation")
    nic_card_mount_4_roll = LaunchConfiguration("nic_card_mount_4_roll")
    nic_card_mount_4_pitch = LaunchConfiguration("nic_card_mount_4_pitch")
    nic_card_mount_4_yaw = LaunchConfiguration("nic_card_mount_4_yaw")

    # Process task board description
    task_board_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            task_board_description_file,
            " ",
            "x:=",
            task_board_x,
            " ",
            "y:=",
            task_board_y,
            " ",
            "z:=",
            task_board_z,
            " ",
            "roll:=",
            task_board_roll,
            " ",
            "pitch:=",
            task_board_pitch,
            " ",
            "yaw:=",
            task_board_yaw,
            " ",
            "lc_mount_rail_0_present:=",
            lc_mount_rail_0_present,
            " ",
            "lc_mount_rail_0_translation:=",
            lc_mount_rail_0_translation,
            " ",
            "lc_mount_rail_0_roll:=",
            lc_mount_rail_0_roll,
            " ",
            "lc_mount_rail_0_pitch:=",
            lc_mount_rail_0_pitch,
            " ",
            "lc_mount_rail_0_yaw:=",
            lc_mount_rail_0_yaw,
            " ",
            "sfp_mount_rail_0_present:=",
            sfp_mount_rail_0_present,
            " ",
            "sfp_mount_rail_0_translation:=",
            sfp_mount_rail_0_translation,
            " ",
            "sfp_mount_rail_0_roll:=",
            sfp_mount_rail_0_roll,
            " ",
            "sfp_mount_rail_0_pitch:=",
            sfp_mount_rail_0_pitch,
            " ",
            "sfp_mount_rail_0_yaw:=",
            sfp_mount_rail_0_yaw,
            " ",
            "sc_mount_rail_0_present:=",
            sc_mount_rail_0_present,
            " ",
            "sc_mount_rail_0_translation:=",
            sc_mount_rail_0_translation,
            " ",
            "sc_mount_rail_0_roll:=",
            sc_mount_rail_0_roll,
            " ",
            "sc_mount_rail_0_pitch:=",
            sc_mount_rail_0_pitch,
            " ",
            "sc_mount_rail_0_yaw:=",
            sc_mount_rail_0_yaw,
            " ",
            "lc_mount_rail_1_present:=",
            lc_mount_rail_1_present,
            " ",
            "lc_mount_rail_1_translation:=",
            lc_mount_rail_1_translation,
            " ",
            "lc_mount_rail_1_roll:=",
            lc_mount_rail_1_roll,
            " ",
            "lc_mount_rail_1_pitch:=",
            lc_mount_rail_1_pitch,
            " ",
            "lc_mount_rail_1_yaw:=",
            lc_mount_rail_1_yaw,
            " ",
            "sfp_mount_rail_1_present:=",
            sfp_mount_rail_1_present,
            " ",
            "sfp_mount_rail_1_translation:=",
            sfp_mount_rail_1_translation,
            " ",
            "sfp_mount_rail_1_roll:=",
            sfp_mount_rail_1_roll,
            " ",
            "sfp_mount_rail_1_pitch:=",
            sfp_mount_rail_1_pitch,
            " ",
            "sfp_mount_rail_1_yaw:=",
            sfp_mount_rail_1_yaw,
            " ",
            "sc_mount_rail_1_present:=",
            sc_mount_rail_1_present,
            " ",
            "sc_mount_rail_1_translation:=",
            sc_mount_rail_1_translation,
            " ",
            "sc_mount_rail_1_roll:=",
            sc_mount_rail_1_roll,
            " ",
            "sc_mount_rail_1_pitch:=",
            sc_mount_rail_1_pitch,
            " ",
            "sc_mount_rail_1_yaw:=",
            sc_mount_rail_1_yaw,
            " ",
            "sc_port_0_present:=",
            sc_port_0_present,
            " ",
            "sc_port_0_translation:=",
            sc_port_0_translation,
            " ",
            "sc_port_0_roll:=",
            sc_port_0_roll,
            " ",
            "sc_port_0_pitch:=",
            sc_port_0_pitch,
            " ",
            "sc_port_0_yaw:=",
            sc_port_0_yaw,
            " ",
            "sc_port_1_present:=",
            sc_port_1_present,
            " ",
            "sc_port_1_translation:=",
            sc_port_1_translation,
            " ",
            "sc_port_1_roll:=",
            sc_port_1_roll,
            " ",
            "sc_port_1_pitch:=",
            sc_port_1_pitch,
            " ",
            "sc_port_1_yaw:=",
            sc_port_1_yaw,
            " ",
            "nic_card_mount_0_present:=",
            nic_card_mount_0_present,
            " ",
            "nic_card_mount_0_translation:=",
            nic_card_mount_0_translation,
            " ",
            "nic_card_mount_0_roll:=",
            nic_card_mount_0_roll,
            " ",
            "nic_card_mount_0_pitch:=",
            nic_card_mount_0_pitch,
            " ",
            "nic_card_mount_0_yaw:=",
            nic_card_mount_0_yaw,
            " ",
            "nic_card_mount_1_present:=",
            nic_card_mount_1_present,
            " ",
            "nic_card_mount_1_translation:=",
            nic_card_mount_1_translation,
            " ",
            "nic_card_mount_1_roll:=",
            nic_card_mount_1_roll,
            " ",
            "nic_card_mount_1_pitch:=",
            nic_card_mount_1_pitch,
            " ",
            "nic_card_mount_1_yaw:=",
            nic_card_mount_1_yaw,
            " ",
            "nic_card_mount_2_present:=",
            nic_card_mount_2_present,
            " ",
            "nic_card_mount_2_translation:=",
            nic_card_mount_2_translation,
            " ",
            "nic_card_mount_2_roll:=",
            nic_card_mount_2_roll,
            " ",
            "nic_card_mount_2_pitch:=",
            nic_card_mount_2_pitch,
            " ",
            "nic_card_mount_2_yaw:=",
            nic_card_mount_2_yaw,
            " ",
            "nic_card_mount_3_present:=",
            nic_card_mount_3_present,
            " ",
            "nic_card_mount_3_translation:=",
            nic_card_mount_3_translation,
            " ",
            "nic_card_mount_3_roll:=",
            nic_card_mount_3_roll,
            " ",
            "nic_card_mount_3_pitch:=",
            nic_card_mount_3_pitch,
            " ",
            "nic_card_mount_3_yaw:=",
            nic_card_mount_3_yaw,
            " ",
            "nic_card_mount_4_present:=",
            nic_card_mount_4_present,
            " ",
            "nic_card_mount_4_translation:=",
            nic_card_mount_4_translation,
            " ",
            "nic_card_mount_4_roll:=",
            nic_card_mount_4_roll,
            " ",
            "nic_card_mount_4_pitch:=",
            nic_card_mount_4_pitch,
            " ",
            "nic_card_mount_4_yaw:=",
            nic_card_mount_4_yaw,
        ]
    )

    # Spawn task board in Gazebo
    gz_spawn_task_board = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            task_board_description_content,
            "-name",
            "task_board",
            "-allow_renaming",
            "true",
            "-x",
            task_board_x,
            "-y",
            task_board_y,
            "-z",
            task_board_z,
            "-R",
            task_board_roll,
            "-P",
            task_board_pitch,
            "-Y",
            task_board_yaw,
        ],
    )

    return [gz_spawn_task_board]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_description"), "urdf", "task_board.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the task board.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_x",
            default_value="0.25",
            description="Task board spawn X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_y",
            default_value="0.0",
            description="Task board spawn Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_z",
            default_value="1.14",
            description="Task board spawn Z position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_roll",
            default_value="0.0",
            description="Task board spawn roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_pitch",
            default_value="0.0",
            description="Task board spawn pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_yaw",
            default_value="0.0",
            description="Task board spawn yaw orientation (radians)",
        )
    )

    # LC Mount Rail 0 arguments (left side, mount rail)
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_0_present",
            default_value="false",
            description="Whether LC Mount Rail 0 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_0_translation",
            default_value="0.0",
            description="LC Mount Rail 0 translation along rail (meters, valid range: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_0_roll",
            default_value="0.0",
            description="LC Mount Rail 0 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_0_pitch",
            default_value="0.0",
            description="LC Mount Rail 0 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_0_yaw",
            default_value="0.0",
            description="LC Mount Rail 0 yaw orientation (radians)",
        )
    )

    # SFP Mount Rail 0 arguments (left side, mount rail)
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_0_present",
            default_value="false",
            description="Whether SFP Mount Rail 0 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_0_translation",
            default_value="0.0",
            description="SFP Mount Rail 0 translation along rail (meters, valid range: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_0_roll",
            default_value="0.0",
            description="SFP Mount Rail 0 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_0_pitch",
            default_value="0.0",
            description="SFP Mount Rail 0 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_0_yaw",
            default_value="0.0",
            description="SFP Mount Rail 0 yaw orientation (radians)",
        )
    )

    # SC Mount Rail 0 arguments (left side, mount rail)
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_0_present",
            default_value="false",
            description="Whether SC Mount Rail 0 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_0_translation",
            default_value="0.0",
            description="SC Mount Rail 0 translation along rail (meters, valid range: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_0_roll",
            default_value="0.0",
            description="SC Mount Rail 0 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_0_pitch",
            default_value="0.0",
            description="SC Mount Rail 0 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_0_yaw",
            default_value="0.0",
            description="SC Mount Rail 0 yaw orientation (radians)",
        )
    )

    # LC Mount Rail 1 arguments (right side, mount rail)
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_1_present",
            default_value="false",
            description="Whether LC Mount Rail 1 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_1_translation",
            default_value="0.0",
            description="LC Mount Rail 1 translation along rail (meters, valid range: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_1_roll",
            default_value="0.0",
            description="LC Mount Rail 1 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_1_pitch",
            default_value="0.0",
            description="LC Mount Rail 1 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_rail_1_yaw",
            default_value="0.0",
            description="LC Mount Rail 1 yaw orientation (radians)",
        )
    )

    # SFP Mount Rail 1 arguments (right side, mount rail)
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_1_present",
            default_value="false",
            description="Whether SFP Mount Rail 1 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_1_translation",
            default_value="0.0",
            description="SFP Mount Rail 1 translation along rail (meters, valid range: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_1_roll",
            default_value="0.0",
            description="SFP Mount Rail 1 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_1_pitch",
            default_value="0.0",
            description="SFP Mount Rail 1 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_rail_1_yaw",
            default_value="0.0",
            description="SFP Mount Rail 1 yaw orientation (radians)",
        )
    )

    # SC Mount Rail 1 arguments (right side, mount rail)
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_1_present",
            default_value="false",
            description="Whether SC Mount Rail 1 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_1_translation",
            default_value="0.0",
            description="SC Mount Rail 1 translation along rail (meters, valid range: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_1_roll",
            default_value="0.0",
            description="SC Mount Rail 1 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_1_pitch",
            default_value="0.0",
            description="SC Mount Rail 1 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_rail_1_yaw",
            default_value="0.0",
            description="SC Mount Rail 1 yaw orientation (radians)",
        )
    )

    # SC Port 0 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_0_present",
            default_value="false",
            description="Whether SC Port 0 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_0_translation",
            default_value="0.0",
            description="SC Port 0 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_0_roll",
            default_value="0.0",
            description="SC Port 0 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_0_pitch",
            default_value="0.0",
            description="SC Port 0 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_0_yaw",
            default_value="0.0",
            description="SC Port 0 yaw orientation (radians)",
        )
    )

    # SC Port 1 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_1_present",
            default_value="false",
            description="Whether SC Port 1 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_1_translation",
            default_value="0.0",
            description="SC Port 1 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_1_roll",
            default_value="0.0",
            description="SC Port 1 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_1_pitch",
            default_value="0.0",
            description="SC Port 1 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_1_yaw",
            default_value="0.0",
            description="SC Port 1 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 0 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_0_present",
            default_value="false",
            description="Whether NIC Card Mount 0 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_0_translation",
            default_value="0.0",
            description="NIC Card Mount 0 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_0_roll",
            default_value="0.0",
            description="NIC Card Mount 0 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_0_pitch",
            default_value="0.0",
            description="NIC Card Mount 0 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_0_yaw",
            default_value="0.0",
            description="NIC Card Mount 0 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 1 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_1_present",
            default_value="false",
            description="Whether NIC Card Mount 1 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_1_translation",
            default_value="0.0",
            description="NIC Card Mount 1 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_1_roll",
            default_value="0.0",
            description="NIC Card Mount 1 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_1_pitch",
            default_value="0.0",
            description="NIC Card Mount 1 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_1_yaw",
            default_value="0.0",
            description="NIC Card Mount 1 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 2 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_2_present",
            default_value="false",
            description="Whether NIC Card Mount 2 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_2_translation",
            default_value="0.0",
            description="NIC Card Mount 2 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_2_roll",
            default_value="0.0",
            description="NIC Card Mount 2 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_2_pitch",
            default_value="0.0",
            description="NIC Card Mount 2 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_2_yaw",
            default_value="0.0",
            description="NIC Card Mount 2 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 3 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_3_present",
            default_value="false",
            description="Whether NIC Card Mount 3 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_3_translation",
            default_value="0.0",
            description="NIC Card Mount 3 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_3_roll",
            default_value="0.0",
            description="NIC Card Mount 3 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_3_pitch",
            default_value="0.0",
            description="NIC Card Mount 3 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_3_yaw",
            default_value="0.0",
            description="NIC Card Mount 3 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 4 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_4_present",
            default_value="false",
            description="Whether NIC Card Mount 4 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_4_translation",
            default_value="0.0",
            description="NIC Card Mount 4 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_4_roll",
            default_value="0.0",
            description="NIC Card Mount 4 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_4_pitch",
            default_value="0.0",
            description="NIC Card Mount 4 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_4_yaw",
            default_value="0.0",
            description="NIC Card Mount 04 yaw orientation (radians)",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
