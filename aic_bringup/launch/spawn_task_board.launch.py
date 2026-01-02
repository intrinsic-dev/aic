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
    lc_mount_01_delta_x = LaunchConfiguration("lc_mount_01_delta_x")
    lc_mount_01_delta_y = LaunchConfiguration("lc_mount_01_delta_y")
    lc_mount_01_delta_z = LaunchConfiguration("lc_mount_01_delta_z")
    sfp_mount_01_delta_x = LaunchConfiguration("sfp_mount_01_delta_x")
    sfp_mount_01_delta_y = LaunchConfiguration("sfp_mount_01_delta_y")
    sfp_mount_01_delta_z = LaunchConfiguration("sfp_mount_01_delta_z")
    sc_mount_01_delta_x = LaunchConfiguration("sc_mount_01_delta_x")
    sc_mount_01_delta_y = LaunchConfiguration("sc_mount_01_delta_y")
    sc_mount_01_delta_z = LaunchConfiguration("sc_mount_01_delta_z")
    lc_mount_02_delta_x = LaunchConfiguration("lc_mount_02_delta_x")
    lc_mount_02_delta_y = LaunchConfiguration("lc_mount_02_delta_y")
    lc_mount_02_delta_z = LaunchConfiguration("lc_mount_02_delta_z")
    sfp_mount_02_delta_x = LaunchConfiguration("sfp_mount_02_delta_x")
    sfp_mount_02_delta_y = LaunchConfiguration("sfp_mount_02_delta_y")
    sfp_mount_02_delta_z = LaunchConfiguration("sfp_mount_02_delta_z")
    sc_mount_02_delta_x = LaunchConfiguration("sc_mount_02_delta_x")
    sc_mount_02_delta_y = LaunchConfiguration("sc_mount_02_delta_y")
    sc_mount_02_delta_z = LaunchConfiguration("sc_mount_02_delta_z")
    sc_port_01_delta_x = LaunchConfiguration("sc_port_01_delta_x")
    sc_port_01_delta_y = LaunchConfiguration("sc_port_01_delta_y")
    sc_port_01_delta_z = LaunchConfiguration("sc_port_01_delta_z")
    sc_port_02_delta_x = LaunchConfiguration("sc_port_02_delta_x")
    sc_port_02_delta_y = LaunchConfiguration("sc_port_02_delta_y")
    sc_port_02_delta_z = LaunchConfiguration("sc_port_02_delta_z")

    # SC Port parameters (0-based indexing)
    sc_port_00_present = LaunchConfiguration("sc_port_00_present")
    sc_port_00_translation = LaunchConfiguration("sc_port_00_translation")
    sc_port_00_roll = LaunchConfiguration("sc_port_00_roll")
    sc_port_00_pitch = LaunchConfiguration("sc_port_00_pitch")
    sc_port_00_yaw = LaunchConfiguration("sc_port_00_yaw")

    sc_port_01_present = LaunchConfiguration("sc_port_01_present")
    sc_port_01_translation = LaunchConfiguration("sc_port_01_translation")
    sc_port_01_roll = LaunchConfiguration("sc_port_01_roll")
    sc_port_01_pitch = LaunchConfiguration("sc_port_01_pitch")
    sc_port_01_yaw = LaunchConfiguration("sc_port_01_yaw")

    # NIC Card Mount parameters (0-based indexing)
    nic_card_mount_00_present = LaunchConfiguration("nic_card_mount_00_present")
    nic_card_mount_00_translation = LaunchConfiguration("nic_card_mount_00_translation")
    nic_card_mount_00_roll = LaunchConfiguration("nic_card_mount_00_roll")
    nic_card_mount_00_pitch = LaunchConfiguration("nic_card_mount_00_pitch")
    nic_card_mount_00_yaw = LaunchConfiguration("nic_card_mount_00_yaw")

    nic_card_mount_01_present = LaunchConfiguration("nic_card_mount_01_present")
    nic_card_mount_01_translation = LaunchConfiguration("nic_card_mount_01_translation")
    nic_card_mount_01_roll = LaunchConfiguration("nic_card_mount_01_roll")
    nic_card_mount_01_pitch = LaunchConfiguration("nic_card_mount_01_pitch")
    nic_card_mount_01_yaw = LaunchConfiguration("nic_card_mount_01_yaw")

    nic_card_mount_02_present = LaunchConfiguration("nic_card_mount_02_present")
    nic_card_mount_02_translation = LaunchConfiguration("nic_card_mount_02_translation")
    nic_card_mount_02_roll = LaunchConfiguration("nic_card_mount_02_roll")
    nic_card_mount_02_pitch = LaunchConfiguration("nic_card_mount_02_pitch")
    nic_card_mount_02_yaw = LaunchConfiguration("nic_card_mount_02_yaw")

    nic_card_mount_03_present = LaunchConfiguration("nic_card_mount_03_present")
    nic_card_mount_03_translation = LaunchConfiguration("nic_card_mount_03_translation")
    nic_card_mount_03_roll = LaunchConfiguration("nic_card_mount_03_roll")
    nic_card_mount_03_pitch = LaunchConfiguration("nic_card_mount_03_pitch")
    nic_card_mount_03_yaw = LaunchConfiguration("nic_card_mount_03_yaw")

    nic_card_mount_04_present = LaunchConfiguration("nic_card_mount_04_present")
    nic_card_mount_04_translation = LaunchConfiguration("nic_card_mount_04_translation")
    nic_card_mount_04_roll = LaunchConfiguration("nic_card_mount_04_roll")
    nic_card_mount_04_pitch = LaunchConfiguration("nic_card_mount_04_pitch")
    nic_card_mount_04_yaw = LaunchConfiguration("nic_card_mount_04_yaw")

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
            "lc_mount_01_delta_x:=",
            lc_mount_01_delta_x,
            " ",
            "lc_mount_01_delta_y:=",
            lc_mount_01_delta_y,
            " ",
            "lc_mount_01_delta_z:=",
            lc_mount_01_delta_z,
            " ",
            "sfp_mount_01_delta_x:=",
            sfp_mount_01_delta_x,
            " ",
            "sfp_mount_01_delta_y:=",
            sfp_mount_01_delta_y,
            " ",
            "sfp_mount_01_delta_z:=",
            sfp_mount_01_delta_z,
            " ",
            "sc_mount_01_delta_x:=",
            sc_mount_01_delta_x,
            " ",
            "sc_mount_01_delta_y:=",
            sc_mount_01_delta_y,
            " ",
            "sc_mount_01_delta_z:=",
            sc_mount_01_delta_z,
            " ",
            "lc_mount_02_delta_x:=",
            lc_mount_02_delta_x,
            " ",
            "lc_mount_02_delta_y:=",
            lc_mount_02_delta_y,
            " ",
            "lc_mount_02_delta_z:=",
            lc_mount_02_delta_z,
            " ",
            "sfp_mount_02_delta_x:=",
            sfp_mount_02_delta_x,
            " ",
            "sfp_mount_02_delta_y:=",
            sfp_mount_02_delta_y,
            " ",
            "sfp_mount_02_delta_z:=",
            sfp_mount_02_delta_z,
            " ",
            "sc_mount_02_delta_x:=",
            sc_mount_02_delta_x,
            " ",
            "sc_mount_02_delta_y:=",
            sc_mount_02_delta_y,
            " ",
            "sc_mount_02_delta_z:=",
            sc_mount_02_delta_z,
            " ",
            "sc_port_00_present:=",
            sc_port_00_present,
            " ",
            "sc_port_00_translation:=",
            sc_port_00_translation,
            " ",
            "sc_port_00_roll:=",
            sc_port_00_roll,
            " ",
            "sc_port_00_pitch:=",
            sc_port_00_pitch,
            " ",
            "sc_port_00_yaw:=",
            sc_port_00_yaw,
            " ",
            "sc_port_01_present:=",
            sc_port_01_present,
            " ",
            "sc_port_01_translation:=",
            sc_port_01_translation,
            " ",
            "sc_port_01_roll:=",
            sc_port_01_roll,
            " ",
            "sc_port_01_pitch:=",
            sc_port_01_pitch,
            " ",
            "sc_port_01_yaw:=",
            sc_port_01_yaw,
            " ",
            "nic_card_mount_00_present:=",
            nic_card_mount_00_present,
            " ",
            "nic_card_mount_00_translation:=",
            nic_card_mount_00_translation,
            " ",
            "nic_card_mount_00_roll:=",
            nic_card_mount_00_roll,
            " ",
            "nic_card_mount_00_pitch:=",
            nic_card_mount_00_pitch,
            " ",
            "nic_card_mount_00_yaw:=",
            nic_card_mount_00_yaw,
            " ",
            "nic_card_mount_01_present:=",
            nic_card_mount_01_present,
            " ",
            "nic_card_mount_01_translation:=",
            nic_card_mount_01_translation,
            " ",
            "nic_card_mount_01_roll:=",
            nic_card_mount_01_roll,
            " ",
            "nic_card_mount_01_pitch:=",
            nic_card_mount_01_pitch,
            " ",
            "nic_card_mount_01_yaw:=",
            nic_card_mount_01_yaw,
            " ",
            "nic_card_mount_02_present:=",
            nic_card_mount_02_present,
            " ",
            "nic_card_mount_02_translation:=",
            nic_card_mount_02_translation,
            " ",
            "nic_card_mount_02_roll:=",
            nic_card_mount_02_roll,
            " ",
            "nic_card_mount_02_pitch:=",
            nic_card_mount_02_pitch,
            " ",
            "nic_card_mount_02_yaw:=",
            nic_card_mount_02_yaw,
            " ",
            "nic_card_mount_03_present:=",
            nic_card_mount_03_present,
            " ",
            "nic_card_mount_03_translation:=",
            nic_card_mount_03_translation,
            " ",
            "nic_card_mount_03_roll:=",
            nic_card_mount_03_roll,
            " ",
            "nic_card_mount_03_pitch:=",
            nic_card_mount_03_pitch,
            " ",
            "nic_card_mount_03_yaw:=",
            nic_card_mount_03_yaw,
            " ",
            "nic_card_mount_04_present:=",
            nic_card_mount_04_present,
            " ",
            "nic_card_mount_04_translation:=",
            nic_card_mount_04_translation,
            " ",
            "nic_card_mount_04_roll:=",
            nic_card_mount_04_roll,
            " ",
            "nic_card_mount_04_pitch:=",
            nic_card_mount_04_pitch,
            " ",
            "nic_card_mount_04_yaw:=",
            nic_card_mount_04_yaw,
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

    # LC Mount 01 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_01_delta_x",
            default_value="0.0",
            description="LC Mount 01 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_01_delta_y",
            default_value="0.0",
            description="LC Mount 01 delta Y position (valid delta: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_01_delta_z",
            default_value="0.0",
            description="LC Mount 01 delta Z position",
        )
    )

    # SFP Mount 01 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_01_delta_x",
            default_value="0.0",
            description="SFP Mount 01 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_01_delta_y",
            default_value="0.0",
            description="SFP Mount 01 delta Y position (valid delta: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_01_delta_z",
            default_value="0.0",
            description="SFP Mount 01 delta Z position",
        )
    )

    # SC Mount 01 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_01_delta_x",
            default_value="0.0",
            description="SC Mount 01 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_01_delta_y",
            default_value="0.0",
            description="SC Mount 01 delta Y position (valid delta: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_01_delta_z",
            default_value="0.0",
            description="SC Mount 01 delta Z position",
        )
    )

    # LC Mount 02 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_02_delta_x",
            default_value="0.0",
            description="LC Mount 02 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_02_delta_y",
            default_value="0.0",
            description="LC Mount 02 delta Y position (valid delta: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lc_mount_02_delta_z",
            default_value="0.0",
            description="LC Mount 02 delta Z position",
        )
    )

    # SFP Mount 02 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_02_delta_x",
            default_value="0.0",
            description="SFP Mount 02 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_02_delta_y",
            default_value="0.0",
            description="SFP Mount 02 delta Y position (valid delta: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sfp_mount_02_delta_z",
            default_value="0.0",
            description="SFP Mount 02 delta Z position",
        )
    )

    # SC Mount 02 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_02_delta_x",
            default_value="0.0",
            description="SC Mount 02 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_02_delta_y",
            default_value="0.0",
            description="SC Mount 02 delta Y position (valid delta: -0.09625 to 0.09625)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_mount_02_delta_z",
            default_value="0.0",
            description="SC Mount 02 delta Z position",
        )
    )

    # SC Port 00 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_00_present",
            default_value="false",
            description="Whether SC Port 00 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_00_translation",
            default_value="0.0",
            description="SC Port 00 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_00_roll",
            default_value="0.0",
            description="SC Port 00 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_00_pitch",
            default_value="0.0",
            description="SC Port 00 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_00_yaw",
            default_value="0.0",
            description="SC Port 00 yaw orientation (radians)",
        )
    )

    # SC Port 01 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_01_present",
            default_value="false",
            description="Whether SC Port 01 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_01_translation",
            default_value="0.0",
            description="SC Port 01 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_01_roll",
            default_value="0.0",
            description="SC Port 01 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_01_pitch",
            default_value="0.0",
            description="SC Port 01 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_01_yaw",
            default_value="0.0",
            description="SC Port 01 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 00 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_00_present",
            default_value="false",
            description="Whether NIC Card Mount 00 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_00_translation",
            default_value="0.0",
            description="NIC Card Mount 00 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_00_roll",
            default_value="0.0",
            description="NIC Card Mount 00 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_00_pitch",
            default_value="0.0",
            description="NIC Card Mount 00 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_00_yaw",
            default_value="0.0",
            description="NIC Card Mount 00 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 01 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_01_present",
            default_value="false",
            description="Whether NIC Card Mount 01 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_01_translation",
            default_value="0.0",
            description="NIC Card Mount 01 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_01_roll",
            default_value="0.0",
            description="NIC Card Mount 01 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_01_pitch",
            default_value="0.0",
            description="NIC Card Mount 01 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_01_yaw",
            default_value="0.0",
            description="NIC Card Mount 01 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 02 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_02_present",
            default_value="false",
            description="Whether NIC Card Mount 02 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_02_translation",
            default_value="0.0",
            description="NIC Card Mount 02 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_02_roll",
            default_value="0.0",
            description="NIC Card Mount 02 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_02_pitch",
            default_value="0.0",
            description="NIC Card Mount 02 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_02_yaw",
            default_value="0.0",
            description="NIC Card Mount 02 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 03 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_03_present",
            default_value="false",
            description="Whether NIC Card Mount 03 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_03_translation",
            default_value="0.0",
            description="NIC Card Mount 03 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_03_roll",
            default_value="0.0",
            description="NIC Card Mount 03 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_03_pitch",
            default_value="0.0",
            description="NIC Card Mount 03 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_03_yaw",
            default_value="0.0",
            description="NIC Card Mount 03 yaw orientation (radians)",
        )
    )

    # NIC Card Mount 04 arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_04_present",
            default_value="false",
            description="Whether NIC Card Mount 04 is present",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_04_translation",
            default_value="0.0",
            description="NIC Card Mount 04 translation along rail (meters)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_04_roll",
            default_value="0.0",
            description="NIC Card Mount 04 roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_04_pitch",
            default_value="0.0",
            description="NIC Card Mount 04 pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_04_yaw",
            default_value="0.0",
            description="NIC Card Mount 04 yaw orientation (radians)",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
