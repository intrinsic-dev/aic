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
    nic_card_mount_01_delta_x = LaunchConfiguration("nic_card_mount_01_delta_x")
    nic_card_mount_01_delta_y = LaunchConfiguration("nic_card_mount_01_delta_y")
    nic_card_mount_01_delta_z = LaunchConfiguration("nic_card_mount_01_delta_z")
    nic_card_mount_02_delta_x = LaunchConfiguration("nic_card_mount_02_delta_x")
    nic_card_mount_02_delta_y = LaunchConfiguration("nic_card_mount_02_delta_y")
    nic_card_mount_02_delta_z = LaunchConfiguration("nic_card_mount_02_delta_z")
    nic_card_mount_03_delta_x = LaunchConfiguration("nic_card_mount_03_delta_x")
    nic_card_mount_03_delta_y = LaunchConfiguration("nic_card_mount_03_delta_y")
    nic_card_mount_03_delta_z = LaunchConfiguration("nic_card_mount_03_delta_z")
    nic_card_mount_04_delta_x = LaunchConfiguration("nic_card_mount_04_delta_x")
    nic_card_mount_04_delta_y = LaunchConfiguration("nic_card_mount_04_delta_y")
    nic_card_mount_04_delta_z = LaunchConfiguration("nic_card_mount_04_delta_z")
    nic_card_mount_05_delta_x = LaunchConfiguration("nic_card_mount_05_delta_x")
    nic_card_mount_05_delta_y = LaunchConfiguration("nic_card_mount_05_delta_y")
    nic_card_mount_05_delta_z = LaunchConfiguration("nic_card_mount_05_delta_z")

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
            "sc_port_01_delta_x:=",
            sc_port_01_delta_x,
            " ",
            "sc_port_01_delta_y:=",
            sc_port_01_delta_y,
            " ",
            "sc_port_01_delta_z:=",
            sc_port_01_delta_z,
            " ",
            "sc_port_02_delta_x:=",
            sc_port_02_delta_x,
            " ",
            "sc_port_02_delta_y:=",
            sc_port_02_delta_y,
            " ",
            "sc_port_02_delta_z:=",
            sc_port_02_delta_z,
            " ",
            "nic_card_mount_01_delta_x:=",
            nic_card_mount_01_delta_x,
            " ",
            "nic_card_mount_01_delta_y:=",
            nic_card_mount_01_delta_y,
            " ",
            "nic_card_mount_01_delta_z:=",
            nic_card_mount_01_delta_z,
            " ",
            "nic_card_mount_02_delta_x:=",
            nic_card_mount_02_delta_x,
            " ",
            "nic_card_mount_02_delta_y:=",
            nic_card_mount_02_delta_y,
            " ",
            "nic_card_mount_02_delta_z:=",
            nic_card_mount_02_delta_z,
            " ",
            "nic_card_mount_03_delta_x:=",
            nic_card_mount_03_delta_x,
            " ",
            "nic_card_mount_03_delta_y:=",
            nic_card_mount_03_delta_y,
            " ",
            "nic_card_mount_03_delta_z:=",
            nic_card_mount_03_delta_z,
            " ",
            "nic_card_mount_04_delta_x:=",
            nic_card_mount_04_delta_x,
            " ",
            "nic_card_mount_04_delta_y:=",
            nic_card_mount_04_delta_y,
            " ",
            "nic_card_mount_04_delta_z:=",
            nic_card_mount_04_delta_z,
            " ",
            "nic_card_mount_05_delta_x:=",
            nic_card_mount_05_delta_x,
            " ",
            "nic_card_mount_05_delta_y:=",
            nic_card_mount_05_delta_y,
            " ",
            "nic_card_mount_05_delta_z:=",
            nic_card_mount_05_delta_z,
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

    # SC Port 01 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_01_delta_x",
            default_value="0.0",
            description="SC Port 01 delta X position (valid delta: -0.055 to 0.055)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_01_delta_y",
            default_value="0.0",
            description="SC Port 01 delta Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_01_delta_z",
            default_value="0.0",
            description="SC Port 01 delta Z position",
        )
    )

    # SC Port 02 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_02_delta_x",
            default_value="0.0",
            description="SC Port 02 delta X position (valid delta: -0.055 to 0.055)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_02_delta_y",
            default_value="0.0",
            description="SC Port 02 delta Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sc_port_02_delta_z",
            default_value="0.0",
            description="SC Port 02 delta Z position",
        )
    )

    # NIC Card Mount 01 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_01_delta_x",
            default_value="0.0",
            description="NIC Card Mount 01 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_01_delta_y",
            default_value="0.0",
            description="NIC Card Mount 01 delta Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_01_delta_z",
            default_value="0.0",
            description="NIC Card Mount 01 delta Z position",
        )
    )

    # NIC Card Mount 02 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_02_delta_x",
            default_value="0.0",
            description="NIC Card Mount 02 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_02_delta_y",
            default_value="0.0",
            description="NIC Card Mount 02 delta Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_02_delta_z",
            default_value="0.0",
            description="NIC Card Mount 02 delta Z position",
        )
    )

    # NIC Card Mount 03 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_03_delta_x",
            default_value="0.0",
            description="NIC Card Mount 03 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_03_delta_y",
            default_value="0.0",
            description="NIC Card Mount 03 delta Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_03_delta_z",
            default_value="0.0",
            description="NIC Card Mount 03 delta Z position",
        )
    )

    # NIC Card Mount 04 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_04_delta_x",
            default_value="0.0",
            description="NIC Card Mount 04 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_04_delta_y",
            default_value="0.0",
            description="NIC Card Mount 04 delta Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_04_delta_z",
            default_value="0.0",
            description="NIC Card Mount 04 delta Z position",
        )
    )

    # NIC Card Mount 05 delta arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_05_delta_x",
            default_value="0.0",
            description="NIC Card Mount 05 delta X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_05_delta_y",
            default_value="0.0",
            description="NIC Card Mount 05 delta Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nic_card_mount_05_delta_z",
            default_value="0.0",
            description="NIC Card Mount 05 delta Z position",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
