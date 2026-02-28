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

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # UR arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    ur_tf_prefix = LaunchConfiguration("ur_tf_prefix")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    spawn_admittance_controller = LaunchConfiguration("spawn_admittance_controller")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")
    robot_roll = LaunchConfiguration("robot_roll")
    robot_pitch = LaunchConfiguration("robot_pitch")
    robot_yaw = LaunchConfiguration("robot_yaw")

    gripper_initial_pos = "0.00655"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            ur_tf_prefix,
            " ",
            "simulation_controllers:=",
            controllers_file,
            " ",
            "x:=",
            robot_x,
            " ",
            "y:=",
            robot_y,
            " ",
            "z:=",
            robot_z,
            " ",
            "roll:=",
            robot_roll,
            " ",
            "pitch:=",
            robot_pitch,
            " ",
            "yaw:=",
            robot_yaw,
            " ",
            "gripper_initial_pos:=",
            gripper_initial_pos,
            " ",
            "hardware_plugin:=",
            "mujoco_ros2_control/MujocoSystem",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Get MuJoCo environment variables from the sourced workspace
    mujoco_plugin_path = os.environ.get("MUJOCO_PLUGIN_PATH", "")
    mujoco_dir = os.environ.get("MUJOCO_DIR", "")

    # Debug: Print environment variables
    print(f"[aic_mujoco_bringup] MUJOCO_PLUGIN_PATH: {mujoco_plugin_path}")
    print(f"[aic_mujoco_bringup] MUJOCO_DIR: {mujoco_dir}")

    # MuJoCo ros2_control node (with visualization)
    mujoco_model_path = PathJoinSubstitution(
        [
            FindPackageShare("aic_mujoco"),
            "mjcf",
            "scene.xml",
        ]
    )

    node_mujoco_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            controllers_file,
            {
                "use_sim_time": True,
                "mujoco_model_path": mujoco_model_path,
            },
        ],
        additional_env={
            # Preload system tinyxml2 to avoid symbol collision with MuJoCo's bundled tinyxml2
            "LD_PRELOAD": "/usr/lib/x86_64-linux-gnu/libtinyxml2.so.10"
        },
        remappings=[
            ("/left_camera/color", "left_camera/image"),
            ("/right_camera/color", "right_camera/image"),
            ("/center_camera/color", "center_camera/image"),
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True, "ignore_timestamp": True},
            robot_description,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    aic_adapter = Node(
        package="aic_adapter",
        executable="aic_adapter",
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    initial_joint_controllers = [initial_joint_controller]
    if IfCondition(spawn_admittance_controller).evaluate(context):
        initial_joint_controllers.append("admittance_controller")

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *initial_joint_controllers,
            "--activate-as-group",
            "-c",
            "/controller_manager",
        ],
        condition=IfCondition(activate_joint_controller),
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *initial_joint_controllers,
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        condition=UnlessCondition(activate_joint_controller),
    )

    # Gripper and FTS controllers - commented out for simplified setup
    # gripper_action_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "gripper_action_controller",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fts_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Start mujoco_ros2_control after robot_state_publisher
    delay_mujoco_after_robot_state_publisher = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[node_mujoco_ros2_control],
        )
    )

    # Launch controllers after mujoco_ros2_control starts
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_mujoco_ros2_control,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_initial_joint_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                initial_joint_controller_spawner_started,
                initial_joint_controller_spawner_stopped,
                fts_broadcaster_spawner,
                # gripper_action_controller_spawner,
            ],
        )
    )

    # Set MuJoCo environment variables for the entire launch file
    set_mujoco_plugin_path = SetEnvironmentVariable(
        name="MUJOCO_PLUGIN_PATH", value=mujoco_plugin_path
    )

    set_mujoco_dir = SetEnvironmentVariable(name="MUJOCO_DIR", value=mujoco_dir)

    nodes_to_start = [
        set_mujoco_plugin_path,
        set_mujoco_dir,
        robot_state_publisher_node,
        delay_mujoco_after_robot_state_publisher,
        delay_joint_state_broadcaster,
        delay_initial_joint_controller,
        delay_rviz_after_joint_state_broadcaster_spawner,
        aic_adapter,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="false",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_bringup"), "config", "aic_ros2_controllers.yaml"]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="aic_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_admittance_controller",
            default_value="false",
            description="If true, then the admittance controller is spawned alongside the initial_joint_controller. Else, only the initial_joint_controller is spawned.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_description"), "urdf", "ur_gz.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_bringup"), "rviz", "aic.rviz"]
            ),
            description="Rviz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_x", default_value="-0.2", description="Robot spawn X position"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_y", default_value="0.2", description="Robot spawn Y position"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_z", default_value="1.14", description="Robot spawn Z position"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_roll",
            default_value="0.0",
            description="Robot spawn roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_pitch",
            default_value="0.0",
            description="Robot spawn pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_yaw",
            default_value="-3.141",
            description="Robot spawn yaw orientation (radians)",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
