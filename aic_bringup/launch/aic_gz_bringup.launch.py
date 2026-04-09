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
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def on_aic_engine_exit(event, context):
    if event.returncode != 0:
        raise RuntimeError(f"aic_engine exited with code {event.returncode}")
    return EmitEvent(
        event=Shutdown(
            reason=f"aic_engine exited cleanly with code {event.returncode})"
        )
    )


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
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ros_gz_bridge_config_file = LaunchConfiguration("ros_gz_bridge_config_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")
    robot_roll = LaunchConfiguration("robot_roll")
    robot_pitch = LaunchConfiguration("robot_pitch")
    robot_yaw = LaunchConfiguration("robot_yaw")
    task_board_x = LaunchConfiguration("task_board_x")
    task_board_y = LaunchConfiguration("task_board_y")
    task_board_z = LaunchConfiguration("task_board_z")
    task_board_roll = LaunchConfiguration("task_board_roll")
    task_board_pitch = LaunchConfiguration("task_board_pitch")
    task_board_yaw = LaunchConfiguration("task_board_yaw")
    cable_x = LaunchConfiguration("cable_x")
    cable_y = LaunchConfiguration("cable_y")
    cable_z = LaunchConfiguration("cable_z")
    cable_roll = LaunchConfiguration("cable_roll")
    cable_pitch = LaunchConfiguration("cable_pitch")
    cable_yaw = LaunchConfiguration("cable_yaw")
    attach_cable_to_gripper = LaunchConfiguration("attach_cable_to_gripper")
    cable_type = LaunchConfiguration("cable_type")
    ground_truth = LaunchConfiguration("ground_truth")
    start_aic_engine = LaunchConfiguration("start_aic_engine")
    shutdown_on_aic_engine_exit = LaunchConfiguration("shutdown_on_aic_engine_exit")
    aic_engine_config_file = LaunchConfiguration("aic_engine_config_file")
    model_discovery_timeout_seconds = LaunchConfiguration(
        "model_discovery_timeout_seconds"
    )
    model_configure_timeout_seconds = LaunchConfiguration(
        "model_configure_timeout_seconds"
    )

    gripper_initial_pos = "0.00655"
    cable_type_str = LaunchConfiguration("cable_type").perform(context)
    if cable_type_str == "sfp_sc_cable":
        gripper_initial_pos = "0.0073"

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
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--service-call-timeout",
            "30",
        ],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
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
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        condition=UnlessCondition(activate_joint_controller),
    )

    fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fts_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--service-call-timeout",
            "30",
        ],
    )

    aic_adapter = Node(
        package="aic_adapter",
        executable="aic_adapter",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    aic_engine = Node(
        package="aic_engine",
        executable="aic_engine",
        output="screen",
        parameters=[
            {
                "config_file_path": aic_engine_config_file,
                "use_sim_time": True,
                "model_discovery_timeout_seconds": model_discovery_timeout_seconds,
                "model_configure_timeout_seconds": model_configure_timeout_seconds,
            },
        ],
        condition=IfCondition(start_aic_engine),
    )

    # Event handler to shutdown launch file when aic_engine exits
    shutdown_on_aic_engine_exit_handler = RegisterEventHandler(
        OnProcessExit(target_action=aic_engine, on_exit=on_aic_engine_exit),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    start_aic_engine,
                    "' == 'true' and '",
                    shutdown_on_aic_engine_exit,
                    "' == 'true'",
                ]
            )
        ),
    )

    # Task board spawning (conditional)
    spawn_task_board = LaunchConfiguration("spawn_task_board")
    task_board_description_file = LaunchConfiguration("task_board_description_file")

    spawn_task_board_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aic_bringup"),
                    "launch",
                    "spawn_task_board.launch.py",
                ]
            )
        ),
        launch_arguments={
            "task_board_description_file": task_board_description_file,
            "task_board_x": task_board_x,
            "task_board_y": task_board_y,
            "task_board_z": task_board_z,
            "task_board_roll": task_board_roll,
            "task_board_pitch": task_board_pitch,
            "task_board_yaw": task_board_yaw,
        }.items(),
        condition=IfCondition(spawn_task_board),
    )

    # Cable spawning (conditional)
    spawn_cable = LaunchConfiguration("spawn_cable")
    cable_description_file = LaunchConfiguration("cable_description_file")

    spawn_cable_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aic_bringup"),
                    "launch",
                    "spawn_cable.launch.py",
                ]
            )
        ),
        launch_arguments={
            "cable_description_file": cable_description_file,
            "cable_x": cable_x,
            "cable_y": cable_y,
            "cable_z": cable_z,
            "cable_roll": cable_roll,
            "cable_pitch": cable_pitch,
            "cable_yaw": cable_yaw,
            "attach_cable_to_gripper": attach_cable_to_gripper,
            "cable_type": cable_type,
        }.items(),
        condition=IfCondition(spawn_cable),
    )

    gz_ip_env = SetEnvironmentVariable(name="GZ_IP", value="127.0.0.1")

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur5e",
            "-allow_renaming",
            "true",
        ],
    )

    gzserver = GzServer(
        world_sdf_file=world_file,
        container_name="ros_gz_container",
        create_own_container="True",
        use_composition="True",
    )

    gzgui = ExecuteProcess(
        cmd=["gz", "sim", "-g"],
        condition=IfCondition(PythonExpression(["'", gazebo_gui, "' == 'true'"])),
        output="screen",
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name="ros_gz_bridge",
        config_file=ros_gz_bridge_config_file,
        container_name="ros_gz_container",
        create_own_container="False",
        use_composition="True",
    )

    ground_truth_tf_relay = Node(
        package="topic_tools",
        executable="relay",
        name="tf_relay",
        output="screen",
        parameters=[
            {
                "input_topic": "/scoring/tf",
                "output_topic": "/tf",
                "lazy": True,
            }
        ],
        condition=IfCondition(ground_truth),
    )

    ground_truth_static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ground_truth_static_tf_publisher",
        output="screen",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "aic_world",
        ],
        condition=IfCondition(ground_truth),
    )

    delay_basic_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, fts_broadcaster_spawner],
        )
    )

    delay_initial_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                initial_joint_controller_spawner_started,
                initial_joint_controller_spawner_stopped,
            ],
        )
    )

    delay_rviz_after_controller_started = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_joint_controller_spawner_started,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    delay_rviz_after_controller_stopped = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_joint_controller_spawner_stopped,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        delay_basic_controllers_after_spawn,
        delay_initial_controller_after_broadcaster,
        delay_rviz_after_controller_started,
        delay_rviz_after_controller_stopped,
        aic_adapter,
        gz_ip_env,
        gzserver,
        gzgui,
        ros_gz_bridge,
        gz_spawn_entity,
        spawn_task_board_launch,
        spawn_cable_launch,
        ground_truth_tf_relay,
        ground_truth_static_tf_publisher,
        aic_engine,
        shutdown_on_aic_engine_exit_handler,
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
            "spawn_task_board",
            default_value="false",
            description="Spawn task board in Gazebo?",
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
            "task_board_description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_description"), "urdf", "task_board.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the task board.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros_gz_bridge_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_bringup"), "config", "ros_gz_bridge_config.yaml"]
            ),
            description="ros_gz bridge config file (absolute path) to use.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_description"), "world", "aic.sdf"]
            ),
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_x",
            default_value="0.15",
            description="Task board spawn X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "task_board_y",
            default_value="-0.2",
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
            default_value="3.1415",
            description="Task board spawn yaw orientation (radians)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "attach_cable_to_gripper",
            default_value="false",
            description="Whether to attach cable to gripper (applicable only if spawn_cable is true)",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_cable",
            default_value="false",
            description="Whether to spawn the cable",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_description"), "urdf", "cable.sdf.xacro"]
            ),
            description="SDF/XACRO file to use for cable.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_x",
            default_value="0.172",
            description="Cable spawn X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_y",
            default_value="0.024",
            description="Cable spawn Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_z",
            default_value="1.518",
            description="Cable spawn Z position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_roll",
            default_value="0.4432",
            description="Cable spawn roll orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_pitch",
            default_value="-0.48",
            description="Cable spawn pitch orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cable_yaw",
            default_value="1.3303",
            description="Cable spawn yaw orientation (radians)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ground_truth",
            default_value="false",
            description="Whether to include ground truth poses in TF topics",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_aic_engine",
            default_value="false",
            description="Whether to start the AIC engine.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "shutdown_on_aic_engine_exit",
            default_value="false",
            description="Whether to shutdown the launch file when aic_engine exits. "
            "Only takes effect when start_aic_engine is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aic_engine_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aic_engine"), "config", "sample_config.yaml"]
            ),
            description="Absolute path to YAML file with the AIC engine configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "model_configure_timeout_seconds",
            default_value="60",
            description="Timeout for model configuration checks.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "model_discovery_timeout_seconds",
            default_value="30",
            description="Timeout for discovering the participant model.",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
