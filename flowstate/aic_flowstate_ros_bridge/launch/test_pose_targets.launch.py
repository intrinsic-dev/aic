import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('aic_flowstate_ros_bridge'),
            'config',
            'test_pose_targets.yaml'
        ),
        description='Path to the YAML configuration file for the test_robot_control_bridge_node'
    )

    return LaunchDescription(
        [
            config_file_arg,
            Node(
                package="aic_flowstate_ros_bridge",
                executable="test_robot_control_bridge.py",
                name="test_robot_control_bridge_node",
                output="screen",
                parameters=[LaunchConfiguration('config_file')],
            )
        ]
    )
