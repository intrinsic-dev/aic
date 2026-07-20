import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aic_flowstate_ros_bridge',
            executable='test_robot_control_bridge.py',
            name='test_robot_control_bridge_node',
            output='screen',
            parameters=[
                {
                    'controller_namespace': 'aic_controller',
                    'test_mode': 'joint',
                    ##########################
                    # Home position:
                    ##########################
                    'test_joint_targets': '['
                    '-1.57, -1.57, -1.57, -1.57, 1.57, -1.57'
                    ']',
                    ##########################
                    # Position 1:
                    ##########################
                    # 'test_joint_targets': '['
                    # '0.0, -0.785, 0.0, -2.356, 0.0, 1.571 '
                    # ']',
                    'log_filepath': '',  # Empty string defaults to tcp_accuracy_<DATETIME>.json
                    'target_pose_stiffness': [1000.0, 150.0],
                    'target_pose_damping': [200.0, 54.7723],
                    'target_joint_stiffness': [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
                    'target_joint_damping': [10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
                    'publish_duration': 3.0,
                    'stop_duration': 2.0
                }
            ]
        )
    ])
