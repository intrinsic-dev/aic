import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="aic_flowstate_ros_bridge",
                executable="test_robot_control_bridge.py",
                name="test_robot_control_bridge_node",
                output="screen",
                parameters=[
                    {
                        "controller_namespace": "aic_controller",
                        "test_mode": "pose",
                        # Example test_pose_targets format: JSON string of a flattened list of poses (or list of lists).
                        # The list is automatically chunked into 7 elements per pose: [x, y, z, qx, qy, qz, qw]
                        ##########################
                        # Home position
                        ##########################
                        "test_pose_targets": "["
                        "-0.1, -0.5, 0.5, 0.7071068, -0.7071068, 0.0, 0.0 "
                        "]",
                        "log_filepath": "",  # Empty string defaults to tcp_accuracy_<DATETIME>.json
                        "target_pose_stiffness": [
                            1000.0,
                            1000.0,
                            1000.0,
                            150.0,
                            150.0,
                            150.0,
                        ],
                        "target_pose_damping": [
                            1562.049935,
                            1562.049935,
                            1562.049935,
                            1096.814453,
                            1096.814453,
                            1096.814453,
                        ],
                        "publish_duration": 5.0,
                        "stop_duration": 2.0,
                    }
                ],
            )
        ]
    )
