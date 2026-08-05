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
                        "test_mode": "joint",
                        # Example test_joint_targets format: JSON string of a flattened list of joint values (or list of lists).
                        # The list is automatically chunked into 6 values per joint target: [j1, j2, j3, j4, j5, j6]
                        ##########################
                        # Home position:
                        ##########################
                        "test_joint_targets": "["
                        "-1.57, -1.57, -1.57, -1.57, 1.57, -1.57"
                        "]",
                        "log_filepath": "",  # Empty string defaults to tcp_accuracy_<DATETIME>.json
                        "target_joint_stiffness": [
                            175.0,
                            175.0,
                            175.0,
                            125.0,
                            50.0,
                            50.0,
                        ],
                        "target_joint_damping": [
                            50.0,
                            50.0,
                            50.0,
                            35.0,
                            10.0,
                            10.0,
                        ],
                        "publish_duration": 3.0,
                        "stop_duration": 2.0,
                    }
                ],
            )
        ]
    )
