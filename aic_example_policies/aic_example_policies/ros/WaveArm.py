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


import numpy as np


from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_control_interfaces.msg import (
    MotionUpdate,
    TrajectoryGenerationMode,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Wrench
from rclpy.duration import Duration


class WaveArm(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("WaveArm.__init__()")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info(f"WaveArm.insert_cable() enter. Task: {task}")
        start_time = self.time_now()
        timeout = Duration(seconds=10.0)
        send_feedback("waving the arm around")
        while (self.time_now() - start_time) < timeout:
            self.sleep_for(0.25)
            observation = get_observation()

            if observation is None:
                self.get_logger().info("No observation received.")
                continue

            t = (
                observation.center_image.header.stamp.sec
                + observation.center_image.header.stamp.nanosec / 1e9
            )
            self.get_logger().info(f"observation time: {t}")

            loop_seconds = 5.0
            loop_fraction = (t % loop_seconds) / loop_seconds
            y_scale = 2 * loop_fraction
            if y_scale > 1.0:
                y_scale = 2.0 - y_scale
            y_scale -= 1.0  # y_scale will move linearly between [-1..1] and back.

            # Move the arm along a line, while looking down at the task board.
            self.set_pose_target(
                move_robot=move_robot,
                pose=Pose(
                    position=Point(x=-0.4, y=0.45 + 0.3 * y_scale, z=0.25),
                    orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
                ),
            )

        self.get_logger().info("WaveArm.insert_cable() exiting...")
        return True
