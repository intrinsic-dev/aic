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


from abc import ABC, abstractmethod
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Wrench, Vector3
from typing import Callable, Protocol
import numpy as np


GetObservationCallback = Callable[[], Observation]


class SetPoseTargetCallback(Protocol):
    def __call__(self, pose: Pose, frame_id: str = "base_link") -> None: ...


SendFeedbackCallback = Callable[[str], None]


class Policy(ABC):
    def __init__(self, parent_node):
        self._parent_node = parent_node
        self.get_logger().info("Policy.__init__()")

    def get_logger(self):
        return self._parent_node.get_logger()

    def get_clock(self):
        return self._parent_node.get_clock()

    @abstractmethod
    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        set_pose_target: SetPoseTargetCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        """Called when the insert_cable task is requested by aic_engine"""
        pass
