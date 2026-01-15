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
from geometry_msgs.msg import WrenchStamped
from rosetta.common.contract_utils import SpecView
from rosetta.common.decoders import _decode_via_names, register_decoder
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def register_decoders():
    """Register decoders for messages not yet supported in rosetta.

    These decoders should be removed once they are upstreamed.
    """

    @register_decoder("geometry_msgs/msg/WrenchStamped")
    def decode_wrench_stamped(msg: WrenchStamped, spec: SpecView):
        return _decode_via_names(msg, spec.names)

    @register_decoder("trajectory_msgs/msg/JointTrajectory")
    def decode_joint_trajecotry(msg: JointTrajectory, spec: SpecView):
        if len(msg.points) == 0:
            return None
        last_point: JointTrajectoryPoint = msg.points[-1]  # type: ignore
        joint_idx = {n: i for i, n in enumerate(msg.joint_names)}
        return np.array(
            [last_point.positions[joint_idx[n]] for n in spec.names], dtype=np.float32
        )
