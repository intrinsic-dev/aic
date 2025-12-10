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
