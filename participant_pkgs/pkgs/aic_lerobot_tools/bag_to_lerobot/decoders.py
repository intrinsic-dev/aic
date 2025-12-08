import geometry_msgs.msg
import numpy as np
from rosetta.common.contract_utils import SpecView
from rosetta.common.decoders import _decode_via_names, register_decoder


def register_decoders():
    """Register decoders for messages not yet supported in rosetta.

    These decoders should be removed once they are upstreamed.
    """

    @register_decoder("geometry_msgs/msg/WrenchStamped")
    def decode_wrench_stamped(msg: geometry_msgs.msg.WrenchStamped, spec: SpecView):
        if spec.names:
            return _decode_via_names(msg, spec.names)
        return np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ],
            dtype=np.float32,
        )
