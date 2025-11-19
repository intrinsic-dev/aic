from rosetta.common.decoders import register_decoder


def register_decoders():
    """Register decoders for messages not yet supported in rosetta.

    These decoders should be removed once they are upstreamed.
    """

    @register_decoder("geometry_msgs/msg/WrenchStamped")
    def decode_wrench_stamped(msg):
        # TODO
        pass
