"""
Shared constants that define the output space of PortKeypointNet.

Output ordering is canonical: (entity, port) pairs in entity-dict order,
then cameras in left/center/right order.  Any code that builds or reads
the (NUM_OUTPUTS, *) tensors must use OUTPUT_KEYS / OUTPUT_KEY_TO_IDX.
"""

CAMERA_NAMES: list[str] = ["left_camera", "center_camera", "right_camera"]

ENTITY_PORTS: dict[str, list[str]] = {
    **{f"nic_card_mount_{i}": ["sfp_port_0", "sfp_port_1"] for i in range(5)},
    **{f"sc_port_{i}": ["sc_port_base"] for i in range(2)},
}

# Flat (entity, port) pairs — 12 total; order determines output slot
ENTITY_PORT_PAIRS: list[tuple[str, str]] = [
    (entity, port)
    for entity, ports in ENTITY_PORTS.items()
    for port in ports
]

NUM_ENTITY_PORT_PAIRS: int = len(ENTITY_PORT_PAIRS)  # 12
NUM_CAMERAS: int = len(CAMERA_NAMES)                  # 3
NUM_OUTPUTS: int = NUM_ENTITY_PORT_PAIRS * NUM_CAMERAS  # 36

# Canonical output key list: index i → (entity, port, camera)
OUTPUT_KEYS: list[tuple[str, str, str]] = [
    (entity, port, cam)
    for entity, port in ENTITY_PORT_PAIRS
    for cam in CAMERA_NAMES
]

OUTPUT_KEY_TO_IDX: dict[tuple[str, str, str], int] = {
    k: i for i, k in enumerate(OUTPUT_KEYS)
}
