#!/usr/bin/env python3
import argparse, random, yaml, os

NIC_RAIL_TRANSLATION_RANGE   = (-0.0215, 0.0234)
SC_RAIL_TRANSLATION_RANGE    = (-0.06,   0.055)
MOUNT_RAIL_TRANSLATION_RANGE = (-0.09425, 0.09425)
BOARD_X_RANGE    = (0.13, 0.20)
BOARD_Y_RANGE    = (-0.25, 0.10)
BOARD_Z          = 1.14
BOARD_YAW_BANDS  = [(2.80,2.95),(2.95,3.05),(3.05,3.20),(3.20,3.35)]
GRIPPER_Z_RANGE  = (0.038, 0.048)
GRIPPER_ROLL, GRIPPER_PITCH, GRIPPER_YAW = 0.4432, -0.4838, 1.3303

def rand(lo, hi): return round(random.uniform(lo, hi), 4)
def rand_t(r):    return rand(*r)

def build_nic_rails(active, trial_idx):
    # entity_name unique per trial to avoid Gazebo name collisions
    entity = f"nic_card_{trial_idx}"
    return {f"nic_rail_{i}": (
        {"entity_present": True, "entity_name": entity,
         "entity_pose": {"translation": rand_t(NIC_RAIL_TRANSLATION_RANGE),
                         "roll": 0.0, "pitch": 0.0,
                         "yaw": round(random.uniform(-0.1, 0.1), 4)}}
        if i == active else {"entity_present": False}
    ) for i in range(5)}

def build_sc_rails(active, trial_idx):
    # sc_port uses sc_mount_<trial_idx> as entity name
    entity = f"sc_mount_{trial_idx}"
    return {f"sc_rail_{i}": (
        {"entity_present": True, "entity_name": entity,
         "entity_pose": {"translation": rand_t(SC_RAIL_TRANSLATION_RANGE),
                         "roll": 0.0, "pitch": 0.0,
                         "yaw": round(random.uniform(-0.15, 0.15), 4)}}
        if i == active else {"entity_present": False}
    ) for i in range(2)}

def build_mount_rails(trial_idx):
    """
    6 mount rails. Each gets a unique entity_name scoped to this trial
    so no two spawned entities in the same trial share a name.
    """
    def maybe(prefix, slot):
        if random.choice([True, False]):
            return {
                "entity_present": True,
                "entity_name": f"{prefix}_{trial_idx}_{slot}",
                "entity_pose": {"translation": rand_t(MOUNT_RAIL_TRANSLATION_RANGE),
                                "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            }
        return {"entity_present": False}

    return {
        "lc_mount_rail_0":  maybe("lc_mount",  0),
        "sfp_mount_rail_0": maybe("sfp_mount", 0),
        "sc_mount_rail_0":  maybe("sc_mount",  0),
        "lc_mount_rail_1":  maybe("lc_mount",  1),
        "sfp_mount_rail_1": maybe("sfp_mount", 1),
        "sc_mount_rail_1":  maybe("sc_mount",  1),
    }

def build_trial(trial_idx, plug_type, nic_rail, sc_rail, yaw_band):
    board = {"x": rand(*BOARD_X_RANGE), "y": rand(*BOARD_Y_RANGE), "z": BOARD_Z,
             "roll": 0.0, "pitch": 0.0, "yaw": rand(*yaw_band)}
    cable_name = f"cable_{trial_idx}"

    if plug_type == "sfp":
        nic_rails  = build_nic_rails(nic_rail, trial_idx)
        sc_rails   = build_sc_rails(sc_rail, trial_idx)
        cable_type = "sfp_sc_cable"
        task = {"cable_type": "sfp_sc", "cable_name": cable_name,
                "plug_type": "sfp", "plug_name": "sfp_tip",
                "port_type": "sfp", "port_name": "sfp_port_0",
                "target_module_name": f"nic_card_mount_{nic_rail}",
                "time_limit": 180}
    else:
        nic_rails  = {f"nic_rail_{i}": {"entity_present": False} for i in range(5)}
        sc_rails   = build_sc_rails(sc_rail, trial_idx)
        cable_type = "sfp_sc_cable_reversed"
        task = {"cable_type": "sfp_sc", "cable_name": cable_name,
                "plug_type": "sc", "plug_name": "sc_tip",
                "port_type": "sc", "port_name": "sc_port_base",
                "target_module_name": f"sc_port_{sc_rail}",
                "time_limit": 180}

    cable = {cable_name: {
        "pose": {"gripper_offset": {"x": 0.0, "y": 0.015385,
                                    "z": rand(*GRIPPER_Z_RANGE)},
                 "roll": GRIPPER_ROLL, "pitch": GRIPPER_PITCH,
                 "yaw": GRIPPER_YAW},
        "attach_cable_to_gripper": True,
        "cable_type": cable_type
    }}

    tb = {"pose": board}
    tb.update(nic_rails)
    tb.update(sc_rails)
    tb.update(build_mount_rails(trial_idx))

    return {"scene": {"task_board": tb, "cables": cable},
            "tasks": {"task_1": task}}

def build_config(num_trials, seed=None):
    if seed is not None:
        random.seed(seed)

    # Stratified pools — guarantee intra-run diversity
    nic_pool = list(range(5)) * ((num_trials // 5) + 1)
    random.shuffle(nic_pool)

    sc_pool = [i % 2 for i in range(num_trials)]
    random.shuffle(sc_pool)

    yaw_pool = BOARD_YAW_BANDS * ((num_trials // len(BOARD_YAW_BANDS)) + 1)
    random.shuffle(yaw_pool)

    # 2/3 sfp, 1/3 sc for class balance
    plugs = ["sc" if i % 3 == 2 else "sfp" for i in range(num_trials)]
    random.shuffle(plugs)

    scoring_topics = [
        {"topic": {"name": "/joint_states",              "type": "sensor_msgs/msg/JointState"}},
        {"topic": {"name": "/tf",                        "type": "tf2_msgs/msg/TFMessage"}},
        {"topic": {"name": "/tf_static",                 "type": "tf2_msgs/msg/TFMessage", "latched": True}},
        {"topic": {"name": "/scoring/tf",                "type": "tf2_msgs/msg/TFMessage"}},
        {"topic": {"name": "/aic/gazebo/contacts/off_limit", "type": "ros_gz_interfaces/msg/Contacts"}},
        {"topic": {"name": "/fts_broadcaster/wrench",    "type": "geometry_msgs/msg/WrenchStamped"}},
        {"topic": {"name": "/aic_controller/joint_commands", "type": "aic_control_interfaces/msg/JointMotionUpdate"}},
        {"topic": {"name": "/aic_controller/pose_commands",  "type": "aic_control_interfaces/msg/MotionUpdate"}},
        {"topic": {"name": "/scoring/insertion_event",   "type": "std_msgs/msg/String"}},
        {"topic": {"name": "/aic_controller/controller_state", "type": "aic_control_interfaces/msg/ControllerState"}},
    ]

    config = {
        "scoring": {"topics": scoring_topics},
        "task_board_limits": {
            "nic_rail":   {"min_translation": -0.0215,  "max_translation": 0.0234},
            "sc_rail":    {"min_translation": -0.06,    "max_translation": 0.055},
            "mount_rail": {"min_translation": -0.09425, "max_translation": 0.09425},
        },
        "trials": {},
        "robot": {"home_joint_positions": {
            "shoulder_pan_joint":  -0.1597,
            "shoulder_lift_joint": -1.3542,
            "elbow_joint":         -1.6648,
            "wrist_1_joint":       -1.6933,
            "wrist_2_joint":        1.5710,
            "wrist_3_joint":        1.4110,
        }}
    }

    for i in range(num_trials):
        config["trials"][f"trial_{i+1}"] = build_trial(
            i, plugs[i], nic_pool[i], sc_pool[i], yaw_pool[i])

    return config

def main():
    parser = argparse.ArgumentParser(
        description="Generate randomized AIC sample_config.yaml")
    parser.add_argument("--trials", type=int, default=6)
    parser.add_argument("--output", type=str, default=os.path.expanduser(
        "~/ws_aic/src/aic/aic_engine/config/sample_config.yaml"))
    parser.add_argument("--seed",    type=int,  default=None)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()

    config = build_config(args.trials, args.seed)

    if args.preview:
        print(yaml.dump(config, default_flow_style=False, sort_keys=False))
    else:
        os.makedirs(os.path.dirname(args.output), exist_ok=True)
        with open(args.output, "w") as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        print(f"[generate_config] Wrote {args.trials} trials → {args.output}")
        for k, v in config["trials"].items():
            t = v["tasks"]["task_1"]
            b = v["scene"]["task_board"]["pose"]
            print(f"  {k}: plug={t['plug_type']:3s}  "
                  f"target={t['target_module_name']:20s}  "
                  f"yaw={b['yaw']:.3f}  y={b['y']:.3f}")

if __name__ == "__main__":
    main()