#!/usr/bin/env python3
"""
Generate a randomised aic_engine YAML config for overnight CheatCode data collection.

Each trial is a randomly drawn (task_type, board_pose, NIC/SC placement) combination.
Both SFP-into-NIC and SC-into-SC-port tasks are supported; pick with --task-type.

Usage:
    cd ~/co/aic
    # 50 SC-only (Trial 3) trials:
    pixi run python3 scripts/generate_collection_config.py --trials 50 --task-type sc --output /tmp/collection.yaml
    # mixed (original 2:1 SFP:SC behaviour):
    pixi run python3 scripts/generate_collection_config.py --trials 50 --task-type mixed --seed 42 --output /tmp/collection.yaml
"""

import argparse
import random
import textwrap
from pathlib import Path


# ── Physical limits ───────────────────────────────────────────────────────────
# NIC rail: sample_config.yaml uses translation = 0.036, which exceeds the
# advertised task_board_limits range of [-0.0215, 0.0234]. The trial values
# are the empirically-valid evidence of what the sim accepts, so widen the
# range symmetrically to match. (task_board_limits appears to be advisory
# rather than strictly enforced.)
NIC_RAIL_TRANS  = (-0.036,   0.036)
SC_RAIL_TRANS   = (-0.060,   0.055)
MOUNT_RAIL_TRANS = (-0.09425, 0.09425)

# SC ports in sample_config.yaml have non-zero yaw (e.g. yaw=0.1). Although
# the qualification doc only mentions translation randomization for Trial 3,
# we mirror the sample's behaviour: ±0.1 rad (~5.7°) yaw jitter on SC ports.
SC_RAIL_YAW_JITTER = 0.1     # set to 0.0 to match qualification doc strictly

# ── Board pose ranges (observed in 10_trials.yaml) ────────────────────────────
BOARD_X   = (0.14, 0.46)
BOARD_Y   = (-0.20, 0.05)
# Task board rests flush on the table; Z is fixed (gravity pins it).
# Per sample_config.yaml, all reference trials use z = 1.14 — this is the
# authoritative table-surface height in the AIC sim environment.
# The original 10_trials.yaml had Z ranging [1.10, 1.22], which corresponds
# to the board floating up to 8 cm above the table — not physically realistic.
# Adjust TABLE_SURFACE_Z only if your sim's table is at a different height.
TABLE_SURFACE_Z   = 1.14
BOARD_Z_JITTER    = 0.000   # set to e.g. 0.002 for ±2 mm placement noise
BOARD_YAW = (-0.35, 3.49)   # 0 to ~200°

# ── Cable gripper offsets (from sample_config.yaml / 10_trials.yaml) ─────────
SFP_CABLE_OFFSET = {"x": 0.0, "y": 0.015385, "z": 0.04245,
                    "roll": 0.4432, "pitch": -0.4838, "yaw": 1.3303}
SC_CABLE_OFFSET  = {"x": 0.0, "y": 0.015385, "z": 0.04045,
                    "roll": 0.4432, "pitch": -0.4838, "yaw": 1.3303}

# ── Grasp pose deviations (~2mm / ~0.04rad per qualification_phase.md §1) ────
GRASP_TRANS_JITTER = 0.002   # metres
GRASP_ROT_JITTER   = 0.04    # radians

# ── NIC entity names pool ─────────────────────────────────────────────────────
NIC_ENTITY_NAMES = ["nic_card_0", "nic_card_1", "nic_card_2", "nic_card_3", "nic_card_4"]
SC_ENTITY_NAMES  = ["sc_mount_0", "sc_mount_1", "sc_mount_2"]

# Probability that a Trial-3 scene also includes the non-target SC port as a
# distractor (qualification_phase.md says "one or both SC ports are mounted").
# Only used when stratification is disabled.
SC_DISTRACTOR_PROB = 0.5


def build_sc_diversity_plan(n_trials: int, rng: random.Random):
    """Exhaustive-cycling plan for SC trials over the discrete config space.

    The discrete space has 2 × 2 × 16 = 64 distinct configs:
      target_rail ∈ {0, 1}
      include_distractor_sc ∈ {False, True}
      nic_rails ∈ all subsets of {0..4} of size 0, 1, or 2  (16 subsets total)

    We enumerate all 64, shuffle, and cycle. This guarantees:
      - if n_trials ≤ 64: every trial has a unique discrete fingerprint
      - if n_trials > 64: configs repeat at most ⌈n_trials / 64⌉ times each

    Returns a list of (target_rail, include_distractor, n_nic_props, nic_rails_tuple).
    """
    from itertools import combinations
    nic_subsets = []
    for k in (0, 1, 2):
        nic_subsets.extend(combinations(range(5), k))

    all_configs = [
        (t, d, len(nic), nic)
        for t in (0, 1)
        for d in (False, True)
        for nic in nic_subsets
    ]
    rng.shuffle(all_configs)

    plan = []
    while len(plan) < n_trials:
        rng.shuffle(all_configs)
        plan.extend(all_configs)
    return plan[:n_trials]

# ── Home robot pose (fixed) ───────────────────────────────────────────────────
HOME_JOINTS = {
    "shoulder_pan_joint":  -0.1597,
    "shoulder_lift_joint": -1.3542,
    "elbow_joint":         -1.6648,
    "wrist_1_joint":       -1.6933,
    "wrist_2_joint":        1.5710,
    "wrist_3_joint":        1.4110,
}


def rand(lo, hi):
    return round(random.uniform(lo, hi), 4)


def rand_trans(lo, hi):
    return round(random.uniform(lo, hi), 4)


def jittered_offset(base: dict, rng: random.Random) -> dict:
    """Apply ±2mm / ±0.04rad noise to a base grasp offset.

    Per qualification_phase.md: 'in practice there will be small deviations
    (~2mm, ~0.04 rad) in the relative grasp pose. We encourage participants
    to develop policies that are robust against these minor differences.'
    """
    return {
        "x":     round(base["x"]     + rng.uniform(-GRASP_TRANS_JITTER, GRASP_TRANS_JITTER), 6),
        "y":     round(base["y"]     + rng.uniform(-GRASP_TRANS_JITTER, GRASP_TRANS_JITTER), 6),
        "z":     round(base["z"]     + rng.uniform(-GRASP_TRANS_JITTER, GRASP_TRANS_JITTER), 6),
        "roll":  round(base["roll"]  + rng.uniform(-GRASP_ROT_JITTER, GRASP_ROT_JITTER), 4),
        "pitch": round(base["pitch"] + rng.uniform(-GRASP_ROT_JITTER, GRASP_ROT_JITTER), 4),
        "yaw":   round(base["yaw"]   + rng.uniform(-GRASP_ROT_JITTER, GRASP_ROT_JITTER), 4),
    }


def build_board_pose():
    z = TABLE_SURFACE_Z
    if BOARD_Z_JITTER > 0:
        z = round(z + random.uniform(-BOARD_Z_JITTER, BOARD_Z_JITTER), 4)
    return {
        "x": rand(*BOARD_X),
        "y": rand(*BOARD_Y),
        "z": z,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": rand(*BOARD_YAW),
    }


def build_sfp_trial(trial_idx: int, rng: random.Random) -> dict:
    """SFP plug → one of 5 NIC rails. 1–3 NIC cards present as distractors."""
    target_rail = rng.randint(0, 4)
    n_distractors = rng.randint(0, 2)
    other_rails = [r for r in range(5) if r != target_rail]
    rng.shuffle(other_rails)
    distractor_rails = other_rails[:n_distractors]
    active_nic_rails = sorted([target_rail] + distractor_rails)

    nic_entity_pool = rng.sample(NIC_ENTITY_NAMES, len(active_nic_rails))

    sc_props = rng.randint(0, 1)
    sc_rail_for_prop = rng.randint(0, 1) if sc_props else None

    board_pose = build_board_pose()
    grasp = jittered_offset(SFP_CABLE_OFFSET, rng)

    return {
        "scene": {
            "task_board": {
                "pose": board_pose,
                **{
                    f"nic_rail_{i}": (
                        {
                            "entity_present": True,
                            "entity_name": nic_entity_pool[active_nic_rails.index(i)],
                            "entity_pose": {
                                "translation": rand_trans(*NIC_RAIL_TRANS),
                                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                            },
                        }
                        if i in active_nic_rails else {"entity_present": False}
                    )
                    for i in range(5)
                },
                **{
                    f"sc_rail_{i}": (
                        {
                            "entity_present": True,
                            "entity_name": rng.choice(SC_ENTITY_NAMES),
                            "entity_pose": {
                                "translation": rand_trans(*SC_RAIL_TRANS),
                                "roll": 0.0, "pitch": 0.0,
                                "yaw": round(rng.uniform(-SC_RAIL_YAW_JITTER, SC_RAIL_YAW_JITTER), 4),
                            },
                        }
                        if i == sc_rail_for_prop else {"entity_present": False}
                    )
                    for i in range(2)
                },
                "lc_mount_rail_0":  {"entity_present": False},
                "sfp_mount_rail_0": {"entity_present": False},
                "sc_mount_rail_0":  {"entity_present": False},
                "lc_mount_rail_1":  {"entity_present": False},
                "sfp_mount_rail_1": {"entity_present": False},
                "sc_mount_rail_1":  {"entity_present": False},
            },
            "cables": {
                "cable_0": {
                    "pose": {
                        "gripper_offset": {"x": grasp["x"], "y": grasp["y"], "z": grasp["z"]},
                        "roll": grasp["roll"],
                        "pitch": grasp["pitch"],
                        "yaw": grasp["yaw"],
                    },
                    "attach_cable_to_gripper": True,
                    "cable_type": "sfp_sc_cable",
                },
            },
        },
        "tasks": {
            "task_1": {
                "cable_type": "sfp_sc",
                "cable_name": "cable_0",
                "plug_type": "sfp",
                "plug_name": "sfp_tip",
                "port_type": "sfp",
                "port_name": "sfp_port_0",
                "target_module_name": f"nic_card_mount_{target_rail}",
                "time_limit": 180,
            },
        },
    }


def build_sc_trial(trial_idx: int, rng: random.Random, plan=None) -> dict:
    """SC plug → one of 2 SC rails.

    Per qualification_phase.md Trial 3:
      - 'One or both SC ports are mounted... Only one SC port will be the target port.'
      - 0-2 NIC cards may be present as visual distractors.
      - Grasp pose has ~2mm / ~0.04rad deviation from nominal.

    If `plan` is provided as (target_rail, include_distractor, n_nic_props,
    nic_rails_tuple) the discrete config is fixed; continuous params still
    randomise. Otherwise discrete config is drawn iid (legacy behaviour).
    """
    if plan is not None:
        target_rail, include_distractor_sc, n_nic_props, nic_rails_planned = plan
        nic_rails = sorted(nic_rails_planned)
    else:
        target_rail = rng.randint(0, 1)
        include_distractor_sc = rng.random() < SC_DISTRACTOR_PROB
        n_nic_props = rng.randint(0, 2)
        nic_rails = sorted(rng.sample(range(5), n_nic_props)) if n_nic_props > 0 else []

    target_module = f"sc_port_{target_rail}"

    # Pick distinct SC entity names for target (and optionally distractor).
    n_sc_entities = 2 if include_distractor_sc else 1
    sc_entities = rng.sample(SC_ENTITY_NAMES, n_sc_entities)
    target_sc_entity = sc_entities[0]
    distractor_sc_entity = sc_entities[1] if include_distractor_sc else None

    nic_entity_pool = rng.sample(NIC_ENTITY_NAMES, n_nic_props) if n_nic_props > 0 else []

    board_pose = build_board_pose()
    grasp = jittered_offset(SC_CABLE_OFFSET, rng)

    def sc_rail_entry(i: int) -> dict:
        if i == target_rail:
            entity = target_sc_entity
        elif include_distractor_sc:
            entity = distractor_sc_entity
        else:
            return {"entity_present": False}
        return {
            "entity_present": True,
            "entity_name": entity,
            "entity_pose": {
                "translation": rand_trans(*SC_RAIL_TRANS),
                "roll": 0.0, "pitch": 0.0,
                "yaw": round(rng.uniform(-SC_RAIL_YAW_JITTER, SC_RAIL_YAW_JITTER), 4),
            },
        }

    def nic_rail_entry(i: int) -> dict:
        if i not in nic_rails:
            return {"entity_present": False}
        return {
            "entity_present": True,
            "entity_name": nic_entity_pool[nic_rails.index(i)],
            "entity_pose": {
                "translation": rand_trans(*NIC_RAIL_TRANS),
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            },
        }

    return {
        "scene": {
            "task_board": {
                "pose": board_pose,
                **{f"nic_rail_{i}": nic_rail_entry(i) for i in range(5)},
                **{f"sc_rail_{i}":  sc_rail_entry(i)  for i in range(2)},
                "lc_mount_rail_0":  {"entity_present": False},
                "sfp_mount_rail_0": {"entity_present": False},
                "sc_mount_rail_0":  {"entity_present": False},
                "lc_mount_rail_1":  {"entity_present": False},
                "sfp_mount_rail_1": {"entity_present": False},
                "sc_mount_rail_1":  {"entity_present": False},
            },
            "cables": {
                "cable_1": {
                    "pose": {
                        "gripper_offset": {"x": grasp["x"], "y": grasp["y"], "z": grasp["z"]},
                        "roll": grasp["roll"],
                        "pitch": grasp["pitch"],
                        "yaw": grasp["yaw"],
                    },
                    "attach_cable_to_gripper": True,
                    "cable_type": "sfp_sc_cable_reversed",
                },
            },
        },
        "tasks": {
            "task_1": {
                "cable_type": "sfp_sc",
                "cable_name": "cable_1",
                "plug_type": "sc",
                "plug_name": "sc_tip",
                "port_type": "sc",
                "port_name": "sc_port_base",
                "target_module_name": target_module,
                "time_limit": 180,
            },
        },
    }


def dict_to_yaml(d, indent=0) -> str:
    """Minimal YAML serialiser (no PyYAML dependency needed here)."""
    lines = []
    pad = "  " * indent
    for k, v in d.items():
        if isinstance(v, dict):
            lines.append(f"{pad}{k}:")
            lines.append(dict_to_yaml(v, indent + 1))
        elif isinstance(v, bool):
            lines.append(f"{pad}{k}: {str(v)}")
        elif isinstance(v, str):
            lines.append(f'{pad}{k}: "{v}"')
        else:
            lines.append(f"{pad}{k}: {v}")
    return "\n".join(lines)


def generate_config(n_trials: int, seed: int, task_type: str, stratify: bool) -> str:
    rng = random.Random(seed)

    # Build a stratified diversity plan for SC trials (if applicable).
    if stratify and task_type in ("sc", "mixed"):
        # For mixed we still build a plan, but only consume entries when an SC trial fires.
        sc_plan = build_sc_diversity_plan(n_trials, rng)
    else:
        sc_plan = None

    # Build scoring topics header (copied from 10_trials.yaml)
    header = textwrap.dedent("""\
        # Auto-generated CheatCode data collection config
        # Trials are randomly sampled combinations of task type, board pose, and NIC/SC placement.
        scoring:
          topics:
            - topic:
                name: "/joint_states"
                type: "sensor_msgs/msg/JointState"
            - topic:
                name: "/left_camera/image_small"
                type: "sensor_msgs/msg/CompressedImage"
            - topic:
                name: "/center_camera/image_small"
                type: "sensor_msgs/msg/CompressedImage"
            - topic:
                name: "/right_camera/image_small"
                type: "sensor_msgs/msg/CompressedImage"
            - topic:
                name: "/tf"
                type: "tf2_msgs/msg/TFMessage"
            - topic:
                name: "/tf_static"
                type: "tf2_msgs/msg/TFMessage"
                latched: true
            - topic:
                name: "/scoring/tf"
                type: "tf2_msgs/msg/TFMessage"
            - topic:
                name: "/aic/gazebo/contacts/off_limit"
                type: "ros_gz_interfaces/msg/Contacts"
            - topic:
                name: "/fts_broadcaster/wrench"
                type: "geometry_msgs/msg/WrenchStamped"
            - topic:
                name: "/aic_controller/joint_commands"
                type: "aic_control_interfaces/msg/JointMotionUpdate"
            - topic:
                name: "/aic_controller/pose_commands"
                type: "aic_control_interfaces/msg/MotionUpdate"
            - topic:
                name: "/scoring/insertion_event"
                type: "std_msgs/msg/String"
            - topic:
                name: "/aic_controller/controller_state"
                type: "aic_control_interfaces/msg/ControllerState"

        task_board_limits:
          nic_rail:
            min_translation: -0.0215
            max_translation: 0.0234
          sc_rail:
            min_translation: -0.06
            max_translation: 0.055
          mount_rail:
            min_translation: -0.09425
            max_translation: 0.09425

        trials:
    """)

    trial_lines = []
    sc_plan_iter = iter(sc_plan) if sc_plan is not None else None
    for i in range(n_trials):
        trial_name = f"trial_{i + 1}"
        if task_type == "sc":
            plan_entry = next(sc_plan_iter) if sc_plan_iter is not None else None
            trial = build_sc_trial(i, rng, plan=plan_entry)
        elif task_type == "sfp":
            trial = build_sfp_trial(i, rng)
        else:  # "mixed"
            if rng.random() < 0.67:
                trial = build_sfp_trial(i, rng)
            else:
                plan_entry = next(sc_plan_iter, None) if sc_plan_iter is not None else None
                trial = build_sc_trial(i, rng, plan=plan_entry)
        trial_lines.append(f"  {trial_name}:")
        trial_lines.append(dict_to_yaml(trial, indent=2))
        trial_lines.append("")

    # Robot home pose
    robot_section = textwrap.dedent("""\
        robot:
          home_joint_positions:
            shoulder_pan_joint: -0.1597
            shoulder_lift_joint: -1.3542
            elbow_joint: -1.6648
            wrist_1_joint: -1.6933
            wrist_2_joint: 1.5710
            wrist_3_joint: 1.4110
    """)

    return header + "\n".join(trial_lines) + "\n" + robot_section


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=50,
                        help="Number of trials to generate (default: 50)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for reproducibility (default: 42)")
    parser.add_argument("--task-type", type=str, default="mixed",
                        choices=["sc", "sfp", "mixed"],
                        help="sc = Trial 3 only, sfp = Trials 1-2 only, mixed = original 2:1 SFP:SC")
    parser.add_argument("--stratify", action="store_true", default=True,
                        help="Stratified sampling over discrete dims (default: on). "
                             "Use --no-stratify to disable.")
    parser.add_argument("--no-stratify", dest="stratify", action="store_false",
                        help="Disable stratified sampling; use iid uniform draws.")
    parser.add_argument("--output", type=str, default="/tmp/collection_config.yaml",
                        help="Output YAML path")
    args = parser.parse_args()

    config_str = generate_config(args.trials, args.seed, args.task_type, args.stratify)
    Path(args.output).write_text(config_str)
    print(f"Generated {args.trials} {args.task_type} trials → {args.output}")
    print(f"  stratification: {'on' if args.stratify else 'off'}, seed={args.seed}")


if __name__ == "__main__":
    main()