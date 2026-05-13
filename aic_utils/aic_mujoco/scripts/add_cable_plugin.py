#!/usr/bin/env python3

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

"""Split aic_world.xml into robot and world XMLs with cable plugin.

Uses pure mjSpec API for all model manipulation including reparenting.
"""

import argparse
import os
import re
import sys
import traceback
import numpy as np
import mujoco


# --- Robot XML Post-Processing ---
def postprocess_robot_xml(xml_str):
    """Apply automated corrections to robot XML (replaces manual edits)."""

    # 1. Add visual settings: <global> and <map> around <headlight>
    xml_str = xml_str.replace(
        '<headlight ambient="0 0 0" diffuse="0 0 0" specular="0 0 0"/>',
        '<global offwidth="1152" offheight="1024"/>\n'
        '    <headlight ambient="0 0 0" diffuse="0 0 0" specular="0 0 0"/>\n'
        '    <map znear="0.0025"/>',
    )

    # 2. Replace backslashes with forward slashes in all attribute values
    xml_str = xml_str.replace("\\", "/")

    # 3. Fix robot body quaternions to clean values
    #    tabletop: near-(0,0,0,-1) → identity
    xml_str = re.sub(
        r'(<body name="tabletop" pos="[^"]*") quat="[^"]*"',
        r'\1 quat="1 0 0 0"',
        xml_str,
    )
    #    shoulder_link: near-identity → remove quat
    xml_str = re.sub(
        r'(<body name="shoulder_link" pos="[^"]*") quat="[^"]*"',
        r"\1",
        xml_str,
    )
    #    upper_arm_link: normalize to exact 90° rotation
    xml_str = re.sub(
        r'(<body name="upper_arm_link" pos="[^"]*") quat="[^"]*"',
        r'\1 quat="0.70710678 0.70710678 0 0"',
        xml_str,
    )
    #    forearm_link: near-Z-rotation → remove quat
    xml_str = re.sub(
        r'(<body name="forearm_link" pos="[^"]*") quat="[^"]*"',
        r"\1",
        xml_str,
    )
    #    wrist_1_link: near-Z-rotation → remove quat
    xml_str = re.sub(
        r'(<body name="wrist_1_link" pos="[^"]*") quat="[^"]*"',
        r"\1",
        xml_str,
    )
    #    wrist_2_link: normalize to exact 90° rotation
    xml_str = re.sub(
        r'(<body name="wrist_2_link" pos="[^"]*") quat="[^"]*"',
        r'\1 quat="0.70710678 0.70710678 0 0"',
        xml_str,
    )
    #    wrist_3_link: normalize to exact 90° rotation
    xml_str = re.sub(
        r'(<body name="wrist_3_link" pos="[^"]*") quat="[^"]*"',
        r'\1 quat="0.70710678 -0.70710678 0 0"',
        xml_str,
    )

    # 4. Add camera attributes (orientation, fov, resolution) to all cameras
    for cam_name in ["center_camera", "left_camera", "right_camera"]:
        xml_str = re.sub(
            rf'<camera name="{cam_name}" class="robot_unused" pos="0 0 0"/>',
            f'<camera name="{cam_name}" class="robot_unused" pos="0 0 0"'
            f' quat="0.5 0.5 -0.5 -0.5" fovy="44.982" resolution="1152 1024"/>',
            xml_str,
        )

    # 5. Add gripper_tcp site before the left finger body
    xml_str = re.sub(
        r'([ \t]*)(<body name="gripper/hande_finger_link_l")',
        r'\1<site name="gripper_tcp" pos="0 0 0.172"/>\n\1\2',
        xml_str,
    )

    # 6. Zero out gripper finger body positions
    xml_str = re.sub(
        r'<body name="gripper/hande_finger_link_l" pos="[^"]*"',
        '<body name="gripper/hande_finger_link_l" pos="0 0 0"',
        xml_str,
    )
    xml_str = re.sub(
        r'<body name="gripper/hande_finger_link_r" pos="[^"]*"',
        '<body name="gripper/hande_finger_link_r" pos="0 0 0"',
        xml_str,
    )

    # 7. Remove the right finger motor actuator line
    xml_str = re.sub(
        r'\n\s*<general name="gripper/right_finger_joint_motor"[^>]*/>', "", xml_str
    )

    # 8. Add armature and light damping to arm joints.
    #    armature=0.1 matches the official MuJoCo UR5e model — provides numerical
    #    stability for implicitfast without adding perceptible inertia.
    #    damping=1.0 adds minimal viscous friction to approximate Bullet-
    #    Featherstone's implicit dissipation without making teleop sluggish.
    arm_joints = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    for joint_name in arm_joints:
        xml_str = re.sub(
            rf'(<joint name="{joint_name}"[^/]*?)(/\s*>)',
            rf'\1 damping="1.0" armature="0.1"\2',
            xml_str,
        )

    # 9. Add equality constraint (mimic joint) and sensor sections
    # Note: </mujoco> has 2 leading spaces in the raw XML, so content
    # placed before it inherits that indent; first line needs no indent.
    equality_and_sensor = (
        "<equality>\n"
        '    <joint joint1="gripper/left_finger_joint"'
        ' joint2="gripper/right_finger_joint"'
        ' polycoef="0 1 0 0 0"/>\n'
        "  </equality>\n"
        "\n"
        "  <sensor>\n"
        '    <force name="AtiForceTorqueSensor_force"'
        ' site="AtiForceTorqueSensor"/>\n'
        '    <torque name="AtiForceTorqueSensor_torque"'
        ' site="AtiForceTorqueSensor"/>\n'
        "  </sensor>\n"
    )
    xml_str = xml_str.replace("</mujoco>", equality_and_sensor + "\n  </mujoco>")

    return xml_str


# --- World XML Post-Processing ---
def postprocess_world_xml(
    xml_str,
    gripper_plug_name="lc_plug_link",
    weld_relpose=None,
    cable_end_pos=None,
    cable_end_quat=None,
):
    """Apply automated corrections to world XML (replaces manual edits)."""

    # 1. Update cable_end_0 pose to tuned values
    if cable_end_pos is None:
        cable_end_pos = "0.171400 0.020606 1.511889"
    if cable_end_quat is None:
        cable_end_quat = "0.712741 0.312130 -0.052173 0.625982"
    xml_str = re.sub(
        r'(<body name="cable_end_0" childclass="cable_default") pos="[^"]*" quat="[^"]*"',
        rf'\1 pos="{cable_end_pos}" quat="{cable_end_quat}"',
        xml_str,
    )

    # 2. Replace <joint name="freejoint" type="free"/> with
    #    <freejoint name="cable_end_0_free"/> and reorder before inertial
    xml_str = re.sub(
        r'([ \t]*)(<inertial[^/]*/>) *\n([ \t]*)<joint name="freejoint" type="free"/>',
        r'\1<freejoint name="cable_end_0_free"/>\n\1\2',
        xml_str,
    )

    # 3. Replace all cable body diaginertia to 1e-6
    #    Normal cable raw export uses 0.01; reversed cable uses 0.001
    for old_diag in ("0.01 0.01 0.01", "0.001 0.001 0.001"):
        xml_str = xml_str.replace(
            f'diaginertia="{old_diag}"', 'diaginertia="1e-6 1e-6 1e-6"'
        )

    # 4. Fix cable_connection_1 plug-end diaginertia to 4e-4
    #    cable_connection_1 has mass=0.01 and holds SC plug end (normal) or LC plug end (reversed)
    xml_str = re.sub(
        r'(<body name="cable_connection_1"[^>]*>\s*'
        r'<inertial pos="0 0 0" mass="0.01") diaginertia="0.01 0.01 0.01"',
        r'\1 diaginertia="4e-4 4e-4 4e-4"',
        xml_str,
    )

    # 4. Add damping to joint_connection_end_0 ball joint
    xml_str = re.sub(
        r'(<joint name="joint_connection_end_0"[^/]*type="ball")(/>)',
        r'\1 damping="0.2"\2',
        xml_str,
    )

    # 6. Add equality weld constraint for gripper-end plug attachment
    if weld_relpose is None:
        weld_relpose = (
            "-0.000711 0.001759 0.168213" " 0.577301 0.816105 -0.021418 -0.015395"
        )
    weld_section = (
        "\n"
        "  <equality>\n"
        f'    <weld body1="ati/tool_link" body2="{gripper_plug_name}"'
        f' relpose="{weld_relpose}"'
        ' solref="0.002 1" solimp="0.99 0.999 0.001"/>\n'
        "  </equality>\n"
    )
    xml_str = xml_str.replace("</mujoco>", weld_section + "</mujoco>")

    # 6. Apply friction childclass to SC Port and NIC Card Mount bodies
    #    (matching Gazebo SDF friction parameters)
    xml_str = re.sub(
        r'(<body name="sc_port_\d+::sc_port_link")',
        r'\1 childclass="sc_port_default"',
        xml_str,
    )
    xml_str = re.sub(
        r'(<body name="nic_card_mount_\d+::nic_card_mount_link")',
        r'\1 childclass="nic_card_default"',
        xml_str,
    )

    return xml_str


def main():
    if "BUILD_WORKSPACE_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKSPACE_DIRECTORY"])

    parser = argparse.ArgumentParser(
        description="Split aic_world.xml into robot and world with plugin."
    )
    parser.add_argument("--input", default="aic_world.xml", help="Input XML file")
    parser.add_argument(
        "--output", default="aic_world_final.xml", help="Output World XML file"
    )
    parser.add_argument("--robot_output", default=None, help="Output Robot XML file")
    parser.add_argument("--scene_output", default=None, help="Output Scene XML file")
    args = parser.parse_args()

    try:
        # Resolve absolute paths
        input_path = os.path.abspath(args.input)
        output_path = os.path.abspath(args.output)
        output_dir = os.path.dirname(output_path)

        if args.robot_output:
            robot_output_path = os.path.abspath(args.robot_output)
        else:
            robot_output_path = os.path.join(output_dir, "aic_robot.xml")

        if args.scene_output:
            scene_output_path = os.path.abspath(args.scene_output)
        else:
            scene_output_path = os.path.join(output_dir, "scene.xml")

        # Change to input directory (so MuJoCo finds assets in CWD)
        base_dir = os.path.dirname(input_path)
        if base_dir:
            os.chdir(base_dir)
            print(f"Changed directory to {base_dir}")

        print(f"Loading model from {input_path}...")

        # --- Asset Partitioning Helpers ---
        robot_keywords = [
            "tabletop",
            "shoulder",
            "upper_arm",
            "upperarm",
            "forearm",
            "wrist",
            "ati",
            "gripper",
            "hande",
            "cam_mount",
            "axia",
            "basler",
            "base-",
            "base1",
            "_base.stl",
            "base_eSeries",
        ]
        env_keywords = [
            "task_board",
            "sc_port",
            "nic_card",
            "plug",
            "sfp_module",
            "enclosure",
            "floor",
            "walls",
            "light",
        ]

        def is_robot_asset(name):
            for ek in env_keywords:
                if ek in name:
                    return False
            for rk in robot_keywords:
                if rk in name:
                    return True
            return False

        # --- String helpers for class renaming ---
        def rename_class(xml_str, old_name, new_name):
            xml_str = re.sub(
                rf'<default\s+class="{old_name}"',
                f'<default class="{new_name}"',
                xml_str,
            )
            xml_str = re.sub(
                rf'\bclass="{old_name}"',
                f'class="{new_name}"',
                xml_str,
            )
            xml_str = re.sub(
                rf'\bchildclass="{old_name}"',
                f'childclass="{new_name}"',
                xml_str,
            )
            return xml_str

        def strip_tag(xml_str, tag):
            pattern = rf"<{tag}[^>]*/>\s*"
            xml_str = re.sub(pattern, "", xml_str)
            pattern = rf"<{tag}[^>]*>.*?</{tag}>\s*"
            xml_str = re.sub(pattern, "", xml_str, flags=re.DOTALL)
            return xml_str

        # --- 1. Create Robot XML ---
        print("Creating robot spec...")
        robot_spec = mujoco.MjSpec.from_file(input_path)

        # Remove everything from worldbody EXCEPT tabletop
        print("Pruning non-robot bodies...")
        bodies_to_delete = []
        for b in robot_spec.worldbody.bodies:
            if b.name != "tabletop":
                bodies_to_delete.append(b)

        for b in bodies_to_delete:
            robot_spec.delete(b)

        # Clean Robot Spec: Delete ENV assets
        print("Cleaning env assets from robot spec...")
        if hasattr(robot_spec, "meshes"):
            for m in list(robot_spec.meshes):
                if not is_robot_asset(m.name):
                    robot_spec.delete(m)
        if hasattr(robot_spec, "materials"):
            for m in list(robot_spec.materials):
                if not is_robot_asset(m.name):
                    robot_spec.delete(m)
        if hasattr(robot_spec, "textures"):
            for t in list(robot_spec.textures):
                if not is_robot_asset(t.name):
                    robot_spec.delete(t)

        # Add motor actuators for robot joints
        print("Adding motor actuators for robot joints...")
        robot_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "gripper\\left_finger_joint",
            "gripper\\right_finger_joint",
        ]
        # Note: right finger motor is removed in postprocess_robot_xml;
        # it is coupled via an equality constraint instead.
        for jname in robot_joints:
            act = robot_spec.add_actuator()
            act.name = f"{jname}_motor"
            act.target = jname
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.dyntype = mujoco.mjtDyn.mjDYN_NONE
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.biastype = mujoco.mjtBias.mjBIAS_NONE
            act.gear = [1, 0, 0, 0, 0, 0]
            print(f"  Added actuator: {act.name}")

        # Add robot contact exclusions
        print("Adding robot contact exclusions...")
        robot_spec.add_exclude(bodyname1="tabletop", bodyname2="shoulder_link")
        robot_spec.add_exclude(
            bodyname1="gripper\\hande_finger_link_l",
            bodyname2="gripper\\hande_finger_link_r",
        )
        print("  Added tabletop <-> shoulder_link exclusion")
        print("  Added gripper finger exclusion")

        # Serialize and clean
        robot_xml = robot_spec.to_xml()
        tags_to_strip = ["extension", "custom", "sensor"]  # Keep contact!
        for tag in tags_to_strip:
            robot_xml = strip_tag(robot_xml, tag)
        # Strip lights from robot (lights live in world XML)
        robot_xml = re.sub(r"<light[^>]*/>\s*", "", robot_xml)
        robot_xml = rename_class(robot_xml, "unused", "robot_unused")

        # Apply automated post-processing (replaces manual edits)
        print("Post-processing robot XML...")
        robot_xml = postprocess_robot_xml(robot_xml)

        print(f"Saving robot XML to {robot_output_path}...")
        with open(robot_output_path, "w") as f:
            f.write(robot_xml)

        # --- 2. Create World XML using full mjSpec decomposition ---
        print("Creating world spec via full mjSpec decomposition...")

        # Load kinematics for pose calculations
        print("Running kinematics for pose computation...")
        orig_model = mujoco.MjModel.from_xml_path(input_path)
        orig_data = mujoco.MjData(orig_model)
        mujoco.mj_kinematics(orig_model, orig_data)

        # Detect cable type by checking which plug is at cable_connection_0
        id_conn0 = mujoco.mj_name2id(
            orig_model, mujoco.mjtObj.mjOBJ_BODY, "cable_connection_0"
        )  # pytype: disable=wrong-arg-types
        id_sc_plug = mujoco.mj_name2id(
            orig_model, mujoco.mjtObj.mjOBJ_BODY, "sc_plug_link"
        )  # pytype: disable=wrong-arg-types

        is_reversed = (
            id_sc_plug != -1
            and id_conn0 != -1
            and orig_model.body_parentid[id_sc_plug] == id_conn0
        )

        if is_reversed:
            gripper_plug_name = "sc_plug_link"
            # Tuned values for reversed cable (sc_plug at gripper end)
            cable_end_pos = "0.172124 0.029369 1.507828"
            cable_end_quat = "0.713143 0.312161 -0.049352 0.625737"
            weld_relpose = (
                "-0.000980 0.000693 0.180020" " 0.170140 -0.684594 0.157796 -0.691002"
            )
            print("Detected reversed cable (sc_plug at cable_connection_0).")
        else:
            gripper_plug_name = "lc_plug_link"
            weld_relpose = None  # Use default hardcoded value
            cable_end_pos = None  # Use default hardcoded value
            cable_end_quat = None
            print("Detected normal cable orientation.")

        # Get body IDs for relative pose calculation
        id_l1 = mujoco.mj_name2id(
            orig_model, mujoco.mjtObj.mjOBJ_BODY, "link_1"
        )  # pytype: disable=wrong-arg-types
        id_c0 = id_conn0

        # Compute relative pose of link_1 w.r.t. cable_connection_0
        rel_pos = None
        rel_quat = None
        if id_l1 != -1 and id_c0 != -1:
            pos_l1 = orig_data.xpos[id_l1].copy()
            quat_l1 = orig_data.xquat[id_l1].copy()
            pos_c0 = orig_data.xpos[id_c0].copy()
            quat_c0 = orig_data.xquat[id_c0].copy()

            quat_c0_inv = np.zeros(4)
            mujoco.mju_negQuat(quat_c0_inv, quat_c0)

            diff_pos = pos_l1 - pos_c0
            rel_pos = np.zeros(3)
            mujoco.mju_rotVecQuat(rel_pos, diff_pos, quat_c0_inv)

            rel_quat = np.zeros(4)
            mujoco.mju_mulQuat(rel_quat, quat_c0_inv, quat_l1)
            print(f"Computed link_1 relative pose: pos={rel_pos}, quat={rel_quat}")

        # Load source spec
        source_spec = mujoco.MjSpec.from_file(input_path)

        # Helper to recursively copy a body and its children
        def copy_body_recursive(
            src_body, dest_parent, skip_bodies=None, reparent_map=None
        ):
            """Recursively copy src_body to dest_parent, applying reparent_map.

            Args:
              src_body: Source MjsBody to copy
              dest_parent: Destination parent (MjsBody or MjsWorldbody)
              skip_bodies: Set of body names to skip entirely
              reparent_map: Dict mapping body_name -> (new_parent_name, new_pos, new_quat)
                            Bodies in this map are NOT copied at their original location.
            Returns:
              The copied body or None if skipped
            """
            skip_bodies = skip_bodies or set()
            reparent_map = reparent_map or {}

            if src_body.name in skip_bodies:
                return None

            # If this body should be reparented, skip it here (will be added later)
            if src_body.name in reparent_map:
                print(f"  Deferring {src_body.name} for reparenting")
                return None

            # Create new body in dest
            new_body = dest_parent.add_body()
            new_body.name = src_body.name
            new_body.pos = list(src_body.pos)
            new_body.quat = list(src_body.quat)
            new_body.mass = src_body.mass
            new_body.inertia = list(src_body.inertia)
            new_body.ipos = list(src_body.ipos)
            new_body.iquat = list(src_body.iquat)
            new_body.gravcomp = src_body.gravcomp
            new_body.mocap = src_body.mocap

            # Copy joints
            for joint in src_body.joints:
                new_joint = new_body.add_joint()
                new_joint.name = joint.name
                new_joint.type = joint.type
                new_joint.pos = list(joint.pos)
                new_joint.axis = list(joint.axis)
                new_joint.range = list(joint.range)
                new_joint.limited = joint.limited
                new_joint.stiffness = joint.stiffness
                new_joint.damping = joint.damping

            # Copy geoms
            for geom in src_body.geoms:
                new_geom = new_body.add_geom()
                new_geom.name = geom.name
                new_geom.type = geom.type
                new_geom.pos = list(geom.pos)
                new_geom.quat = list(geom.quat)
                new_geom.size = list(geom.size)
                new_geom.rgba = list(geom.rgba)
                new_geom.mesh = geom.mesh
                new_geom.material = geom.material
                new_geom.contype = geom.contype
                new_geom.conaffinity = geom.conaffinity

            # Copy sites
            for site in src_body.sites:
                new_site = new_body.add_site()
                new_site.name = site.name
                new_site.pos = list(site.pos)
                new_site.quat = list(site.quat)
                new_site.type = site.type
                new_site.size = list(site.size)

            # Recursively copy children
            for child in src_body.bodies:
                copy_body_recursive(child, new_body, skip_bodies, reparent_map)

            # After copying this body's regular children, check if any deferred
            # bodies should be added here
            for defer_name, (
                target_parent,
                defer_pos,
                defer_quat,
            ) in reparent_map.items():
                if target_parent == src_body.name:
                    # Find the deferred body in source
                    defer_body = find_body(source_spec.worldbody, defer_name)
                    if defer_body:
                        print(
                            f"  Adding deferred body {defer_name} under {src_body.name}"
                        )
                        # Copy with modified pose
                        copy_deferred_body(defer_body, new_body, defer_pos, defer_quat)

            return new_body

        def find_body(parent, name):
            """Find a body by name in the hierarchy."""
            for b in parent.bodies:
                if b.name == name:
                    return b
                found = find_body(b, name)
                if found:
                    return found
            return None

        def copy_deferred_body(src_body, dest_parent, new_pos, new_quat):
            """Copy a body with overridden pose and all its children."""
            new_body = dest_parent.add_body()
            new_body.name = src_body.name
            new_body.pos = list(new_pos) if new_pos is not None else list(src_body.pos)
            new_body.quat = (
                list(new_quat) if new_quat is not None else list(src_body.quat)
            )
            new_body.mass = src_body.mass
            new_body.inertia = list(src_body.inertia)
            new_body.ipos = list(src_body.ipos)
            new_body.iquat = list(src_body.iquat)
            new_body.gravcomp = src_body.gravcomp
            new_body.mocap = src_body.mocap

            for joint in src_body.joints:
                new_joint = new_body.add_joint()
                new_joint.name = joint.name
                new_joint.type = joint.type
                new_joint.pos = list(joint.pos)
                new_joint.axis = list(joint.axis)
                new_joint.range = list(joint.range)
                new_joint.limited = joint.limited
                new_joint.stiffness = joint.stiffness
                new_joint.damping = joint.damping

            for geom in src_body.geoms:
                new_geom = new_body.add_geom()
                new_geom.name = geom.name
                new_geom.type = geom.type
                new_geom.pos = list(geom.pos)
                new_geom.quat = list(geom.quat)
                new_geom.size = list(geom.size)
                new_geom.rgba = list(geom.rgba)
                new_geom.mesh = geom.mesh
                new_geom.material = geom.material
                new_geom.contype = geom.contype
                new_geom.conaffinity = geom.conaffinity

            for site in src_body.sites:
                new_site = new_body.add_site()
                new_site.name = site.name
                new_site.pos = list(site.pos)
                new_site.quat = list(site.quat)
                new_site.type = site.type
                new_site.size = list(site.size)

            # Copy children directly (no reparenting)
            for child in src_body.bodies:
                copy_deferred_body(child, new_body, None, None)

            return new_body

        # Create world spec from scratch - but this is complex (compiler settings, etc.)
        # Alternative: Load source, delete tabletop, then do restructuring via delete+add
        # Actually the simplest approach: Just load source, perform restructuring

        # Load world spec from source
        world_spec = mujoco.MjSpec.from_file(input_path)

        # Remove tabletop
        print("Removing tabletop from world spec...")
        for b in list(world_spec.worldbody.bodies):
            if b.name == "tabletop":
                world_spec.delete(b)
                break

        # Clean robot assets
        print("Cleaning robot assets from world spec...")
        for m in list(world_spec.meshes):
            if is_robot_asset(m.name):
                world_spec.delete(m)
        for m in list(world_spec.materials):
            if is_robot_asset(m.name):
                world_spec.delete(m)
        for t in list(world_spec.textures):
            if is_robot_asset(t.name):
                world_spec.delete(t)

        # --- Reparenting link_1 via XML manipulation since mjSpec can't do it ---
        # Generate XML, perform reparent, reload
        print("Generating intermediate XML for reparenting...")
        temp_xml = world_spec.to_xml()

        # Use string manipulation to reparent link_1 under cable_connection_0
        # This is the one operation that mjSpec can't handle natively
        import xml.etree.ElementTree as ET

        root = ET.fromstring(temp_xml)

        # Find bodies
        cable_end = None
        conn_0 = None
        link_1 = None
        for b in root.iter("body"):
            if b.get("name") == "cable_end_0":
                cable_end = b
            if b.get("name") == "cable_connection_0":
                conn_0 = b
            if b.get("name") == "link_1":
                link_1 = b

        if cable_end is not None and conn_0 is not None and link_1 is not None:
            # Lift cable_end_0 by 5cm
            pos_str = cable_end.get("pos")
            if pos_str:
                pos_vals = [float(x) for x in pos_str.split()]
                pos_vals[2] += 0.05
                cable_end.set("pos", " ".join(f"{x:.15g}" for x in pos_vals))
                print(f"Lifted cable_end_0 by 5cm")

            # Update link_1 pose and move it under conn_0
            if rel_pos is not None and rel_quat is not None:
                link_1.set("pos", " ".join(f"{x:.15g}" for x in rel_pos))
                link_1.set("quat", " ".join(f"{x:.15g}" for x in rel_quat))
                # Remove any conflicting orientation attrs
                for attr in ["euler", "axis", "angle", "xyaxes", "zaxis"]:
                    if link_1.get(attr):
                        del link_1.attrib[attr]

            # Move link_1 from cable_end to conn_0 as FIRST child
            cable_end.remove(link_1)
            conn_0.insert(0, link_1)
            print("Reparented link_1 under cable_connection_0")

        reparented_xml = ET.tostring(root, encoding="unicode")

        # Reload into mjSpec
        print("Reloading reparented XML into mjSpec...")
        world_spec = mujoco.MjSpec.from_string(reparented_xml)

        # Add contact exclusions
        # Dynamically find sc_port link
        sc_port_link_name = None
        # Try finding an sc_port link with various indices
        for i in range(10):
            candidate = f"sc_port_{i}::sc_port_link"
            if find_body(world_spec.worldbody, candidate):
                sc_port_link_name = candidate
                break

            if sc_port_link_name:
                world_spec.add_exclude(
                    bodyname1=sc_port_link_name, bodyname2="sc_plug_link"
                )
                print(f"Added exclusion: {sc_port_link_name} <-> sc_plug_link")
            else:
                print("Warning: Could not find sc_port link for exclusion.")
        world_spec.add_exclude(bodyname1="cable_end_0", bodyname2="link_1")
        print("Added exclusion: cable_end_0 <-> link_1")

        # Activate plugin
        print("Activating cable plugin...")
        try:
            world_spec.activate_plugin("mujoco.elasticity.cable")
        except ValueError:
            pass  # Already activated

        print("Adding plugin instance...")
        plugin = world_spec.add_plugin(
            name="composite",
            plugin_name="mujoco.elasticity.cable",
            active=True,
            info="Cable composite plugin",
        )
        plugin.config = {"twist": "1e2", "bend": "4e1"}

        # Add cable_default
        root_default = world_spec.default
        cable_default = world_spec.add_default("cable_default", root_default)
        cable_default.joint.damping = 0.2
        print("Added 'cable_default' with joint damping 0.2.")

        # Add friction defaults for task board components (matching Gazebo SDF)
        sc_port_default = world_spec.add_default("sc_port_default", root_default)
        sc_port_default.geom.friction = [0.5, 0.5, 0.0001]
        print("Added 'sc_port_default' with friction [0.5, 0.5, 0.0001].")

        nic_card_default = world_spec.add_default("nic_card_default", root_default)
        nic_card_default.geom.friction = [0.1, 0.005, 0.1]
        print("Added 'nic_card_default' with friction [0.1, 0.005, 0.1].")

        # Set plugin and childclass on cable bodies
        print("Setting plugin on cable bodies...")

        def traverse_find_links(body, target_plugin):
            count = 0

            # Skip plug links — they are rigid attachments, not part of the
            # elastic cable chain.  Including them would create a branch and
            # violate the cable plugin's single-chain requirement
            if body.name in ("lc_plug_link", "sc_plug_link"):
                return 0

            is_cable_body = body.name.startswith("cable_end_") or body.name.startswith(
                "cable_connection_"
            )
            if body.name.startswith("link_"):
                try:
                    idx = int(body.name.split("_")[1])
                    if 1 <= idx <= 20:
                        is_cable_body = True
                except ValueError:
                    pass

            if is_cable_body:
                body.plugin = target_plugin
                body.plugin.active = True
                body.plugin.name = target_plugin.name
                body.childclass = "cable_default"
                count += 1

            if hasattr(body, "bodies"):
                for child in body.bodies:
                    count += traverse_find_links(child, target_plugin)
            return count

        bodies_found = traverse_find_links(world_spec.worldbody, plugin)
        print(f"Attached plugin to {bodies_found} bodies.")

        # --- Generate and Save ---
        print("Generating World XML...")
        xml_str = world_spec.to_xml()
        xml_str = rename_class(xml_str, "unused", "world_default")

        def strip_class_from_cable_children(xml):
            for i in range(1, 21):
                for elem_type in ["joint", "geom"]:
                    patterns = [
                        f'name="{elem_type}_{i}"',
                        f'name="{elem_type}_end_0"',
                        f'name="link_{i}_collision"',
                        f'name="link_{i}_vis"',
                        f'name="sphere_{i}_collision"',
                        f'name="sphere_{i}_vis"',
                    ]
                    for pat in patterns:
                        xml = xml.replace(f'{pat} class="world_default"', pat)
            return xml

        xml_str = strip_class_from_cable_children(xml_str)

        # Apply automated post-processing (replaces manual edits)
        print("Post-processing world XML...")
        xml_str = postprocess_world_xml(
            xml_str, gripper_plug_name, weld_relpose, cable_end_pos, cable_end_quat
        )

        print(f"Saving world XML to {output_path}...")
        with open(output_path, "w") as f:
            f.write(xml_str)

        # --- Generate Scene XML ---
        print(f"Generating Scene XML to {scene_output_path}...")
        scene_dir = os.path.dirname(scene_output_path)
        rel_robot = os.path.relpath(robot_output_path, scene_dir)
        rel_world = os.path.relpath(output_path, scene_dir)

        scene_xml = f"""<mujoco model="Scene">
  <!--
    Physics tuning for Gazebo (Bullet-Featherstone) behavioral match:
    - integrator="implicitfast": Implicit Coriolis/centrifugal treatment better
      matches Bullet-Featherstone's articulated body algorithm which also handles
      these forces implicitly.
    - solver="Newton": High-accuracy constraint solver.
    - iterations/tolerance: Tight solver for accurate force resolution.
    - timestep="0.002": Matches Gazebo's 2ms step (500 Hz physics).
  -->
  <option integrator="implicitfast" timestep="0.002" solver="Newton" iterations="200" tolerance="1e-10"/>
  <include file="{rel_robot}"/>
  <include file="{rel_world}"/>
</mujoco>"""

        with open(scene_output_path, "w") as f:
            f.write(scene_xml)

        print("Done.")

    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
