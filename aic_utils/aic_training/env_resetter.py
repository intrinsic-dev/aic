#!/usr/bin/env python3
"""
Environment reset node for RL training.

This node runs INSIDE the distrobox/docker container where simulation_interfaces
is available. It provides a /env/reset service (std_srvs/SetBool) that performs
the full reset sequence:

  data=False (deterministic):
    1. Delete cable entity from Gazebo
    2. Delete task board (only if delete_task_board param is True)
    3. Deactivate aic_controller
    4. Reset robot joints to home positions
    5. Reactivate aic_controller
    6. Tare force/torque sensor
    7. Re-spawn cable at fixed pose (from ROS2 params)
    8. Wait for robot joints to stabilize

  data=True (random, requires config_path param):
    1. Delete cable entity from Gazebo
    2. Delete task board
    3. Deactivate aic_controller
    4. Reset robot joints to home positions
    5. Reactivate aic_controller
    6. Tare force/torque sensor
    7. Re-spawn task board with randomly sampled trial scene
       (NIC rails, SC rails, mount rails, board pose)
    8. Re-spawn cable with randomly sampled trial cable config
    9. Wait for robot joints to stabilize

Usage (inside distrobox):
    source /ws_aic/install/setup.bash
    python3 env_resetter.py [--ros-args -p use_sim_time:=true]
"""

import copy
from collections import deque
import json
import math
import random
import subprocess
import sys
import time
from threading import Event

import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from aic_engine_interfaces.srv import ResetJoints
from controller_manager_msgs.srv import SwitchController
from simulation_interfaces.srv import SpawnEntity, DeleteEntity
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import tf2_ros


# ---- Default Configuration ----

# Home joint positions (from ur_gz.urdf.xacro initial_positions)
# and similar to sample_config.yaml
HOME_JOINT_POSITIONS = {
    "shoulder_pan_joint": -0.1597,
    "shoulder_lift_joint": -1.3542,
    "elbow_joint": -1.6648,
    "wrist_1_joint": -1.6933,
    "wrist_2_joint": 1.5710,
    "wrist_3_joint": 1.4110,
}

# Cable spawn config: offset relative to gripper TCP
CABLE_GRIPPER_OFFSET = {"x": 0.0, "y": 0.015385, "z": 0.04245}
CABLE_ORIENTATION = {"roll": 0.4432, "pitch": -0.4838, "yaw": 1.3303}
CABLE_TYPE = "sfp_sc_cable"
ATTACH_CABLE_TO_GRIPPER = True

# Entity names managed by the resetter
CABLE_ENTITY_NAME = "cable_0"

# Timeouts
SERVICE_TIMEOUT_SEC = 10.0
STABILIZE_TIMEOUT_SEC = 20.0
JOINT_VELOCITY_THRESHOLD = 1e-3

# Trials that use procedural randomization instead of their fixed YAML scene config.
# These are SFP insertion trials (SFP_MODULE → SFP_PORT via NIC card).
PROCEDURALLY_RANDOMIZED_TRIALS = {"trial_1", "trial_2"}

# SC port insertion trials (trial_3: SC_TIP → SC_PORT via SC port on rail).
SC_RANDOMIZED_TRIALS = {"trial_3"}

# Board pose bounds for procedurally randomized trials.
# Chosen so the target NIC port stays within robot camera view.
RANDOM_BOARD_X_RANGE = (0.12, 0.22)   # meters, forward distance from robot base
RANDOM_BOARD_Y_RANGE = (-0.25, 0.15)  # meters, lateral offset
RANDOM_BOARD_Z = 1.14                  # fixed — table surface height
RANDOM_BOARD_YAW_RANGE = (2.8, 3.4)   # radians, facing roughly toward robot (around π)

# NIC card randomization for procedurally randomized trials.
RANDOM_NIC_MIN_COUNT = 1   # always at least 1 NIC card (the insertion target)
RANDOM_NIC_MAX_COUNT = 3   # at most 3 NIC cards simultaneously
RANDOM_NIC_YAW_RANGE = (-0.15, 0.15)  # radians, small yaw offset per card

# SC port randomization for trial_3.
# SC_PORT_0 is placed on SC_RAIL_0, SC_PORT_1 on SC_RAIL_1 (when present).
# Either one or both ports may be present; exactly one is the target.
RANDOM_SC_YAW_RANGE = (-0.15, 0.15)   # radians, small yaw offset per SC port


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def generate_task_board_urdf(task_board_cfg: dict, limits: dict) -> str:
    """Run xacro to generate task board URDF from a trial's scene.task_board config.

    Maps YAML keys to xacro args:
      nic_rail_N       → nic_card_mount_N_*   (N = 0..4)
      sc_rail_N        → sc_port_N_*          (N = 0..1)
      lc_mount_rail_N  → lc_mount_rail_N_*    (N = 0..1)
      sfp_mount_rail_N → sfp_mount_rail_N_*   (N = 0..1)
      sc_mount_rail_N  → sc_mount_rail_N_*    (N = 0..1)

    Translations are clamped to the limits from task_board_limits in the config.
    """
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "aic_description", "--share"],
            capture_output=True, text=True, check=True,
        )
        aic_desc_share = result.stdout.strip()
        xacro_file = f"{aic_desc_share}/urdf/task_board.urdf.xacro"

        pose = task_board_cfg.get("pose", {})

        nic_lim = limits.get("nic_rail", {})
        nic_min = float(nic_lim.get("min_translation", -0.0215))
        nic_max = float(nic_lim.get("max_translation", 0.0234))

        sc_lim = limits.get("sc_rail", {})
        sc_min = float(sc_lim.get("min_translation", -0.06))
        sc_max = float(sc_lim.get("max_translation", 0.055))

        mount_lim = limits.get("mount_rail", {})
        mount_min = float(mount_lim.get("min_translation", -0.09425))
        mount_max = float(mount_lim.get("max_translation", 0.09425))

        args = [
            f"x:={pose.get('x', 0.15)}",
            f"y:={pose.get('y', -0.2)}",
            f"z:={pose.get('z', 1.14)}",
            f"roll:={pose.get('roll', 0.0)}",
            f"pitch:={pose.get('pitch', 0.0)}",
            f"yaw:={pose.get('yaw', 3.1415)}",
        ]

        # NIC rails (0-4) → nic_card_mount_N
        for i in range(5):
            cfg = task_board_cfg.get(f"nic_rail_{i}", {})
            present = cfg.get("entity_present", False)
            args.append(f"nic_card_mount_{i}_present:={'true' if present else 'false'}")
            if present:
                print(f"insert NIC card {i}")
                ep = cfg.get("entity_pose", {})
                t = max(nic_min, min(nic_max, float(ep.get("translation", 0.0))))
                args += [
                    f"nic_card_mount_{i}_translation:={t}",
                    f"nic_card_mount_{i}_roll:={ep.get('roll', 0.0)}",
                    f"nic_card_mount_{i}_pitch:={ep.get('pitch', 0.0)}",
                    f"nic_card_mount_{i}_yaw:={ep.get('yaw', 0.0)}",
                ]

        # SC rails (0-1) → sc_port_N
        for i in range(2):
            cfg = task_board_cfg.get(f"sc_rail_{i}", {})
            present = cfg.get("entity_present", False)
            args.append(f"sc_port_{i}_present:={'true' if present else 'false'}")
            if present:
                print(f"insert SC rail {i}")
                ep = cfg.get("entity_pose", {})
                t = max(sc_min, min(sc_max, float(ep.get("translation", 0.0))))
                args += [
                    f"sc_port_{i}_translation:={t}",
                    f"sc_port_{i}_roll:={ep.get('roll', 0.0)}",
                    f"sc_port_{i}_pitch:={ep.get('pitch', 0.0)}",
                    f"sc_port_{i}_yaw:={ep.get('yaw', 0.0)}",
                ]

        # Mount rails: lc_mount, sfp_mount, sc_mount (0-1 each)
        for rail_type in ("lc_mount", "sfp_mount", "sc_mount"):
            for i in range(2):
                key = f"{rail_type}_rail_{i}"
                cfg = task_board_cfg.get(key, {})
                present = cfg.get("entity_present", False)
                args.append(f"{key}_present:={'true' if present else 'false'}")
                if present:
                    ep = cfg.get("entity_pose", {})
                    t = max(mount_min, min(mount_max, float(ep.get("translation", 0.0))))
                    args += [
                        f"{key}_translation:={t}",
                        f"{key}_roll:={ep.get('roll', 0.0)}",
                        f"{key}_pitch:={ep.get('pitch', 0.0)}",
                        f"{key}_yaw:={ep.get('yaw', 0.0)}",
                    ]

        cmd = ["xacro", xacro_file] + args
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return result.stdout
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"xacro failed: {e.stderr}") from e


def generate_cable_sdf(cable_type: str, attach_to_gripper: bool) -> str:
    """Run xacro to generate cable SDF string."""
    try:
        # Find the aic_description share directory
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "aic_description", "--share"],
            capture_output=True, text=True, check=True,
        )
        aic_desc_share = result.stdout.strip()
        xacro_file = f"{aic_desc_share}/urdf/cable.sdf.xacro"

        cmd = [
            "xacro", xacro_file,
            f"attach_cable_to_gripper:={'true' if attach_to_gripper else 'false'}",
            f"cable_type:={cable_type}",
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return result.stdout
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"xacro failed: {e.stderr}") from e


class EnvResetter(Node):
    """ROS2 node that resets the simulation environment for RL training."""

    def __init__(self):
        super().__init__("env_resetter")

        # Declare parameters
        self.declare_parameter("cable_type", CABLE_TYPE)
        self.declare_parameter("attach_cable_to_gripper", ATTACH_CABLE_TO_GRIPPER)
        self.declare_parameter("cable_entity_name", CABLE_ENTITY_NAME)
        self.declare_parameter("cable_offset_x", CABLE_GRIPPER_OFFSET["x"])
        self.declare_parameter("cable_offset_y", CABLE_GRIPPER_OFFSET["y"])
        self.declare_parameter("cable_offset_z", CABLE_GRIPPER_OFFSET["z"])
        self.declare_parameter("cable_roll", CABLE_ORIENTATION["roll"])
        self.declare_parameter("cable_pitch", CABLE_ORIENTATION["pitch"])
        self.declare_parameter("cable_yaw", CABLE_ORIENTATION["yaw"])
        self.declare_parameter("delete_task_board", False)
        self.declare_parameter("gripper_frame", "gripper/tcp")
        self.declare_parameter("config_path", "")

        self._cb_group = ReentrantCallbackGroup()

        # Service clients
        self._reset_joints_client = self.create_client(
            ResetJoints, "/scoring/reset_joints", callback_group=self._cb_group,
        )
        self._switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller",
            callback_group=self._cb_group,
        )
        self._spawn_entity_client = self.create_client(
            SpawnEntity, "/gz_server/spawn_entity", callback_group=self._cb_group,
        )
        self._delete_entity_client = self.create_client(
            DeleteEntity, "/gz_server/delete_entity", callback_group=self._cb_group,
        )
        self._tare_ft_client = self.create_client(
            Trigger, "/aic_controller/tare_force_torque_sensor",
            callback_group=self._cb_group,
        )

        # TF for gripper pose lookup
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Joint state subscriber (for stabilization check)
        self._joints_stable = Event()
        self._joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_states_cb, 10,
        )

        # Pre-generate cable SDFs for all known cable types
        attach = self.get_parameter("attach_cable_to_gripper").value
        self._cable_sdfs: dict[str, str] = {}
        for ct in ("sfp_sc_cable", "sfp_sc_cable_reversed"):
            self.get_logger().info(f"Generating cable SDF (type={ct}, attach={attach})...")
            try:
                self._cable_sdfs[ct] = generate_cable_sdf(ct, attach)
                self.get_logger().info(
                    f"Cable SDF '{ct}' generated ({len(self._cable_sdfs[ct])} bytes)"
                )
            except RuntimeError as e:
                self.get_logger().warn(f"Could not pre-generate SDF for '{ct}': {e}")

        # Default SDF (used when not doing random reset)
        default_ct = self.get_parameter("cable_type").value
        if default_ct not in self._cable_sdfs:
            self.get_logger().info(f"Generating default cable SDF (type={default_ct})...")
            self._cable_sdfs[default_ct] = generate_cable_sdf(default_ct, attach)

        # Load trial configs and task board limits from sample_config.yaml for random reset
        self._trials: list[tuple[str, dict]] = []  # (trial_name, trial_dict)
        self._task_board_limits: dict = {}
        config_path = self.get_parameter("config_path").value
        if config_path:
            try:
                with open(config_path) as f:
                    cfg = yaml.safe_load(f)
                self._trials = list(cfg.get("trials", {}).items())
                self._task_board_limits = cfg.get("task_board_limits", {})
                self.get_logger().info(
                    f"Loaded {len(self._trials)} trials from {config_path}: "
                    f"{[name for name, _ in self._trials]}"
                )
            except Exception as e:
                self.get_logger().warn(f"Failed to load config '{config_path}': {e}")

        # Reset service (SetBool: data=True → random reset, data=False → deterministic)
        self._reset_service = self.create_service(
            SetBool, "/env/reset", self._reset_callback,
            callback_group=self._cb_group,
        )

        # Track the actual entity name returned by the last successful cable spawn.
        # Gazebo may rename entities (e.g. cable_0 → cable_0_0) when allow_renaming=True,
        # so we must delete by the name Gazebo assigned, not the name we requested.
        self._last_cable_name: str | None = None

        # Queue of pre-generated reset configs.
        # Each entry is a dict with keys: trial_name, task_board_cfg, task_info,
        # cable_override, spawn_board.  When empty, _fill_reset_queue() generates
        # a fresh board config and enqueues 2 episodes that share the same physical
        # board layout but target different ports.
        self._reset_queue: deque[dict] = deque()

        # Track whether this is the first reset (skip delete on first call
        # if entities were spawned by the launch file)
        self._first_reset = True

        self.get_logger().info("EnvResetter ready. Call /env/reset to reset environment.")

    def _joint_states_cb(self, msg: JointState):
        if not msg.velocity:
            return
        if all(abs(v) < JOINT_VELOCITY_THRESHOLD for v in msg.velocity):
            self._joints_stable.set()

    def _wait_for_service(self, client, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error(f"Service {name} not available")
            return False
        return True

    def _call_service(self, client, request, name: str):
        """Synchronous service call with timeout.

        Uses a busy-wait instead of rclpy.spin_until_future_complete because
        the node is already being spun by a MultiThreadedExecutor. Calling
        spin_until_future_complete from within an executor callback causes
        the service response to never be sent back.
        """
        future = client.call_async(request)
        t0 = time.monotonic()
        while not future.done():
            time.sleep(0.05)
            if time.monotonic() - t0 > SERVICE_TIMEOUT_SEC:
                self.get_logger().error(f"Service {name} timed out")
                return None
        return future.result()

    def _delete_entity(self, entity_name: str) -> bool:
        if not self._wait_for_service(self._delete_entity_client, "delete_entity"):
            return False
        req = DeleteEntity.Request()
        req.entity = entity_name
        resp = self._call_service(self._delete_entity_client, req, "delete_entity")
        if resp is None:
            return False
        # Result code 1 = OK in simulation_interfaces
        if resp.result.result != 1:
            self.get_logger().warn(
                f"Delete entity '{entity_name}' returned: {resp.result.error_message}"
            )
            return False
        self.get_logger().info(f"Deleted entity '{entity_name}'")
        return True

    def _switch_controllers(self, activate: list, deactivate: list) -> bool:
        if not self._wait_for_service(
            self._switch_controller_client, "switch_controller"
        ):
            return False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        resp = self._call_service(
            self._switch_controller_client, req, "switch_controller"
        )
        if resp is None or not resp.ok:
            self.get_logger().error("Failed to switch controllers")
            return False
        return True

    def _reset_joints(self) -> bool:
        if not self._wait_for_service(self._reset_joints_client, "reset_joints"):
            return False
        req = ResetJoints.Request()
        req.joint_names = list(HOME_JOINT_POSITIONS.keys())
        req.initial_positions = list(HOME_JOINT_POSITIONS.values())
        resp = self._call_service(self._reset_joints_client, req, "reset_joints")
        if resp is None or not resp.success:
            self.get_logger().error(f"Failed to reset joints: {getattr(resp, 'message', 'timeout')}")
            return False
        self.get_logger().info("Joints reset to home positions")
        return True

    def _tare_ft_sensor(self) -> bool:
        if not self._wait_for_service(self._tare_ft_client, "tare_ft"):
            return False
        req = Trigger.Request()
        resp = self._call_service(self._tare_ft_client, req, "tare_ft")
        if resp is None or not resp.success:
            self.get_logger().error("Failed to tare F/T sensor")
            return False
        self.get_logger().info("F/T sensor tared")
        return True

    def _get_gripper_pose(self):
        """Get current gripper TCP pose in world frame."""
        gripper_frame = self.get_parameter("gripper_frame").value
        try:
            t = self._tf_buffer.lookup_transform("world", gripper_frame, rclpy.time.Time())
            return t.transform.translation
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    def _spawn_cable(
        self,
        offset_x: float = None,
        offset_y: float = None,
        offset_z: float = None,
        roll: float = None,
        pitch: float = None,
        yaw: float = None,
        entity_name: str = None,
        cable_type: str = None,
    ) -> bool:
        if not self._wait_for_service(self._spawn_entity_client, "spawn_entity"):
            return False

        # Fall back to ROS parameters for any value not explicitly provided
        if offset_x is None:
            offset_x = self.get_parameter("cable_offset_x").value
        if offset_y is None:
            offset_y = self.get_parameter("cable_offset_y").value
        if offset_z is None:
            offset_z = self.get_parameter("cable_offset_z").value
        if roll is None:
            roll = self.get_parameter("cable_roll").value
        if pitch is None:
            pitch = self.get_parameter("cable_pitch").value
        if yaw is None:
            yaw = self.get_parameter("cable_yaw").value
        if entity_name is None:
            entity_name = self.get_parameter("cable_entity_name").value
        if cable_type is None:
            cable_type = self.get_parameter("cable_type").value

        cable_sdf = self._cable_sdfs.get(cable_type)
        if cable_sdf is None:
            self.get_logger().error(f"No pre-generated SDF for cable type '{cable_type}'")
            return False

        # Get gripper position for cable placement
        gripper_pos = self._get_gripper_pose()
        if gripper_pos is None:
            return False

        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

        req = SpawnEntity.Request()
        req.name = entity_name
        req.allow_renaming = True
        req.resource_string = cable_sdf
        req.entity_namespace = ""
        req.initial_pose.header.frame_id = "world"
        req.initial_pose.pose.position.x = gripper_pos.x + offset_x
        req.initial_pose.pose.position.y = gripper_pos.y + offset_y
        req.initial_pose.pose.position.z = gripper_pos.z + offset_z
        req.initial_pose.pose.orientation.x = qx
        req.initial_pose.pose.orientation.y = qy
        req.initial_pose.pose.orientation.z = qz
        req.initial_pose.pose.orientation.w = qw

        resp = self._call_service(self._spawn_entity_client, req, "spawn_entity")
        if resp is None or resp.result.result != 1:
            err = getattr(resp, "result", None)
            self.get_logger().error(
                f"Failed to spawn cable: {err.error_message if err else 'timeout'}"
            )
            return False

        self._last_cable_name = resp.entity_name
        self.get_logger().info(f"Cable spawned as '{resp.entity_name}'")
        return True

    def _spawn_task_board(self, task_board_cfg: dict) -> bool:
        """Delete and re-spawn the task board using the given scene config dict."""
        if not self._wait_for_service(self._spawn_entity_client, "spawn_entity"):
            return False

        try:
            urdf = generate_task_board_urdf(task_board_cfg, self._task_board_limits)
        except RuntimeError as e:
            self.get_logger().error(f"Task board xacro generation failed: {e}")
            return False

        pose = task_board_cfg.get("pose", {})
        qx, qy, qz, qw = rpy_to_quaternion(
            float(pose.get("roll", 0.0)),
            float(pose.get("pitch", 0.0)),
            float(pose.get("yaw", 3.1415)),
        )

        req = SpawnEntity.Request()
        req.name = "task_board"
        req.allow_renaming = True
        req.resource_string = urdf
        req.entity_namespace = ""
        req.initial_pose.header.frame_id = "world"
        req.initial_pose.pose.position.x = float(pose.get("x", 0.15))
        req.initial_pose.pose.position.y = float(pose.get("y", -0.2))
        req.initial_pose.pose.position.z = float(pose.get("z", 1.14))
        req.initial_pose.pose.orientation.x = qx
        req.initial_pose.pose.orientation.y = qy
        req.initial_pose.pose.orientation.z = qz
        req.initial_pose.pose.orientation.w = qw

        resp = self._call_service(self._spawn_entity_client, req, "spawn_entity")
        if resp is None or resp.result.result != 1:
            err = getattr(resp, "result", None)
            self.get_logger().error(
                f"Failed to spawn task board: {err.error_message if err else 'timeout'}"
            )
            return False

        self.get_logger().info(f"Task board spawned as '{resp.entity_name}'")
        return True

    def _wait_for_stabilization(self) -> bool:
        """Wait for robot joints to settle (velocities near zero)."""
        self._joints_stable.clear()
        settled = self._joints_stable.wait(timeout=STABILIZE_TIMEOUT_SEC)
        if not settled:
            self.get_logger().warning("Robot did not stabilize within timeout")
        return settled

    def _pick_random_trial(self) -> tuple[str, dict] | tuple[None, None]:
        """Pick a random trial from the loaded config.
        Returns (trial_name, trial_dict), or (None, None) if no trials are loaded."""
        if not self._trials:
            self.get_logger().warn(
                "Random reset requested but no trials loaded (set config_path parameter)"
            )
            return None, None

        #return self._trials[-1]
        return random.choice(self._trials)

    def _cable_config_from_trial(self, trial: dict) -> dict | None:
        """Extract cable kwargs for _spawn_cable() from a trial dict."""
        cables = trial.get("scene", {}).get("cables", {})
        if not cables:
            self.get_logger().warn("Trial has no cables section, using defaults")
            return None
        cable_name, cable_cfg = next(iter(cables.items()))
        pose = cable_cfg.get("pose", {})
        offset = pose.get("gripper_offset", {})
        return {
            "offset_x": float(offset.get("x", CABLE_GRIPPER_OFFSET["x"])),
            "offset_y": float(offset.get("y", CABLE_GRIPPER_OFFSET["y"])),
            "offset_z": float(offset.get("z", CABLE_GRIPPER_OFFSET["z"])),
            "roll": float(pose.get("roll", CABLE_ORIENTATION["roll"])),
            "pitch": float(pose.get("pitch", CABLE_ORIENTATION["pitch"])),
            "yaw": float(pose.get("yaw", CABLE_ORIENTATION["yaw"])),
            "entity_name": cable_name,
            "cable_type": cable_cfg.get("cable_type", CABLE_TYPE),
        }

    def _randomize_sfp_trial_board(
        self, base_cfg: dict,
    ) -> tuple[dict, list[int]]:
        """Return (randomized_cfg, chosen_nic_rails) for SFP insertion trials.

        Randomizes board pose and NIC rail layout.  Caller picks the target rail(s)
        from the returned chosen_nic_rails list.
        """
        cfg = copy.deepcopy(base_cfg)

        # --- Board pose ---
        new_x = random.uniform(*RANDOM_BOARD_X_RANGE)
        new_y = random.uniform(*RANDOM_BOARD_Y_RANGE)
        new_yaw = random.uniform(*RANDOM_BOARD_YAW_RANGE)
        cfg["pose"] = {
            "x": new_x,
            "y": new_y,
            "z": RANDOM_BOARD_Z,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": new_yaw,
        }

        # --- NIC rails ---
        nic_lim = self._task_board_limits.get("nic_rail", {})
        nic_min = float(nic_lim.get("min_translation", -0.0215))
        nic_max = float(nic_lim.get("max_translation", 0.0234))

        num_nics = random.randint(RANDOM_NIC_MIN_COUNT, RANDOM_NIC_MAX_COUNT)
        chosen_rails = sorted(random.sample(range(5), num_nics))

        for i in range(5):
            if i in chosen_rails:
                cfg[f"nic_rail_{i}"] = {
                    "entity_present": True,
                    "entity_name": f"nic_card_{i}",
                    "entity_pose": {
                        "translation": random.uniform(nic_min, nic_max),
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": random.uniform(*RANDOM_NIC_YAW_RANGE),
                    },
                }
            else:
                cfg[f"nic_rail_{i}"] = {"entity_present": False}

        self.get_logger().info(
            f"  board pose: x={new_x:.3f} y={new_y:.3f} yaw={new_yaw:.3f} | "
            f"NIC rails populated: {chosen_rails}"
        )
        return cfg, chosen_rails

    def _randomize_sc_trial_board(
        self, base_cfg: dict, num_sc: int | None = None,
    ) -> tuple[dict, list[int]]:
        """Return (randomized_cfg, chosen_sc_rails) for SC port insertion trials (trial_3).

        Randomizes board pose, SC rail layout, and NIC distractor layout.
        Caller picks the target SC rail(s) from the returned chosen_sc_rails list.

        Args:
            base_cfg: Trial scene.task_board config to randomize on top of.
            num_sc:   Number of SC ports to place (1 or 2).  If None, chosen randomly.
                      Pass 2 when generating episode pairs so two distinct targets are
                      always available.
        """
        cfg = copy.deepcopy(base_cfg)

        # --- Board pose ---
        new_x = random.uniform(*RANDOM_BOARD_X_RANGE)
        new_y = random.uniform(*RANDOM_BOARD_Y_RANGE)
        new_yaw = random.uniform(*RANDOM_BOARD_YAW_RANGE)
        cfg["pose"] = {
            "x": new_x,
            "y": new_y,
            "z": RANDOM_BOARD_Z,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": new_yaw,
        }

        # --- SC rails ---
        sc_lim = self._task_board_limits.get("sc_rail", {})
        sc_min = float(sc_lim.get("min_translation", -0.06))
        sc_max = float(sc_lim.get("max_translation", 0.055))

        if num_sc is None:
            num_sc = random.randint(1, 2)
        chosen_sc_rails = sorted(random.sample([0, 1], num_sc))

        for i in range(2):
            if i in chosen_sc_rails:
                cfg[f"sc_rail_{i}"] = {
                    "entity_present": True,
                    "entity_name": f"sc_mount_{i}",
                    "entity_pose": {
                        "translation": random.uniform(sc_min, sc_max),
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": random.uniform(*RANDOM_SC_YAW_RANGE),
                    },
                }
            else:
                cfg[f"sc_rail_{i}"] = {"entity_present": False}

        # --- NIC rails (distractors) ---
        nic_lim = self._task_board_limits.get("nic_rail", {})
        nic_min = float(nic_lim.get("min_translation", -0.0215))
        nic_max = float(nic_lim.get("max_translation", 0.0234))

        num_nics = random.randint(0, RANDOM_NIC_MAX_COUNT)
        chosen_nic_rails = sorted(random.sample(range(5), num_nics))

        for i in range(5):
            if i in chosen_nic_rails:
                cfg[f"nic_rail_{i}"] = {
                    "entity_present": True,
                    "entity_name": f"nic_card_{i}",
                    "entity_pose": {
                        "translation": random.uniform(nic_min, nic_max),
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": random.uniform(*RANDOM_NIC_YAW_RANGE),
                    },
                }
            else:
                cfg[f"nic_rail_{i}"] = {"entity_present": False}

        self.get_logger().info(
            f"  board pose: x={new_x:.3f} y={new_y:.3f} yaw={new_yaw:.3f} | "
            f"SC rails populated: {chosen_sc_rails} | "
            f"NIC distractors: {chosen_nic_rails}"
        )
        return cfg, chosen_sc_rails

    def _fill_reset_queue(self) -> None:
        """Pick a random trial, generate one board config, and enqueue 2 episodes.

        Both episodes share the same physical board layout (same task_board_cfg) but
        target different modules/ports.  The first entry in the pair gets
        spawn_board=True (board must be (re-)spawned); the second gets spawn_board=False
        (board is already in place from the first episode, so spawning is skipped).
        """
        trial_name, trial = self._pick_random_trial()
        if not trial:
            return

        cable_override = self._cable_config_from_trial(trial)
        task_board_cfg = trial.get("scene", {}).get("task_board")
        base_task = next(iter(trial.get("tasks", {}).values()), {})

        entries: list[dict] = []

        if trial_name in SC_RANDOMIZED_TRIALS and task_board_cfg:
            # Always place both SC ports so we always have 2 distinct targets.
            board_cfg, chosen_sc_rails = self._randomize_sc_trial_board(
                task_board_cfg, num_sc=2
            )
            targets = random.sample(chosen_sc_rails, min(2, len(chosen_sc_rails)))
            for target_sc_rail in targets:
                entries.append({
                    "trial_name": trial_name,
                    "task_board_cfg": board_cfg,
                    "cable_override": cable_override,
                    "task_info": {
                        "trial_name": trial_name,
                        "cable_type": base_task.get("cable_type", "sc"),
                        "cable_name": cable_override["entity_name"] if cable_override else CABLE_ENTITY_NAME,
                        "plug_type": base_task.get("plug_type", "sc"),
                        "plug_name": base_task.get("plug_name", "sc_tip"),
                        "port_type": base_task.get("port_type", "sc"),
                        "port_name": "sc_port_base",
                        "target_module_name": f"sc_port_{target_sc_rail}",
                        "time_limit": base_task.get("time_limit", 180),
                    },
                })

        elif trial_name in PROCEDURALLY_RANDOMIZED_TRIALS and task_board_cfg:
            board_cfg, chosen_rails = self._randomize_sfp_trial_board(task_board_cfg)
            # Two pairing strategies:
            #   - len == 1, or 50% chance when len > 1: same NIC rail, different port
            #   - 50% chance when len > 1: two different NIC rails, each with a random port
            if len(chosen_rails) > 1 and random.random() < 0.5:
                target_rails = random.sample(chosen_rails, 2)
                pairs = [
                    (target_rails[0], random.choice(["sfp_port_0", "sfp_port_1"])),
                    (target_rails[1], random.choice(["sfp_port_0", "sfp_port_1"])),
                ]
            else:
                target_rail = random.choice(chosen_rails)
                pairs = [
                    (target_rail, "sfp_port_0"),
                    (target_rail, "sfp_port_1"),
                ]
            for target_rail, port_name in pairs:
                entries.append({
                    "trial_name": trial_name,
                    "task_board_cfg": board_cfg,
                    "cable_override": cable_override,
                    "task_info": {
                        "trial_name": trial_name,
                        "cable_type": base_task.get("cable_type", "sfp_sc"),
                        "cable_name": cable_override["entity_name"] if cable_override else CABLE_ENTITY_NAME,
                        "plug_type": base_task.get("plug_type", "sfp"),
                        "plug_name": base_task.get("plug_name", "sfp_tip"),
                        "port_type": base_task.get("port_type", "sfp"),
                        "port_name": port_name,
                        "target_module_name": f"nic_card_mount_{target_rail}",
                        "time_limit": base_task.get("time_limit", 180),
                    },
                })

        else:
            # Fixed scene config — only one episode per board config.
            entries.append({
                "trial_name": trial_name,
                "task_board_cfg": task_board_cfg,
                "cable_override": cable_override,
                "task_info": {
                    "trial_name": trial_name,
                    "cable_type": base_task.get("cable_type", ""),
                    "cable_name": base_task.get("cable_name", CABLE_ENTITY_NAME),
                    "plug_type": base_task.get("plug_type", ""),
                    "plug_name": base_task.get("plug_name", ""),
                    "port_type": base_task.get("port_type", ""),
                    "port_name": base_task.get("port_name", ""),
                    "target_module_name": base_task.get("target_module_name", ""),
                    "time_limit": base_task.get("time_limit", 180),
                },
            })

        # First entry spawns the board; subsequent entries reuse it.
        for idx, entry in enumerate(entries):
            entry["spawn_board"] = (idx == 0)
            self._reset_queue.append(entry)

        self.get_logger().info(
            f"Filled reset queue with {len(entries)} episode(s) "
            f"(trial='{trial_name}', "
            f"targets={[e['task_info']['target_module_name'] for e in entries]})"
        )

    def _reset_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Full environment reset sequence.

        request.data=True  → randomly pick a trial from sample_config.yaml; resets
                             the task board scene (NIC/SC rails, mount positions, board
                             pose) AND the cable configuration.
        request.data=False → use fixed default parameters (cable only; task board is
                             left as-is unless delete_task_board param is set).
        """
        random_reset = request.data
        self.get_logger().info(
            f"=== Starting environment reset (random={random_reset}) ==="
        )

        trial_name: str | None = None
        cable_override: dict | None = None
        task_board_cfg: dict | None = None
        task_info: dict = {}
        spawn_board: bool = True

        if random_reset:
            # Refill the queue when empty, then dequeue the next episode.
            if not self._reset_queue:
                self._fill_reset_queue()

            if self._reset_queue:
                entry = self._reset_queue.popleft()
                trial_name = entry["trial_name"]
                task_board_cfg = entry["task_board_cfg"]
                cable_override = entry["cable_override"]
                task_info = entry["task_info"]
                spawn_board = entry["spawn_board"]
                self.get_logger().info(
                    f"Random reset: dequeued episode "
                    f"(trial='{trial_name}', "
                    f"target='{task_info.get('target_module_name')}', "
                    f"port='{task_info.get('port_name')}', "
                    f"spawn_board={spawn_board}, "
                    f"queue_remaining={len(self._reset_queue)})"
                )
            else:
                self.get_logger().warn(
                    "Random reset requested but queue is still empty after fill — "
                    "no trials loaded?"
                )

        success = True
        message_parts = []

        # Step 1: Delete cable.
        # Always delete by _last_cable_name (the actual Gazebo-assigned name from the
        # previous spawn by this node). On the very first reset, _last_cable_name is None
        # so fall back to the cable_entity_name ROS param — that is the name used by the
        # launch file, and is independent of whichever trial was just picked.
        entity_to_delete = self._last_cable_name or self.get_parameter("cable_entity_name").value
        self.get_logger().info(f"Step 1: Deleting cable '{entity_to_delete}'...")
        if not self._delete_entity(entity_to_delete):
            message_parts.append("cable deletion failed (may not exist)")
            # Not fatal - cable might not exist on first reset

        # Step 2: Delete task board
        #   - for random reset: only when spawn_board=True (first episode of a pair
        #     spawns a fresh board; second episode reuses the existing board)
        #   - for deterministic reset: only when delete_task_board param is True
        if (random_reset and spawn_board) or self.get_parameter("delete_task_board").value:
            self.get_logger().info("Step 2: Deleting task board...")
            if not self._delete_entity("task_board"):
                message_parts.append("task board deletion failed (may not exist)")
                # Not fatal
        elif random_reset:
            self.get_logger().info("Step 2: Skipping task board deletion (reusing board from previous episode)")

        # Step 3: Deactivate controller
        self.get_logger().info("Step 3: Deactivating aic_controller...")
        if not self._switch_controllers([], ["aic_controller"]):
            success = False
            message_parts.append("controller deactivation failed")

        # Step 4: Reset joints to home
        if success:
            self.get_logger().info("Step 4: Resetting joints to home...")
            if not self._reset_joints():
                success = False
                message_parts.append("joint reset failed")

        # Step 5: Reactivate controller
        self.get_logger().info("Step 5: Reactivating aic_controller...")
        if not self._switch_controllers(["aic_controller"], []):
            success = False
            message_parts.append("controller reactivation failed")

        # Step 6: Tare F/T sensor (before cable/board add weight)
        if success:
            self.get_logger().info("Step 6: Taring F/T sensor...")
            if not self._tare_ft_sensor():
                message_parts.append("FT tare failed")
                # Not fatal

        # Step 7: Spawn task board
        #   - for random reset: only when spawn_board=True (first episode of a pair)
        #   - for deterministic reset: no-op (task board stays as launched)
        if success and random_reset and spawn_board and task_board_cfg:
            self.get_logger().info("Step 7: Spawning task board from trial config...")
            if not self._spawn_task_board(task_board_cfg):
                success = False
                message_parts.append("task board spawn failed")
        elif random_reset and not spawn_board:
            self.get_logger().info("Step 7: Skipping task board spawn (reusing board from previous episode)")

        # Step 8: Spawn cable
        if success:
            self.get_logger().info("Step 8: Spawning cable...")
            spawn_ok = (
                self._spawn_cable(**cable_override)
                if cable_override
                else self._spawn_cable()
            )
            if not spawn_ok:
                success = False
                message_parts.append("cable spawn failed")

        # Step 9: Wait for stabilization
        if success:
            self.get_logger().info("Step 9: Waiting for stabilization...")
            self._wait_for_stabilization()

        self._first_reset = False

        if success:
            warnings = f" (warnings: {'; '.join(message_parts)})" if message_parts else ""
            self.get_logger().info(f"=== Environment reset complete{warnings} ===")
            if task_info:
                self.get_logger().info(f"Task info: {task_info}")
            # Derive which NIC cards and SC ports are physically present on the board.
            if task_board_cfg and task_info:
                present_entities = []
                for i in range(5):
                    if task_board_cfg.get(f"nic_rail_{i}", {}).get("entity_present", False):
                        present_entities.append(f"nic_card_mount_{i}")
                for i in range(2):
                    if task_board_cfg.get(f"sc_rail_{i}", {}).get("entity_present", False):
                        present_entities.append(f"sc_port_{i}")
                task_info["present_entities"] = present_entities
            # Encode task details as JSON so callers can parse structured data.
            # On failure the message is a plain error string instead.
            response.message = json.dumps(task_info) if task_info else "{}"
        else:
            msg = f"Environment reset failed: {'; '.join(message_parts)}"
            self.get_logger().error(f"=== {msg} ===")
            response.message = msg

        response.success = success
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EnvResetter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
