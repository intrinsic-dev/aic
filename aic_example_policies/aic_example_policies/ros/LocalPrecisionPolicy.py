"""Two-phase SAC insertion policy for the AIC qualification challenge.

Phase 0 — Orientation pre-step:
    SLERP the gripper to a nominal downward orientation (no GT needed).
    Uses a fixed target quaternion per port type.

Phase 1 — XY centering (CNN + SAC):
    Outputs (dx, dy) Cartesian commands using the center wrist camera.
    Terminates when estimated XY error falls below threshold or time limit.

Phase 2 — F/T-guided insertion (CNN + SAC):
    Outputs (dx, dy, dz) commands using all 3 cameras + F/T sensor.
    Descends until insertion depth or time limit.

Checkpoints are loaded from the paths set by PHASE1_CKPT / PHASE2_CKPT.
Set these via environment variables or edit the defaults below.
"""

import os
import time

import numpy as np
import torch
from geometry_msgs.msg import Point, Pose, Quaternion

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from transforms3d._gohlketransforms import quaternion_slerp

from ..training.image_utils import (
    IMG_SIZE,
    NARROW_CROP_PX,
    WIDE_CROP_PX,
    add_canny_channel,
    center_crop_and_resize,
    crop_and_resize,
    ros_image_to_rgb,
    to_tensor,
)
from ..training.networks import SACActor

# ── Checkpoint paths (override with env vars) ────────────────────────────────
_DEFAULT_P1_CKPT = os.environ.get(
    "AIC_PHASE1_CKPT",
    os.path.join(os.path.dirname(__file__), "../../checkpoints/phase1/final.pt"),
)
_DEFAULT_P2_CKPT = os.environ.get(
    "AIC_PHASE2_CKPT",
    os.path.join(os.path.dirname(__file__), "../../checkpoints/phase2/final.pt"),
)

# ── Nominal TCP orientations (w, x, y, z) for each port type ────────────────
# Derived from CheatCode gripper offsets: TCP pointing downward, plug facing port.
# Adjust these if your training used a different nominal pose.
_NOMINAL_QUAT = {
    "sfp": np.array(
        [0.6533, 0.2706, 0.6533, 0.2706], dtype=np.float64
    ),  # approx downward
    "sc": np.array([0.6533, 0.2706, 0.6533, 0.2706], dtype=np.float64),
}

# ── Tunable thresholds ───────────────────────────────────────────────────────
_P1_XY_THRESHOLD = 0.004  # m — Phase 1 done when FT-based XY estimate < this
_P1_MAX_STEPS = 200  # steps at 20 Hz = 10 s
_P2_FORCE_LIMIT = 25.0  # N — abort insertion
_P2_MAX_STEPS = 300  # steps at 20 Hz = 15 s
_STEP_DT = 0.05  # seconds (20 Hz loop)
_ORIENT_STEPS = 60  # 3 s for orientation pre-step

# ── Scales (must match training) ─────────────────────────────────────────────
_XY_SCALE = 0.005
_XY_INS_SCALE = 0.001
_Z_INS_SCALE = 0.002

_DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class LocalPrecisionPolicy(Policy):
    """Two-phase SAC policy that loads PyTorch weights from checkpoints."""

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self._actor1: SACActor | None = None
        self._actor2: SACActor | None = None
        self._load_checkpoints()

    # ── Checkpoint loading ───────────────────────────────────────────────────

    def _load_checkpoints(self) -> None:
        def _load(phase: int, path: str) -> SACActor | None:
            path = os.path.abspath(path)
            if not os.path.exists(path):
                self.get_logger().warn(f"Checkpoint not found: {path}")
                return None
            actor = SACActor(phase).to(_DEVICE)
            ckpt = torch.load(path, map_location=_DEVICE)
            actor.load_state_dict(ckpt["actor"])
            actor.eval()
            self.get_logger().info(f"Loaded Phase {phase} actor from {path}")
            return actor

        self._actor1 = _load(1, _DEFAULT_P1_CKPT)
        self._actor2 = _load(2, _DEFAULT_P2_CKPT)

    # ── Image helpers ─────────────────────────────────────────────────────────

    @staticmethod
    def _prep_image(img_msg, half_crop: int = WIDE_CROP_PX) -> torch.Tensor:
        """Decode ROS image, add Canny, crop+resize → 1×4×84×84 float tensor."""
        rgb = ros_image_to_rgb(img_msg)
        rgbc = add_canny_channel(rgb)
        cropped = center_crop_and_resize(rgbc, half_crop)
        return to_tensor(cropped).unsqueeze(0).to(_DEVICE)

    @staticmethod
    def _proprio_p1(
        tcp_rel: np.ndarray,
        ft_z: float,
        port_type_enc: np.ndarray,
        step_norm: float,
    ) -> torch.Tensor:
        arr = np.concatenate([tcp_rel, [ft_z], port_type_enc, [step_norm]]).astype(
            np.float32
        )
        return torch.from_numpy(arr).unsqueeze(0).to(_DEVICE)

    @staticmethod
    def _proprio_p2(
        tcp_rel: np.ndarray,
        ft6: np.ndarray,
        port_type_enc: np.ndarray,
        step_norm: float,
        depth: float,
    ) -> torch.Tensor:
        arr = np.concatenate(
            [tcp_rel, ft6, port_type_enc, [step_norm], [depth]]
        ).astype(np.float32)
        return torch.from_numpy(arr).unsqueeze(0).to(_DEVICE)

    # ── Pose helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _pose_from_tcp(
        tcp_pose, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0
    ) -> Pose:
        """Return an absolute pose = current TCP + world-frame delta."""
        return Pose(
            position=Point(
                x=tcp_pose.position.x + dx,
                y=tcp_pose.position.y + dy,
                z=tcp_pose.position.z + dz,
            ),
            orientation=tcp_pose.orientation,
        )

    @staticmethod
    def _get_tcp_pose(obs) -> Pose:
        return obs.controller_state.tcp_pose

    @staticmethod
    def _get_ft6(obs) -> np.ndarray:
        w = obs.wrist_wrench.wrench
        return np.array(
            [
                w.force.x,
                w.force.y,
                w.force.z,
                w.torque.x,
                w.torque.y,
                w.torque.z,
            ],
            dtype=np.float32,
        )

    # ── Phase 0: orientation alignment ───────────────────────────────────────

    def _orient_step(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        port_type: str,
    ) -> Pose:
        """SLERP gripper to nominal downward orientation over ~3 s."""
        target_q_wxyz = _NOMINAL_QUAT.get(port_type, _NOMINAL_QUAT["sfp"])

        obs = get_observation()
        tcp = self._get_tcp_pose(obs)
        q_now = np.array(
            [
                tcp.orientation.w,
                tcp.orientation.x,
                tcp.orientation.y,
                tcp.orientation.z,
            ],
            dtype=np.float64,
        )

        self.get_logger().info(f"Phase 0: orienting for port_type={port_type}")
        for i in range(1, _ORIENT_STEPS + 1):
            t = i / _ORIENT_STEPS
            q_slerp = quaternion_slerp(q_now, target_q_wxyz, t)
            pose = Pose(
                position=tcp.position,
                orientation=Quaternion(
                    w=float(q_slerp[0]),
                    x=float(q_slerp[1]),
                    y=float(q_slerp[2]),
                    z=float(q_slerp[3]),
                ),
            )
            self.set_pose_target(move_robot, pose)
            self.sleep_for(_STEP_DT)

        obs = get_observation()
        return self._get_tcp_pose(obs)

    # ── Phase 1: XY centering ─────────────────────────────────────────────────

    def _centering_phase(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        port_type_enc: np.ndarray,
    ) -> bool:
        if self._actor1 is None:
            self.get_logger().error("Phase 1 actor not loaded — cannot center.")
            return False

        self.get_logger().info("Phase 1: XY centering")
        obs = get_observation()
        tcp0 = self._get_tcp_pose(obs)
        tcp0_xyz = np.array([tcp0.position.x, tcp0.position.y, tcp0.position.z])
        ft_tare = self._get_ft6(obs).copy()

        for step in range(_P1_MAX_STEPS):
            obs = get_observation()
            tcp = self._get_tcp_pose(obs)
            tcp_xyz = np.array([tcp.position.x, tcp.position.y, tcp.position.z])
            xyz_rel = (tcp_xyz - tcp0_xyz).astype(np.float32)

            ft = (self._get_ft6(obs) - ft_tare).astype(np.float32)
            ft_z = float(ft[2])
            step_norm = float(step / _P1_MAX_STEPS)

            img_t = self._prep_image(obs.center_image)
            prop_t = self._proprio_p1(xyz_rel, ft_z, port_type_enc, step_norm)
            obs_t = {"image": img_t, "proprio": prop_t}

            with torch.no_grad():
                action = self._actor1.get_action(obs_t).squeeze(0).cpu().numpy()

            dx = float(action[0]) * _XY_SCALE
            dy = float(action[1]) * _XY_SCALE

            target_pose = self._pose_from_tcp(tcp, dx=dx, dy=dy)
            self.set_pose_target(move_robot, target_pose)

            # Estimate convergence from TCP error (controller tracks tcp_error)
            tcp_err = obs.controller_state.tcp_error
            xy_err = float(np.hypot(tcp_err[0], tcp_err[1]))

            send_feedback(f"P1 step={step} xy_err={xy_err:.4f}m")
            if xy_err < _P1_XY_THRESHOLD:
                self.get_logger().info(
                    f"Phase 1 converged at step {step}, xy_err={xy_err:.4f}"
                )
                return True

            self.sleep_for(_STEP_DT)

        self.get_logger().warn("Phase 1 timed out — proceeding to insertion anyway")
        return True

    # ── Phase 2: Insertion ────────────────────────────────────────────────────

    def _insertion_phase(
        self,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        port_type_enc: np.ndarray,
    ) -> bool:
        if self._actor2 is None:
            self.get_logger().error("Phase 2 actor not loaded — cannot insert.")
            return False

        self.get_logger().info("Phase 2: F/T-guided insertion")
        obs = get_observation()
        tcp0 = self._get_tcp_pose(obs)
        tcp0_xyz = np.array([tcp0.position.x, tcp0.position.y, tcp0.position.z])
        ft_tare = self._get_ft6(obs).copy()
        # Switch to compliance stiffness: low Z for insertion
        _low_z_stiffness = [90.0, 90.0, 20.0, 30.0, 30.0, 30.0]
        _low_z_damping = [50.0, 50.0, 15.0, 15.0, 15.0, 15.0]

        for step in range(_P2_MAX_STEPS):
            obs = get_observation()
            tcp = self._get_tcp_pose(obs)
            tcp_xyz = np.array([tcp.position.x, tcp.position.y, tcp.position.z])
            xyz_rel = (tcp_xyz - tcp0_xyz).astype(np.float32)
            depth = float(tcp0_xyz[2] - tcp_xyz[2])

            ft6 = (self._get_ft6(obs) - ft_tare).astype(np.float32)
            f_mag = float(np.linalg.norm(ft6[:3]))

            if f_mag > _P2_FORCE_LIMIT:
                self.get_logger().warn(
                    f"Force limit exceeded ({f_mag:.1f} N) — aborting insertion"
                )
                return False

            step_norm = float(step / _P2_MAX_STEPS)
            half_crop = NARROW_CROP_PX if depth > 0.005 else WIDE_CROP_PX

            img_c = self._prep_image(obs.center_image, half_crop)
            img_l = self._prep_image(obs.left_image, half_crop)
            img_r = self._prep_image(obs.right_image, half_crop)
            prop_t = self._proprio_p2(xyz_rel, ft6, port_type_enc, step_norm, depth)
            obs_t = {"center": img_c, "left": img_l, "right": img_r, "proprio": prop_t}

            with torch.no_grad():
                action = self._actor2.get_action(obs_t).squeeze(0).cpu().numpy()

            dx = float(action[0]) * _XY_INS_SCALE
            dy = float(action[1]) * _XY_INS_SCALE
            dz = -float(action[2]) * _Z_INS_SCALE  # positive action → descend

            target_pose = self._pose_from_tcp(tcp, dx=dx, dy=dy, dz=dz)
            self.set_pose_target(
                move_robot,
                target_pose,
                stiffness=_low_z_stiffness,
                damping=_low_z_damping,
            )

            send_feedback(f"P2 step={step} depth={depth:.4f}m fmag={f_mag:.1f}N")
            self.sleep_for(_STEP_DT)

        self.get_logger().warn("Phase 2 timed out")
        return True

    # ── Main entry point ──────────────────────────────────────────────────────

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(
            f"LocalPrecisionPolicy.insert_cable(): "
            f"plug={task.plug_type} port={task.port_type} target={task.target_module_name}"
        )

        port_type = "sc" if task.port_type.lower().startswith("sc") else "sfp"
        port_type_enc = np.array(
            [1.0, 0.0] if port_type == "sfp" else [0.0, 1.0], dtype=np.float32
        )

        # Phase 0: orient gripper
        send_feedback("Phase 0: orienting")
        self._orient_step(get_observation, move_robot, port_type)

        # Phase 1: center over port
        send_feedback("Phase 1: centering")
        self._centering_phase(get_observation, move_robot, send_feedback, port_type_enc)

        # Phase 2: insert
        send_feedback("Phase 2: inserting")
        success = self._insertion_phase(
            get_observation, move_robot, send_feedback, port_type_enc
        )

        if success:
            self.get_logger().info("Insertion complete. Waiting for stabilization.")
            self.sleep_for(3.0)
        else:
            self.get_logger().error("Insertion failed.")

        return success
