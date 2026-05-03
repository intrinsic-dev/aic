"""MuJoCo Gymnasium environment for Phase 2: force-guided insertion.

The robot starts centered above the port (as if Phase 1 just succeeded) and
must descend into the port while using all 3 wrist cameras and F/T feedback.
Full physics simulation (mj_step) is used here because cable elasticity matters.

Body/site/sensor names may need adjusting to match your generated scene.xml.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional

try:
    import mujoco
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as e:
    raise ImportError(f"Training deps missing: {e}") from e

from .image_utils import (
    IMG_SIZE,
    WIDE_CROP_PX,
    add_canny_channel,
    center_crop_and_resize,
)

_ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
_HOME_QPOS = np.array([-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110])
_SFP_TCP_TO_TIP = np.array([0.0, 0.015385, -0.04245])
_SC_TCP_TO_TIP = np.array([0.0, 0.015385, -0.04045])

# Approximate insertion depth (how far below port surface success is)
_INSERTION_DEPTH = 0.015  # 15 mm

# Force limits
_FORCE_PENALTY_THRESHOLD = 10.0  # N — start penalizing above this
_FORCE_LIMIT = 30.0  # N — terminate episode (too dangerous)


@dataclass
class InsertionConfig:
    scene_path: str
    port_type: str = "sfp"
    sfp_port_body: str = "sfp_port_0"  # verify!
    sc_port_body: str = "sc_port_0"  # verify!
    tcp_site: str = "gripper_tcp"
    center_camera: str = "center_camera"
    left_camera: str = "left_camera"
    right_camera: str = "right_camera"
    ft_sensor_name: str = "force_torque"  # sensor name in scene.xml — verify!
    max_steps: int = 300
    xy_offset_range: tuple = (-0.003, 0.003)  # small residual offset at start
    z_start_offset: float = 0.005  # start just above port surface
    substeps: int = 5  # physics steps per policy step
    arm_joint_names: list = field(default_factory=lambda: list(_ARM_JOINT_NAMES))
    render_width: int = 320
    render_height: int = 240


class InsertionEnv(gym.Env):
    """Phase 2 insertion environment."""

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, config: InsertionConfig):
        super().__init__()
        self.cfg = config

        self.model = mujoco.MjModel.from_xml_path(config.scene_path)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(
            self.model, height=config.render_height, width=config.render_width
        )

        self._tcp_site_id = self.model.site(config.tcp_site).id
        self._port_body_name = (
            config.sfp_port_body if config.port_type == "sfp" else config.sc_port_body
        )

        try:
            self._port_body_id = self.model.body(self._port_body_name).id
        except Exception:
            raise ValueError(
                f"Port body '{self._port_body_name}' not found in {config.scene_path}."
            )

        self._cam_ids = {
            "center": mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, config.center_camera
            ),
            "left": mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, config.left_camera
            ),
            "right": mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, config.right_camera
            ),
        }

        self._ft_sensor_adr = self._find_sensor_adr(config.ft_sensor_name)
        self._arm_joint_ids = self._resolve_joint_ids(config.arm_joint_names)

        port_type_enc = np.array(
            [1.0, 0.0] if config.port_type == "sfp" else [0.0, 1.0], dtype=np.float32
        )
        self._port_type_enc = port_type_enc
        self._tcp_to_tip = (
            _SFP_TCP_TO_TIP if config.port_type == "sfp" else _SC_TCP_TO_TIP
        )

        self.observation_space = spaces.Dict(
            {
                "center": spaces.Box(0, 255, (IMG_SIZE, IMG_SIZE, 4), dtype=np.uint8),
                "left": spaces.Box(0, 255, (IMG_SIZE, IMG_SIZE, 4), dtype=np.uint8),
                "right": spaces.Box(0, 255, (IMG_SIZE, IMG_SIZE, 4), dtype=np.uint8),
                "proprio": spaces.Box(-np.inf, np.inf, (13,), dtype=np.float32),
            }
        )
        self.action_space = spaces.Box(-1.0, 1.0, (3,), dtype=np.float32)

        self._step = 0
        self._start_tcp_pos: Optional[np.ndarray] = None
        self._tare_ft: Optional[np.ndarray] = None

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _find_sensor_adr(self, name: str) -> int:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return self.model.sensor_adr[sid] if sid >= 0 else -1

    def _resolve_joint_ids(self, names: list[str]) -> np.ndarray:
        ids = []
        for name in names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in scene.")
            ids.append(self.model.jnt_dofadr[jid])
        return np.array(ids, dtype=int)

    def _get_tcp_pos(self) -> np.ndarray:
        return self.data.site_xpos[self._tcp_site_id].copy()

    def _get_tcp_mat(self) -> np.ndarray:
        return self.data.site_xmat[self._tcp_site_id].reshape(3, 3).copy()

    def _get_port_pos(self) -> np.ndarray:
        return self.data.xpos[self._port_body_id].copy()

    def _get_plug_tip_pos(self) -> np.ndarray:
        return self._get_tcp_pos() + self._get_tcp_mat() @ self._tcp_to_tip

    def _get_ft(self) -> np.ndarray:
        if self._ft_sensor_adr < 0 or self.model.nsensor == 0:
            return np.zeros(6, dtype=np.float32)
        raw = self.data.sensordata[self._ft_sensor_adr : self._ft_sensor_adr + 6].copy()
        if self._tare_ft is not None:
            raw -= self._tare_ft
        return raw.astype(np.float32)

    def _apply_delta_ik(self, delta: np.ndarray) -> None:
        """Apply (dx, dy, dz) world-frame TCP delta via damped IK."""
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, None, self._tcp_site_id)
        J = jacp[:, self._arm_joint_ids]
        lam = 0.05
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), delta)
        self.data.qpos[self._arm_joint_ids] += dq

    def _render_camera(self, cam_id: int) -> np.ndarray:
        self.renderer.update_scene(self.data, camera=cam_id)
        rgb = self.renderer.render()
        rgbc = add_canny_channel(rgb)
        return center_crop_and_resize(rgbc, WIDE_CROP_PX)

    def _get_obs(self) -> dict:
        imgs = {k: self._render_camera(v) for k, v in self._cam_ids.items()}
        tcp_pos = self._get_tcp_pos()
        xyz_rel = (tcp_pos - self._start_tcp_pos).astype(np.float32)
        ft = self._get_ft()
        depth = float(self._start_tcp_pos[2] - tcp_pos[2])  # positive = descended
        step_norm = np.float32(self._step / self.cfg.max_steps)
        proprio = np.concatenate(
            [xyz_rel, ft, self._port_type_enc, [step_norm], [depth]]
        ).astype(np.float32)
        imgs["proprio"] = proprio
        return imgs

    # ── Gymnasium API ───────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        for i, jid in enumerate(self._arm_joint_ids):
            self.data.qpos[jid] = _HOME_QPOS[i]
        mujoco.mj_forward(self.model, self.data)

        # Place TCP just above port center (small XY residual)
        port_pos = self._get_port_pos()
        rng = self.np_random
        dx = rng.uniform(*self.cfg.xy_offset_range)
        dy = rng.uniform(*self.cfg.xy_offset_range)
        target = np.array(
            [port_pos[0] + dx, port_pos[1] + dy, port_pos[2] + self.cfg.z_start_offset]
        )
        self._move_tcp_to(target)
        mujoco.mj_forward(self.model, self.data)

        self._start_tcp_pos = self._get_tcp_pos().copy()
        self._tare_ft = self._get_ft().copy()  # zero out baseline force
        self._step = 0
        return self._get_obs(), {}

    def _move_tcp_to(self, target: np.ndarray, max_iter: int = 80) -> None:
        for _ in range(max_iter):
            err = target - self._get_tcp_pos()
            if np.linalg.norm(err) < 1e-4:
                break
            self._apply_delta_ik(err * 0.8)
            mujoco.mj_forward(self.model, self.data)

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0)
        from .networks import XY_INS_SCALE, Z_INS_SCALE

        delta = np.array(
            [
                float(action[0]) * XY_INS_SCALE,
                float(action[1]) * XY_INS_SCALE,
                -float(action[2]) * Z_INS_SCALE,  # positive action → descend
            ]
        )
        self._apply_delta_ik(delta)
        for _ in range(self.cfg.substeps):
            mujoco.mj_step(self.model, self.data)

        plug_tip = self._get_plug_tip_pos()
        port_pos = self._get_port_pos()
        ft = self._get_ft()
        fz = float(ft[2])
        f_mag = float(np.linalg.norm(ft[:3]))

        # Depth below port surface (positive = inside)
        depth = float(port_pos[2] - plug_tip[2])
        xy_err = float(np.linalg.norm(plug_tip[:2] - port_pos[:2]))

        # Reward shaping
        reward = 0.0
        reward += 5.0 * max(0.0, depth)  # depth progress
        reward -= 2.0 * xy_err  # stay centered
        if f_mag > _FORCE_PENALTY_THRESHOLD:
            reward -= 0.5 * (f_mag - _FORCE_PENALTY_THRESHOLD)

        terminated = False
        if depth > _INSERTION_DEPTH and xy_err < 0.005:
            reward += 50.0
            terminated = True

        too_much_force = f_mag > _FORCE_LIMIT
        if too_much_force:
            reward -= 20.0
            terminated = True

        self._step += 1
        truncated = self._step >= self.cfg.max_steps

        obs = self._get_obs()
        info = {
            "xy_error": xy_err,
            "depth": depth,
            "ft_mag": f_mag,
            "fz": fz,
        }
        return obs, reward, terminated, truncated, info

    def render(self):
        self.renderer.update_scene(self.data, camera=self._cam_ids["center"])
        return self.renderer.render()

    def close(self):
        self.renderer.close()
