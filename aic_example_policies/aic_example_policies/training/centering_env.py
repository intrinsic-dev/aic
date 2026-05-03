"""MuJoCo Gymnasium environment for Phase 1: XY centering above the port.

The robot starts at a random XY offset (~1–3 cm) from the port center at
a fixed Z height. The policy must output (dx, dy) actions to align the plug
tip with the port center using only the center wrist camera.

Usage:
    env = CenteringEnv(
        scene_path="/path/to/scene.xml",
        port_type="sfp",   # or "sc"
    )
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(action)

Body/site names in the MJCF may need adjusting to match the generated scene.
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
    raise ImportError(
        f"Training dependencies missing ({e}). Run: pip install mujoco gymnasium"
    ) from e

from .image_utils import (
    IMG_SIZE,
    WIDE_CROP_PX,
    add_canny_channel,
    center_crop_and_resize,
)

# ── Defaults (verify against your generated scene.xml) ─────────────────────
_ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# Home joint config from sample_config.yaml
_HOME_QPOS = np.array([-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110])

# Gripper offset from TCP to plug tip (approximate; from sample_config.yaml)
_SFP_TCP_TO_TIP = np.array([0.0, 0.015385, -0.04245])  # in TCP frame
_SC_TCP_TO_TIP = np.array([0.0, 0.015385, -0.04045])


@dataclass
class CenteringConfig:
    scene_path: str
    port_type: str = "sfp"  # "sfp" or "sc"
    sfp_port_body: str = "sfp_port_0"  # body name in scene.xml — verify!
    sc_port_body: str = "sc_port_0"  # body name in scene.xml — verify!
    tcp_site: str = "gripper_tcp"  # site name in scene.xml
    center_camera: str = "center_camera"
    max_steps: int = 200
    # Starting XY offset range (meters)
    xy_offset_range: tuple = (-0.025, 0.025)
    # Fixed Z offset above port during centering (meters)
    z_hover: float = 0.06
    # Convergence threshold (meters)
    xy_threshold: float = 0.003
    arm_joint_names: list = field(default_factory=lambda: list(_ARM_JOINT_NAMES))
    render_width: int = 320
    render_height: int = 240


class CenteringEnv(gym.Env):
    """Phase 1 centering environment."""

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, config: CenteringConfig):
        super().__init__()
        self.cfg = config

        self.model = mujoco.MjModel.from_xml_path(config.scene_path)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(
            self.model, height=config.render_height, width=config.render_width
        )

        # Cache IDs
        self._tcp_site_id = self.model.site(config.tcp_site).id
        self._cam_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, config.center_camera
        )
        self._port_body_name = (
            config.sfp_port_body if config.port_type == "sfp" else config.sc_port_body
        )

        try:
            self._port_body_id = self.model.body(self._port_body_name).id
        except Exception:
            raise ValueError(
                f"Port body '{self._port_body_name}' not found in {config.scene_path}. "
                "Adjust sfp_port_body / sc_port_body in CenteringConfig."
            )

        self._arm_joint_ids = self._resolve_joint_ids(config.arm_joint_names)

        port_type_enc = np.array(
            [1.0, 0.0] if config.port_type == "sfp" else [0.0, 1.0], dtype=np.float32
        )
        self._port_type_enc = port_type_enc
        self._tcp_to_tip = (
            _SFP_TCP_TO_TIP if config.port_type == "sfp" else _SC_TCP_TO_TIP
        )

        # Spaces
        self.observation_space = spaces.Dict(
            {
                "image": spaces.Box(0, 255, (IMG_SIZE, IMG_SIZE, 4), dtype=np.uint8),
                "proprio": spaces.Box(-np.inf, np.inf, (7,), dtype=np.float32),
            }
        )
        self.action_space = spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)

        self._step = 0
        self._start_tcp_pos: Optional[np.ndarray] = None

    # ── Internal helpers ────────────────────────────────────────────────────

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
        tcp_pos = self._get_tcp_pos()
        tcp_mat = self._get_tcp_mat()
        return tcp_pos + tcp_mat @ self._tcp_to_tip

    def _apply_delta_ik(self, dx_world: float, dy_world: float) -> None:
        """Move TCP by (dx, dy) in world XY using damped least-squares IK."""
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self._tcp_site_id)

        J = jacp[:, self._arm_joint_ids]  # 3×6
        dp = np.array([dx_world, dy_world, 0.0])

        # Damped least-squares
        lam = 0.05
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), dp)

        qpos_arm = self.data.qpos[self._arm_joint_ids].copy()
        qpos_arm = np.clip(qpos_arm + dq, -np.pi * 2, np.pi * 2)
        self.data.qpos[self._arm_joint_ids] = qpos_arm
        mujoco.mj_forward(self.model, self.data)

    def _render_center_camera(self) -> np.ndarray:
        """Returns IMG_SIZE×IMG_SIZE×4 (RGBC) uint8 array."""
        self.renderer.update_scene(self.data, camera=self._cam_id)
        rgb = self.renderer.render()  # H×W×3
        rgbc = add_canny_channel(rgb)
        return center_crop_and_resize(rgbc, WIDE_CROP_PX)

    def _get_obs(self) -> dict:
        image = self._render_center_camera()
        tcp_pos = self._get_tcp_pos()
        xyz_rel = (tcp_pos - self._start_tcp_pos).astype(np.float32)
        # FT z-force (tared to zero at episode start → not available in pure kinematics sim)
        ft_z = (
            np.float32(self.data.sensordata[2])
            if self.model.nsensor > 0
            else np.float32(0.0)
        )
        step_norm = np.float32(self._step / self.cfg.max_steps)
        proprio = np.concatenate(
            [xyz_rel, [ft_z], self._port_type_enc, [step_norm]]
        ).astype(np.float32)
        return {"image": image, "proprio": proprio}

    # ── Gymnasium API ───────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        # Set arm to home
        for i, jid in enumerate(self._arm_joint_ids):
            self.data.qpos[jid] = _HOME_QPOS[i]
        mujoco.mj_forward(self.model, self.data)

        # Move TCP to hover height above port with random XY offset
        port_pos = self._get_port_pos()
        rng = self.np_random
        dx = rng.uniform(*self.cfg.xy_offset_range)
        dy = rng.uniform(*self.cfg.xy_offset_range)

        target_tcp = np.array(
            [
                port_pos[0] + dx,
                port_pos[1] + dy,
                port_pos[2] + self.cfg.z_hover,
            ]
        )
        self._move_tcp_to(target_tcp)
        mujoco.mj_forward(self.model, self.data)

        self._start_tcp_pos = self._get_tcp_pos().copy()
        self._step = 0
        return self._get_obs(), {}

    def _move_tcp_to(self, target_xyz: np.ndarray, max_iter: int = 50) -> None:
        """Iterative IK to place TCP at target_xyz (approximate)."""
        for _ in range(max_iter):
            err = target_xyz - self._get_tcp_pos()
            if np.linalg.norm(err) < 1e-4:
                break
            self._apply_delta_ik(err[0] * 0.8, err[1] * 0.8)

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0)
        from .networks import XY_SCALE

        dx = float(action[0]) * XY_SCALE
        dy = float(action[1]) * XY_SCALE
        self._apply_delta_ik(dx, dy)

        plug_tip = self._get_plug_tip_pos()
        port_pos = self._get_port_pos()
        xy_err = np.linalg.norm(plug_tip[:2] - port_pos[:2])

        reward = float(-10.0 * xy_err)
        terminated = False
        if xy_err < self.cfg.xy_threshold:
            reward += 10.0
            terminated = True

        self._step += 1
        truncated = self._step >= self.cfg.max_steps

        obs = self._get_obs()
        info = {"xy_error": xy_err, "plug_tip": plug_tip, "port_pos": port_pos}
        return obs, reward, terminated, truncated, info

    def render(self):
        self.renderer.update_scene(self.data, camera=self._cam_id)
        return self.renderer.render()

    def close(self):
        self.renderer.close()
