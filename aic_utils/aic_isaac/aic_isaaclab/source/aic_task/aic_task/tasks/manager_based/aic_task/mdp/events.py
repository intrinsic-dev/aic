from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING
from pxr import UsdGeom, Gf

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def sample_object_poses(
    num_objects: int,
    min_separation: float = 0.0,
    pose_range: dict[str, tuple[float, float]] = {},
    max_sample_tries: int = 5000,
):
    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    pose_list = []

    for i in range(num_objects):
        for j in range(max_sample_tries):
            sample = [random.uniform(range[0], range[1]) for range in range_list]

            # Accept pose if it is the first one, or if reached max num tries
            if len(pose_list) == 0 or j == max_sample_tries - 1:
                pose_list.append(sample)
                break

            # Check if pose of object is sufficiently far away from all other objects
            separation_check = [
                math.dist(sample[:3], pose[:3]) > min_separation for pose in pose_list
            ]
            if False not in separation_check:
                pose_list.append(sample)
                break

    return pose_list


def randomize_object_pose(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfgs: list[SceneEntityCfg],
    min_separation: float = 0.0,
    pose_range: dict[str, tuple[float, float]] = {},
    max_sample_tries: int = 5000,
):
    if env_ids is None:
        return

    # Randomize poses in each environment independently
    for cur_env in env_ids.tolist():
        pose_list = sample_object_poses(
            num_objects=len(asset_cfgs),
            min_separation=min_separation,
            pose_range=pose_range,
            max_sample_tries=max_sample_tries,
        )

        # Randomize pose for each object
        for i in range(len(asset_cfgs)):
            asset_cfg = asset_cfgs[i]
            asset = env.scene[asset_cfg.name]

            # Write pose to simulation
            pose_tensor = torch.tensor([pose_list[i]], device=env.device)
            positions = pose_tensor[:, 0:3] + env.scene.env_origins[cur_env, 0:3]
            orientations = math_utils.quat_from_euler_xyz(
                pose_tensor[:, 3], pose_tensor[:, 4], pose_tensor[:, 5]
            )
            asset.write_root_pose_to_sim(
                torch.cat([positions, orientations], dim=-1),
                env_ids=torch.tensor([cur_env], device=env.device),
            )
            asset.write_root_velocity_to_sim(
                torch.zeros(1, 6, device=env.device),
                env_ids=torch.tensor([cur_env], device=env.device),
            )


def randomize_xform_position(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_name: str,
    pose_range: dict[str, tuple[float, float]],
    default_pos: tuple[float, float, float],
):
    xform_view: XformPrimView = env.scene.extras[asset_name]

    range_list = [pose_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z"]]
    ranges = torch.tensor(range_list, device=env.device)
    rand_offsets = math_utils.sample_uniform(
        ranges[:, 0], ranges[:, 1], (len(env_ids), 3), device=env.device
    )

    default = torch.tensor(default_pos, device=env.device).unsqueeze(0)
    positions = default + env.scene.env_origins[env_ids] + rand_offsets

    # Convert env_ids to a CPU list — XformPrimView indices must NOT be CUDA tensors
    xform_view.set_world_poses(positions=positions, indices=env_ids.tolist())


import omni.usd
from pxr import UsdLux, Gf


def randomize_dome_light(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    intensity_range: tuple[float, float] = (1500.0, 3500.0),
    color_range: tuple[tuple[float, float, float], tuple[float, float, float]] = (
        (0.5, 0.5, 0.5),
        (1.0, 1.0, 1.0),
    ),
):
    """Randomize dome light intensity and color at reset.

    Note: Since there's only one shared light (not per-env), this changes
    the light globally for all environments on each reset.
    """
    stage = omni.usd.get_context().get_stage()
    light_prim = stage.GetPrimAtPath("/World/light")

    if not light_prim.IsValid():
        return

    light = UsdLux.DomeLight(light_prim)

    # Randomize intensity
    intensity = torch.empty(1).uniform_(intensity_range[0], intensity_range[1]).item()
    light.GetIntensityAttr().Set(intensity)

    # Randomize color
    color_min, color_max = color_range
    r = torch.empty(1).uniform_(color_min[0], color_max[0]).item()
    g = torch.empty(1).uniform_(color_min[1], color_max[1]).item()
    b = torch.empty(1).uniform_(color_min[2], color_max[2]).item()
    light.GetColorAttr().Set(Gf.Vec3f(r, g, b))


def reset_asset_base_position(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    prim_path: str = "{ENV_REGEX_NS}/Table",
    pose_range: dict = {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)},
    default_pos: tuple = (0.5, 0.0, 1.05),
):
    """Reset a static asset's position via USD APIs."""
    stage = omni.usd.get_context().get_stage()

    for env_id in env_ids.tolist():
        # resolve the per-env prim path
        resolved_path = prim_path.replace("{ENV_REGEX_NS}", f"/World/envs/env_{env_id}")
        prim = stage.GetPrimAtPath(resolved_path)
        if not prim.IsValid():
            continue

        xform = UsdGeom.Xformable(prim)
        # sample random offsets
        x = (
            default_pos[0]
            + torch.empty(1).uniform_(*pose_range.get("x", (0.0, 0.0))).item()
        )
        y = (
            default_pos[1]
            + torch.empty(1).uniform_(*pose_range.get("y", (0.0, 0.0))).item()
        )
        z = (
            default_pos[2]
            + torch.empty(1).uniform_(*pose_range.get("z", (0.0, 0.0))).item()
        )

        # set the translate op
        xform_ops = xform.GetOrderedXformOps()
        for op in xform_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(x, y, z))
                break


def _set_prim_translate(stage, prim_path: str, pos: tuple[float, float, float]):
    """Helper: set the translate xform op on a USD prim."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
            return


def _sample_axis(pose_range: dict, snap_step: dict, axis: str) -> float:
    """Sample a random offset for an axis. If snap_step has a value for this axis,
    snap to the nearest multiple of that step within the range."""
    lo, hi = pose_range.get(axis, (0.0, 0.0))
    step = snap_step.get(axis, 0.0)
    if step > 0 and (hi - lo) > 0:
        n_lo = math.ceil(lo / step)
        n_hi = math.floor(hi / step)
        n = random.randint(n_lo, n_hi)
        return n * step
    return torch.empty(1).uniform_(lo, hi).item()


def randomize_board_and_parts(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    board_path: str = "{ENV_REGEX_NS}/task_board",
    board_default_pos: tuple = (0.16141, -0.041, 0.0),
    board_range: dict = {"x": (0.0, 0.0), "y": (0.0, 0.0)},
    parts: list[dict] = (),
):
    """Randomize the task board position, then place parts relative to it.

    Only changes translation via USD xform ops — rotation is never touched,
    so each object keeps the rotation from its init_state.

    Each entry in *parts* is a dict with:
        prim_path      – USD path template (with {ENV_REGEX_NS})
        offset         – (dx, dy, dz) default offset from the board
        pose_range     – {"x": (lo, hi), "y": (lo, hi)} local perturbation
        snap_step      – (optional) {"x": step, "y": step} snap to discrete positions
    """
    stage = omni.usd.get_context().get_stage()

    for env_id in env_ids.tolist():
        bx = (
            board_default_pos[0]
            + torch.empty(1).uniform_(*board_range.get("x", (0.0, 0.0))).item()
        )
        by = (
            board_default_pos[1]
            + torch.empty(1).uniform_(*board_range.get("y", (0.0, 0.0))).item()
        )
        bz = board_default_pos[2]

        resolved_board = board_path.replace(
            "{ENV_REGEX_NS}", f"/World/envs/env_{env_id}"
        )
        _set_prim_translate(stage, resolved_board, (bx, by, bz))

        for part in parts:
            ox, oy, oz = part["offset"]
            pr = part.get("pose_range", {})
            snap = part.get("snap_step", {})
            px = bx + ox + _sample_axis(pr, snap, "x")
            py = by + oy + _sample_axis(pr, snap, "y")
            pz = bz + oz
            resolved = part["prim_path"].replace(
                "{ENV_REGEX_NS}", f"/World/envs/env_{env_id}"
            )
            _set_prim_translate(stage, resolved, (px, py, pz))
