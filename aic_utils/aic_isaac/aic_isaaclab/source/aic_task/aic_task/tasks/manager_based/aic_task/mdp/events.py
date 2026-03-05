from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING

import omni.usd
import torch
from pxr import Gf, UsdLux

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


_cached_orientations: dict[str, torch.Tensor] = {}


def randomize_board_and_parts(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    board_scene_name: str = "task_board",
    board_default_pos: tuple = (0.0, 0.0, 0.0),
    board_range: dict = {"x": (0.0, 0.0), "y": (0.0, 0.0)},
    parts: list[dict] = (),
):
    """Reset and randomize the task board + parts positions.

    On the first invocation, captures the true orientations from the live
    PhysX state (which includes composed USD child transforms).  These are
    cached and reused on every subsequent reset so we never write a wrong
    orientation.  Only positions are randomized; orientations are preserved.
    """
    global _cached_orientations
    device = env.device
    n = len(env_ids)
    env_origins = env.scene.env_origins[env_ids]

    board_asset = env.scene[board_scene_name]

    all_names = [board_scene_name] + [p["scene_name"] for p in parts]
    if not _cached_orientations:
        for name in all_names:
            asset = env.scene[name]
            _cached_orientations[name] = asset.data.root_state_w[:, 3:7].clone()
    board_rot = _cached_orientations[board_scene_name][env_ids]

    board_pos = torch.tensor([board_default_pos], device=device).expand(n, -1).clone()
    bx_off = torch.empty(n, device=device).uniform_(*board_range.get("x", (0.0, 0.0)))
    by_off = torch.empty(n, device=device).uniform_(*board_range.get("y", (0.0, 0.0)))
    board_pos[:, 0] += bx_off
    board_pos[:, 1] += by_off

    board_world_pos = board_pos + env_origins
    board_pose = torch.cat([board_world_pos, board_rot], dim=-1)
    board_asset.write_root_pose_to_sim(board_pose, env_ids=env_ids)
    board_asset.write_root_velocity_to_sim(
        torch.zeros(n, 6, device=device), env_ids=env_ids
    )

    for part_cfg in parts:
        pname = part_cfg["scene_name"]
        part_asset = env.scene[pname]
        part_rot = _cached_orientations[pname][env_ids]

        ox, oy, oz = part_cfg["offset"]
        pr = part_cfg.get("pose_range", {})
        snap = part_cfg.get("snap_step", {})

        part_pos = board_world_pos.clone()
        for idx in range(n):
            dx = _sample_axis(pr, snap, "x")
            dy = _sample_axis(pr, snap, "y")
            part_pos[idx, 0] += ox + dx
            part_pos[idx, 1] += oy + dy
            part_pos[idx, 2] = board_world_pos[idx, 2] + oz

        part_pose = torch.cat([part_pos, part_rot], dim=-1)
        part_asset.write_root_pose_to_sim(part_pose, env_ids=env_ids)
        part_asset.write_root_velocity_to_sim(
            torch.zeros(n, 6, device=device), env_ids=env_ids
        )
