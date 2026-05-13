from __future__ import annotations

import math
import random
import re
from typing import TYPE_CHECKING

import omni.usd
import torch
from pxr import Gf, Sdf, UsdGeom, UsdLux

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

# Matches the regex form Isaac Lab uses to instantiate per-env prim paths.
_ENV_REGEX_RE = re.compile(r"env_(?:\.\*|\[\^/\]\*)")

# Orientations captured from PhysX on the first reset and reused on every
# subsequent reset. Holding the quaternion fixed keeps the composed child
# transforms from referenced USDs (e.g. port frames) correctly aligned.
_cached_orientations: dict[str, torch.Tensor] = {}


def randomize_dome_light(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    intensity_range: tuple[float, float] = (1500.0, 3500.0),
    color_range: tuple[tuple[float, float, float], tuple[float, float, float]] = (
        (0.5, 0.5, 0.5),
        (1.0, 1.0, 1.0),
    ),
) -> None:
    """Randomize the dome light's intensity and color on reset.

    The light is a single shared prim, so the randomization is global across
    all environments regardless of ``env_ids``.
    """
    stage = omni.usd.get_context().get_stage()
    light_prim = stage.GetPrimAtPath("/World/light")
    if not light_prim.IsValid():
        return
    light = UsdLux.DomeLight(light_prim)

    intensity = torch.empty(1).uniform_(intensity_range[0], intensity_range[1]).item()
    light.GetIntensityAttr().Set(intensity)

    color_min, color_max = color_range
    r = torch.empty(1).uniform_(color_min[0], color_max[0]).item()
    g = torch.empty(1).uniform_(color_min[1], color_max[1]).item()
    b = torch.empty(1).uniform_(color_min[2], color_max[2]).item()
    light.GetColorAttr().Set(Gf.Vec3f(r, g, b))


def _sample_axis(pose_range: dict, snap_step: dict, axis: str) -> float:
    """Sample an axis offset, snapping to a grid step when configured."""
    lo, hi = pose_range.get(axis, (0.0, 0.0))
    step = snap_step.get(axis, 0.0)
    if step > 0 and (hi - lo) > 0:
        n_lo = math.ceil(lo / step)
        n_hi = math.floor(hi / step)
        return random.randint(n_lo, n_hi) * step
    return torch.empty(1).uniform_(lo, hi).item()


def _write_usd_xform_pose(
    stage,
    prim_path_template: str,
    env_ids: torch.Tensor,
    env_origins: torch.Tensor,
    world_pos: torch.Tensor,
    world_rot: torch.Tensor,
) -> None:
    """Mirror a per-env rigid body pose onto its USD Xform.

    The prim translate is authored relative to its env root, so the world
    position is converted to env-local coordinates before writing.
    """
    ids = env_ids.tolist()
    local_pos = (world_pos - env_origins).tolist()
    rot = world_rot.tolist()

    for i, env_id in enumerate(ids):
        prim_path = _ENV_REGEX_RE.sub(f"env_{env_id}", prim_path_template)
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            continue

        xf = UsdGeom.Xformable(prim)
        tx, ty, tz = local_pos[i]
        qw, qx, qy, qz = rot[i]

        for op in xf.GetOrderedXformOps():
            name = op.GetOpName()
            if "translate" in name:
                if op.GetTypeName() == Sdf.ValueTypeNames.Float3:
                    op.Set(Gf.Vec3f(tx, ty, tz))
                else:
                    op.Set(Gf.Vec3d(tx, ty, tz))
            elif "orient" in name:
                if op.GetTypeName() == Sdf.ValueTypeNames.Quatf:
                    op.Set(Gf.Quatf(qw, qx, qy, qz))
                else:
                    op.Set(Gf.Quatd(qw, qx, qy, qz))


def randomize_board_and_parts(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    board_scene_name: str = "task_board",
    board_default_pos: tuple = (0.0, 0.0, 0.0),
    board_range: dict = {"x": (0.0, 0.0), "y": (0.0, 0.0)},
    parts: list[dict] = (),
    sync_usd_xforms: bool = True,
) -> None:
    """Randomize the task board and its attached parts on reset.

    The board position is drawn from ``board_range`` around ``board_default_pos``.
    Each part is offset from the board by a fixed ``offset`` plus a random
    delta from ``pose_range`` (optionally snapped to ``snap_step``).

    When ``sync_usd_xforms`` is True (default) the pose is mirrored onto the
    USD Xform so the viewport tracks physics state. Training workloads should
    set this False to skip the per-env USD writes.
    """
    device = env.device
    n = len(env_ids)
    env_origins = env.scene.env_origins[env_ids]
    stage = omni.usd.get_context().get_stage() if sync_usd_xforms else None

    all_names = [board_scene_name] + [p["scene_name"] for p in parts]
    if not _cached_orientations:
        for name in all_names:
            _cached_orientations[name] = (
                env.scene[name].data.root_state_w[:, 3:7].clone()
            )

    # Board pose.
    board_asset = env.scene[board_scene_name]
    board_rot = _cached_orientations[board_scene_name][env_ids]
    board_pos = torch.tensor([board_default_pos], device=device).expand(n, -1).clone()
    board_pos[:, 0] += torch.empty(n, device=device).uniform_(
        *board_range.get("x", (0.0, 0.0))
    )
    board_pos[:, 1] += torch.empty(n, device=device).uniform_(
        *board_range.get("y", (0.0, 0.0))
    )
    board_world_pos = board_pos + env_origins

    board_asset.write_root_pose_to_sim(
        torch.cat([board_world_pos, board_rot], dim=-1), env_ids=env_ids
    )
    board_asset.write_root_velocity_to_sim(
        torch.zeros(n, 6, device=device), env_ids=env_ids
    )
    if sync_usd_xforms:
        _write_usd_xform_pose(
            stage,
            board_asset.cfg.prim_path,
            env_ids,
            env_origins,
            board_world_pos,
            board_rot,
        )

    # Part poses, anchored to the board.
    for part_cfg in parts:
        pname = part_cfg["scene_name"]
        part_asset = env.scene[pname]
        part_rot = _cached_orientations[pname][env_ids]

        ox, oy, oz = part_cfg["offset"]
        pr = part_cfg.get("pose_range", {})
        snap = part_cfg.get("snap_step", {})

        part_pos = board_world_pos.clone()
        for idx in range(n):
            part_pos[idx, 0] += ox + _sample_axis(pr, snap, "x")
            part_pos[idx, 1] += oy + _sample_axis(pr, snap, "y")
            part_pos[idx, 2] = board_world_pos[idx, 2] + oz

        part_asset.write_root_pose_to_sim(
            torch.cat([part_pos, part_rot], dim=-1), env_ids=env_ids
        )
        part_asset.write_root_velocity_to_sim(
            torch.zeros(n, 6, device=device), env_ids=env_ids
        )
        if sync_usd_xforms:
            _write_usd_xform_pose(
                stage,
                part_asset.cfg.prim_path,
                env_ids,
                env_origins,
                part_pos,
                part_rot,
            )
