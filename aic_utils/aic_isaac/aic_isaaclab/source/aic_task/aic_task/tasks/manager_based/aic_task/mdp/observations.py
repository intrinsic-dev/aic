# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Observation functions for the AIC task (e.g. contact sensing)."""

from __future__ import annotations

import re
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def contact_net_forces(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Net contact forces (world frame) from the contact sensor, flattened for policy obs.

    Uses the current timestep net forces (no history). Body selection is via sensor_cfg.body_ids
    if set by the manager, or sensor_cfg.body_names matched against the sensor's body_names.

    Returns:
        Tensor of shape (num_envs, num_bodies * 3) in world frame (x,y,z per body).
    """
    from isaaclab.sensors import ContactSensor

    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net = contact_sensor.data.net_forces_w  # (N, B, 3)
    body_ids = sensor_cfg.body_ids
    if body_ids is None or body_ids == slice(None):
        if getattr(sensor_cfg, "body_names", None) is not None:
            names = (
                [sensor_cfg.body_names]
                if isinstance(sensor_cfg.body_names, str)
                else sensor_cfg.body_names
            )
            pattern = re.compile(names[0] if len(names) == 1 else "|".join(names))
            body_ids = [
                i for i, b in enumerate(contact_sensor.body_names) if pattern.search(b)
            ]
            if body_ids:
                net = net[:, body_ids, :]
    else:
        net = net[:, body_ids, :]
    return net.reshape(env.num_envs, -1)
