# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Utilities for XR (OpenXR/SteamVR) teleop sessions."""

from __future__ import annotations

import queue
import select
import sys
import termios
import threading
import tty


def spawn_stdin_reader(
    command_queue: queue.SimpleQueue[str],
    stop_event: threading.Event,
) -> threading.Thread:
    """Read single-key commands from stdin without blocking the simulation loop.

    XR hand-tracking devices do not emit Isaac Lab START/STOP/RESET messages,
    so this thread provides terminal hotkeys as a fallback control channel.
    """

    def _reader() -> None:
        if not sys.stdin.isatty():
            return
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not stop_event.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch:
                        command_queue.put(ch.lower())
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    thread = threading.Thread(target=_reader, name="xr-stdin-reader", daemon=True)
    thread.start()
    return thread


def remove_xr_cameras(env_cfg):
    """Remove camera scene entries and observation terms for XR sessions.

    Isaac Lab's ``remove_camera_configs`` expects ``*_camera`` observation names.
    AIC uses ``*_rgb`` instead, so this handles both naming conventions.
    """
    from isaaclab.devices.openxr import remove_camera_configs

    try:
        return remove_camera_configs(env_cfg)
    except AttributeError:
        for attr in ("center_camera", "left_camera", "right_camera"):
            if hasattr(env_cfg.scene, attr):
                setattr(env_cfg.scene, attr, None)
        for attr in ("center_rgb", "left_rgb", "right_rgb"):
            if hasattr(env_cfg.observations.policy, attr):
                delattr(env_cfg.observations.policy, attr)
        return env_cfg
