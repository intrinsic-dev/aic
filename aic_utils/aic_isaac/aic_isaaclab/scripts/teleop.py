# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Teleoperation script for Isaac Lab environments (with aic_task support)."""

"""Launch Isaac Sim Simulator first."""

import argparse
from collections.abc import Callable

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Teleoperation for Isaac Lab environments."
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to simulate."
)
parser.add_argument(
    "--teleop_device",
    type=str,
    default="keyboard",
    help="Teleop device. Built-ins: keyboard, spacemouse, gamepad.",
)
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--sensitivity", type=float, default=1.0, help="Sensitivity factor."
)
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument(
    "--enable_pinocchio",
    action="store_true",
    default=False,
    help="Enable Pinocchio.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher_args = vars(args_cli)

if args_cli.enable_pinocchio:
    import pinocchio  # noqa: F401
if "handtracking" in args_cli.teleop_device.lower():
    app_launcher_args["xr"] = True

app_launcher = AppLauncher(app_launcher_args)
simulation_app = app_launcher.app

"""Rest everything follows."""

import logging
import queue
import threading

import gymnasium as gym
import torch

from isaaclab.devices import (
    Se3Gamepad,
    Se3GamepadCfg,
    Se3Keyboard,
    Se3KeyboardCfg,
    Se3SpaceMouse,
    Se3SpaceMouseCfg,
)
from isaaclab.devices.teleop_device_factory import create_teleop_device
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.utils import parse_env_cfg

import aic_task.tasks  # noqa: F401

if args_cli.enable_pinocchio:
    import isaaclab_tasks.manager_based.locomanipulation.pick_place  # noqa: F401
    import isaaclab_tasks.manager_based.manipulation.pick_place  # noqa: F401

logger = logging.getLogger(__name__)


def main() -> None:
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    env_cfg.env_name = args_cli.task
    if not isinstance(env_cfg, ManagerBasedRLEnvCfg):
        raise ValueError(
            "Teleoperation is only supported for ManagerBasedRLEnv environments. "
            f"Received environment config type: {type(env_cfg).__name__}"
        )
    # env_cfg.terminations.time_out = None  # disabled: causes simulation view crash with some robots
    if "Lift" in args_cli.task:
        env_cfg.commands.object_pose.resampling_time_range = (1.0e9, 1.0e9)
        env_cfg.terminations.object_reached_goal = DoneTerm(
            func=mdp.object_reached_goal
        )

    if args_cli.xr:
        from xr_utils import configure_xr_env, remove_xr_cameras

        env_cfg = configure_xr_env(env_cfg)
        env_cfg = remove_xr_cameras(env_cfg)
        env_cfg.sim.render.antialiasing_mode = "DLSS"

    try:
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        if "Reach" in args_cli.task:
            logger.warning(
                f"The environment '{args_cli.task}' does not support gripper control. The device command will be"
                " ignored."
            )
    except Exception as e:
        logger.error(f"Failed to create environment: {e}")
        simulation_app.close()
        return

    should_reset = False
    teleoperation_active = True

    def reset_recording_instance() -> None:
        nonlocal should_reset
        should_reset = True
        print("Reset triggered - Environment will reset on next step")

    def start_teleoperation() -> None:
        nonlocal teleoperation_active
        teleoperation_active = True
        print("Teleoperation activated")

    def stop_teleoperation() -> None:
        nonlocal teleoperation_active
        teleoperation_active = False
        print("Teleoperation deactivated")

    teleoperation_callbacks: dict[str, Callable[[], None]] = {
        "R": reset_recording_instance,
        "START": start_teleoperation,
        "STOP": stop_teleoperation,
        "RESET": reset_recording_instance,
    }

    if args_cli.xr:
        teleoperation_active = False

    teleop_interface = None
    try:
        if (
            hasattr(env_cfg, "teleop_devices")
            and args_cli.teleop_device in env_cfg.teleop_devices.devices
        ):
            teleop_interface = create_teleop_device(
                args_cli.teleop_device,
                env_cfg.teleop_devices.devices,
                teleoperation_callbacks,
            )
        else:
            logger.warning(
                f"No teleop device '{args_cli.teleop_device}' found in environment config. Creating default."
            )
            sensitivity = args_cli.sensitivity
            if args_cli.teleop_device.lower() == "keyboard":
                teleop_interface = Se3Keyboard(
                    Se3KeyboardCfg(
                        pos_sensitivity=0.025 * sensitivity,
                        rot_sensitivity=0.025 * sensitivity,
                    )
                )
            elif args_cli.teleop_device.lower() == "spacemouse":
                teleop_interface = Se3SpaceMouse(
                    Se3SpaceMouseCfg(
                        pos_sensitivity=0.025 * sensitivity,
                        rot_sensitivity=0.025 * sensitivity,
                    )
                )
            elif args_cli.teleop_device.lower() == "gamepad":
                teleop_interface = Se3Gamepad(
                    Se3GamepadCfg(
                        pos_sensitivity=0.05 * sensitivity,
                        rot_sensitivity=0.05 * sensitivity,
                    )
                )
            else:
                logger.error(f"Unsupported teleop device: {args_cli.teleop_device}")
                env.close()
                simulation_app.close()
                return

            for key, callback in teleoperation_callbacks.items():
                try:
                    teleop_interface.add_callback(key, callback)
                except (ValueError, TypeError) as e:
                    logger.warning(f"Failed to add callback for key {key}: {e}")
    except Exception as e:
        logger.error(f"Failed to create teleop device: {e}")
        env.close()
        simulation_app.close()
        return

    if teleop_interface is None:
        logger.error("Failed to create teleop interface")
        env.close()
        simulation_app.close()
        return

    print(f"Using teleop device: {teleop_interface}")

    env.reset()
    teleop_interface.reset()

    # For XR sessions, start a background stdin reader for terminal hotkeys
    # (hand-tracking devices do not emit START/STOP/RESET messages).
    command_queue: queue.SimpleQueue[str] | None = None
    stop_event: threading.Event | None = None
    if args_cli.xr:
        from xr_utils import spawn_stdin_reader

        command_queue = queue.SimpleQueue()
        stop_event = threading.Event()
        spawn_stdin_reader(command_queue, stop_event)
        print("Teleoperation started. Hotkeys: s=start, p=pause, r=reset, q=quit")
    else:
        print("Teleoperation started. Press 'R' to reset the environment.")

    while simulation_app.is_running():
        if stop_event is not None and stop_event.is_set():
            break

        if command_queue is not None:
            while True:
                try:
                    cmd = command_queue.get_nowait()
                except queue.Empty:
                    break
                if cmd == "s":
                    start_teleoperation()
                elif cmd == "p":
                    stop_teleoperation()
                elif cmd == "r":
                    reset_recording_instance()
                elif cmd == "q":
                    stop_event.set()

        try:
            with torch.inference_mode():
                action = teleop_interface.advance()

                if teleoperation_active:
                    actions = action.repeat(env.num_envs, 1)
                    env.step(actions)
                else:
                    env.sim.render()

                if should_reset:
                    env.reset()
                    teleop_interface.reset()
                    should_reset = False
                    print("Environment reset complete")
        except Exception as e:
            logger.error(f"Error during simulation step: {e}")
            break

    env.close()
    print("Environment closed")


if __name__ == "__main__":
    main()
    simulation_app.close()
