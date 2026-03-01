# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to record demonstrations via human teleoperation.

Recorded demonstrations are stored as episodes in an HDF5 file.

required arguments:
    --task                    Name of the task.

optional arguments:
    -h, --help                Show this help message and exit
    --teleop_device           Device for interacting with environment. (default: keyboard)
    --dataset_file            File path to export recorded demos. (default: ./datasets/dataset.hdf5)
    --step_hz                 Environment stepping rate in Hz. (default: 30)
    --num_demos               Number of demonstrations to record. 0 = infinite. (default: 0)
    --num_success_steps       Consecutive success steps to conclude a demo. (default: 10)
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import contextlib

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Record demonstrations for Isaac Lab environments."
)
parser.add_argument("--task", type=str, required=True, help="Name of the task.")
parser.add_argument(
    "--teleop_device",
    type=str,
    default="keyboard",
    help="Teleop device. Built-ins: keyboard, spacemouse, gamepad.",
)
parser.add_argument(
    "--dataset_file",
    type=str,
    default="./datasets/dataset.hdf5",
    help="File path to export recorded demos.",
)
parser.add_argument(
    "--step_hz", type=int, default=30, help="Environment stepping rate in Hz."
)
parser.add_argument(
    "--num_demos",
    type=int,
    default=0,
    help="Number of demonstrations to record. Set to 0 for infinite.",
)
parser.add_argument(
    "--num_success_steps",
    type=int,
    default=10,
    help="Consecutive steps with task success to conclude a demo as successful.",
)
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import logging
import os
import time
from collections.abc import Callable

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
from isaaclab.envs import DirectRLEnvCfg, ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg
from isaaclab.managers import DatasetExportMode

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

import aic_task.tasks  # noqa: F401

logger = logging.getLogger(__name__)


class RateLimiter:
    """Convenience class for enforcing rates in loops."""

    def __init__(self, hz: int):
        self.hz = hz
        self.last_time = time.time()
        self.sleep_duration = 1.0 / hz
        self.render_period = min(0.033, self.sleep_duration)

    def sleep(self, env: gym.Env):
        next_wakeup_time = self.last_time + self.sleep_duration
        while time.time() < next_wakeup_time:
            time.sleep(self.render_period)
            env.sim.render()

        self.last_time = self.last_time + self.sleep_duration
        if self.last_time < time.time():
            while self.last_time < time.time():
                self.last_time += self.sleep_duration


def main() -> None:
    """Collect demonstrations from the environment using teleop interfaces."""

    rate_limiter = RateLimiter(args_cli.step_hz)

    # Output directories
    output_dir = os.path.dirname(args_cli.dataset_file)
    output_file_name = os.path.splitext(os.path.basename(args_cli.dataset_file))[0]
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    # Parse environment config
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=1,
        use_fabric=not args_cli.disable_fabric,
    )
    env_cfg.env_name = args_cli.task.split(":")[-1]

    # Extract success termination (used to mark demos, not to end episodes)
    success_term = None
    if hasattr(env_cfg.terminations, "success"):
        success_term = env_cfg.terminations.success
        env_cfg.terminations.success = None
    else:
        logger.warning(
            "No success termination term found. Cannot mark demos as successful."
        )

    # Run indefinitely until goal or manual reset
    env_cfg.terminations.time_out = None
    env_cfg.observations.policy.concatenate_terms = False

    # Configure recorder
    env_cfg.recorders = ActionStateRecorderManagerCfg()
    env_cfg.recorders.dataset_export_dir_path = output_dir
    env_cfg.recorders.dataset_filename = output_file_name
    env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY

    # Create environment
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # State for the recording loop
    current_recorded_demo_count = 0
    success_step_count = 0
    should_reset = False
    running = True

    def reset_recording():
        nonlocal should_reset
        should_reset = True
        print("Recording reset requested")

    def start_recording():
        nonlocal running
        running = True
        print("Recording started")

    def stop_recording():
        nonlocal running
        running = False
        print("Recording paused")

    callbacks: dict[str, Callable] = {
        "R": reset_recording,
        "START": start_recording,
        "STOP": stop_recording,
        "RESET": reset_recording,
    }

    # Set up teleop device
    teleop_interface = None
    if (
        hasattr(env_cfg, "teleop_devices")
        and args_cli.teleop_device in env_cfg.teleop_devices.devices
    ):
        teleop_interface = create_teleop_device(
            args_cli.teleop_device, env_cfg.teleop_devices.devices, callbacks
        )
    else:
        logger.warning(
            f"No teleop device '{args_cli.teleop_device}' in env config. Creating default."
        )
        sensitivity = 1.0
        if args_cli.teleop_device.lower() == "keyboard":
            teleop_interface = Se3Keyboard(
                Se3KeyboardCfg(
                    pos_sensitivity=0.05 * sensitivity,
                    rot_sensitivity=0.05 * sensitivity,
                )
            )
        elif args_cli.teleop_device.lower() == "spacemouse":
            teleop_interface = Se3SpaceMouse(
                Se3SpaceMouseCfg(
                    pos_sensitivity=0.05 * sensitivity,
                    rot_sensitivity=0.05 * sensitivity,
                )
            )
        elif args_cli.teleop_device.lower() == "gamepad":
            teleop_interface = Se3Gamepad(
                Se3GamepadCfg(
                    pos_sensitivity=0.1 * sensitivity, rot_sensitivity=0.1 * sensitivity
                )
            )
        else:
            logger.error(f"Unsupported teleop device: {args_cli.teleop_device}")
            env.close()
            simulation_app.close()
            return

        for key, cb in callbacks.items():
            try:
                teleop_interface.add_callback(key, cb)
            except (ValueError, TypeError):
                pass

    if teleop_interface is None:
        logger.error("Failed to create teleop interface")
        env.close()
        simulation_app.close()
        return

    teleop_interface.add_callback("R", reset_recording)

    # Reset before starting
    env.sim.reset()
    env.reset()
    teleop_interface.reset()

    print(f"Using teleop device: {teleop_interface}")
    print("Recording demonstrations. Press 'R' to reset/discard current episode.")
    print(f"Recorded 0 successful demonstrations so far.")

    with contextlib.suppress(KeyboardInterrupt) and torch.inference_mode():
        while simulation_app.is_running():
            action = teleop_interface.advance()
            actions = action.repeat(env.num_envs, 1)

            if running:
                env.step(actions)
            else:
                env.sim.render()

            # Check success condition
            if success_term is not None:
                if bool(success_term.func(env, **success_term.params)[0]):
                    success_step_count += 1
                    if success_step_count >= args_cli.num_success_steps:
                        env.recorder_manager.record_pre_reset(
                            [0], force_export_or_skip=False
                        )
                        env.recorder_manager.set_success_to_episodes(
                            [0],
                            torch.tensor([[True]], dtype=torch.bool, device=env.device),
                        )
                        env.recorder_manager.export_episodes([0])
                        print("Success condition met! Episode recorded.")
                        should_reset = True
                else:
                    success_step_count = 0

            # Update demo count
            if (
                env.recorder_manager.exported_successful_episode_count
                > current_recorded_demo_count
            ):
                current_recorded_demo_count = (
                    env.recorder_manager.exported_successful_episode_count
                )
                print(
                    f"Recorded {current_recorded_demo_count} successful demonstrations."
                )

            # Check if target reached
            if (
                args_cli.num_demos > 0
                and current_recorded_demo_count >= args_cli.num_demos
            ):
                print(
                    f"All {current_recorded_demo_count} demonstrations recorded. Exiting."
                )
                target_time = time.time() + 0.8
                while time.time() < target_time:
                    rate_limiter.sleep(env)
                break

            # Handle reset
            if should_reset:
                print("Resetting environment...")
                env.sim.reset()
                env.recorder_manager.reset()
                env.reset()
                success_step_count = 0
                should_reset = False

            if env.sim.is_stopped():
                break

            rate_limiter.sleep(env)

    env.close()
    print(
        f"Recording session completed with {current_recorded_demo_count} successful demonstrations"
    )
    print(f"Demonstrations saved to: {args_cli.dataset_file}")


if __name__ == "__main__":
    main()
    simulation_app.close()
