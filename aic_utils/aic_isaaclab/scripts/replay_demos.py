# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to replay recorded demonstrations in Isaac Lab environments.

required arguments:
    --dataset_file            HDF5 dataset file to replay.

optional arguments:
    -h, --help                Show this help message and exit
    --task                    Force a specific task (overrides dataset metadata).
    --num_envs                Number of environments to replay episodes. (default: 1)
    --select_episodes         List of episode indices to replay. Empty = all. (default: [])
    --validate_states         Compare dataset states vs runtime states. Only with --num_envs 1.
    --validate_success_rate   Check success rate using task termination criteria.
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Replay demonstrations in Isaac Lab environments."
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to replay episodes."
)
parser.add_argument(
    "--task", type=str, default=None, help="Force to use the specified task."
)
parser.add_argument(
    "--select_episodes",
    type=int,
    nargs="+",
    default=[],
    help="Episode indices to replay. Empty = replay all.",
)
parser.add_argument(
    "--dataset_file",
    type=str,
    default="datasets/dataset.hdf5",
    help="Dataset file to be replayed.",
)
parser.add_argument(
    "--validate_states",
    action="store_true",
    default=False,
    help="Validate states match between dataset and runtime. Only valid with --num_envs 1.",
)
parser.add_argument(
    "--validate_success_rate",
    action="store_true",
    default=False,
    help="Validate replay success rate using task termination criteria.",
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

import contextlib
import os

import gymnasium as gym
import torch

from isaaclab.devices import Se3Keyboard, Se3KeyboardCfg
from isaaclab.utils.datasets import EpisodeData, HDF5DatasetFileHandler

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

import aic_task.tasks  # noqa: F401

is_paused = False


def play_cb():
    global is_paused
    is_paused = False


def pause_cb():
    global is_paused
    is_paused = True


def compare_states(
    state_from_dataset, runtime_state, runtime_env_index
) -> tuple[bool, str]:
    """Compare states from dataset and runtime.

    Returns:
        Tuple of (states_matched, log_message).
    """
    states_matched = True
    output_log = ""
    for asset_type in ["articulation", "rigid_object"]:
        for asset_name in runtime_state[asset_type].keys():
            for state_name in runtime_state[asset_type][asset_name].keys():
                runtime_asset_state = runtime_state[asset_type][asset_name][state_name][
                    runtime_env_index
                ]
                dataset_asset_state = state_from_dataset[asset_type][asset_name][
                    state_name
                ]
                if len(dataset_asset_state) != len(runtime_asset_state):
                    raise ValueError(
                        f"State shape of {state_name} for asset {asset_name} don't match"
                    )
                for i in range(len(dataset_asset_state)):
                    if abs(dataset_asset_state[i] - runtime_asset_state[i]) > 0.01:
                        states_matched = False
                        output_log += f'\tState ["{asset_type}"]["{asset_name}"]["{state_name}"][{i}] mismatch\r\n'
                        output_log += f"\t  Dataset:\t{dataset_asset_state[i]}\r\n"
                        output_log += f"\t  Runtime: \t{runtime_asset_state[i]}\r\n"
    return states_matched, output_log


def main():
    """Replay episodes loaded from a file."""
    global is_paused

    if not os.path.exists(args_cli.dataset_file):
        raise FileNotFoundError(
            f"The dataset file {args_cli.dataset_file} does not exist."
        )

    dataset_file_handler = HDF5DatasetFileHandler()
    dataset_file_handler.open(args_cli.dataset_file)
    env_name = dataset_file_handler.get_env_name()
    episode_count = dataset_file_handler.get_num_episodes()

    if episode_count == 0:
        print("No episodes found in the dataset.")
        return

    episode_indices_to_replay = list(args_cli.select_episodes)
    if len(episode_indices_to_replay) == 0:
        episode_indices_to_replay = list(range(episode_count))

    if args_cli.task is not None:
        env_name = args_cli.task.split(":")[-1]
    if env_name is None:
        raise ValueError("Task/env name was not specified nor found in the dataset.")

    num_envs = args_cli.num_envs

    env_cfg = parse_env_cfg(
        env_name,
        device=args_cli.device,
        num_envs=num_envs,
        use_fabric=not args_cli.disable_fabric,
    )

    # Extract success termination for validation
    success_term = None
    if args_cli.validate_success_rate:
        if hasattr(env_cfg.terminations, "success"):
            success_term = env_cfg.terminations.success
            env_cfg.terminations.success = None
        else:
            print("No success termination term found. Cannot validate success rate.")

    # Disable recorders and terminations for replay
    env_cfg.recorders = {}
    env_cfg.terminations = {}

    env = gym.make(args_cli.task or env_name, cfg=env_cfg).unwrapped

    teleop_interface = Se3Keyboard(
        Se3KeyboardCfg(pos_sensitivity=0.1, rot_sensitivity=0.1)
    )
    teleop_interface.add_callback("N", play_cb)
    teleop_interface.add_callback("B", pause_cb)
    print('Press "B" to pause and "N" to resume the replayed actions.')

    state_validation_enabled = False
    if args_cli.validate_states and num_envs == 1:
        state_validation_enabled = True
    elif args_cli.validate_states and num_envs > 1:
        print(
            "Warning: State validation is only supported with a single environment. Skipping."
        )

    if hasattr(env_cfg, "idle_action"):
        idle_action = env_cfg.idle_action.repeat(num_envs, 1)
    else:
        idle_action = torch.zeros(env.action_space.shape)

    env.reset()
    teleop_interface.reset()

    episode_names = list(dataset_file_handler.get_episode_names())
    replayed_episode_count = 0
    recorded_episode_count = 0
    current_episode_indices = [None] * num_envs
    failed_demo_ids = []

    with contextlib.suppress(KeyboardInterrupt) and torch.inference_mode():
        while simulation_app.is_running() and not simulation_app.is_exiting():
            env_episode_data_map = {index: EpisodeData() for index in range(num_envs)}
            first_loop = True
            has_next_action = True
            episode_ended = [False] * num_envs

            while has_next_action:
                actions = idle_action
                has_next_action = False

                for env_id in range(num_envs):
                    env_next_action = env_episode_data_map[env_id].get_next_action()

                    if env_next_action is None:
                        # Check success after episode completes
                        if (
                            success_term is not None
                            and current_episode_indices[env_id] is not None
                            and not episode_ended[env_id]
                        ):
                            if bool(
                                success_term.func(env, **success_term.params)[env_id]
                            ):
                                recorded_episode_count += 1
                                s = "s" if recorded_episode_count > 1 else ""
                                print(
                                    f"Successfully replayed {recorded_episode_count} episode{s}"
                                    f" out of {replayed_episode_count} demos."
                                )
                            else:
                                if (
                                    current_episode_indices[env_id] is not None
                                    and current_episode_indices[env_id]
                                    not in failed_demo_ids
                                ):
                                    failed_demo_ids.append(
                                        current_episode_indices[env_id]
                                    )
                            episode_ended[env_id] = True

                        # Load next episode
                        next_episode_index = None
                        while episode_indices_to_replay:
                            next_episode_index = episode_indices_to_replay.pop(0)
                            if next_episode_index < episode_count:
                                episode_ended[env_id] = False
                                break
                            next_episode_index = None

                        if next_episode_index is not None:
                            replayed_episode_count += 1
                            current_episode_indices[env_id] = next_episode_index
                            print(
                                f"{replayed_episode_count:4}: Loading #{next_episode_index} episode to env_{env_id}"
                            )
                            episode_data = dataset_file_handler.load_episode(
                                episode_names[next_episode_index], env.device
                            )
                            env_episode_data_map[env_id] = episode_data
                            initial_state = episode_data.get_initial_state()
                            env.reset_to(
                                initial_state,
                                torch.tensor([env_id], device=env.device),
                                is_relative=True,
                            )
                            env_next_action = env_episode_data_map[
                                env_id
                            ].get_next_action()
                            has_next_action = True
                        else:
                            continue
                    else:
                        has_next_action = True

                    actions[env_id] = env_next_action

                if first_loop:
                    first_loop = False
                else:
                    while is_paused:
                        env.sim.render()
                        continue

                env.step(actions)

                if state_validation_enabled:
                    state_from_dataset = env_episode_data_map[0].get_next_state()
                    if state_from_dataset is not None:
                        print(
                            f"Validating states at action-index: {env_episode_data_map[0].next_state_index - 1:4}",
                            end="",
                        )
                        current_runtime_state = env.scene.get_state(is_relative=True)
                        states_matched, comparison_log = compare_states(
                            state_from_dataset, current_runtime_state, 0
                        )
                        if states_matched:
                            print("\t- matched.")
                        else:
                            print("\t- mismatched.")
                            print(comparison_log)
            break

    s = "s" if replayed_episode_count > 1 else ""
    print(f"Finished replaying {replayed_episode_count} episode{s}.")

    if success_term is not None:
        print(
            f"Successfully replayed: {recorded_episode_count}/{replayed_episode_count}"
        )
        if failed_demo_ids:
            print(f"\nFailed demo IDs ({len(failed_demo_ids)} total):")
            print(f"  {sorted(failed_demo_ids)}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
