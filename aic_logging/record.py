#!/usr/bin/env python3
"""
AIC External Bag Recorder CLI

Parses YAML recording configuration and launches `ros2 bag record` with the specified
topics, services, actions, and storage settings on the external recording machine.
"""

import argparse
import datetime
import os
import subprocess
import sys
import yaml


def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch ROS 2 bag recording using YAML configuration."
    )
    parser.add_argument(
        "-c",
        "--config",
        default="recording_config.yaml",
        help="Path to the YAML recording configuration file (default: recording_config.yaml)",
    )
    parser.add_argument(
        "-n",
        "--name",
        default="",
        help="Optional suffix tag for the bag output folder name",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default="",
        help="Override output directory specified in YAML config",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the generated ros2 bag record command without executing it",
    )
    return parser.parse_args()


def load_config(config_path):
    if not os.path.exists(config_path):
        print(f"Error: Configuration file '{config_path}' not found.", file=sys.stderr)
        sys.exit(1)

    with open(config_path, "r") as f:
        try:
            return yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(f"Error parsing YAML config '{config_path}': {e}", file=sys.stderr)
            sys.exit(1)


def build_command(config, args):
    cmd = ["ros2", "bag", "record"]

    storage_cfg = config.get("storage", {})
    topics_cfg = config.get("topics", {})
    services_cfg = config.get("services", {})
    actions_cfg = config.get("actions", {})
    adv_cfg = config.get("advanced", {})

    # Storage format
    storage_id = storage_cfg.get("identifier", "mcap")
    cmd.extend(["-s", storage_id])

    # Preset profile
    preset = storage_cfg.get("preset_profile", "")
    if preset:
        cmd.extend(["--storage-preset-profile", preset])

    # Output directory & naming
    base_dir = args.output_dir or storage_cfg.get("output_dir", "/tmp/aic_bags")
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    name_suffix = args.name or "recording"
    bag_name = f"{timestamp}_{name_suffix}"
    output_path = os.path.join(base_dir, bag_name)
    cmd.extend(["-o", output_path])

    # Max size and duration
    max_size = storage_cfg.get("max_bag_size", 0)
    if max_size > 0:
        cmd.extend(["-b", str(max_size)])

    max_dur = storage_cfg.get("max_bag_duration", 0)
    if max_dur > 0:
        cmd.extend(["-d", str(max_dur)])

    max_cache = storage_cfg.get("max_cache_size", 0)
    if max_cache > 0:
        cmd.extend(["--max-cache-size", str(max_cache)])

    # Topics
    if topics_cfg.get("record_all", False):
        cmd.append("--all-topics")
    else:
        inc_topics = topics_cfg.get("include", [])
        if inc_topics:
            cmd.append("--topics")
            cmd.extend(inc_topics)

    exc_topics = topics_cfg.get("exclude", [])
    if exc_topics:
        cmd.append("--exclude-topics")
        cmd.extend(exc_topics)

    # Services
    if services_cfg.get("record_all", False):
        cmd.append("--all-services")
    else:
        inc_services = services_cfg.get("include", [])
        if inc_services:
            cmd.append("--services")
            cmd.extend(inc_services)

    exc_services = services_cfg.get("exclude", [])
    if exc_services:
        cmd.append("--exclude-services")
        cmd.extend(exc_services)

    # Actions
    if actions_cfg.get("record_all", False):
        cmd.append("--all-actions")
    else:
        inc_actions = actions_cfg.get("include", [])
        if inc_actions:
            cmd.append("--actions")
            cmd.extend(inc_actions)

    exc_actions = actions_cfg.get("exclude", [])
    if exc_actions:
        cmd.append("--exclude-actions")
        cmd.extend(exc_actions)

    # Advanced filters
    regex = adv_cfg.get("regex", "")
    if regex:
        cmd.extend(["-e", regex])

    ex_regex = adv_cfg.get("exclude_regex", "")
    if ex_regex:
        cmd.extend(["--exclude-regex", ex_regex])

    node_name = adv_cfg.get("node_name", "")
    if node_name:
        cmd.extend(["--node-name", node_name])

    if adv_cfg.get("include_hidden_topics", False):
        cmd.append("--include-hidden-topics")

    return cmd, output_path


def main():
    args = parse_args()

    # Find config file location relative to script if default
    config_path = args.config
    if not os.path.isabs(config_path) and not os.path.exists(config_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        rel_path = os.path.join(script_dir, config_path)
        if os.path.exists(rel_path):
            config_path = rel_path

    config = load_config(config_path)
    cmd, output_path = build_command(config, args)

    print("==================================================")
    print(" AIC External Bag Recorder")
    print("==================================================")
    print(f"Config File : {os.path.abspath(config_path)}")
    print(f"Output Path : {os.path.abspath(output_path)}")
    print(f"Command     : {' '.join(cmd)}")
    print("==================================================")

    if args.dry_run:
        print("Dry-run mode enabled. Command not executed.")
        return

    # Ensure parent output directory exists
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing recorder: {e}", file=sys.stderr)
        sys.exit(e.returncode)


if __name__ == "__main__":
    main()
