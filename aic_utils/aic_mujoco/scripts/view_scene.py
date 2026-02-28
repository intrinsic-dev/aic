#!/usr/bin/env python3

#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

"""
Launch MuJoCo viewer with an MJCF scene file.

This script loads a MuJoCo XML scene and launches the interactive viewer.
Useful when running in the pixi environment where drag-and-drop is otherwise needed.

Usage:
    python3 view_scene.py <path_to_scene.xml>
    python3 view_scene.py ~/aic_mujoco_world/scene.xml
"""

import sys
import argparse
from pathlib import Path

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("Error: MuJoCo Python package not found.")
    print("Install with: pip install mujoco")
    print("Or in pixi environment: pixi add mujoco")
    sys.exit(1)


def launch_viewer(scene_path: str):
    """
    Load MuJoCo scene and launch interactive viewer.

    Args:
        scene_path: Path to the MJCF scene XML file
    """
    scene_path = Path(scene_path).expanduser().resolve()

    if not scene_path.exists():
        print(f"Error: Scene file not found: {scene_path}")
        sys.exit(1)

    print(f"Loading MuJoCo scene: {scene_path}")

    try:
        # Load the model
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)

        print(f"Scene loaded successfully!")
        print(f"  Bodies: {model.nbody}")
        print(f"  Joints: {model.njnt}")
        print(f"  DOFs: {model.nv}")
        print(f"  Actuators: {model.nu}")
        print("\nLaunching viewer (paused by default)...")
        print("Controls:")
        print("  - Left mouse: rotate")
        print("  - Right mouse: zoom")
        print("  - Middle mouse: pan")
        print("  - Double click: select body")
        print("  - Space: pause/resume")
        print("  - Backspace: reset")

        # Launch interactive viewer in paused mode
        # The viewer starts paused by default
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Start paused
            viewer.sync()
            while viewer.is_running():
                viewer.sync()

    except Exception as e:
        print(f"Error loading scene: {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Launch MuJoCo viewer with an MJCF scene file",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 view_scene.py scene.xml
  python3 view_scene.py ~/aic_mujoco_world/scene.xml
  python3 view_scene.py ~/ws_aic/aic_world/scene.xml
        """,
    )
    parser.add_argument("scene", help="Path to the MuJoCo MJCF scene XML file")

    args = parser.parse_args()
    launch_viewer(args.scene)


if __name__ == "__main__":
    main()
