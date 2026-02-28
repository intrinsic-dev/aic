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
Convert AIC world exported from Gazebo into MuJoCo MJCF format.

This script wraps the sdformat_mjcf tool and applies necessary post-processing
fixes to the generated MJCF files.

Usage:
    python3 convert_world.py /tmp/aic.sdf ~/aic_mujoco_world
"""

import sys
import argparse
import subprocess
import os
from pathlib import Path


def convert_sdf_to_mjcf(sdf_path, output_dir):
    """
    Convert an SDF file to MuJoCo MJCF format using sdformat_mjcf.

    Args:
        sdf_path: Path to the input SDF file
        output_dir: Directory where MJCF files will be created
    """
    print(f"Converting SDF world from: {sdf_path}")
    print(f"Output directory: {output_dir}")

    # Ensure output directory exists
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    # Run sdformat_mjcf conversion
    try:
        cmd = ["sdformat_mjcf", sdf_path, "--output-dir", output_dir]
        print(f"\nRunning: {' '.join(cmd)}")
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(result.stdout)
        if result.stderr:
            print("Warnings:", result.stderr)
    except subprocess.CalledProcessError as e:
        print(f"Error during conversion: {e}")
        print(f"stdout: {e.stdout}")
        print(f"stderr: {e.stderr}")
        return False
    except FileNotFoundError:
        print("Error: sdformat_mjcf tool not found.")
        print("Make sure you've built gz-mujoco and sourced the workspace.")
        print("  source ~/ws_aic/install/setup.bash")
        return False

    print("\n✓ Conversion complete!")
    return True


def apply_post_processing_fixes(output_dir):
    """
    Apply manual fixes to the generated MJCF files.

    TODO: Automate common fixes like:
    - Adding shell="0" to specific geometries
    - Adjusting contact parameters
    - Fixing material properties
    """
    print("\n⚠ Manual post-processing required:")
    print('  1. Open the generated MJCF files and add shell="0" to relevant geometries')
    print("  2. Adjust contact/friction parameters if needed")
    print("  3. Verify mesh file paths are correct")
    print("\nSee docs/integration.md for detailed post-processing instructions.")


def main():
    parser = argparse.ArgumentParser(
        description="Convert AIC Gazebo world to MuJoCo MJCF format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  # Convert default exported world
  python3 convert_world.py /tmp/aic.sdf ~/aic_mujoco_world

  # View the result
  python -m mujoco.viewer ~/aic_mujoco_world/scene.xml
        """,
    )
    parser.add_argument(
        "sdf_file",
        type=str,
        help="Path to exported SDF file",
        default="/tmp/aic.sdf",
        nargs="?",
    )
    parser.add_argument(
        "output_dir",
        type=str,
        help="Output directory for MJCF files",
        default="./aic_mujoco_world",
        nargs="?",
    )
    parser.add_argument(
        "--skip-post-process",
        action="store_true",
        help="Skip post-processing instructions",
    )

    args = parser.parse_args()

    # Check if input file exists
    if not os.path.exists(args.sdf_file):
        print(f"Error: SDF file not found: {args.sdf_file}")
        print("\nGenerate the world file using:")
        print("  ros2 launch aic_bringup aic_gz_bringup.launch.py \\")
        print("    spawn_task_board:=true spawn_cable:=true")
        sys.exit(1)

    # Convert SDF to MJCF
    success = convert_sdf_to_mjcf(args.sdf_file, args.output_dir)

    if not success:
        sys.exit(1)

    # Show post-processing instructions
    if not args.skip_post_process:
        apply_post_processing_fixes(args.output_dir)

    print(f"\n✓ MuJoCo world ready at: {args.output_dir}/scene.xml")
    print(f"\nTo view: python -m mujoco.viewer {args.output_dir}/scene.xml")


if __name__ == "__main__":
    main()
