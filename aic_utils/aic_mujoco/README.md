# AIC MuJoCo Integration

This package provides documentation, scripts, and utilities for training policies in MuJoCo simulation using the AI for Industry Challenge (AIC) environment.

## Overview

MuJoCo (Multi-Joint dynamics with Contact) is a physics engine designed for research and development in robotics, biomechanics, graphics and animation. In collaboration with **Google DeepMind**, this integration enables participants to:

- Convert Gazebo SDF worlds to MuJoCo MJCF format using `sdformat_mjcf`
- Load the AIC task board and robot from exported Gazebo worlds (`/tmp/aic.sdf`)
- Train policies using MuJoCo's fast physics simulation
- Control the UR5e robot using the same `aic_controller` interface
- Leverage domain randomization across multiple simulators (Gazebo, MuJoCo, IsaacLab)

## Workflow Summary

1. **Export from Gazebo**: Launch `aic_gz_bringup` with desired domain randomization parameters
2. **Automatic SDF Export**: Gazebo saves complete world to `/tmp/aic.sdf`
3. **Convert to MJCF**: Use `sdformat_mjcf` tool to convert SDF → MuJoCo XML + mesh assets
4. **Manual Refinements**: Apply necessary MJCF fixes (e.g., `shell="0"` for certain geometries)
5. **Load in MuJoCo**: Open the converted `scene.xml` in MuJoCo viewer or simulation
6. **Train Policy**: Use same control interface as Gazebo
7. **Validate**: Test trained policy back in Gazebo before submission

## Prerequisites

- **Operating System:** Ubuntu 24.04 (Noble Numbat)
- **ROS 2:** ROS 2 Kilted Kaiju
- **Existing AIC Workspace:** Follow the [Getting Started](../../docs/getting_started.md) guide to set up your base workspace

> **Note:** MuJoCo integration requires a native Ubuntu 24.04 installation with ROS 2 built from source or installed via apt. The pixi-based workflow does not support the necessary ROS 2 Control packages.

## Installation

### 1. Import MuJoCo Dependencies

From your ROS 2 workspace, import the required repositories:

```bash
cd ~/ws_aic/src
vcs import < aic/aic_utils/aic_mujoco/mujoco.repos
```

This adds:
- `mujoco_vendor` (v0.0.6) - ROS 2 wrapper for MuJoCo 3.x
- `mujoco_ros2_control` - Integration between MuJoCo and ros2_control
- `gz-mujoco` (with `sdformat_mjcf` tool) - Converts Gazebo SDF files to MuJoCo MJCF format

### 2. Install Dependencies

```bash
cd ~/ws_aic
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

The `mujoco_vendor` package requires the `MUJOCO_DIR` environment variable during build:

```bash
source /opt/ros/kilted/setup.bash

# Build mujoco_vendor first to install MuJoCo
colcon build --packages-select mujoco_vendor

# Source to get MUJOCO_DIR in environment
source install/setup.bash

# Build mujoco_ros2_control and other packages
colcon build

# Source the complete workspace
source install/setup.bash
```

### 4. Verify Installation

```bash
# Check MuJoCo installation
echo $MUJOCO_DIR
# Should output something like:
# /home/user/ws_aic/install/mujoco_vendor/share/mujoco_vendor/mujoco-3.x.x

# Check sdformat_mjcf tool
which sdformat_mjcf
# Should output:
# /home/user/ws_aic/install/gz-mujoco/bin/sdformat_mjcf
```

## Usage

### Exporting the AIC World from Gazebo

First, generate the world state from Gazebo with your desired configuration:

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'

# Example: Spawn task board with cable
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  spawn_task_board:=true \
  task_board_x:=0.3 task_board_y:=-0.1 task_board_z:=1.2 \
  spawn_cable:=true \
  cable_type:=sfp_sc_cable \
  attach_cable_to_gripper:=true \
  ground_truth:=true

# The world is automatically exported to /tmp/aic.sdf
```

### Converting SDF to MJCF

Use the `sdformat_mjcf` tool to convert the exported SDF file to MuJoCo format:

```bash
source ~/ws_aic/install/setup.bash

# Convert SDF to MJCF (creates MJCF XML files and extracts mesh assets)
sdformat_mjcf /tmp/aic.sdf --output-dir ~/aic_mujoco_world

# Or use the convenience script:
python3 aic_utils/aic_mujoco/scripts/convert_world.py /tmp/aic.sdf ~/aic_mujoco_world
```

This creates:
- `scene.xml` - Main MuJoCo scene file
- `aic_robot.xml` - Robot model
- `*.stl` files - Extracted mesh geometry
- `aic_world.xml` - World configuration

### Manual MJCF Refinements

After conversion, some manual fixes may be needed:

1. **Shell geometries**: Set `shell="0"` for certain mesh geometries to avoid rendering artifacts
2. **Contact parameters**: Adjust friction, damping for cable behavior
3. **Visual refinements**: Material colors, textures

These refinements will be automated in future releases.

### Loading in MuJoCo

```bash
# Open in MuJoCo viewer
python -m mujoco.viewer ~/aic_mujoco_world/scene.xml

# Or use MuJoCo simulate binary
simulate ~/aic_mujoco_world/scene.xml
```

## Training with MuJoCo

MuJoCo's integration with ros2_control allows you to use the same `aic_controller` interface as in Gazebo, ensuring your policy code remains simulator-agnostic.

Key benefits for training:
- **Faster simulation** than Gazebo for large-scale training
- **Domain randomization** across Gazebo, MuJoCo, and IsaacLab
- **Same control interface** as evaluation environment

## Directory Structure

```
aic_mujoco/
├── README.md              # This file
├── mujoco.repos          # VCS repositories for MuJoCo dependencies
├── scripts/              # Utility scripts (to be added)
├── examples/             # Example training setups (to be added)
└── docs/                 # Additional documentation (to be added)
```

## Tested Configuration

This integration has been tested with:
- **mujoco_vendor:** v0.0.6 (MuJoCo 3.x)
- **mujoco_ros2_control:** commit `c811dcca2039cf6a88af3077e3f3dbbbd9c37a66`
- **gz-mujoco:** branch `fix_mesh_uri_handling`
- **ROS 2:** Kilted Kaiju
- **Ubuntu:** 24.04 LTS

## Known Limitations

- **SDF Conversion**: Not all Gazebo features convert perfectly (e.g., some plugins, advanced materials)
- **Manual Fixes Required**: Some MJCF refinements needed post-conversion (e.g., `shell="0"` for geometries)
- **Cable Physics**: Deformable cable behavior differs from Gazebo - intentional for domain randomization
- **Mesh URIs**: Ensure all mesh paths are resolved correctly during conversion
- **Contact Models**: May need tuning for insertion tasks

## Future Automation

Planned improvements to automate the workflow:
- Automated post-conversion MJCF fixes
- Pre-configured contact parameters for cable insertion
- One-command conversion script
- Docker container with complete MuJoCo training environment
- ROS 2 launch files for MuJoCo simulation with aic_controller

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [mujoco_ros2_control GitHub](https://github.com/ros-controls/mujoco_ros2_control)
- [AIC Getting Started Guide](../../docs/getting_started.md)
- [AIC Scene Description](../../docs/scene_description.md)

## Contributing

This is an experimental integration. Contributions, bug reports, and feedback are welcome. Please open issues in the main AIC repository.

## Next Steps

After setting up MuJoCo:
1. Generate diverse training scenarios by varying launch parameters in Gazebo
2. Export each configuration to different SDF files
3. Load and train policies in MuJoCo
4. Test your trained policy back in Gazebo for sim-to-sim validation
5. Submit your containerized policy for evaluation
