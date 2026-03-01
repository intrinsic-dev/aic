# AIC MuJoCo Integration

This package provides documentation, scripts, and utilities for loading the AI for Industry Challenge (AIC) environment in MuJoCo.

## Overview

[MuJoCo](https://mujoco.org/) is a physics engine designed for research and development in robotics, biomechanics, graphics and animation. In collaboration with **Google DeepMind**, this integration enables participants to:

- Convert Gazebo SDF worlds to MuJoCo MJCF format using `sdformat_mjcf`
- Load the AIC task board and robot from exported Gazebo worlds (`/tmp/aic.sdf`)
- Access camera images, joint states, FT sensor data, and command the simulated robot over the same ROS topics
- Collect data and run policies unchanged between Gazebo and MuJoCo



## Setting up Mujoco with ROS 2 Control

![](../../../media/wave_arm_policy_mujoco.gif)

MuJoCo's integration with `ros2_control` allows you to control the UR5e robot using the same `aic_controller` interface as in Gazebo, ensuring your policy code remains simulator-agnostic.

### Additional Installation Steps

#### 1. Import MuJoCo Dependencies

From your ROS 2 workspace, import the required repositories:

```bash
cd ~/ws_aic/src
vcs import < aic/aic_utils/aic_mujoco/mujoco.repos
```

This adds:
- `mujoco_vendor` (v0.0.6) - ROS 2 wrapper for MuJoCo 3.x with plugins (elasticity, actuator, sensor, SDF)
- `mujoco_ros2_control` - Integration between MuJoCo and ros2_control
- `gz-mujoco` (with `sdformat_mjcf` tool) - Converts Gazebo SDF files to MuJoCo MJCF format

#### 2. Install Dependencies

Install dependencies for the newly imported MuJoCo packages:

```bash
cd ~/ws_aic
rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev"
```

#### 3. Build the Workspace

With the package dependencies properly configured, building should work automatically:

```bash
cd ~/ws_aic
source /opt/ros/kilted/setup.bash

# Build all packages (including aic_mujoco)
GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install --packages-ignore lerobot_robot_aic
```

### 4. Verify Installation

```bash
# Source the workspace (if not already done)
source ~/ws_aic/install/setup.bash

# Check MUJOCO_DIR is automatically set by the environment hook
echo $MUJOCO_DIR
# Should output something like:
# /home/user/ws_aic/install/opt/mujoco_vendor

# Check MUJOCO_PLUGIN_PATH is set (this is how MuJoCo finds plugins)
echo $MUJOCO_PLUGIN_PATH
# Should output something like:
# /home/user/ws_aic/install/opt/mujoco_vendor/lib

# Check MuJoCo installation directory
ls $MUJOCO_DIR
# Should show: bin, include, lib, share, simulate directories

# Check that plugin libraries are installed
ls $MUJOCO_DIR/lib/*.so
# Should show: libelasticity.so, lipython3 ~/intrinsic_ws/src/aic/aic_utils/aic_mujoco/scripts/add_cable_plugin.py --input ~/intrinsic_ws/src/aic/aic_utils/aic_mujoco/mjcf/aic_world.xml --output ~/intrinsic_ws/src/aic/aic_utils/aic_mujoco/mjcf/aic_world.xml --robot_output ~/intrinsic_ws/src/aic/aic_utils/aic_mujoco/mjcf/aic_robot.xml --scene_output ~/intrinsic_ws/src/aic/aic_utils/aic_mujoco/mjcf/scene.xml

bactuator.so, libsensor.so, libsdf_plugin.so, libmujoco.so*

# Verify MuJoCo simulate binary works
which simulate
# Should output:
# /home/user/ws_aic/install/opt/mujoco_vendor/bin/simulate
```

> **⚠️ Important:** If you have a previous MuJoCo installation, it may conflict with `mujoco_vendor`. Check for and remove any existing `MUJOCO_PATH`, `MUJOCO_PLUGIN_PATH`, or `MUJOCO_DIR` environment variables from your shell configuration (`~/.bashrc`, `~/.zshrc`, etc.) before building. After cleaning the environment, restart your shell and rebuild the workspace:
> ```bash
> # Check for conflicting environment variables
> env | grep MUJOCO
>
> # If you see MUJOCO_PATH or MUJOCO_PLUGIN_PATH pointing to a different location,
> # remove those exports from ~/.bashrc (or ~/.zshrc) and restart shell
>
> # Then rebuild mujoco_vendor
> cd ~/ws_aic
> colcon build --packages-select mujoco_vendor --cmake-clean-cache
> source install/setup.bash
>
> # Verify the correct MUJOCO_PLUGIN_PATH is set
> echo $MUJOCO_PLUGIN_PATH
> # Should point to: /home/user/ws_aic/install/opt/mujoco_vendor/lib
> ```


## Workflow

> **⚠️ Important:** Please ensure you have completed the [Additional Installation Steps](#additional-installation-steps) below before starting this workflow.

### 1. Export from Gazebo

- Launch `aic_gz_bringup` with your desired domain randomization parameters. For example: 
```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py spawn_task_board:=true spawn_cable:=true   cable_type:=sfp_sc_cable   attach_cable_to_gripper:=true   ground_truth:=true
```
- Gazebo will export the world to `/tmp/aic.sdf`.

See [Scene Description](../../docs/scene_description.md) for more details.

### 2. Install `sdformat_mjcf` Dependencies

The `sdf2mjcf` CLI tool (provided by the `sdformat_mjcf` Python package in `gz-mujoco`) requires Python bindings for SDFormat and Gazebo Math that are **not** resolved by `rosdep`. Install them from the OSRF Gazebo apt repository:

```bash
# Add the OSRF Gazebo stable apt repository (if not already added)
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

# Install required Python bindings
sudo apt install -y python3-sdformat16 python3-gz-math9
```

Verify the bindings are importable:

```bash
python3 -c "import sdformat16; print('sdformat OK')"
python3 -c "from gz.math9 import Vector3d; print('gz.math OK')"
```

If the workspace was already built before installing these packages, rebuild `sdformat_mjcf`:

```bash
cd ~/ws_aic
source /opt/ros/kilted/setup.bash
colcon build --packages-select sdformat_mjcf
source install/setup.bash
```

### 3. Fix Exported SDF

The exported `/tmp/aic.sdf` contains two known URI corruption issues that must be fixed before conversion.

#### Issue 1: `<urdf-string>` in mesh URIs

When models are spawned from URDF strings (via `ros_gz_sim create -string`), the SDFormat parser uses the placeholder path `<urdf-string>` as the file source. On world export, this leaks into mesh URIs as `file://<urdf-string>/model://...`, which breaks XML parsing because `<urdf-string>` is interpreted as an XML tag.

```bash
# Fix corrupted model:// URIs
sed -i 's|file://<urdf-string>/model://|model://|g' /tmp/aic.sdf
```

#### Issue 2: Broken relative mesh URIs

Some included models (SC Plug, LC Plug, SFP Module) use relative mesh URIs (e.g., `<uri>sc_plug_visual.glb</uri>`). When the world is exported, these lose their model-relative context and become root-path URIs like `file:///sc_plug_visual.glb`, which point to nonexistent files.

```bash
# Fix broken mesh URIs by pointing to the actual files in aic_assets
sed -i 's|file:///lc_plug_visual.glb|model://LC Plug/lc_plug_visual.glb|g' /tmp/aic.sdf
sed -i 's|file:///sc_plug_visual.glb|model://SC Plug/sc_plug_visual.glb|g' /tmp/aic.sdf
sed -i 's|file:///sfp_module_visual.glb|model://SFP Module/sfp_module_visual.glb|g' /tmp/aic.sdf
```

> **Note:** These issues originate in the SDFormat library's handling of string-parsed URDFs and relative URIs during world save. They will occur every time you re-export the world from Gazebo.

### 4. Convert SDF to MJCF

- Use the `sdf2mjcf` CLI tool to convert the fixed `/tmp/aic.sdf` to MJCF format:
  ```bash
  source ~/ws_aic/install/setup.bash
  mkdir -p ~/aic_mujoco_world
  sdf2mjcf /tmp/aic.sdf ~/aic_mujoco_world/aic_world.xml
  ```
- This generates MJCF XML files and mesh assets in `~/aic_mujoco_world`.

### 5. Organize MJCF Files

> **Note:** The `sdformat_mjcf` tool generates several XML files. Most of these have already been **manually corrected and committed** to the `mjcf` folder inside `aic_mujoco` (e.g., `scene.xml`, `aic_robot.xml`, `aic_world.xml`). You only need to overwrite these XML files if you are updating the scene or robot structure.

- However, you **must always** copy or symlink the generated mesh assets (`.obj` and `.png` files) from `~/aic_mujoco_world` into the `mjcf` folder so MuJoCo can find them.
  ```bash
  cp ~/aic_mujoco_world/* ~/ws_aic/src/aic/aic_utils/aic_mujoco/mjcf
  ```

### 6. Generate Final MJCF Files

- To split and refine the MJCF files, use the `add_cable_plugin.py` script. Make sure you run this without sourcing the ROS2 workspace in new terminal:
  ```bash
  cd ~/ws_aic/src/aic/aic_utils/aic_mujoco/
  python3 scripts/add_cable_plugin.py --input mjcf/aic_world.xml --output mjcf/aic_world.xml --robot_output mjcf/aic_robot.xml --scene_output mjcf/scene.xml
  cd ~/ws_aic && colcon build --packages-select aic_mujoco
  ```
  - `--input`: Path to the initial MJCF world file (usually `aic_world.xml`).
  - `--output`: Path for the final world file (`aic_world.xml`).
  - `--robot_output`: Path for the robot-only file (`aic_robot.xml`).
  - `--scene_output`: Path for the scene file (`scene.xml`).



### 7. Load in MuJoCo:

#### Using pixi environment

The Python viewer starts in **paused mode by default**. Press Space to start/pause simulation.

```bash
# Enter pixi shell
pixi shell

# Option 1: Launch empty viewer (then drag and drop scene.xml into the window)
python -m mujoco.viewer

# Option 2: Use the provided convenience script (starts paused)
cd ~/ws_aic
python src/aic/aic_utils/aic_mujoco/scripts/view_scene.py ~/aic_mujoco_world/scene.xml

# Option 3: Use a one-liner Python command (paused mode)
python -c "import mujoco, mujoco.viewer; m = mujoco.MjModel.from_xml_path('~/aic_mujoco_world/scene.xml'); d = mujoco.MjData(m); v = mujoco.viewer.launch_passive(m, d); v.sync(); exec('while v.is_running(): v.sync()')"
```

> **Tip:** Press Space in the viewer to start/pause simulation, Backspace to reset.

#### Using native ROS 2 workspace

The `simulate` binary (from `mujoco_vendor`) can load scenes directly from command line and starts **paused by default**:

```bash
# Load scene (paused by default)
simulate ~/ws_aic/src/aic/aic_utils/aic_mujoco/mjcf/scene.xml
```

> **Tip:** Press Space to start/pause simulation in the viewer.

### Launching MuJoCo with ros2_control

The `aic_mujoco_bringup.launch.py` launch file starts MuJoCo simulation with ros2_control, loading the same controllers as the Gazebo simulation.

#### Basic Launch Example

```bash
# terminal 1: Start the Zenoh router if not already running
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run rmw_zenoh_cpp rmw_zenohd
```

```bash
# terminal 2: Launch MuJoCo simulation with ros2_control
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_mujoco aic_mujoco_bringup.launch.py
```

The robot can now be teleoperated using the `aic_teleoperation` package. See the [teleoperation](../../docs/teleoperation.md) section for details.

Any of the policies in `aic_example_policies` can be used to control the robot in MuJoCo. See the [example policies](../../docs/example_policies.md) section for details.

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [mujoco_ros2_control GitHub](https://github.com/ros-controls/mujoco_ros2_control)
- [AIC Getting Started Guide](../../docs/getting_started.md)
- [AIC Scene Description](../../docs/scene_description.md)
