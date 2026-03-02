# AIC MuJoCo Integration

This package provides documentation, scripts, and utilities for loading the AI for Industry Challenge (AIC) environment in MuJoCo.

## Overview

[MuJoCo](https://mujoco.org/) is a physics engine designed for research and development in robotics, biomechanics, graphics and animation. In collaboration with **Google DeepMind**, this integration enables participants to:

- Convert Gazebo SDF worlds to MuJoCo MJCF format using `sdformat_mjcf`
- Load the AIC task board and robot from exported Gazebo worlds (`/tmp/aic.sdf`)
- Access camera images, joint states, FT sensor data, and command the simulated robot over the same ROS topics
- Collect data and run policies unchanged between Gazebo and MuJoCo

This guide is split into two independent parts:

| | What | ROS 2 Control needed? |
|---|---|---|
| [**Part 1**](#part-1-building-the-mujoco-scene) | Generate the MJCF scene from Gazebo and view it in MuJoCo | No |
| [**Part 2**](#part-2-mujoco-with-ros-2-control) | Run the scene with `ros2_control` (same controller interface as Gazebo) | Yes |

## Import MuJoCo Dependencies

From your ROS 2 workspace, import all required MuJoCo repositories:

```bash
cd ~/ws_aic/src
vcs import < aic/aic_utils/aic_mujoco/mujoco.repos
```

This adds:
- `gz-mujoco` (with `sdformat_mjcf` tool) — Converts Gazebo SDF files to MuJoCo MJCF format
- `mujoco_vendor` (v0.0.6) — ROS 2 wrapper for MuJoCo 3.x with plugins (elasticity, actuator, sensor, SDF) and the `simulate` binary
- `mujoco_ros2_control` — Integration between MuJoCo and ros2_control

---

## Part 1: Building the MuJoCo Scene

This section covers generating and viewing the AIC scene in MuJoCo **without** requiring `ros2_control`. You only need the `sdformat_mjcf` converter and a MuJoCo viewer.

### Prerequisites

#### 1. Install `sdformat_mjcf` Python Bindings

The `sdf2mjcf` CLI tool requires Python bindings for SDFormat and Gazebo Math that are **not** resolved by `rosdep`. Install them from the OSRF Gazebo apt repository:

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

#### 2. Build the Converter

Build the `sdformat_mjcf` package:

```bash
cd ~/ws_aic
source /opt/ros/kilted/setup.bash
colcon build --packages-select sdformat_mjcf
source install/setup.bash
```

### Scene Generation Workflow

#### 1. Export from Gazebo

- Launch `aic_gz_bringup` with your desired domain randomization parameters. For example:
```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py spawn_task_board:=true spawn_cable:=true   cable_type:=sfp_sc_cable   attach_cable_to_gripper:=true   ground_truth:=true
```
- Gazebo will export the world to `/tmp/aic.sdf`.

See [Scene Description](../../docs/scene_description.md) for more details.

#### 2. Fix Exported SDF

The exported `/tmp/aic.sdf` contains two known URI corruption issues that must be fixed before conversion.

##### Issue 1: `<urdf-string>` in mesh URIs

When models are spawned from URDF strings (via `ros_gz_sim create -string`), the SDFormat parser uses the placeholder path `<urdf-string>` as the file source. On world export, this leaks into mesh URIs as `file://<urdf-string>/model://...`, which breaks XML parsing because `<urdf-string>` is interpreted as an XML tag.

```bash
# Fix corrupted model:// URIs
sed -i 's|file://<urdf-string>/model://|model://|g' /tmp/aic.sdf
```

##### Issue 2: Broken relative mesh URIs

Some included models (SC Plug, LC Plug, SFP Module) use relative mesh URIs (e.g., `<uri>sc_plug_visual.glb</uri>`). When the world is exported, these lose their model-relative context and become root-path URIs like `file:///sc_plug_visual.glb`, which point to nonexistent files.

```bash
# Fix broken mesh URIs by pointing to the actual files in aic_assets
sed -i 's|file:///lc_plug_visual.glb|model://LC Plug/lc_plug_visual.glb|g' /tmp/aic.sdf
sed -i 's|file:///sc_plug_visual.glb|model://SC Plug/sc_plug_visual.glb|g' /tmp/aic.sdf
sed -i 's|file:///sfp_module_visual.glb|model://SFP Module/sfp_module_visual.glb|g' /tmp/aic.sdf
```

> **Note:** These issues originate in the SDFormat library's handling of string-parsed URDFs and relative URIs during world save. They will occur every time you re-export the world from Gazebo.

#### 3. Convert SDF to MJCF

- Use the `sdf2mjcf` CLI tool to convert the fixed `/tmp/aic.sdf` to MJCF format:
  ```bash
  source ~/ws_aic/install/setup.bash
  mkdir -p ~/aic_mujoco_world
  sdf2mjcf /tmp/aic.sdf ~/aic_mujoco_world/aic_world.xml
  ```
- This generates MJCF XML file and mesh assets in `~/aic_mujoco_world`.

#### 4. Organize MJCF Files


- You **must always**  copy or symlink the generated mesh assets (`.obj` and `.png` files) from `~/aic_mujoco_world` into the `mjcf` folder so MuJoCo can find them.
  ```bash
  cp ~/aic_mujoco_world/* ~/ws_aic/src/aic/aic_utils/aic_mujoco/mjcf
  ```

#### 5. Generate Final MJCF Files

The `sdformat_mjcf` converter produces a single monolithic MJCF file. The `add_cable_plugin.py` script splits and refines it into separate robot/world/scene files and applies corrections that the converter cannot handle automatically:

- **Splits into three files:** Separates the monolithic `aic_world.xml` into `aic_robot.xml` (robot bodies, actuators, sensors), `aic_world.xml` (environment, task board, cable), and `scene.xml` (top-level file that includes both).
- **Adds motor actuators:** Inserts position-controlled actuators for all 6 UR5e joints and the Robotiq gripper finger joints.
- **Adds gripper mimic joint:** Couples the right finger to the left finger via an equality constraint (removing the redundant right finger motor).
- **Adds FT sensor:** Attaches force and torque sensors to the `AtiForceTorqueSensor` site.
- **Adds `gripper_tcp` site:** Inserts a tool-center-point site at the gripper tip for policy use.
- **Fixes robot quaternions:** Normalizes near-identity and noisy quaternions on robot links (e.g., `shoulder_link`, `upper_arm_link`, `wrist_*_link`) to clean values.
- **Configures cameras:** Adds orientation (`quat`), field of view (`fovy`), and resolution to the center/left/right cameras.
- **Configures the cable plugin:** Activates `mujoco.elasticity.cable`, sets twist/bend stiffness, adds joint damping, and attaches the plugin to all cable bodies.
- **Reparents cable link_1:** Moves `link_1` from `cable_end_0` to `cable_connection_0` with a computed relative pose (required for correct cable attachment).
- **Tunes cable physics:** Reduces cable body inertias from `0.01` to `1e-6`, sets `cable_connection_1` (SC plug end) inertia to `4e-4`, adds damping to `joint_connection_end_0`, and lifts `cable_end_0` by 5cm.
- **Adds weld constraint:** Welds the LC plug to the gripper tool link (`ati/tool_link`) with a tuned relative pose.
- **Adds contact exclusions:** Prevents self-collision between `tabletop`↔`shoulder_link`, gripper fingers, `sc_port`↔`sc_plug`, and `cable_end_0`↔`link_1`.
- **Partitions assets:** Assigns meshes, materials, and textures to the correct file (robot vs. world) based on keyword matching.

Make sure you run this without sourcing the ROS 2 workspace in a new terminal (use a virtual env as necessary):

  ```bash
  cd ~/ws_aic/src/aic/aic_utils/aic_mujoco/
  python3 scripts/add_cable_plugin.py --input mjcf/aic_world.xml --output mjcf/aic_world.xml --robot_output mjcf/aic_robot.xml --scene_output mjcf/scene.xml
  cd ~/ws_aic && colcon build --packages-select aic_mujoco
  ```
  - `--input`: Path to the initial MJCF world file (usually `aic_world.xml`).
  - `--output`: Path for the final world file (`aic_world.xml`).
  - `--robot_output`: Path for the robot-only file (`aic_robot.xml`).
  - `--scene_output`: Path for the scene file (`scene.xml`).



#### 6. View in MuJoCo

At this point you can view the generated scene in MuJoCo **without** any ROS 2 control setup.

##### Using pixi environment

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

##### Using the `simulate` binary

> **Note:** The `simulate` binary is provided by `mujoco_vendor`, which is built in [Part 2](#part-2-mujoco-with-ros-2-control). If you have already completed Part 2, you can also use:

```bash
simulate ~/ws_aic/src/aic/aic_utils/aic_mujoco/mjcf/scene.xml
```

---

## Part 2: MuJoCo with ROS 2 Control

![](../../../media/wave_arm_policy_mujoco.gif)

MuJoCo's integration with `ros2_control` allows you to control the UR5e robot using the same `aic_controller` interface as in Gazebo, ensuring your policy code remains simulator-agnostic.

### Installation Steps

> **Note:** If you already imported dependencies via `mujoco.repos` in the [Import MuJoCo Dependencies](#import-mujoco-dependencies) step above, the repositories are already cloned. Continue with the steps below to install and build.

#### 1. Install Dependencies

Install dependencies for the MuJoCo packages:

```bash
cd ~/ws_aic
rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev"
```

#### 2. Build the Workspace

```bash
cd ~/ws_aic
source /opt/ros/kilted/setup.bash

# Build all packages (including aic_mujoco)
GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install --packages-ignore lerobot_robot_aic
```

#### 3. Verify Installation

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
# Should show: libelasticity.so, libactuator.so, libsensor.so, libsdf_plugin.so, libmujoco.so*

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

The robot can now be teleoperated using the `aic_teleoperation` package. See the [teleoperation](../../docs/teleoperation.md) section for details. For cartesian teleop use:

```bash
source ~/ws_aic/install/setup.bash
ros2 run aic_teleoperation cartesian_keyboard_teleop
```

Any of the policies in `aic_example_policies` can be used to control the robot in MuJoCo. See the [example policies](../../docs/example_policies.md) section for details.

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [mujoco_ros2_control GitHub](https://github.com/ros-controls/mujoco_ros2_control)
- [AIC Getting Started Guide](../../docs/getting_started.md)
- [AIC Scene Description](../../docs/scene_description.md)
