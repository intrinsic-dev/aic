# Building the Evaluation Component from Source

This guide is for advanced users who want to build the Evaluation Component locally on Ubuntu 24.04 instead of using the provided Docker container.

> [!NOTE]
> **For most users, we recommend using the pre-built `aic_eval` Docker container** as described in the [Getting Started](./getting_started.md) guide. The container provides a consistent, tested environment that matches the official evaluation setup.

## Why Build from Source?

Building locally may be useful if you:
- Want to modify or debug the evaluation component
- Prefer native development without containers
- Need to integrate with other tools on your host system

> [!IMPORTANT]
> Changes to the Evaluation Component will **not** be reflected in the official evaluation. Only your Participant Model (submitted as a container) will be evaluated.

---

## Prerequisites

| Dependency | Release / Distro |
| ---------- | ------- |
| Operating System | [Ubuntu 24.04 (Noble Numbat)](https://releases.ubuntu.com/noble/) |
| ROS 2 | [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html) |

---

## Setup Instructions

If your system currently has ROS 2 Kilted and Gazebo binaries installed, you must remove them before starting the source installation. This is necessary because building specific repositories from source while pre-installed binaries exist can cause environment conflicts.

To purge the relevant binaries, run the following command:

```bash
sudo apt purge ros-kilted-ros2-control* ros-kilted-control* ros-kilted-kinematics* ros-kilted-joint-state-publisher ros-kilted-realtime-tools ros-kilted-gz*
```

### 1. Add Gazebo Repository

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

### 2. Clone and Build Workspace

```bash
# Create workspace
sudo apt update && sudo apt upgrade -y
mkdir -p ~/ws_aic/src
cd ~/ws_aic/src

# Clone the repository
git clone https://github.com/intrinsic-dev/aic

# Import dependencies
vcs import . < aic/aic.repos --recursive

# Install Gazebo dependencies
sudo apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')

# Install ROS 2 dependencies
cd ~/ws_aic
sudo rosdep init  # Only if running rosdep for the first time
rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev rosetta"

# Install rmw_zenoh_cpp middleware and additional dependencies
sudo apt install -y ros-kilted-rmw-zenoh-cpp python3-pynput

# Build the workspace
source /opt/ros/kilted/setup.bash
GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install --packages-ignore lerobot_robot_aic
```

### 3. Configure Environment

Add these environment variables to your shell configuration (e.g., `~/.bashrc`):

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/shared_memory/transport_optimization/pool_size=536870912'
```

Then reload your shell configuration:
```bash
source ~/.bashrc
```

> [!NOTE]
> This challenge uses [rmw_zenoh](https://github.com/ros2/rmw_zenoh) as the ROS 2 middleware. You must set the `RMW_IMPLEMENTATION` environment variable to `rmw_zenoh_cpp` in all terminals.

---

## Running the System

You'll need three terminals. In each terminal, source the workspace:

```bash
source ~/ws_aic/install/setup.bash
```

> [!TIP]
> If you didn't add the environment variables to `~/.bashrc` in step 3, you'll need to export them in each terminal:
> ```bash
> export RMW_IMPLEMENTATION=rmw_zenoh_cpp
> export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/shared_memory/transport_optimization/pool_size=536870912'
> ```

Then run the following commands in their respective terminals:

### Terminal 1 - Start Zenoh Router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 2 - Launch Evaluation Environment

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py ground_truth:=false start_aic_engine:=true
```

This launches Gazebo with the robot arm and end-of-arm tooling. The `TaskBoard` and `Cable` will be spawned by `aic_engine` when your model is ready.

### Terminal 3 - Run Your Policy

```bash
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.WaveArm
```

Replace `aic_example_policies.ros.WaveArm` with your policy implementation.

---

## Next Steps

Now that you have the evaluation environment running locally:

- Explore the [Scene Description](./scene_description.md) to learn how to customize and explore the environment
- Read the [Policy Integration Guide](./policy.md) to understand how to create your own policy node
- Check out [`aic_example_policies/`](../aic_example_policies/) for reference implementations
- Review [AIC Interfaces](./aic_interfaces.md) to understand available sensors and actuators
- Consult [AIC Controller](./aic_controller.md) to learn about motion commands
- Run the [Scoring Test Examples](./scoring_tests.md) to see expected results for each baseline policy

---

## Troubleshooting

If you encounter issues:

1. **Build Errors**: Ensure all dependencies are installed correctly
2. **Runtime Issues**: Verify environment variables are set in all terminals
3. **ROS 2 Communication**: Check that Zenoh router is running and middleware is configured

For more help, see [Troubleshooting](./troubleshooting.md) or report issues on [GitHub](https://github.com/intrinsic-dev/aic/issues).
