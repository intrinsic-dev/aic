# Getting Started

Welcome to the AI for Industry Challenge! This guide will help you set up your development environment and get started with building your solution.

The challenge workflow relies on two distinct components:

### 1. Evaluation Container (`aic_eval`)
This container hosts:
- Gazebo simulation environment
- Robot arm and end-of-arm tooling
- Task board spawning and management
- Trial orchestration via `aic_engine`
- Scoring system

### 2. Participant workspace
This is your development workspace where you implement your policy.

**What you'll do:**
- Implement your policy in the `aic_model` package
- Test locally against the evaluation container
- Build and submit your container image for official evaluation

### Requirements

* [docker](https://www.docker.com/)
* [nvidia-continer-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html)
* [distrobox](https://distrobox.it/)
* [pixi](https://prefix.dev/tools/pixi)

#### Install docker

Refer to [Docker Engine Installation](https://docs.docker.com/engine/install/).

#### Install and configure nvidia-container-toolkit

Follow https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html to install nvidia-container-toolkit.

After installing, be sure to enable it for docker:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

#### Install distrobox

If your distro already supports distrobox ([supported distros](https://distrobox.it/#installation)). It is recommended to install with the package manager.

For example, in ubuntu:

```bash
sudo apt install distrobox
```

If it is not supported, refer to [Alternative methods](https://distrobox.it/#alternative-methods) to install distrobox.

#### Install pixi

```bash
curl -fsSL https://pixi.sh/install.sh | sh
# Restart the terminal
```

Also see [Alternative Installation Methods](https://pixi.prefix.dev/latest/installation/#alternative-installation-methods).

### Quick Start

1. **Start the evaluation container:**
   ```bash
   # Set up Docker container manager
   export DBX_CONTAINER_MANAGER=docker

   # Create and enter the eval container
   docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest
   distrobox create -r -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
   distrobox enter -r aic_eval

   # Inside the container, start the environment
   /entrypoint.sh
   ```

   <!-- TODO: Update instruction to disable ACL after https://github.com/intrinsic-dev/aic/pull/190 or https://github.com/intrinsic-dev/aic/pull/171 is merged. -->

   > [!Note]
   > You may need to [login to ghcr.io](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-with-a-personal-access-token-classic).

   <!-- TODO: Shouldn't need to login after we make it public -->

2. **Set up pixi workspace:**
   ```bash
   # Clone this repo
   mkdir -p ~/ws_aic/src
   cd ~/ws_aic/src
   git clone https://github.com/intrinsic-dev/aic

   # Install and build dependencies
   cd ~/ws_aic/src/arc
   pixi install
   ```

3. **Run an example policy:**
   ```bash
   pixi run ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
   ```

---

## Next Steps

Now that you have your environment set up:

1. **📚 Read the Documentation**
   - [Qualification Phase Details](./qualification_phase.md) - Understand the trials you'll be evaluated on
   - [Challenge Rules](./challenge_rules.md) - Ensure your policy complies with all requirements
   - [Policy Integration Guide](./policy.md) - Learn how to implement your policy

2. **💻 Start Developing**
   - Explore `aic_example_policies/` for reference implementations
   - Review [AIC Interfaces](./aic_interfaces.md) to understand available sensors and actuators
   - Consult [AIC Controller](./aic_controller.md) to learn about motion commands
   - Follow the [Creating a new policy node tutorial](./policy_tutorial.md) to learn how to create your own policy node.

3. **🧪 Test and Iterate**
   - Use the example configurations in `aic_engine/config/` to test different scenarios
   - Monitor your policy's behavior with ground truth data during development
   - Refer to [Troubleshooting](./troubleshooting.md) if you encounter issues

4. **📦 Prepare for Submission**
   - Package your solution following [Submission Guidelines](./submission.md)
   - Test your container before submitting
   - Submit through the official portal

---

## Advanced Topics

### Native Ubuntu 24.04 Installation

If you prefer you can build and run everything locally on Ubuntu 24.04.

#### Prerequisites
- **Operating System:** [Ubuntu 24.04 (Noble Numbat)](https://releases.ubuntu.com/noble/)
- **ROS 2:** [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

#### 1. Add Gazebo Repository

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

#### 2. Clone and Build the Workspace

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

# Install ROS dependencies
cd ~/ws_aic
sudo rosdep init # if running rosdep for the first time.
rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev rosetta"

# Build the workspace
source /opt/ros/kilted/setup.bash
GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install --packages-ignore lerobot_robot_aic
```

#### 3. Running the Evaluation Environment

> [!NOTE]
> For detailed information about all available launch files and their configurable parameters, see the [aic_bringup README](../aic_bringup/README.md).

> [!NOTE]
> We rely on [rmw_zenoh](https://github.com/ros2/rmw_zenoh) as the ROS 2 middleware for this application. Please ensure the `RMW_IMPLEMENTATION` environment variable is set to `rmw_zenoh_cpp` in all terminals.

Start the Zenoh router.

```bash
source ~/ws_aic/install/setup.bash
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run rmw_zenoh_cpp rmw_zenohd
```

#### Evaluator bringup

> [!NOTE]
> Update with launch commands to start Zenoh router with ACLs.

To launch the evaluator,

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup aic_gz_bringup.launch.py ground_truth:=false start_aic_engine:=true
```

This will launch Gazebo with the robot arm and end-of-arm tooling together with all required drivers.
The `TaskBoard` and `Cable` will be spawned by `aic_engine`, the orchestrator for the challenge.
Note that you will need to bring up your model for the `aic_engine` to work correctly, see [Submission Bringup](#submission-bringup).


#### Training Bringup

During training, you can bring up the scene with randomized poses of the `TaskBoard` and `Cables`.
The simulation automatically exports the complete world state to `/tmp/aic.sdf` after spawning all entities, which can be imported into other simulators like IsaacLab or MuJoCo for AI policy training.

The layout of the `TaskBoard` can be configured at runtime.
For the full list of configurable parameters, see the [aic_bringup README](../aic_bringup/README.md).
Ground truth poses of ports and plugs can be made available in TF tree as well.

**Example:** Launch with a fully configured task board and cable:


```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup aic_gz_bringup.launch.py \
	spawn_task_board:=true \
	task_board_x:=0.3 task_board_y:=-0.1 task_board_z:=1.2 \
	task_board_roll:=0.0 task_board_pitch:=0.0 task_board_yaw:=0.785 \
	lc_mount_rail_0_present:=true lc_mount_rail_0_translation:=-0.05 \
	lc_mount_rail_0_roll:=0.0 lc_mount_rail_0_pitch:=0.0 lc_mount_rail_0_yaw:=0.0 \
	sfp_mount_rail_0_present:=true sfp_mount_rail_0_translation:=-0.08 \
	sfp_mount_rail_0_roll:=0.0 sfp_mount_rail_0_pitch:=0.0 sfp_mount_rail_0_yaw:=0.0 \
	sc_mount_rail_0_present:=true sc_mount_rail_0_translation:=-0.09 \
	sc_mount_rail_0_roll:=0.0 sc_mount_rail_0_pitch:=0.0 sc_mount_rail_0_yaw:=0.0 \
	lc_mount_rail_1_present:=true lc_mount_rail_1_translation:=0.05 \
	lc_mount_rail_1_roll:=0.0 lc_mount_rail_1_pitch:=0.0 lc_mount_rail_1_yaw:=0.0 \
	sfp_mount_rail_1_present:=true sfp_mount_rail_1_translation:=0.08 \
	sfp_mount_rail_1_roll:=0.0 sfp_mount_rail_1_pitch:=0.0 sfp_mount_rail_1_yaw:=0.0 \
	sc_mount_rail_1_present:=true sc_mount_rail_1_translation:=0.09 \
	sc_mount_rail_1_roll:=0.0 sc_mount_rail_1_pitch:=0.0 sc_mount_rail_1_yaw:=0.0 \
	sc_port_0_present:=true sc_port_0_translation:=-0.04 \
	sc_port_0_roll:=0.0 sc_port_0_pitch:=0.0 sc_port_0_yaw:=0.0 \
	sc_port_1_present:=true sc_port_1_translation:=0.04 \
	sc_port_1_roll:=0.0 sc_port_1_pitch:=0.0 sc_port_1_yaw:=0.0 \
	nic_card_mount_0_present:=true nic_card_mount_0_translation:=0.005 \
	nic_card_mount_0_roll:=0.0 nic_card_mount_0_pitch:=0.0 nic_card_mount_0_yaw:=0.0 \
	nic_card_mount_1_present:=true nic_card_mount_1_translation:=-0.008 \
	nic_card_mount_1_roll:=0.0 nic_card_mount_1_pitch:=0.0 nic_card_mount_1_yaw:=0.0 \
	nic_card_mount_2_present:=true nic_card_mount_2_translation:=0.012 \
	nic_card_mount_2_roll:=0.0 nic_card_mount_2_pitch:=0.0 nic_card_mount_2_yaw:=0.0 \
	nic_card_mount_3_present:=true nic_card_mount_3_translation:=-0.015 \
	nic_card_mount_3_roll:=0.0 nic_card_mount_3_pitch:=0.0 nic_card_mount_3_yaw:=0.0 \
	nic_card_mount_4_present:=true nic_card_mount_4_translation:=0.01 \
	nic_card_mount_4_roll:=0.0 nic_card_mount_4_pitch:=0.0 nic_card_mount_4_yaw:=0.0 \
	spawn_cable:=true cable_type:=sfp_sc_cable attach_cable_to_gripper:=true \
	ground_truth:=true start_aic_engine:=false

# The complete world state is automatically saved to /tmp/aic.sdf
# This file contains the robot, enclosure, task board, and cable with the specified configuration
# Import this file into IsaacLab, MuJoCo, or other simulators for training
```

**Creating multiple training scenarios:** Run the launch command with different parameter combinations to generate diverse training environments. Each launch will overwrite `/tmp/aic.sdf` with the new configuration, so copy it to a different location if you want to preserve multiple scenarios.

#### Submission bringup

Run a minimal `aic_model` demo. This demo `aic_model` implementation should wave the arm back and forth for 30 seconds, before the goal

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
```

---

### Testing Your Policy

After setting up your environment, you can test your policy implementation:

#### 1. Start the Evaluation Environment

**Terminal 1 - Start Zenoh Router:**
```bash
source ~/ws_aic/install/setup.bash
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Terminal 2 - Launch Evaluation Environment:**
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup aic_gz_bringup.launch.py ground_truth:=false start_aic_engine:=true
```

#### 2. Run Your Policy

**Terminal 3 - Start Your aic_model:**
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
```

Or rely on 🐍 Pixi

```bash
pixi run ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
```

Replace `aic_example_policies.ros.WaveArm` with your policy implementation.

To manually submit a task,

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
cd ~/ws_aic/src/aic/aic_model/test
./create_and_cancel_task.py
```


#### 3. Monitor Progress

- Watch the Gazebo window for robot movement.
- Check terminal output for task progress and scoring information.
- Results will be saved to `$HOME/aic_results/` (or `$AIC_RESULTS_DIR` if set).


### LeRobot Support

A LeRobot interface is available to train a policy using [LeRobot](https://huggingface.co/lerobot). See [lerobot_robot_aic](../aic_utils/lerobot_robot_aic/README.md).

### Debugging Commands

**Send a reference wrench command (10N in the positive z-axis) to the controller:**
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup move_to_contact.launch.py contact_force_z:=10.0
```

**Control the gripper via ROS 2 Action (range: 0.0 to 0.025m):**
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup gripper_action.launch.py use_position:=true position:=0.024
```

**Send a joint-position command to the arm:**
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
# Switch to joint target mode on the controller
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode '{target_mode: 1}'
# Send joint target
ros2 topic pub /aic_controller/joint_commands aic_control_interfaces/msg/JointMotionUpdate '{target_state:
{positions: [0.0, -1.57, -1.57, -1.57, 1.57, 0] }, target_stiffness: [100.0, 100.0, 100.0, 50.0, 50.0, 50.0], target_damping: [40.0, 40.0, 40.0, 15.0, 15.0, 15.0], trajectory_generation_mode: {mode: 2}, time_to_target_seconds: 1.0 }' --once
```

---

## Need Help?

- **Documentation**: Check the [main README](../README.md) for links to all documentation
- **Issues**: Report problems via [GitHub Issues](https://github.com/intrinsic-dev/aic/issues)
- **Community**: Join discussions at [Open Robotics Discourse](https://discourse.openrobotics.org/c/competitions/ai-for-industry-challenge/)
