# Getting Started

## Container Setup

The challenge workflow relies on two distinct Docker containers. We provide scripts to build both, ensuring your development environment matches the evaluation environment.

### 1. Evaluation Container (`aic_eval`)
This container hosts the simulation environment (Gazebo), manages task randomization, and acts as the "referee." It sends requests to your model to begin insertion and evaluates the result.

* **Pre-built Image:** For convenience, we publish this image to the repository. We recommend downloading it directly to save time.
* **Build from Source:** Alternatively, you can build it locally using the provided scripts if you need to inspect the environment internals.

### 2. Participant Container (`aic_model`)
This is your development workspace. It is designed to package the `aic_model` package, which contains your custom solution.

* **Development:** You will implement your solution within this container.
* **Submission:** Once your solution is ready, you will build this container image and upload it via our **Submission Portal**.

> **Note:** Detailed build and run instructions for these containers can be found in the [docker/aic_eval](../docker/aic_eval/README.md) and [docker/aic_model](../docker/aic_model/README.md) directory of the repository.

**TODO**

---

## Local setup

### Requirements
- [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
- [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)
- [pixi](https://pixi.prefix.dev/latest/installation/)

### Install

Add `packages.osrfoundation.org` to the apt sources list:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

Build the workspace
```bash
sudo apt update && sudo apt upgrade -y
mkdir ~/ws_aic/src -p
cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic
vcs import . < aic/aic.repos --recursive
# Install Gazebo dependencies.
sudo apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
cd ~/ws_aic
# Install ROS dependencies using rosdep.
rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev rosetta"
source /opt/ros/kilted/setup.bash
GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install --packages-ignore lerobot_robot_aic aic_lerobot_tools
```

### Launch

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

During training, one might want to bring up the scene with randomized poses of the `TaskBoard` and `Cables`.
The scene may then be exported to other simulation environments.
The layout of the `TaskBoard` can be configured at runtime.
For the full list of configurable parameters, see the [aic_bringup README](../aic_bringup/README.md).
Ground truth poses of ports and plugs can be made available in TF tree as well.


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
```

#### Submission bringup

Run a minimal `aic_model` demo. This demo `aic_model` implementation should wave the arm back and forth for 30 seconds, before the goal

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
```


Run this in a different shell, for convenience
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
cd ~/ws_aic/src/aic/aic_model/test
./create_and_cancel_task.py
```

### LeRobot Support

A LeRobot interface is available to train a policy using [LeRobot](https://huggingface.co/lerobot). See [lerobot_robot_aic](../aic_utils/lerobot_robot_aic/README.md).

#### Debugging

Send a reference wrench command (10N in the positive z-axis) to the controller
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup move_to_contact.launch.py contact_force_z:=10.0
```

Control the gripper via a ROS 2 Action. The joint range of the gripper is from 0.0 to 0.025m
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup gripper_action.launch.py use_position:=true position:=0.024
```

Send a joint-position command to the arm as a single-point 1-second trajectory:
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
