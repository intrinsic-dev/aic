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
rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev"
source /opt/ros/kilted/setup.bash
GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install
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

Then bringup the simulator.

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

> [!NOTE]
> To spawn a cable and attach it to the gripper, pass `spawn_cable:=True` and `attach_cable_to_gripper:=True` to the launch command.


#### Debugging

Send a reference wrench command (10N in the positive z-axis) to the controller
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup move_to_contact.launch.py contact_force_z:=10.0
```

Control the gripper via a ROS2 Action. The joint range of the gripper is from 0.0 to 0.025m
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
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{ joint_names: ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], points: [ {positions: [0.0, -1.57, -1.57, -1.57, 1.57, 0], time_from_start: {sec: 1} } ] }' --once
```

Spawn a task board
# To bringup the simulation without a default task board launch with spawn_task_board:=False
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup spawn_task_board.launch.py \
  task_board_x:=0.3 task_board_y:=-0.1 task_board_z:=1.2 task_board_yaw:=0.785 \
  lc_mount_01_delta_y:=-0.05 sfp_mount_01_delta_y:=-0.08 sc_mount_01_delta_y:=-0.09 \
  lc_mount_02_delta_y:=0.05 sfp_mount_02_delta_y:=0.08 sc_mount_02_delta_y:=0.09 \
  sc_port_01_delta_x:=-0.04 sc_port_02_delta_x:=0.04 \
  nic_card_mount_01_delta_x:=0.005 nic_card_mount_02_delta_x:=-0.008 \
  nic_card_mount_03_delta_x:=0.012 nic_card_mount_04_delta_x:=-0.015 \
  nic_card_mount_05_delta_x:=0.01
```
