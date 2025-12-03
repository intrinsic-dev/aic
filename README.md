# aic

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
GZ_BUILD_FROM_SOURCE=1 colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

### Launch

> [!NOTE]
> We rely on [rmw_zenoh](https://github.com/ros2/rmw_zenoh) as the ROS 2 middleware for this application. Please ensure the `RMW_IMPLEMENTATION` environment variable is set to `rmw_zenoh_cpp` in all terminals.

Start the Zenoh router.

```bash
source ~/ws_aic/install/setup.bash
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run rmw_zenoh_cpp rmw_zenohd`
```

Then bringup the simulator.

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

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
