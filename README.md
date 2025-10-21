# aic

## Local setup

### Requirements
- [Ubunutu 24.04](https://releases.ubuntu.com/noble/)
- [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### Install

```bash
sudo apt update && sudo apt upgrade -y && sudo apt install ros-jazzy-rmw-zenoh-cpp -y
mkdir ~/ws_aic/src -p
cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic
vcs import . < aic/aic.repos --recursive
cd ~/ws_aic
rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

### Launch

Make sure to already have the zenoh router up by running `ros2 run rmw_zenoh_cpp rmw_zenohd`.

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

## Admittance Control

### Launch

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py initial_joint_controller:=admittance_controller
```

### Force-torque sensor output
The force-torque sensor in Gazebo has been attached to the joint `wrist_3_joint` of the UR5e robot and it's gz topic has been bridged to the ROS Topic `/wrist_3_joint/force_torque`.

### TODO
1. Fix kinematics interface plugin
2. Tuning of parameters for admittance controller
