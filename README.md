# aic

## Local setup

### Requirements
- [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
- [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### Install
Purge existing `gz-harmonic`, `ros-jazzy-gz-*-vendor` and `ros-jazzy-ros2-control` binaries
```bash
sudo apt purge gz-harmonic
sudo apt purge ros-jazzy-gz-*-vendor ros-jazzy-gz-ros2-control
```

Build the workspace
```bash
sudo apt update && sudo apt upgrade -y && sudo apt install ros-jazzy-rmw-zenoh-cpp -y
mkdir ~/ws_aic/src -p
cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic
cd ~/ws_aic
vcs import . < aic/aic.repos --recursive
rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

### Launch

Make sure to already have the zenoh router up by running `ros2 run rmw_zenoh_cpp rmw_zenohd`.

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

Send a reference wrench command (10N in the positive z-axis) to the controller
```bash
ros2 launch aic_bringup move_to_contact.launch.py contact_force_z:=10.0
```


