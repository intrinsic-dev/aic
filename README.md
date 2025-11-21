# aic

## Local setup

### Requirements
- [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
- [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

### Install

Build the workspace
```bash
sudo apt update && sudo apt upgrade -y && sudo apt install ros-kilted-rmw-zenoh-cpp -y
mkdir ~/ws_aic/src -p
cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic
vcs import . < aic/aic.repos --recursive
# Install ROS dependencies using rosdep.
cd ~/ws_aic
rosdep install --from-paths src --ignore-src --rosdistro kilted -yr
source /opt/ros/kilted/setup.bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
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
