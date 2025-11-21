# aic

## Local setup

### Requirements
- [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
- [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### Install

Purge existing Gazebo binaries
```bash
sudo apt purge gz-harmonic
sudo apt purge ros-jazzy-gz-*-vendor ros-jazzy-gz-ros2-control
```

Add `packages.osrfoundation.org` to the apt sources list:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

Build the workspace
```bash
sudo apt update && sudo apt upgrade -y && sudo apt install ros-jazzy-rmw-zenoh-cpp -y
mkdir ~/ws_aic/src -p
cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic
vcs import . < aic/aic.repos --recursive
# Install Gazebo dependencies.
sudo apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
cd ~/ws_aic
# Install ROS dependencies using rosdep.
rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev"
source /opt/ros/jazzy/setup.bash
GZ_BUILD_FROM_SOURCE=1 colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
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

Control the gripper via a ROS2 Action. The joint range of the gripper is from 0.0 to 0.025m
```bash
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 50.0}}"
```
