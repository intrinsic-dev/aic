# ieec

## Local setup

### Requirements
- Ubunutu 24.04
- ROS 2 Jazzy Jalisco

### Install

```bash
sudo apt update && sudo apt install ros-jazzy-rmw-zenoh-cpp -y
mkdir ~/ws_ieec/src -p
cd ~/ws_ieec/src
git clone https://github.com/intrinsic-dev/ieec
vcs import . < ieec/ieec.repos --recursive
cd ~/ws_ieec
rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

### Launch

Make sure to already have the zenoh router up by running `ros2 run rmw_zenoh_cpp rmw_zenohd`.

```bash
ros2 launch ieec_bringup ieec_gz_bringup.launch.py
```

### Troubleshooting

- When encountering erros with loading `ros2_control` libraries from Gazebo, make sure to upgrade your existing packages by running `sudo apt upgrade`. An example error is shown below:
```
[gazebo-5] Error while loading the library [/opt/ros/jazzy/lib/libgz_ros2_control-system.so]: /opt/ros/jazzy/lib/libgz_ros2_control-system.so: undefined symbol: _ZN18hardware_interface15ResourceManager30load_and_initialize_componentsERKNS_21ResourceManagerParamsE
```

