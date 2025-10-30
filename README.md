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

### Debugging

Send a target reference force of 13N in the z-axis to the  controller
```bash
ros2 topic pub --once /admittance_controller/wrench_reference geometry_msgs/msg/WrenchStamped "{
    header: {
        stamp: {sec: 0, nanosec: 0},
        frame_id: 'wrist_3_link'
    },
    wrench: {
        force:  {x: 0.0, y: 0.0, z: 13.0},
        torque: {x: 0.0,  y: 0.0, z: 0.0}
    }
}"
```


