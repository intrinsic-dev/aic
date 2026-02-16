# Participant Utilities

## Teleoperation

### aic_teleoperation

- [aic_teleoperation](../aic_utils/aic_teleoperation/README.md): Keyboard-based teleoperation for joint-space and Cartesian-space control

### lerobot_robot_aic

- [lerobot_robot_aic](../aic_utils/lerobot_robot_aic/README.md#teleoperating-with-lerobot): LeRobot-based teleoperation for joint-space and Cartesian-space control (using keyboard or SpaceMouse device)
- Enables dataset recording for training LeRobot policies

### Additional Examples

- Command robot to specified poses or joint configurations: [test_impedance.py](../aic_bringup/scripts/test_impedance.py), [home_robot.py](../aic_bringup/scripts/home_robot.py)

## LeRobot Data collection and Training

- [lerobot_robot_aic](../aic_utils/lerobot_robot_aic/README.md#recording-training-data): [LeRobot](https://huggingface.co/lerobot) integration with AIC, which enables teleoperation and dataset recording using LeRobot

## Plotting

- [PlotJuggler](https://github.com/facontidavide/PlotJuggler): for visualizing time series data from ROS topics

## RViz

- [RViz](https://docs.ros.org/en/kilted/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) is a vizualizer for ROS 2. The RViz configuration file provided (`aic.rviz`) only displays the center camera stream due to bandwidth concerns, but you may find it helpful to add views for the other two cameras.

## ROS 2 CLI Tools

ROS 2 provides a comprehensive set of command-line tools for introspecting and debugging your system:

- **[ROS 2 Beginner CLI Tools](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools.html)**: Essential tutorials covering:
  - `ros2 node` - List and inspect running nodes
  - `ros2 topic` - View topics, echo messages, and monitor publication rates
  - `ros2 service` - Call services and view service types
  - `ros2 param` - Get and set node parameters
  - `ros2 action` - Interact with actions
  - `ros2 bag` - Record and replay data
  - `ros2 launch` - Launch multiple nodes
  - `ros2 interface` - Inspect message/service/action types

**Quick examples:**
```bash
# List all active nodes
ros2 node list

# Echo a topic
ros2 topic echo /aic_controller/state

# Get node parameters
ros2 param list /aic_controller

# Record data to a bag file
ros2 bag record -o my_recording /aic_controller/state /camera/image
```