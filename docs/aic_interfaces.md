# AI Challenge Interfaces

This document defines all the interfaces available for participants to work with in the AI for Industry Challenge. It includes both standard ROS 2 interfaces and new interfaces specifically defined for this challenge.

The aic_interfaces folder contains custom message and action definitions that bridge the hardware and the Insertion Policy. These interfaces are crucial for developing solutions that interact with the robot and task environment.

## Interface Overview

The challenge utilizes a combination of standard ROS 2 interfaces and custom interfaces defined in the [aic_interfaces](../aic_interfaces/) folder:

### Standard ROS 2 Interfaces
- **[sensor_msgs/msg/Image](https://github.com/ros2/common_interfaces/blob/kilted/sensor_msgs/msg/Image.msg)** - For camera image data
- **[sensor_msgs/msg/CameraInfo](https://github.com/ros2/common_interfaces/blob/kilted/sensor_msgs/msg/CameraInfo.msg)** - For camera calibration data
- **[geometry_msgs/msg/WrenchStamped](https://github.com/ros2/common_interfaces/blob/kilted/geometry_msgs/msg/WrenchStamped.msg)** - For force/torque sensor data
- **[sensor_msgs/msg/JointState](https://github.com/ros2/common_interfaces/blob/kilted/sensor_msgs/msg/JointState.msg)** - For joint state information
- **[tf2_msgs/msg/TFMessage](https://github.com/ros2/geometry2/blob/kilted/tf2_msgs/msg/TFMessage.msg)** - For transformation data

### Custom Interfaces (defined in [aic_interfaces](../aic_interfaces/))
* **[aic_task_interfaces/action/InsertCable.action](../aic_interfaces/aic_task_interfaces/action/InsertCable.action)**
    * An Action interface used to trigger the Insertion Policy to perform the cable insertion task.
* **[aic_task_interfaces/msg/Task.msg](../aic_interfaces/aic_task_interfaces/msg/Task.msg)**
    * Describes the specific parameters and state of the cable insertion task.
* **[aic_control_interfaces/msg/MotionUpdate.msg](../aic_interfaces/aic_control_interfaces/msg/MotionUpdate.msg)**
    * Describes a target pose and the associated tolerances for Cartesian-space control.
* **[aic_control_interfaces/msg/JointMotionUpdate.msg](../aic_interfaces/aic_control_interfaces/msg/JointMotionUpdate.msg)**
    * Describes a target joint configuration and the associated tolerances for joint-space control.
* **[aic_model_interfaces/msg/Observation.msg](../aic_interfaces/aic_model_interfaces/msg/Observation.msg)**
    * A snapshot of the world that the `aic_model` node subscribes to.

---

## Inputs

The following topics provide sensory data and state information to the model.

### Sensor Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/left_camera/image` | `sensor_msgs/msg/Image` | Rectified image data from the left wrist camera. |
| `/left_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for the left wrist camera. |
| `/center_camera/image` | `sensor_msgs/msg/Image` | Rectified image data from the center wrist camera. |
| `/center_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for the center wrist camera. |
| `/right_camera/image` | `sensor_msgs/msg/Image` | Rectified image data from the right wrist camera. |
| `/right_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for the right wrist camera. |
| `/fts_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | Force/Torque sensor data. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Current state of the robot joints. |
| `/gripper_state` | `sensor_msgs/msg/JointState` | Current state of the end-effector/gripper. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform data for dynamic coordinate frames. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Transform data for static coordinate frames. |

### Action Servers

| Action Name | Action Type | Description |
| :--- | :--- | :--- |
| `/insert_cable` | `aic_task_interfaces/action/InsertCable` | Trigger for the autonomous insertion task. |

### Controller Topics

The following topic provides high-frequency and real-time state telemetry data for monitoring and debugging.

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/aic_controller/controller_state` | `aic_control_interfaces/msg/ControllerState` | Data on current TCP pose and velocity, reference TCP pose, TCP tracking error and reference joint efforts. |

---

## Outputs

The Insertion Policy controls the robot by publishing to the following topics.

### Command Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/aic_controller/joint_commands` | `aic_control_interfaces/msg/JointMotionUpdate` | Target configurations for joint-space control. |
| `/aic_controller/pose_commands` | `aic_control_interfaces/msg/MotionUpdate` | Target poses for Cartesian-space control. |

> **Note:** The controller operates in mutually exclusive modes. For example, if the controller is in `Cartesian` target mode, it will process messages from the `/aic_controller/pose_commands` topic and ignore messages from `/aic_controller/joint_commands`. You must set the active target mode via the `/aic_controller/change_target_mode` service before the controller will accept commands of that type.

---

## Controller Configuration

### Services

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `/aic_controller/change_target_mode` | `aic_control_interfaces/srv/ChangeTargetMode` | Select the target mode (Cartesian or joint) to define the expected input. The controller will subscribe to either `/aic_controller/pose_commands` or `/aic_controller/joint_commands` accordingly. |
| `/aic_controller/tare_ft_sensor` | `std_srvs/srv/Trigger` | Service to tare the force/torque sensor. This service will be disabled during evaluation. The evaluation system will automatically call this service before the cable is spawned in the environment. |
