# aic_interfaces

This package contains additional ROS 2 interface definitions relevant to the AI Challenge. It defines the custom messages and actions required to bridge the robot hardware and the Insertion Policy.

## Interface Definitions

The package defines the following custom interfaces:

* **[action/InsertCable.action](./action/InsertCable.action)**
    * An Action interface used to trigger the Insertion Policy to perform the cable insertion task.
* **[msg/Task.msg](./msg/Task.msg)**
    * Describes the specific parameters and state of the cable insertion task.

---

### Inputs

The following topics provide sensory data and state information to the model.

**Sensor Topics**

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/wrist_camera_1/image_rect` | `sensor_msgs/msg/Image` | Rectified image data from wrist camera 1. |
| `/wrist_camera_1/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for wrist camera 1. |
| `/wrist_camera_2/image_rect` | `sensor_msgs/msg/Image` | Rectified image data from wrist camera 2. |
| `/wrist_camera_2/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for wrist camera 2. |
| `/axia80_m20/wrench` | `geometry_msgs/msg/WrenchStamped` | Force/Torque sensor data. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Current state of the robot joints. |
| `/gripper_state` | `sensor_msgs/msg/JointState` | Current state of the end-effector/gripper. |

**Action Servers**

| Action Name | Action Type | Description |
| :--- | :--- | :--- |
| `/insert_cable` | `aic_interfaces/action/InsertCable` | Trigger for the autonomous insertion task. |

### Outputs

The Insertion Policy controls the robot by publishing to the following topics.

**Command Topics**

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/joint_commands` | `aic_interfaces/msg/JointMotionUpdate` | Target configurations for joint-space control. |
| `/pose_commands` | `aic_interfaces/msg/MotionUpdate` | Target poses for Cartesian-space control. |

> **Note:** The model can command the robot using either joint configurations (via `/joint_commands`) or Cartesian poses (via `/pose_commands`). Publishing references to both topics simultaneously is discouraged to avoid control conflicts.
