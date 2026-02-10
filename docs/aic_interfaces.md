# aic_interfaces

Additional ROS 2 interface definitions relevant to the AI Challenge.
It defines the custom messages and actions required to bridge the robot hardware and the Insertion Policy.

## Interface Definitions

The following interfaces are defined.

* **[action/InsertCable.action](../aic_interfaces/aic_task_interfaces/action/InsertCable.action)**
    * An Action interface used to trigger the Insertion Policy to perform the cable insertion task.
* **[msg/Task.msg](../aic_interfaces/aic_task_interfaces/msg/Task.msg)**
    * Describes the specific parameters and state of the cable insertion task.
* **[msg/MotionUpdate.msg](../aic_interfaces/aic_control_interfaces/msg/MotionUpdate.msg)**
    * Describes a target pose and the associated tolerances for Cartesian-space control.
* **[msg/JointMotionUpdate.msg](../aic_interfaces/aic_control_interfaces/msg/JointMotionUpdate.msg)**
    * Describes a target joint configuration and the associated tolerances for joint-space control.
* **[msg/Observation.msg](../aic_interfaces/aic_model_interfaces/msg/Observation.msg)**
    * A snapshot of the world that the `aic_model` node subscribes to.
---

### Inputs

The following topics provide sensory data and state information to the model.

**Sensor Topics**

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/left_camera/image` | `sensor_msgs/msg/Image` | Rectified image data from the left wrist camera. |
| `/left_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for the left wrist camera. |
| `/center_camera/image` | `sensor_msgs/msg/Image` | Rectified image data from the center wrist camera. |
| `/center_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for the center wrist camera. |
| `/right_camera/image` | `sensor_msgs/msg/Image` | Rectified image data from the right wrist camera. |
| `/right_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration data for the right wrist camera. |
| `/axia80_m20/wrench` | `geometry_msgs/msg/WrenchStamped` | Force/Torque sensor data. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Current state of the robot joints. |
| `/gripper_state` | `sensor_msgs/msg/JointState` | Current state of the end-effector/gripper. |

**Action Servers**

| Action Name | Action Type | Description |
| :--- | :--- | :--- |
| `/insert_cable` | `aic_task_interfaces/action/InsertCable` | Trigger for the autonomous insertion task. |

### Outputs

The Insertion Policy controls the robot by publishing to the following topics.

**Command Topics**

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/aic_controller/joint_commands` | `aic_control_interfaces/msg/JointMotionUpdate` | Target configurations for joint-space control. |
| `/aic_controller/pose_commands` | `aic_control_interfaces/msg/MotionUpdate` | Target poses for Cartesian-space control. |

> **Note:** The model can command the robot using either joint configurations (via `/aic_controller/joint_commands`) or Cartesian poses (via `/aic_controller/pose_commands`). Publishing references to both topics simultaneously is discouraged to avoid control conflicts.

> **Note:** You must set the active target mode via the `/aic_controller/change_target_mode` ROS 2 service before the controller will accept commands of that type. For example, to publish joint targets via `/aic_controller/joint_commands`, first call the service to switch the controller to joint mode.

### Controller Configuration

**Services**

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `/aic_controller/change_target_mode` | `aic_control_interfaces/srv/ChangeTargetMode` | Select the target mode (Cartesian or joint) to define the expected input. The controller will subscribe to either `/aic_controller/pose_commands` or `/aic_controller/joint_commands` accordingly.|
