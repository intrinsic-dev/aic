# aic_controller

The [aic_controller](../aic_controller/) package is a controller for ROS 2. It receives target commands (either Joint or Cartesian) from a control policy or planner at $\approx 10 \to 30\text{ Hz}$ and bridges them to the robot hardware at $\approx 500\text{ Hz}$. The targets go through safety checks and smoothing before impedance control is applied.

## Architecture

A high-level overview of its architecture is provided in the following diagram:
<img width="1889" height="437" alt="image" src="../../media/aic_controller.png" />

### Control Pipeline

1. **Command Clamping**: Incoming targets are clamped to stay within safety bounds.
    - Joint targets are clamped to the limits defined in the robot's URDF/description.
    - Cartesian targets are clamped to user-specified parameters.

2. **Command Interpolation**: The clamped targets are then smoothed out to turn slow policy commands into smooth high-speed setpoints.

3. **Impedance Control**: Smoothed setpoints are processed by either [CartesianImpedanceAction](../aic_controller/include/aic_controller/actions/cartesian_impedance_action.hpp) or [JointImpedanceAction](../aic_controller/include/aic_controller/actions/joint_impedance_action.hpp) to calculate the joint torques needed.

4. **Gravity Compensation**: Additional torque is calculated using [GravityCompensationAction](../aic_controller/include/aic_controller/actions/gravity_compensation_action.hpp) to counteract gravity on the robot links.

5. **Command Execution**: The impedance and gravity compensation torques are added together and sent to the robot joints.

### Cartesian Impedance Control

Cartesian targets are handled by `CartesianImpedanceAction` which calculates joint torques based on the difference between where the end-effector is and where it should be.

$$
\tau = \mathbf{J}^T \Big[ \mathbf{K}_p (\mathbf{x}_{des} - \mathbf{x}) + \mathbf{K}_d (\dot{\mathbf{x}}_{des} - \dot{\mathbf{x}}) + \mathbf{W}_f \Big] + \tau_{null}
$$

**Where**:
- $\tau \in \mathbb{R}^n$: Calculated joint torque.
- $\mathbf{J} \in \mathbb{R}^{6 \times n}$: Jacobian matrix of the robot arm.
- $\mathbf{K}_p, \mathbf{K}_d \in \mathbb{R}^{6 \times 6}$: Stiffness and damping matrices.
- $\mathbf{x}_{des}, \mathbf{x} \in \mathbb{R}^6$: Target and current end-effector pose.
- $\mathbf{W}_f \in \mathbb{R}^6$: Additional external force/torque.
- $\tau_{null} \in \mathbb{R}^n$: Extra torque for secondary tasks like avoiding joint limits.

### Joint Impedance Control

Joint targets are handled by `JointImpedanceAction` which calculates joint torques based on the difference between target and current joint positions.

$$
\tau = \mathbf{K}_p (\mathbf{q}_{des} - \mathbf{q}) + \mathbf{K}_d (\dot{\mathbf{q}}_{des} - \dot{\mathbf{q}}) + \tau_f
$$

**Where**:
- $\tau \in \mathbb{R}^n$: Calculated joint torque.
- $\mathbf{K}_p, \mathbf{K}_d \in \mathbb{R}^n$: Stiffness and damping for each joint.
- $\mathbf{q}_{des}, \mathbf{q} \in \mathbb{R}^n$: Target and current joint positions.
- $\dot{\mathbf{q}}_{des}, \dot{\mathbf{q}} \in \mathbb{R}^n$: Target and current joint velocities.
- $\tau_f \in \mathbb{R}^n$: Additional joint torque.


### ROS 2 Interfaces

#### Command Interfaces

The `aic_controller` accepts commands via two ROS 2 Topics. For detailed message definitions, refer to [Controller Target Parameters](#controller-target-parameters).

- **Cartesian Targets** ([`MotionUpdate`](../aic_interfaces/aic_control_interfaces/msg/MotionUpdate.msg)): `/aic_controller/pose_commands`
- **Joint Targets** ([`JointMotionUpdate`](../aic_interfaces/aic_control_interfaces/msg/JointMotionUpdate.msg)): `/aic_controller/joint_commands`

#### Switching between joint and Cartesian target modes

To switch between joint and Cartesian control, send a ROS 2 service request to `/aic_controller/change_target_mode`. The controller starts in **Cartesian** mode by default.

Send a service call to switch the controller's target mode using the ROS 2 CLI:
```bash
# Send a service request to switch to Cartesian target mode
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode "{target_mode: {mode: 1}}"

# Send a service request to switch to joint target mode
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode "{target_mode: {mode: 2}}"
```

> **Note:** The controller can only be in one mode at a time. For example, if the controller is in `Cartesian` mode, it will only listen to `/aic_controller/pose_commands` and ignore messages from `/aic_controller/joint_commands`. You must switch modes using the `/aic_controller/change_target_mode` service before the controller will accept that type of command. See [Controller Configuration](../docs/aic_interfaces.md#Controller-Configuration) for more details.

#### State feedback

The controller publishes real-time data to `/aic_controller/controller_state` ([`ControllerState`](../aic_interfaces/aic_control_interfaces/msg/ControllerState.msg)). This message includes:
- Current TCP pose and velocity
- Target TCP pose
- Error between current and target TCP pose
- Target joint torques

#### Force-Torque Sensor Tare

The controller provides a service to tare (zero) the force-torque sensor at `/aic_controller/tare_ft_sensor`. This service resets the current force/torque readings to zero, which is useful for calibrating the sensor or removing sensor bias. The tared offset is published in the [`ControllerState`](../aic_interfaces/aic_control_interfaces/msg/ControllerState.msg) message as `fts_tare_offset`.

> **Note:** Before the start of each training episode (i.e. before teleoperation or spawning cables in the environment), it is important to tare the Force/Torque Sensor (F/T Sensor) for accurate force-torque feedback.

```bash
# Tare the FT sensor
ros2 service call /aic_controller/tare_ft_sensor std_srvs/srv/Trigger
```

> **Important:** This service will **not be available** during the evaluation. The force-torque sensor readings are used for scoring, and participants cannot tare the sensor during competition runs.

## Controller Target Parameters

The table below shows the main controller parameters that policies typically need to modify for different tasks.

### MotionUpdate

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `header` | `std_msgs/Header` | The `frame_id` must be either `gripper/tcp` (TCP frame) or `base_link` (global frame). <br /> The `stamp` field should have the current timestamp. |
| `pose` | `geometry_msgs/Pose` |The target cartesian pose for the TCP. <br /> Used when `trajectory_generation_mode` is `MODE_POSITION`. If `frame_id` is `base_link`, the pose is relative to the robot base. If `frame_id` is `gripper/tcp`, the pose is an offset from the current TCP position. |
| `velocity` | `geometry_msgs/Twist` |The target velocity for the TCP. <br /> Used when `trajectory_generation_mode` is `MODE_VELOCITY`. The velocities are relative to the frame specified in `frame_id`. |
| `target_stiffness` | `float64[36]` | The 6x6 stiffness matrix that controls how strongly the robot resists moving away from the target pose. <br /> Higher values = stiffer control, lower values = more compliant control. |
| `target_damping` | `float64[36]` | The 6x6 damping matrix that reduces oscillations. <br /> Usually tuned relative to `target_stiffness` to prevent wobbling and ensure stable motion.|
| `feedforward_wrench_at_tip` | `geometry_msgs/Wrench` | Optional external force/torque at the TCP. <br /> Useful for contact tasks like applying constant downward force or dealing with known tool-environment interactions. |
| `wrench_feedback_gains_at_tip` | `float64[6]` | Feedback gains on force/torque measured by the sensor. |
| `trajectory_generation_mode` | `TrajectoryGenerationMode` | How the target should be interpreted. <br /> `MODE_POSITION` follows the `pose` values. <br /> `MODE_VELOCITY` follows the `velocity` values. |

#### Examples

To publish a pose target via the [`MotionUpdate`](../aic_interfaces/aic_control_interfaces/msg/MotionUpdate.msg) message using the ROS 2 CLI:
```bash
# Send a service request to switch to Cartesian target mode
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode "{target_mode: {mode: 1}}"

# Send a Cartesian pose target
ros2 topic pub --once /aic_controller/pose_commands aic_control_interfaces/msg/MotionUpdate "{
  header: {
    frame_id: 'base_link'
  },
  pose: {
    position: {x: -0.501, y: -0.175, z: 0.2},
    orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}
  },
  target_stiffness: [
    85.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 85.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 85.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 85.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 85.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 85.0
  ],
  target_damping: [
    75.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 75.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 75.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 75.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 75.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 75.0
  ],
  feedforward_wrench_at_tip: {
    force: {x: 0.0, y: 0.0, z: 0.0},
    torque: {x: 0.0, y: 0.0, z: 0.0}
  },
  wrench_feedback_gains_at_tip: [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  ],
  trajectory_generation_mode: {mode: 2}
}"
```

Similarly, publishing a velocity target is as simple as:
```bash
# The command below will move the TCP at 0.025 m/s along the x-axis and rotate it about the z-axis at 0.25 rad/s, until another target overrides it:
ros2 topic pub --once /aic_controller/pose_commands aic_control_interfaces/msg/MotionUpdate "{
  header: {
    frame_id: 'gripper/tcp'
  },
  velocity: {
    linear: {x: 0.025, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.25}
  },
  target_stiffness: [
    85.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 85.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 85.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 85.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 85.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 85.0
  ],
  target_damping: [
    75.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 75.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 75.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 75.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 75.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 75.0
  ],
  trajectory_generation_mode: {mode: 1}
}"
```

Refer to the `generate_motion_update()` function within the [test_impedance.py](../aic_bringup/scripts/test_impedance.py) script for an example with `rclpy`.

### JointMotionUpdate

| Parameter |  Type | Description |
| :--- | :--- | :--- |
| `target_state` | `trajectory_msgs/JointTrajectoryPoint` | The target joint values for each robot joint. <br /> The `positions` field is used when `trajectory_generation_mode` is `MODE_POSITION`. <br /> The `velocities` field is used when `trajectory_generation_mode` is `MODE_VELOCITY`. |
| `target_stiffness` | `float64[]` | The stiffness for each robot joint. <br /> Higher values = stiffer control, lower values = more compliant control. Array size should match the number of joints. |
| `target_damping` | `float64[]` | The damping for each robot joint. <br /> Usually tuned relative to `target_stiffness` to prevent wobbling and ensure stable motion. Array size should match the number of joints. |
| `target_feedforward_torque` | `float64[]` | Optional external torque for each robot joint.  <br /> Useful for contact tasks like applying constant force or dealing with known tool-environment interactions. |
| `trajectory_generation_mode` | `TrajectoryGenerationMode` | How the target should be interpreted. <br /> `MODE_POSITION` follows the `target_state.positions` values. <br /> `MODE_VELOCITY` follows the `target_state.velocities` values. |

#### Examples

Refer to the `generate_joint_motion_update()` function within [test_impedance.py](../aic_bringup/scripts/test_impedance.py).

To publish a joint position target via the [`JointMotionUpdate`](../aic_interfaces/aic_control_interfaces/msg/JointMotionUpdate.msg) message using the ROS 2 CLI:
```bash
# Send a service request to switch to joint target mode
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode "{target_mode: {mode: 2}}"

# Send a joint position target
ros2 topic pub --once /aic_controller/joint_commands aic_control_interfaces/msg/JointMotionUpdate "{
  target_state: {
    positions: [0.0, -1.57, -1.57, -1.57, 1.57, 0]
  },
  target_stiffness: [85.0, 85.0, 85.0, 85.0, 85.0, 85.0],
  target_damping: [75.0, 75.0, 75.0, 75.0, 75.0, 75.0], trajectory_generation_mode: {mode: 2}
}"
```

Similarly, publishing a joint velocity target is as simple as:
```bash
# The command below will rotate all joints at 0.025 rad/s until another target overrides it:
ros2 topic pub --once /aic_controller/joint_commands aic_control_interfaces/msg/JointMotionUpdate "{
  target_state: {
    velocities: [0.025, 0.025, 0.025, 0.025, 0.025, 0.025]
  },
  target_stiffness: [85.0, 85.0, 85.0, 85.0, 85.0, 85.0],
  target_damping: [75.0, 75.0, 75.0, 75.0, 75.0, 75.0], trajectory_generation_mode: {mode: 1}
}"
```

## Controller Configuration Parameters

The `aic_controller` uses ROS 2 parameters to set values for target limiting, smoothing, and impedance control. These are defined in [aic_controller_parameters.yaml](../aic_controller/src/aic_controller_parameters.yaml) along with their descriptions and data types.

> **Note:** The configuration is fixed during the evaluation and all participants will use the same controller settings.
