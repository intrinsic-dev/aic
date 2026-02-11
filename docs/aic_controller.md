# aic_controller

The [aic_controller](../aic_controller/) package provides a robust controller within the ROS 2 control framework. It acts as a high-frequency bridge between low-frequency planning/policy layers and the robot's hardware interface by interpolating sparse joint and cartesian-space commands then performing impedance control for a robot manipulator.

## Architecture

A high-level overview of its architecture is provided in the following diagram:
<img width="1889" height="437" alt="image" src="../../media/aic_controller.png" />

The controller subscribes to targets (either Joint or Cartesian) typically published by a control policy or planner at $\approx 10 \to 30\text{ Hz}$. To ensure smooth and safe execution at the hardware level ($\approx 500\text{ Hz}$), these targets undergo a pipeline of safety clamping and interpolation before the impedance control law is applied.

### Control Pipeline

1. **Command Clamping**: Incoming targets are clamped to ensure that they remain within safety limits and operational bounds.
    - Joint targets are clamped to the limits defined in the robot's URDF/description.
    - Cartesian targets are clamped to user-specified parameters.

2. **Command Interpolation**: The clamped targets are then interpolated to upsample the low-frequency policy commands into smooth high-frequency setpoints.

3. **Impedance Control**: Interpolated setpoints are then processed by either [CartesianImpedanceAction](../aic_controller/include/aic_controller/actions/cartesian_impedance_action.hpp) or [JointImpedanceAction](../aic_controller/include/aic_controller/actions/joint_impedance_action.hpp) to compute the joint torques of the robot

4. **Gravity Compensation**: An additional torque term is computed using via [GravityCompensationAction](../aic_controller/include/aic_controller/actions/gravity_compensation_action.hpp) to offset compensate for the gravitational forces of the robot links.

5. **Command Execution**: The impedance and gravity compensation torques are then summed and written to the effort interfaces for the robot joints.

### Cartesian Impedance Control

Cartesian targets are handled by `CartesianImpedanceAction` which computes joint torques based on the error between the current and desired end-effector pose.

$$
\tau = \mathbf{J}^T \Big[ \mathbf{K}_p (\mathbf{x}_{des} - \mathbf{x}) + \mathbf{K}_d (\dot{\mathbf{x}}_{des} - \dot{\mathbf{x}}) + \mathbf{W}_f \Big] + \tau_{null}
$$

**Where**:
- $\tau \in \mathbb{R}^n$: Computed joint control torque.
- $\mathbf{J} \in \mathbb{R}^{6 \times n}$: Geometric Jacobian of the robot arm.
- $\mathbf{K}_p, \mathbf{K}_d \in \mathbb{R}^{6 \times 6}$: Cartesian stiffness and damping matrices.
- $\mathbf{x}_{des}, \mathbf{x} \in \mathbb{R}^6$: Desired and current Cartesian pose.
- $\mathbf{W}_f \in \mathbb{R}^6$: Cartesian feedforward wrench.
- $\tau_{null} \in \mathbb{R}^n$: Nullspace torque (used for secondary objectives like joint limit avoidance).

### Joint Impedance Control

Joint targets are handled by `JointImpedanceAction` which computes joint torques based on joint-space errors.

$$
\tau = \mathbf{K}_p (\mathbf{q}_{des} - \mathbf{q}) + \mathbf{K}_d (\dot{\mathbf{q}}_{des} - \dot{\mathbf{q}}) + \tau_f
$$

**Where**:
- $\tau \in \mathbb{R}^n$: Computed joint control torque.
- $\mathbf{K}_p, \mathbf{K}_d \in \mathbb{R}^n$: Joint stiffness and damping vectors.
- $\mathbf{q}_{des}, \mathbf{q} \in \mathbb{R}^n$: Desired and current joint positions.
- $\dot{\mathbf{q}}_{des}, \dot{\mathbf{q}} \in \mathbb{R}^n$: Desired and current joint velocities.
- $\tau_f \in \mathbb{R}^n$: Feedforward joint torque.


### ROS 2 Interfaces

#### Command Interfaces

The `aic_controller` accepts commands via two ROS 2 Topics. For detailed message definitions, refer to [Controller Target Parameters](#controller-target-parameters).

- **Cartesian Targets** ([`MotionUpdate`](../aic_interfaces/aic_control_interfaces/msg/MotionUpdate.msg)): `/aic_controller/pose_commands`
- **Joint Targets** ([`JointMotionUpdate`](../aic_interfaces/aic_control_interfaces/msg/JointMotionUpdate.msg)): `/aic_controller/joint_commands`

#### Switching between joint and Cartesian target modes

To switch between joint and Cartesian control, a ROS 2 service request is sent to the service `/aic_controller/change_target_mode`. The initial target mode is **Cartesian** by default.

Send a service call to switch the controller's target mode using the ROS 2 CLI:
```bash
# Send a service request to switch to Cartesian target mode
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode "{target_mode: 0}"

# Send a service request to switch to joint target mode
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode "{target_mode: 1}"
```

> **Note:** The controller operates in mutually exclusive modes. For example, if the controller is in `Cartesian` target mdoe, it will process messages from the `/aic_controller/pose_commands` topic and ignore messages from `/aic_controller/joint_commands`. You must set the active target mode via the `/aic_controller/change_target_mode` service before the controller will accept commands of that type. See [Controller Configuration](../docs/aic_interfaces.md#Controller-Configuration) for more details.

#### State feedback

The controller publishes real-time telemetry to `/aic_controller/controller_state` ([`ControllerState`](../aic_interfaces/aic_control_interfaces/msg/ControllerState.msg)). This message includes:
- Current TCP pose and velocity
- Reference TCP pose
- Tracking error between current and reference TCP pose
- Reference joint efforts

## Controller Target Parameters

The table below shows some relevant controller target parameters that will typically need to be modified by the policy for the task.

### MotionUpdate

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `header` | `std_msgs/Header` | The `frame_id` must be set to either `gripper/tcp` (TCP frame) or `base_link` (global frame). the pose will be set to <br /> The `stamp` field needs to be populated with the latest timestamp for data synchronization. |
| `pose` | `geometry_msgs/Pose` |The target Cartesian pose for the TCP. <br /> This is used when the `trajectory_generation_mode` is set to `MODE_POSITION`. If the `frame_id` is set to `base_link`, the pose of the TCP will be relative the `base_link` of the robot. However, if the `frame_id` is set to `gripper/tcp`, then the pose values are specified as an offset to the current TCP pose. |
| `velocity` | `geometry_msgs/Twist` |The target Cartesian velocity for the TCP. <br /> This is used when the `trajectory_generation_mode` is set to `MODE_VELOCITY`. Depending on the `frame_id` specified, the target velocities will be relative to that frame. |
| `target_stiffness` | `float64[36]` | The 6x6 stiffness matrix that controls how strongly the robot resists deviations from the target pose. <br /> Higher values correspond to stiffer control and lower values corresponds to more compliant control. |
| `target_damping` | `float64[36]` | The 6x6 damping matrix controls the suppression of oscillations. <br /> The damping term is usually tuned relative to `target_stiffness` to ensure stability (e.g., critical damping) and prevent oscillations.|
| `feedforward_wrench_at_tip` | `geometry_msgs/Wrench` | An optional external wrench term at the TCP frame. <br /> This might be useful for contact tasks such as applying a constant downward force or compensating for known tool-environment interactions. |
| `wrench_feedback_gains_at_tip` | `geometry_msgs/Wrench` | Feedback gains on wrench measured by force torque sensor. |
| `trajectory_generation_mode` | `TrajectoryGenerationMode` | Defines how the target is interpreted. <br /> `MODE_POSITION` tracks the `pose` target values. <br /> `MODE_VELOCITY` tracks the `velocity` target values. |

#### Examples

To publish a pose target via the [`MotionUpdate`](../aic_interfaces/aic_control_interfaces/msg/MotionUpdate.msg) message using the ROS 2 CLI:
```bash
# Send a service request to switch to Cartesian target mode
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode "{target_mode: 0}"

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
  wrench_feedback_gains_at_tip: {
    force: {x: 0.0, y: 0.0, z: 0.0},
    torque: {x: 0.0, y: 0.0, z: 0.0}
  },
  trajectory_generation_mode: {mode: 2}
}"
```

Similarly, publishing a velocity target is as simple as:
```bash
# The command below will move the TCP at a constant velocity of 0.025 m/s along the x-axis and rotate the TCP about the z-axis at 0.25 rad/s . That is, until another pose or velocity target overrides it:
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
| `target_state` | `trajectory_msgs/JointTrajectoryPoint` | The target joint configurations for each robot joint. <br /> The `positions` field is used when the `trajectory_generation_mode` is set to `MODE_POSITION`. <br /> The `velocities` field is used when the `trajectory_generation_mode` is set to `MODE_VELOCITY`. |
| `target_stiffness` | `float64[]` | The stiffness vector for each robot joint. <br /> Higher values correspond to stiffer control and lower values corresponds to more compliant control. The size should be equal to the number of robot joints. |
| `target_damping` | `float64[]` | The damping vector for each robot joint. <br /> The damping term is usually tuned relative to `target_stiffness` to ensure stability (e.g., critical damping) and prevent oscillations. The size should be equal to the number of robot joints. |
| `target_feedforward_torque` | `float64[]` | An optional external torque term for each robot joint.  <br /> This might be useful for contact tasks such as applying a constant downward force or compensating for known tool-environment interactions. |
| `trajectory_generation_mode` | `TrajectoryGenerationMode` | Defines how the target is interpreted. <br /> `MODE_POSITION` tracks the `target_state.positions` target values. <br /> `MODE_VELOCITY` tracks the `target_state.velocities` target values. |

#### Examples

Refer to the `generate_joint_motion_update()` function within [test_impedance.py](../aic_bringup/scripts/test_impedance.py).

To publish a joint position target via the [`JointMotionUpdate`](../aic_interfaces/aic_control_interfaces/msg/JointMotionUpdate.msg) message using the ROS 2 CLI:
```bash
# Send a service request to switch to joint target mode
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode "{target_mode: 1}"

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
# The command below will rotate all joints at a constant velocity of 0.025 rad/s until another position or velocity target overrides it:
ros2 topic pub --once /aic_controller/joint_commands aic_control_interfaces/msg/JointMotionUpdate "{
  target_state: {
    velocities: [0.025, 0.025, 0.025, 0.025, 0.025, 0.025]
  },
  target_stiffness: [85.0, 85.0, 85.0, 85.0, 85.0, 85.0],
  target_damping: [75.0, 75.0, 75.0, 75.0, 75.0, 75.0], trajectory_generation_mode: {mode: 1}
}"
```

## Controller Configuration Parameters

The `aic_controller` uses the ROS 2 parameter interface to define constants used in target clamping, interpolation and impedance control. These are defined within [aic_controller_parameters.yaml](../aic_controller/src/aic_controller_parameters.yaml) along with their descriptions and expected data types.

> **Note:** The configuration is fixed during the eval stage and all participants will use the same controller settings.
