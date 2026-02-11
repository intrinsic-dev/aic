# aic_controller

The [aic_controller](../aic_controller/) package provides a robust controller within the ROS2 control framework. It acts as a high-frequency bridge between low-frequency planning/policy layers and the robot's hardware interface by interpolating sparse joint and cartesian-space commands then performing impedance control for a robot manipulator.

## Architecture

A high-level overview of its architecture is provided in the following diagram:
<img width="1889" height="437" alt="image" src="../../media/aic_controller.png" />

The controller subscribes to targets (either Joint or Cartesian) typically published by a control policy or planner at $\approx 10\text{ Hz}$. To ensure smooth and safe execution at the hardware level ($\approx 500\text{ Hz}$), these targets undergo a pipeline of safety clamping and interpolation before the impedance control law is applied.

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
- $\tau \in \mathbb{R}^n$: Cmoputed joint control torque.
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
- $\tau \in \mathbb{R}^n$: Cmoputed joint control torque.
- $\mathbf{K}_p, \mathbf{K}_d \in \mathbb{R}^n$: Joint stiffness and damping vectors.
- $\mathbf{q}_{des}, \mathbf{q} \in \mathbb{R}^n$: Desired and current joint positions.
- $\dot{\mathbf{q}}_{des}, \dot{\mathbf{q}} \in \mathbb{R}^n$: Desired and current joint velocities.
- $\tau_f \in \mathbb{R}^n$: Feedforward joint torque.


### ROS2 Interfaces

#### Command Interfaces

The `aic_controller` accepts commands via two ROS2 Topics. For detailed message definitions, refer to [Controller Target Parameters](#Controller-Target-Parameters).

- **Joint Targets**: `/aic_controller/joint_commands`
- **Cartesian Targets**: `/aic_controller/pose_commands`

#### Switching between joint and Cartesian target modes

To switch between joint and Cartesian control, a ROS2 service request is sent to the service `/aic_controller/change_target_mode`.

> **Note:** The controller operates in mutually exclusive modes. For example, if the controller is in `Cartesian` target mdoe, it will process messages from the `/aic_controller/pose_commands` topic and ignore messages from ``/aic_controller/joint_commands`. You must set the active target mode via the `/aic_controller/change_target_mode` service before the controller will accept commands of that type. See [Controller Configuration](../docs/aic_interfaces.md#Controller-Configuration) for more details.

#### State feedback

The controller publishes real-time telemetry to `/aic_controller/controller_state`. This message includes:
- Current TCP pose and velocity
- Reference TCP pose
- Tracking error between current and reference TCP pose
- Reference joint efforts

## Controller Target Parameters

The table below shows some relevant controller target parameters that will typically need to be modified by the policy for the task.

### MotionUpdate

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `header` | `std_msgs/Header` | The `frame_id` must be set to either `gripper/tcp` (TCP frame) or `base_link` (global frame). <br /> The `stamp` field needs to be populated with the latest timestamp for data synchronization. |
| `pose` | `geometry_msgs/Pose` |The target Cartesian pose. <br /> This is used when the `trajectory_generation_mode` is set to `MODE_POSITION`. |
| `velocity` | `geometry_msgs/Twist` |The target Cartesian velocity. <br /> This is used when the `trajectory_generation_mode` is set to `MODE_VELOCITY`. |
| `target_stiffness` | `float64[36]` | The 6x6 stiffness matrix that controls how strongly the robot resists deviations from the target pose. <br /> Higher values correspond to stiffer control and lower values corresponds to more compliant control. |
| `target_damping` | `float64[36]` | The 6x6 damping matrix controls the suppression of oscillations. <br /> The damping term is usually tuned relative to `target_stiffness` to ensure stability (e.g., critical damping) and prevent oscillations.|
| `feedforward_wrench_at_tip` | `geometry_msgs/Wrench` | An optional external wrench term at the TCP frame. <br /> This might be useful for contact tasks such as applying a constant downward force or compensating for known tool-environment interactions. |
| `wrench_feedback_gains_at_tip` | `geometry_msgs/Wrench` | Feedback gains on wrench measured by force torque sensor. |
| `trajectory_generation_mode` | `TrajectoryGenerationMode` | Defines how the target is interpreted. <br /> `MODE_POSITION` tracks the `pose` target values. <br /> `MODE_VELOCITY` tracks the `velocity` target values. |

### JointMotionUpdate

| Parameter |  Type | Description |
| :--- | :--- | :--- |
| `target_state` | `trajectory_msgs/JointTrajectoryPoint` | The target joint configurations for each robot joint. <br /> The `positions` field is used when the `trajectory_generation_mode` is set to `MODE_POSITION`. <br /> The `velocities` field is used when the `trajectory_generation_mode` is set to `MODE_VELOCITY`. |
| `target_stiffness` | `float64[]` | The stiffness vector for each robot joint. <br /> Higher values correspond to stiffer control and lower values corresponds to more compliant control. The size should be equal to the number of robot joints. |
| `target_damping` | `float64[]` | The damping vector for each robot joint. <br /> The damping term is usually tuned relative to `target_stiffness` to ensure stability (e.g., critical damping) and prevent oscillations. The size should be equal to the number of robot joints. |
| `target_feedforward_torque` | `float64[]` | An optional external torque term for each robot joint.  <br /> This might be useful for contact tasks such as applying a constant downward force or compensating for known tool-environment interactions. |
| `trajectory_generation_mode` | `TrajectoryGenerationMode` | Defines how the target is interpreted. <br /> `MODE_POSITION` tracks the `target_state.positions` target values. <br /> `MODE_VELOCITY` tracks the `target_state.velocities` target values. |

## Controller Configuration Parameters

The `aic_controller` uses the ROS2 parameter interface to define constants used in target clamping, interpolation and impedance control. These are defined within [aic_controller_parameters.yaml](../aic_controller/src/aic_controller_parameters.yaml) along with their descriptions and expected data types. An example configuration can be found within [aic_ros2_controllers.yaml](../aic_bringup/config/aic_ros2_controllers.yaml).
