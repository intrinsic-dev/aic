
# AIC Toolkit Glossary

## Core Architecture Terms

**Adapter (aic_adapter)**
- A bridge component that handles sensor fusion and data synchronization between the robot hardware and the participant's policy implementation. Processes multi-source sensor data into unified observations.

**Evaluation Component**
- The infrastructure provided by challenge organizers that includes the orchestration engine, launch systems, robot controller, and sensor pipeline. Participants do not modify this component.

**Participant Model Component (aic_model)**
- The policy implementation developed by challenge participants. This is where custom logic processes sensor data and commands the robot to perform cable insertion tasks.

**Policy**
- The algorithm or AI model implemented by participants that processes sensor observations and generates robot motion commands. Submitted as a containerized solution.

## Task & Environment

**Cable Insertion Task**
- The primary challenge objective: autonomously routing and inserting fiber optic cables into network interface cards using a robotic arm with force sensing capabilities. During qualification trials, only one plug end of the cable is tested for insertion while the other end remains free and unconnected.

**Task Board**
- The modular, reconfigurable platform where the cable insertion challenge takes place. Divided into four functional zones: assembly targets (Zones 1 & 2) and pick locations (Zones 3 & 4) with randomizable component positions and orientations.

**Zone (Task Board Zones)**
- Four functional areas on the task board that simulate a complete electronics assembly workflow:
  - **Zone 1:** Network Interface Cards (NICs) with SFP ports - represents the server compute tray where data links are established
  - **Zone 2:** SC optical ports - emulates the optical patch panel or backplane of a server rack
  - **Zones 3 & 4:** Pick locations - organized supply areas where SFP modules, LC plugs, and SC plugs are staged on adjustable mounts before routing and insertion

**Gazebo**
- The primary simulation environment used for evaluation and training. Provides realistic physics simulation of the robot and cable insertion task. World state can be exported to `/tmp/aic.sdf` for use in other simulators.

## Connector Types & Components

**SFP (Small Form-factor Pluggable)**
- Transceiver module type used in network interface cards. The SFP module end of the cable plugs into SFP ports on NIC cards in Zone 1.

**LC (Lucent Connector)**
- Fiber optic connector type with a small form factor. LC plugs are staged in pick locations (Zones 3 & 4) for cable assembly tasks.

**SC (Subscriber Connector)**
- Fiber optic connector type with a larger form factor than LC. SC plugs insert into SC ports in Zone 2, emulating optical patch panel connections.

**NIC (Network Interface Card)**
- Network hardware component with SFP ports that serve as primary insertion targets in Zone 1. Up to five dual-port NICs can be mounted on adjustable rails.

**Port**
- Insertion target on the task board. Types include SFP ports (on NIC cards) and SC ports (on optical panel). Each port requires precise alignment for successful insertion.

**Plug**
- The connector end of a cable being manipulated and inserted. Types include SFP modules, LC plugs, and SC plugs. The robot grasps one plug while the other end remains free during insertion.

**Rails**
- Adjustable mounting tracks on the task board allowing components to slide with randomized positions:
  - **NIC Rails** (5 total): Hold network interface cards with translation range [0, 0.062] meters and rotation [-10, +10] degrees
  - **SC Rails** (2 total): Hold SC ports with translation range [0, 0.115] meters
  - **Fixture Rails** (in Zones 3 & 4): Hold component mounts with translation range [0, 0.188] meters and rotation [-60, +60] degrees

**Fixture/Mount**
- Specialized holders securing components on the task board. In Zones 3 & 4, fixtures hold LC plugs, SC plugs, and SFP modules in organized pick locations with adjustable positions and orientations.

## Robot & Control

**Robot Controller (aic_controller)**
- Low-level control system managing robot motion, force management, and actuator commands. Handles both joint-space and Cartesian-space control.

**End-Effector/Gripper**
- The robot's attachment point for grasping and manipulating cables and components, also known as the **Tool Center Point (TCP)**, with the corresponding frame being `gripper/tcp`. The gripper state is monitored via the `/gripper_state` topic. 

**Joint-Space Control**
- Robot motion commanded using target joint configurations. Commands published to `/aic_controller/joint_commands`.

**Cartesian-Space Control**
- Robot motion commanded using target poses (position and orientation) or linear/angular velocities in 3D space. Commands published to `/aic_controller/pose_commands`.

**Motion Update**
- A control command message (`aic_control_interfaces/msg/MotionUpdate`) specifying target pose and control parameters for Cartesian-space control.

**Joint Motion Update**
- A control command message (`aic_control_interfaces/msg/JointMotionUpdate`) specifying target joint configuration and control parameters for joint-space control.

**Impedance Control**
- Compliance-based robot control that allows force management during manipulation, critical for delicate cable insertion without damage.

## Sensing & Perception

**Sensor Fusion**
- The process of combining multiple sensor inputs (cameras, force/torque sensors, joint states) into coherent environmental understanding.

**Observation**
- A snapshot of the robot's sensory environment (aic_model_interfaces/msg/Observation) including camera images, joint states, force measurements, and transform frames.

**Force/Torque Sensor (F/T Sensor)**
- Measures forces and torques applied during manipulation. Data published to `/fts_broadcaster/wrench` topic, enabling sensitive force feedback control. The F/T sensor is typically tared before each training episode, see [Taring before Training](../docs/scene_description.md#Taring-before-training) for more details.

**Wrist Cameras**
- Three RGB cameras mounted on the robot's wrist:
  - Left camera (`/left_camera/image`)
  - Center camera (`/center_camera/image`)
  - Right camera (`/right_camera/image`)
- Each provides calibration data via corresponding `/camera_info` topics.

**Joint State**
- Current configuration and velocity of all robot joints, published to `/joint_states` topic.

## Communication & Interfaces

**ROS 2 (Robot Operating System 2)**
- The middleware framework enabling communication between all toolkit components through topics, services, and actions.

**Topic**
- A named channel for pub/sub message passing between ROS 2 nodes. Used for sensor streaming and command broadcasting.

**Action**
- A ROS 2 communication pattern enabling goal-based requests with feedback and results. Used for task triggering (e.g., `/insert_cable`).

**Message (ROS Message)**
- Structured data format for ROS communication. Examples: `InsertCable.action`, `Task.msg`, `MotionUpdate.msg`.

**aic_interfaces**
- Package defining all custom ROS 2 messages, services, and actions used in the challenge for consistent protocol definitions.

**InsertCable Action**
- ROS 2 action interface (`aic_task_interfaces/action/InsertCable`) that triggers the insertion policy to perform the cable insertion task.

**Task Message**
- ROS 2 message (`aic_task_interfaces/msg/Task`) describing specific parameters and state of the cable insertion task.

## Development & Submission

**Container/Docker**
- Containerization technology for packaging the participant's policy solution with all dependencies. Enables reproducible evaluation across different environments.

**Dockerfile**
- Configuration file defining how to build the participant's policy container image for submission.

**aic_engine**
- Trial orchestration system that manages the complete trial lifecycle including: spawning randomized task boards, validating policy behavior and lifecycle conformance, monitoring task execution, triggering the InsertCable action, and collecting performance scoring data from multiple ROS 2 topics.

**Qualification Phase**
- Initial simulation-only competition phase where participants train models and submit solutions for evaluation against three specific trials. Conducted entirely in Gazebo, with mirror environments available in IsaacLab (NVIDIA) and MuJoCo (Google DeepMind) for training robust policies through domain randomization.

**Trial**
- A single execution of the cable insertion task where only one plug end of the cable is tested for insertion into a randomized target port. Each trial evaluates policy performance under specific board configurations and component placements.

## Specialized Components

**aic_bringup**
- Launch file package containing configurations to start the complete challenge environment including simulation, robot, sensors, and scoring.

**aic_example_policies**
- Reference implementations demonstrating different approaches and techniques for solving the cable insertion task.

**aic_description**
- Contains URDF/SDF descriptions of the robot, task board, and environment for simulation purposes.

**aic_assets**
- Repository of 3D models and visual assets used in the Gazebo simulation environment.

**aic_scoring**
- System implementation for calculating performance metrics and evaluating trial success according to challenge criteria.

**aic_utils**
- Utility packages and helper tools used across the toolkit.

## Simulation & Training

**IsaacLab**
- NVIDIA's alternative simulation environment providing a mirror of the AIC challenge for policy training. Enables domain randomization across different physics engines to improve sim-to-real transfer.

**MuJoCo**
- Google DeepMind's alternative simulation environment providing another mirror of the AIC challenge. Used alongside Gazebo and IsaacLab for multi-simulator training strategies.

**Domain Randomization**
- Training technique where policies are exposed to variations across different simulators (Gazebo, IsaacLab, MuJoCo). Physical discrepancies between simulators serve as natural randomization, preparing models for sim-to-sim-to-real transfer.

## Evaluation Metrics

**Jerk**
- Smoothness metric computed as the rate of change of joint accelerations (third derivative of position). Calculated from `/joint_states` topic data. Lower jerk values indicate smoother, more controlled trajectories and receive higher scores.

**Convergence**
- Performance metric measuring the Euclidean distance between plug tip and target port over time. Sourced from `/ground_truth_poses` topic. Higher scores awarded for minimizing distance and achieving faster convergence rates.

**Task Completion Time**
- Duration from task start to successful insertion completion. Monitored via `TaskState` messages from aic_engine. Faster completion times receive higher scores.

**Smoothness**
- Overall trajectory quality metric based on jerk calculations. Evaluates how gracefully the robot moves during insertion attempts.

**Success Rate**
- Binary metric indicating successful cable insertion with correct alignment and full seating within tolerance thresholds. Verified via contact sensors, force/torque feedback, and Gazebo plugins. Forms the primary Tier 3 evaluation criterion.

**Collision Penalty**
- Score deduction for unintended contact with environment or task board. Detected by custom Gazebo plugin monitoring all contacts (excluding intentional cable-related contacts). Severity proportional to collision frequency and force.

**Force Safety**
- Monitoring of forces applied during insertion to ensure they remain within component safety limits. Excessive forces beyond thresholds result in score deductions based on magnitude and duration.

**Command Safety**
- Validation of motion commands sent to aic_controller for excessive values. Monitored from `MotionUpdate` and `JointMotionUpdate` messages. Unsafe command values result in score deductions.

## Transformation & Coordinates

**TF (Transform Frames)**
- ROS 2 system for tracking coordinate frame relationships between robot, sensors, and environment. Enables proper sensor-to-robot-to-world coordinate conversions.

**Pose**
- Complete specification of an object's position (x, y, z) and orientation (quaternion), typically in 3D Cartesian space.

**Quaternion**
- Mathematical representation of 3D orientation using four components (x, y, z, w), avoiding gimbal lock issues inherent in Euler angles.
