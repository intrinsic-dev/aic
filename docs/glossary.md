
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
- The primary challenge objective: autonomously routing and inserting fiber optic cables into network interface cards using a robotic arm with force sensing capabilities.

**Task Board**
- The physical environment where the cable insertion challenge takes place, composed of multiple zones with network interface cards, cable routing paths, and component pick locations.

**Zone (Task Board Zones)**
- Designated areas on the task board serving different functions:
  - **Zone 1:** Network Interface Cards (NICs) where cables are inserted
  - **Zone 2:** Cable routing paths and management areas
  - **Zones 3 & 4:** Pick locations for components (LC plugs, SC plugs, and SFP modules) before they are routed and inserted

**Gazebo**
- The simulation environment used for training models and evaluation. Provides realistic physics simulation of the robot and cable insertion task.

## Robot & Control

**Robot Controller (aic_controller)**
- Low-level control system managing robot motion, force management, and actuator commands. Handles both joint-space and Cartesian-space control.

**End-Effector/Gripper**
- The robot's attachment point for grasping and manipulating cables and components. Its state is monitored via `/gripper_state` topic.

**Joint-Space Control**
- Robot motion commanded using target joint configurations. Commands published to `/aic_controller/joint_commands`.

**Cartesian-Space Control**
- Robot motion commanded using target poses (position and orientation) in 3D space. Commands published to `/aic_controller/pose_commands`.

**Motion Update**
- A control command message (aic_control_interfaces/msg/MotionUpdate) specifying target pose and associated tolerances for Cartesian-space control.

**Joint Motion Update**
- A control command message (aic_control_interfaces/msg/JointMotionUpdate) specifying target joint configuration and tolerances for joint-space control.

**Impedance Control**
- Compliance-based robot control that allows force management during manipulation, critical for delicate cable insertion without damage.

## Sensing & Perception

**Sensor Fusion**
- The process of combining multiple sensor inputs (cameras, force/torque sensors, joint states) into coherent environmental understanding.

**Observation**
- A snapshot of the robot's sensory environment (aic_model_interfaces/msg/Observation) including camera images, joint states, force measurements, and transform frames.

**Force/Torque Sensor (F/T Sensor)**
- Measures forces and torques applied during manipulation. Data published to `/axia80_m20/wrench` topic, enabling sensitive force feedback control.

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
- ROS 2 action interface (aic_task_interfaces/action/InsertCable) that triggers the insertion policy to perform the cable insertion task.

**Task Message**
- ROS 2 message (aic_task_interfaces/msg/Task) describing specific parameters and state of the cable insertion task.

## Development & Submission

**Container/Docker**
- Containerization technology for packaging the participant's policy solution with all dependencies. Enables reproducible evaluation across different environments.

**Dockerfile**
- Configuration file defining how to build the participant's policy container image for submission.

**aic_engine**
- Trial orchestration system that manages experiment execution, validates participant models, and collects performance scoring data.

**Qualification Phase**
- Initial competition phase where participants train models in simulation and submit solutions for evaluation against baseline tests.

**Trial**
- A single execution of the cable insertion task under specified conditions, used to evaluate policy performance.

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

## Transformation & Coordinates

**TF (Transform Frames)**
- ROS 2 system for tracking coordinate frame relationships between robot, sensors, and environment. Enables proper sensor-to-robot-to-world coordinate conversions.

**Pose**
- Complete specification of an object's position (x, y, z) and orientation (quaternion), typically in 3D Cartesian space.

**Quaternion**
- Mathematical representation of 3D orientation using four components (x, y, z, w), avoiding gimbal lock issues inherent in Euler angles.

