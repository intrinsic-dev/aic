# AIC Bringup Package

This package contains launch files for bringing up the AI for Industry Challenge (AIC) simulation environment.

## Launch Files

### 1. `aic_gz_bringup.launch.py`

Main launch file that brings up the complete AIC simulation environment including Gazebo, UR robot, task board, and cable.

#### Usage
```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

#### Configurable Parameters

**Robot Spawn Position:**
- `robot_x` (default: `"0.0"`) - Robot spawn X position (meters)
- `robot_y` (default: `"0.0"`) - Robot spawn Y position (meters)
- `robot_z` (default: `"0.0"`) - Robot spawn Z position (meters)
- `robot_roll` (default: `"0.0"`) - Robot spawn roll orientation (radians)
- `robot_pitch` (default: `"0.0"`) - Robot spawn pitch orientation (radians)
- `robot_yaw` (default: `"0.0"`) - Robot spawn yaw orientation (radians)

**Controller Configuration:**
- `controllers_file` (default: `"ur_controllers.yaml"`) - YAML file with controller configuration
- `activate_joint_controller` (default: `"true"`) - Activate joint controller on startup
- `initial_joint_controller` (default: `"aic_controller"`) - Initial controller to activate
- `description_file` (default: `"ur.urdf.xacro"`) - Robot description file
- `spawn_admittance_controller` (default: `"false"`) - Spawns the Admittance Controller alongside the initial controller defined by the `initial_joint_controller` parameter. This value should be set to `false` if using the impedance control mode on the `aic_controller`.

**Task Board Configuration:**
- `spawn_task_board` (default: `"false"`) - Whether to spawn the task board
- `task_board_description_file` (default: `"task_board.urdf.xacro"`) - Task board URDF/XACRO file
- `task_board_x` (default: `"0.25"`) - Task board spawn X position (meters)
- `task_board_y` (default: `"0.0"`) - Task board spawn Y position (meters)
- `task_board_z` (default: `"1.14"`) - Task board spawn Z position (meters)
- `task_board_roll` (default: `"0.0"`) - Task board spawn roll orientation (radians)
- `task_board_pitch` (default: `"0.0"`) - Task board spawn pitch orientation (radians)
- `task_board_yaw` (default: `"0.0"`) - Task board spawn yaw orientation (radians)

**Cable Configuration:**
- `spawn_cable` (default: `"false"`) - Whether to spawn the cable
- `cable_description_file` (default: `"cable.sdf.xacro"`) - Cable SDF/XACRO file
- `attach_cable_to_gripper` (default: `"false"`) - Whether to attach cable to gripper
- `cable_type` (default: `"sfp_sc_cable"`) - Type of cable to spawn. Options: [`sfp_sc_cable`, `sfp_sc_cable_reversed`]
- `cable_x` (default: `"0.1956"`) - Cable spawn X position (meters)
- `cable_y` (default: `"-0.2112"`) - Cable spawn Y position (meters)
- `cable_z` (default: `"1.53"`) - Cable spawn Z position (meters)
- `cable_roll` (default: `"0.4432"`) - Cable spawn roll orientation (radians)
- `cable_pitch` (default: `"-0.4838"`) - Cable spawn pitch orientation (radians)
- `cable_yaw` (default: `"-1.8112"`) - Cable spawn yaw orientation (radians)

**Gazebo Configuration:**
- `world_file` (default: `"aic.sdf"`) - Gazebo world file
- `gazebo_gui` (default: `"true"`) - Launch Gazebo GUI
- `ros_gz_bridge_config_file` (default: `"ros_gz_bridge.yaml"`) - ROS-Gazebo bridge configuration

**Visualization:**
- `launch_rviz` (default: `"false"`) - Launch RViz for visualization
- `rviz_config_file` (default: `"view_robot.rviz"`) - RViz configuration file


**Ground Truth:**
- `ground_truth` (default: `"false"`) - Include ground truth pose data in TF topics


**AIC Engine:**
- `start_aic_engine` (default: `"false"`) - Start the `aic_engine` orchestrator node for evaluation.
- `aic_engine_config_file` (default: `"aic_engine/config/sample_config.yaml"`) - Absolute path to YAML file with the AIC engine configuration.

---

### 2. `spawn_task_board.launch.py`

Standalone launch file for spawning the task board in an existing Gazebo simulation.

#### Usage
```bash
ros2 launch aic_bringup spawn_task_board.launch.py
```

#### Configurable Parameters

**Task Board Base Configuration:**
- `task_board_description_file` (default: `"task_board.urdf.xacro"`) - Task board URDF/XACRO description file
- `task_board_x` (default: `"0.25"`) - Task board spawn X position (meters)
- `task_board_y` (default: `"0.0"`) - Task board spawn Y position (meters)
- `task_board_z` (default: `"1.14"`) - Task board spawn Z position (meters)
- `task_board_roll` (default: `"0.0"`) - Task board spawn roll orientation (radians)
- `task_board_pitch` (default: `"0.0"`) - Task board spawn pitch orientation (radians)
- `task_board_yaw` (default: `"0.0"`) - Task board spawn yaw orientation (radians)

**Mount Rails (LC/SFP/SC):**

The task board has 6 mount rails for LC, SFP, and SC connector mounts. Each rail has configurable presence, translation along the rail, and orientation.

*LC Mount Rail 0 (left side):*
- `lc_mount_rail_0_present` (default: `"false"`) - Whether LC mount is present on rail 0
- `lc_mount_rail_0_translation` (default: `"0.0"`) - Translation along rail (meters, range: -0.09625 to 0.09625)
- `lc_mount_rail_0_roll` (default: `"0.0"`) - Roll orientation (radians)
- `lc_mount_rail_0_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `lc_mount_rail_0_yaw` (default: `"0.0"`) - Yaw orientation (radians)

*SFP Mount Rail 0 (left side):*
- `sfp_mount_rail_0_present` (default: `"false"`) - Whether SFP mount is present on rail 0
- `sfp_mount_rail_0_translation` (default: `"0.0"`) - Translation along rail (meters, range: -0.09625 to 0.09625)
- `sfp_mount_rail_0_roll` (default: `"0.0"`) - Roll orientation (radians)
- `sfp_mount_rail_0_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `sfp_mount_rail_0_yaw` (default: `"0.0"`) - Yaw orientation (radians)

*SC Mount Rail 0 (left side):*
- `sc_mount_rail_0_present` (default: `"false"`) - Whether SC mount is present on rail 0
- `sc_mount_rail_0_translation` (default: `"0.0"`) - Translation along rail (meters, range: -0.09625 to 0.09625)
- `sc_mount_rail_0_roll` (default: `"0.0"`) - Roll orientation (radians)
- `sc_mount_rail_0_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `sc_mount_rail_0_yaw` (default: `"0.0"`) - Yaw orientation (radians)

*LC Mount Rail 1 (right side):*
- `lc_mount_rail_1_present` (default: `"false"`) - Whether LC mount is present on rail 1
- `lc_mount_rail_1_translation` (default: `"0.0"`) - Translation along rail (meters, range: -0.09625 to 0.09625)
- `lc_mount_rail_1_roll` (default: `"0.0"`) - Roll orientation (radians)
- `lc_mount_rail_1_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `lc_mount_rail_1_yaw` (default: `"0.0"`) - Yaw orientation (radians)

*SFP Mount Rail 1 (right side):*
- `sfp_mount_rail_1_present` (default: `"false"`) - Whether SFP mount is present on rail 1
- `sfp_mount_rail_1_translation` (default: `"0.0"`) - Translation along rail (meters, range: -0.09625 to 0.09625)
- `sfp_mount_rail_1_roll` (default: `"0.0"`) - Roll orientation (radians)
- `sfp_mount_rail_1_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `sfp_mount_rail_1_yaw` (default: `"0.0"`) - Yaw orientation (radians)

*SC Mount Rail 1 (right side):*
- `sc_mount_rail_1_present` (default: `"false"`) - Whether SC mount is present on rail 1
- `sc_mount_rail_1_translation` (default: `"0.0"`) - Translation along rail (meters, range: -0.09625 to 0.09625)
- `sc_mount_rail_1_roll` (default: `"0.0"`) - Roll orientation (radians)
- `sc_mount_rail_1_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `sc_mount_rail_1_yaw` (default: `"0.0"`) - Yaw orientation (radians)

**SC Port Rails:**

Two SC port rails for attaching SC port modules.

*SC Port 0:*
- `sc_port_0_present` (default: `"false"`) - Whether SC port is present on rail 0
- `sc_port_0_translation` (default: `"0.0"`) - Translation along rail (meters)
- `sc_port_0_roll` (default: `"0.0"`) - Roll orientation (radians)
- `sc_port_0_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `sc_port_0_yaw` (default: `"0.0"`) - Yaw orientation (radians)

*SC Port 1:*
- `sc_port_1_present` (default: `"false"`) - Whether SC port is present on rail 1
- `sc_port_1_translation` (default: `"0.0"`) - Translation along rail (meters)
- `sc_port_1_roll` (default: `"0.0"`) - Roll orientation (radians)
- `sc_port_1_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `sc_port_1_yaw` (default: `"0.0"`) - Yaw orientation (radians)

**NIC Card Mount Rails:**

Five NIC card mount rails for attaching network interface cards.

*NIC Card Mount 0-4:* (Parameters repeat for each mount: 0, 1, 2, 3, 4)
- `nic_card_mount_N_present` (default: `"false"`) - Whether NIC card mount N is present
- `nic_card_mount_N_translation` (default: `"0.0"`) - Translation along rail (meters)
- `nic_card_mount_N_roll` (default: `"0.0"`) - Roll orientation (radians)
- `nic_card_mount_N_pitch` (default: `"0.0"`) - Pitch orientation (radians)
- `nic_card_mount_N_yaw` (default: `"0.0"`) - Yaw orientation (radians)

---

### 3. `spawn_cable.launch.py`

Standalone launch file for spawning a cable in an existing Gazebo simulation.

#### Usage
```bash
ros2 launch aic_bringup spawn_cable.launch.py
```

#### Configurable Parameters

- `cable_description_file` (default: `"cable.sdf.xacro"`) - Cable URDF/XACRO description file
- `cable_x` (default: `"-0.35"`) - Cable spawn X position (meters)
- `cable_y` (default: `"0.4"`) - Cable spawn Y position (meters)
- `cable_z` (default: `"1.15"`) - Cable spawn Z position (meters)
- `cable_roll` (default: `"0.0"`) - Cable spawn roll orientation (radians)
- `cable_pitch` (default: `"0.0"`) - Cable spawn pitch orientation (radians)
- `cable_yaw` (default: `"0.0"`) - Cable spawn yaw orientation (radians)
- `attach_cable_to_gripper` (default: `"false"`) - Whether to attach cable to gripper

---

### 4. `gripper_action.launch.py`

Launch file for sending gripper action commands to control the gripper position and/or effort.

#### Usage
```bash
ros2 launch aic_bringup gripper_action.launch.py use_position:=true position:=0.5
```

#### Configurable Parameters

- `gripper_name` (default: `"gripper"`) - Name of the gripper
- `gripper_action_name` (default: `"/gripper_action_controller/gripper_cmd"`) - Action server name for gripper control
- `use_position` (default: `"false"`) - If enabled, send a position command
- `position` (default: `"0.0"`) - Gripper position command (0.0 = closed, 1.0 = open)
- `use_effort` (default: `"false"`) - If enabled, send an effort command
- `effort` (default: `"0.0"`) - Gripper effort command (Newtons)

**Note:** At least one of `use_position` or `use_effort` should be set to `true` for the action to have effect.

---

### 5. `move_to_contact.launch.py`

Launch file for moving the robot tool frame until contact is detected based on force feedback.

#### Usage
```bash
ros2 launch aic_bringup move_to_contact.launch.py contact_force_z:=15.0
```

#### Configurable Parameters

- `controller_namespace` (default: `"admittance_controller"`) - Namespace for the admittance controller node
- `tool_frame` (default: `"tool0"`) - Tool frame for the move-to-contact behavior
- `contact_force_z` (default: `"10.0"`) - Contact detection threshold force along Z-axis (Newtons)

---

## Example Usage

### Basic Simulation Launch
Start the complete simulation with default parameters:
```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

### Launch with Custom Robot Position
```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py robot_x:=1.0 robot_y:=0.5
```

### Launch with Task Board and Cable
```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py spawn_task_board:=true spawn_cable:=true
```

### Spawn Task Board with LC Mount on Rail 0
```bash
ros2 launch aic_bringup spawn_task_board.launch.py \
  lc_mount_rail_0_present:=true \
  lc_mount_rail_0_translation:=0.05
```

### Open Gripper
```bash
ros2 launch aic_bringup gripper_action.launch.py use_position:=true position:=0.8
```

---

## Launching with the Impedance Controller

### Start the Simulation with the AIC Controller

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

### Start a script to send both joint (`JointMotionUpdate`) and Cartesian targets (`MotionUpdate`).

This script also calls the `ChangeTargetMode` service to change the target mode between Joint and Cartesian, in between sending the targets.
```bash
ros2 run aic_bringup test_impedance.py
```

---

## Notes

- All position values are in meters
- All orientation values are in radians
- Translation ranges for mount rails are limited to prevent collisions: -0.09625 to 0.09625 meters
- Mount rails are type-specific: LC, SFP, and SC mounts can only be attached to their respective rails
- Port rails (sc_port and nic_card_mount) are separate from mount rails
