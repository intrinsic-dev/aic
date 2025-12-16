# Scene Description

The simulation environment is defined in the [aic_description](./aic_description) package and includes a robot, a task board, and various objects relevant to the cable insertion task. The 3D models for the scene are located in the [aic_assets](./aic_assets) package.

### Robot

The robot used in this challenge is a Universal Robots UR5e, equipped with a Robotiq Hand-E gripper and an Axia80 force-torque sensor. The robot's configuration is described in the `ur_gz.urdf.xacro` file.

### Task Board

The central element of the challenge is the task board, described in `task_board.urdf.xacro`. The task board is a modular platform where various connectors and modules are mounted. The following components can be found on the task board:

*   **LC, SC, and SFP Connectors**: Standard fiber optic connectors.
*   **NIC Cards**: Network Interface Cards.
*   **Mounts**: Various mounts for the different connectors and modules.

The `aic_bringup` package contains launch files to spawn the task board in the simulation with a default layout. The layout can be customized by passing arguments to the `spawn_task_board.launch.py` launch file.

### Environment

The overall simulation world is defined in `aic.sdf`, which includes lighting, physics properties, and the general setup of the environment.
