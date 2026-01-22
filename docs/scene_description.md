# Scene Description

![](../../media/aic_scene.png)

The simulation environment is defined in the [`aic_description`](./../aic_description) package and comprises a robot, a task board, and various objects required for the cable insertion task. All 3D models for the scene are stored in the [`aic_assets`](./../aic_assets) package.

### Robot

The challenge utilizes a **Universal Robots UR5e** robotic arm, equipped with a **Robotiq Hand-E gripper** and an **Axia80 force-torque sensor**.

* **Configuration:** The robot's physical properties and setup are defined in the [`ur_gz.urdf.xacro`](../aic_description/urdf/ur_gz.urdf.xacro) file.
* **Control:** The robot is operated via the `aic_controller`. For detailed interface and usage instructions, please refer to the [AIC Controller documentation](./aic_controller.md).

### Task Board

The core component of the challenge is the task board, defined in [`task_board.urdf.xacro`](../aic_description/urdf/task_board.urdf.xacro). This modular platform hosts various mounts, connectors, and modules required for the tasks.

**Key Components:**
* **Connectors:** Standard fiber optic connectors including LC, SC, and SFP types.
* **NIC Cards:** Network Interface Cards.
* **Mounts:** Specialized fixtures for securing the connectors and modules.

**Spawning the Board:**
The [`aic_bringup`](../aic_bringup/) package provides launch files to spawn the task board in simulation with a default layout. You can customize this layout by passing arguments to the [`spawn_task_board.launch.py`](../aic_bringup/launch/spawn_task_board.launch.py) launch file.

### Environment

The global simulation settings—including lighting, physics properties, and general world setup—are defined in the [`aic.sdf`](../aic_description/world/aic.sdf) file.

### Simulation Launch

To view the complete scene in Gazebo, first follow the instructions in [Getting Started](./getting_started.md) to build your workspace.
Once the build is complete, launch the simulator with the following command `ros2 launch aic_bringup aic_gz_bringup.launch.py`.

The launch file does the following:

- Initializes the World: Loads Gazebo with the [`aic.sdf`](../aic_description/world/aic.sdf) environment settings.
- Parses Xacros: Processes the robot and task board Xacro files into URDF format.
- Spawns Entities: Automatically places the robot arm and the task board into the simulation scene.
- Starts various ROS 2 nodes including the Gazebo<->ROS bridge and ROS 2 controller to command the robot.
