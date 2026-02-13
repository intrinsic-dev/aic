# Scene Description

![](../../media/aic_scene.png)

> [!NOTE]
> This guide assumes you've completed the [Getting Started](./getting_started.md) guide and have a running evaluation environment.

The simulation environment is defined in the [`aic_description`](./../aic_description) package and comprises a robot, a task board, and various objects required for the cable insertion task. All 3D models for the scene are stored in the [`aic_assets`](./../aic_assets) package.

## Scene Components

### Robot

The challenge utilizes a **Universal Robots UR5e** robotic arm, equipped with a **Robotiq Hand-E gripper** and an **Axia80 force-torque sensor**.

* **Configuration:** The robot's physical properties and setup are defined in the [`ur_gz.urdf.xacro`](../aic_description/urdf/ur_gz.urdf.xacro) file.
* **Control:** The robot is operated via the `aic_controller`. For detailed interface and usage instructions, please refer to the [AIC Controller documentation](./aic_controller.md).

### Task Board

The core component of the challenge is the task board, defined in [`task_board.urdf.xacro`](../aic_description/urdf/task_board.urdf.xacro). This modular platform hosts various mounts, connectors, and modules required for the tasks.

**Key Components:**
* **Connectors:** Standard fiber optic connectors including SC and SFP types.
* **NIC Cards:** Network Interface Cards.
* **Mounts:** Specialized fixtures for securing the connectors and modules.

See [Task Board Description](./task_board_description.md) for detailed specifications.

### Environment

The global simulation settings—including lighting, physics properties, and general world setup—are defined in the [`aic.sdf`](../aic_description/world/aic.sdf) file.

---

## Exploring the Environment

Now that you have the basic environment running, you can explore different configurations to understand the challenge better and create diverse training scenarios.

### Customizing the Environment

You can customize the simulation environment by passing parameters to the launch command. Whether you're using the **eval container** or a **source build**, the parameters are the same.

**In the eval container (via distrobox):**
```bash
/entrypoint.sh [parameters]
```

**From source build:**
```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py [parameters]
```

> [!TIP]
> To check if your current terminal is inside the eval container:
> ```bash
> echo $CONTAINER_ID  # Should output: aic_eval
> ```

### Example: Custom Task Board Configuration

Here's a complete example spawning a task board with various components:

```bash
spawn_task_board:=true \
    task_board_x:=0.3 task_board_y:=-0.1 task_board_z:=1.2 \
    task_board_roll:=0.0 task_board_pitch:=0.0 task_board_yaw:=0.785 \
    sfp_mount_rail_0_present:=true sfp_mount_rail_0_translation:=-0.08 \
    sc_mount_rail_0_present:=true sc_mount_rail_0_translation:=-0.09 \
    nic_card_mount_0_present:=true nic_card_mount_0_translation:=0.005 \
    sc_port_0_present:=true sc_port_0_translation:=-0.04 \
    spawn_cable:=true cable_type:=sfp_sc_cable attach_cable_to_gripper:=true \
    ground_truth:=true start_aic_engine:=false
```

**Key parameters for exploration:**
- `ground_truth:=true` - Enables ground truth TF frames for easier debugging during development
- `start_aic_engine:=false` - Disables automatic trial orchestration so you can freely explore
- `spawn_task_board:=true` - Spawns the task board immediately
- `spawn_cable:=true` - Spawns a cable in the scene
- `attach_cable_to_gripper:=true` - Attaches the cable to the gripper
- `cable_type:=sfp_sc_cable` - Type of cable (options: `sfp_sc_cable`, `sfp_sc_cable_reversed`)

For the complete list of configurable parameters, see the [aic_bringup README](../aic_bringup/README.md).

### Creating Training Scenarios

**Generate diverse training environments** by varying the parameters:

1. **Launch with different configurations** to create randomized scenarios
2. **The complete world state is automatically saved** to `/tmp/aic.sdf` after spawning
3. **Copy the file to preserve multiple scenarios:**
   ```bash
   cp /tmp/aic.sdf ~/training_scenarios/scenario_001.sdf
   ```
4. **Import into other simulators** like IsaacLab or MuJoCo for training

**Example workflow:**
```bash
# Scenario 1: NIC card in slot 2
/entrypoint.sh spawn_task_board:=true nic_card_mount_2_present:=true \
    spawn_cable:=true cable_type:=sfp_sc_cable ground_truth:=true start_aic_engine:=false
cp /tmp/aic.sdf ~/training_scenarios/nic_slot_2.sdf

# Scenario 2: SC connector on right rail with different pose
/entrypoint.sh spawn_task_board:=true task_board_yaw:=1.57 \
    sc_mount_rail_1_present:=true spawn_cable:=true ground_truth:=true start_aic_engine:=false
cp /tmp/aic.sdf ~/training_scenarios/sc_right_rotated.sdf
```

### Teleoperation

To manually control the robot and get familiar with the environment:

**Teleoperate the robot** in joint-space or Cartesian-space to:
- Explore the workspace
- Test cable insertion manually
- Understand the robot's reach and limitations
- Practice with and without cable attached

Before teleoperating, we recommend reading the [AIC Controller Guide](./aic_controller.md) to understand the controller used in the challenge.

See the [Robot Teleoperation Guide](../aic_utils/aic_teleoperation/README.md) for detailed instructions.

---

## Exporting World State for AI Training

The simulation includes a world plugin that automatically exports the complete world state after all entities (robot, task board, cable) are spawned. This feature is particularly useful for AI policy training and cross-platform simulation workflows.

**Key Benefits:**
- **Reproducible Scenarios:** Capture randomized configurations created via launch parameters for consistent training environments.
- **Cross-Platform Compatibility:** Import the exported SDF file into other simulators like IsaacLab or MuJoCo.
- **Training Data Generation:** Create diverse training scenarios by varying launch parameters and exporting each configuration.

**Export Details:**
- **Default Location:** `/tmp/aic.sdf`
- **Plugin Configuration:** Defined in [`aic.sdf`](../aic_description/world/aic.sdf) with parameters:
  - `<save_world_path>`: Path where the world file is saved (default: `/tmp/aic.sdf`)
  - `<save_world_delay_s>`: Delay in simulation seconds before exporting (default: `0.0`)

---

## Next Steps

Now that you understand the scene:

- **Develop your policy:** See [Policy Integration Guide](./policy.md)
- **Understand the interfaces:** Review [AIC Interfaces](./aic_interfaces.md)
- **Learn about scoring:** Read [Scoring](./scoring.md)
- **Explore example policies:** Check out [`aic_example_policies/`](../aic_example_policies/)
