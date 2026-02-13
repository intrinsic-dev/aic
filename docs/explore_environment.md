# Explore Environment

This guide shows you how to customize and test the environment using the eval container and Pixi workspace. You'll learn how to run simulations with different configurations, manually submit tasks, and monitor results.

> [!TIP]
> Prefer building from source? See [Building the Evaluation Component from Source](./build_eval.md) for native installation instructions.

## Environment Configurations

### 1. Randomize Environment

You can launch the simulation in evaluation container with custom configurations. Make sure you are in aic_eval container via distrobox. Refer [Quick start eval container with pixi workspace](./getting_started.md#quick-start-eval-container-with-pixi-workspace) if you are not sure how to work with distrobox.

> [!TIP]
> To check if you current terminal is inside distrobox following
> command should result in **aic_eval**:
> ```bash
> echo $CONTAINER_ID
> ```

Here's a complete example with all available task board parameters:
```bash
/entrypoint.sh spawn_task_board:=true \
    task_board_x:=0.3 task_board_y:=-0.1 task_board_z:=1.2 \
    task_board_roll:=0.0 task_board_pitch:=0.0 task_board_yaw:=0.785 \
    lc_mount_rail_0_present:=true lc_mount_rail_0_translation:=-0.05 \
    lc_mount_rail_0_roll:=0.0 lc_mount_rail_0_pitch:=0.0 lc_mount_rail_0_yaw:=0.0 \
    sfp_mount_rail_0_present:=true sfp_mount_rail_0_translation:=-0.08 \
    sfp_mount_rail_0_roll:=0.0 sfp_mount_rail_0_pitch:=0.0 sfp_mount_rail_0_yaw:=0.0 \
    sc_mount_rail_0_present:=true sc_mount_rail_0_translation:=-0.09 \
    sc_mount_rail_0_roll:=0.0 sc_mount_rail_0_pitch:=0.0 sc_mount_rail_0_yaw:=0.0 \
    lc_mount_rail_1_present:=true lc_mount_rail_1_translation:=0.05 \
    lc_mount_rail_1_roll:=0.0 lc_mount_rail_1_pitch:=0.0 lc_mount_rail_1_yaw:=0.0 \
    sfp_mount_rail_1_present:=true sfp_mount_rail_1_translation:=0.08 \
    sfp_mount_rail_1_roll:=0.0 sfp_mount_rail_1_pitch:=0.0 sfp_mount_rail_1_yaw:=0.0 \
    sc_mount_rail_1_present:=true sc_mount_rail_1_translation:=0.09 \
    sc_mount_rail_1_roll:=0.0 sc_mount_rail_1_pitch:=0.0 sc_mount_rail_1_yaw:=0.0 \
    sc_port_0_present:=true sc_port_0_translation:=-0.04 \
    sc_port_0_roll:=0.0 sc_port_0_pitch:=0.0 sc_port_0_yaw:=0.0 \
    sc_port_1_present:=true sc_port_1_translation:=0.04 \
    sc_port_1_roll:=0.0 sc_port_1_pitch:=0.0 sc_port_1_yaw:=0.0 \
    nic_card_mount_0_present:=true nic_card_mount_0_translation:=0.005 \
    nic_card_mount_0_roll:=0.0 nic_card_mount_0_pitch:=0.0 nic_card_mount_0_yaw:=0.0 \
    nic_card_mount_1_present:=true nic_card_mount_1_translation:=-0.008 \
    nic_card_mount_1_roll:=0.0 nic_card_mount_1_pitch:=0.0 nic_card_mount_1_yaw:=0.0 \
    nic_card_mount_2_present:=true nic_card_mount_2_translation:=0.012 \
    nic_card_mount_2_roll:=0.0 nic_card_mount_2_pitch:=0.0 nic_card_mount_2_yaw:=0.0 \
    nic_card_mount_3_present:=true nic_card_mount_3_translation:=-0.015 \
    nic_card_mount_3_roll:=0.0 nic_card_mount_3_pitch:=0.0 nic_card_mount_3_yaw:=0.0 \
    nic_card_mount_4_present:=true nic_card_mount_4_translation:=0.01 \
    nic_card_mount_4_roll:=0.0 nic_card_mount_4_pitch:=0.0 nic_card_mount_4_yaw:=0.0 \
    spawn_cable:=true cable_type:=sfp_sc_cable attach_cable_to_gripper:=true \
    ground_truth:=true start_aic_engine:=false
```

The complete world state is automatically saved to `/tmp/aic.sdf`, which can be imported into other simulators like IsaacLab or MuJoCo for training.

**Creating multiple training scenarios:** Run the launch command with different parameter combinations to generate diverse training environments. Each launch will overwrite `/tmp/aic.sdf` with the new configuration, so copy it to a different location if you want to preserve multiple scenarios.

For the full list of configurable parameters, see the [aic_bringup README](../aic_bringup/README.md).

### 2. Manual Task Submission

For testing, you can manually submit tasks. Make sure the eval container is running.

```bash
cd ~/ws_aic/src/aic/aic_model/test
pixi run ./create_and_cancel_task.py
```

### 3. Monitoring and Results

- Watch the Gazebo window for robot movement
- Check terminal output for task progress and scoring information
- Results are saved to `$HOME/aic_results/` (or `$AIC_RESULTS_DIR` if set)

### 4. Teleoperation

If you would like to teleoperate the robot in either joint-space or Cartesian-space, refer to the [Robot Teleoperation Guide](../aic_utils/aic_teleoperation/README.md).
