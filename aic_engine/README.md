# AIC Engine

The AIC Engine is the orchestrator for the AI for Industry Challenge.
It manages trial execution, validates participant models, spawns task boards in simulation, and monitors task completion.

## Overview

The engine operates as a state machine that progresses through the following states:

1. **Uninitialized** → **Initialized** → **Running** → **Completed** (or **Error**)

For each trial, the engine executes these steps:

1. **Model Ready**: Validates that the participant's lifecycle node is available and in the correct state
2. **Endpoints Ready**: Ensures all required ROS nodes, topics, and services are available
3. **Simulator Ready**: Spawns the task board with configured components in Gazebo
4. **Scoring Ready**: Prepares the scoring system
5. **Task Started**: Sends task goals to the participant model
6. **Task Completed**: Monitors and validates task completion

## How It Works

### Lifecycle Node Validation

The engine validates that the participant model:
- Is a properly implemented ROS 2 lifecycle node
- Exposes standard lifecycle services (`get_state`, `change_state`)
- Starts in the `unconfigured` state
- Remains stationary (no robot movement) while unconfigured
- Rejects action goals while in the `configured` state (before activation)

### Task Board Spawning

The engine dynamically generates task boards based on YAML configuration files, supporting:
- Configurable pose (position and orientation)
- NIC card mounts on 5 rails (nic_rail_0 through nic_rail_4)
- SC ports on 2 rails (sc_rail_0 and sc_rail_1)
- LC, SFP, and SC mounts on 6 mount rails (lc_mount_rail_0/1, sfp_mount_rail_0/1, sc_mount_rail_0/1)
- Adjustable translation and rotation for each component
- Ground truth pose publishing (optional)

### Trial Execution

Each trial is executed sequentially:
1. Load trial configuration from YAML
2. Validate configuration structure
3. Progress through trial states
4. Clean up (remove spawned entities)
5. Move to next trial or complete

## Usage

### Running the Engine

```bash
ros2 run aic_engine aic_engine --ros-args \
  -p config_file_path:=/path/to/config.yaml \
  -p model_node_name:=aic_model \
  -p ground_truth:=false \
  -p endpoint_ready_timeout_seconds:=10 \
  -p model_discovery_timeout_seconds:=30 \
  -p model_configure_timeout_seconds:=60 \
  -p use_sim_time:=true
```

### ROS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `config_file_path` | string | "" | **Required**. Path to the trial configuration YAML file |
| `model_node_name` | string | "aic_model" | Name of the participant's lifecycle node |
| `adapter_node_name` | string | "aic_adapter_node" | Name of the adapter node (future use) |
| `gripper_frame_name` | string | "gripper/tcp" | Name of the gripper frame |
| `ground_truth` | bool | false | Whether to publish ground truth poses from the task board |
| `skip_model_ready` | bool | false | Skip model readiness checks (for testing only) |
| `skip_ready_simulator` | bool | false | Skip simulator readiness and entity spawning/deletion (for testing only) |
| `endpoint_ready_timeout_seconds` | int | 10 | Timeout for waiting for required endpoints |
| `model_discovery_timeout_seconds` | int | 30 | Timeout for discovering the participant model |
| `model_configure_timeout_seconds` | int | 60 | Timeout for model configuration checks |
| `model_activate_timeout_seconds` | int | 60 | Timeout for model activation |
| `model_deactivate_timeout_seconds` | int | 60 | Timeout for model deactivation |
| `model_cleanup_timeout_seconds` | int | 60 | Timeout for model cleanup |
| `model_shutdown_timeout_seconds` | int | 60 | Timeout for model shutdown |

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `AIC_RESULTS_DIR` | `$HOME/aic_results` | Directory where scoring data and bag files will be written. If not set or empty, defaults to `$HOME/aic_results` |


### Testing

Run with a sample configuration:

```bash
ros2 run aic_engine aic_engine --ros-args \
  -p config_file_path:=$(ros2 pkg prefix aic_engine)/share/aic_engine/config/sample_config.yaml \
  -p skip_model_ready:=false \
  -p skip_ready_simulator:=false \
  -p use_sim_time:=true
```
