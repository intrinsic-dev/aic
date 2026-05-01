# Cloud Setup

Steps to get the AIC eval running on a fresh Ubuntu 24.04 cloud instance with an NVIDIA GPU.

## Requirements

- Ubuntu 24.04
- NVIDIA GPU (RTX 2070+ or equivalent, 8GB+ VRAM)
- sudo access

## One-Time Setup

```bash
git clone <your-repo-url>
cd aic-lift
./scripts/setup_cloud.sh
```

This installs Docker, NVIDIA Container Toolkit, Distrobox, and Pixi, then pulls the `aic_eval` image and creates the distrobox container.

> If Docker was just installed, log out and back in before continuing so the `docker` group takes effect.

## Running the Eval

Open two terminals.

**Terminal 1 — start the simulation:**
```bash
./scripts/start_eval.sh
```

Wait until you see the log message `No node with name 'aic_model' found. Retrying...` before starting the policy.

**Terminal 2 — run a policy:**
```bash
./scripts/run_policy.sh aic_example_policies.ros.WaveArm
```

## Customizing Launch Args

Pass ROS launch args directly to `start_eval.sh`:

```bash
# Run with AIC engine (for full scored evaluation)
./scripts/start_eval.sh ground_truth:=false start_aic_engine:=true

# Headless (no Gazebo or RViz windows)
./scripts/start_eval.sh gazebo_gui:=false launch_rviz:=false
```

Pass a different policy class to `run_policy.sh`:

```bash
./scripts/run_policy.sh aic_example_policies.ros.SACPolicy
```

## Monitoring a Running Policy

Run these in a third terminal while the eval and policy are running. Set the Zenoh env vars first so `ros2` commands can reach the simulation:

```bash
export ZENOH_ROUTER_CONFIG_URI="$(pwd)/docker/aic_eval/aic_zenoh_config.json5"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
```

**See all active topics:**
```bash
pixi run ros2 topic list
```

**Stream joint states:**
```bash
pixi run ros2 topic echo /joint_states
```

**Stream policy actions (gripper + arm commands):**
```bash
pixi run ros2 topic echo /gripper_command
pixi run ros2 topic echo /joint_trajectory
```

**Check publish rates:**
```bash
pixi run ros2 topic hz /joint_states
pixi run ros2 topic hz /joint_trajectory
```

**Inspect the policy node:**
```bash
pixi run ros2 node info /aic_model
```

**Print full topic info (type + publishers/subscribers):**
```bash
pixi run ros2 topic info -v /joint_states
```

## Resetting the Container

`start_eval.sh` always removes and recreates the container on each run, so there's no manual cleanup needed.
