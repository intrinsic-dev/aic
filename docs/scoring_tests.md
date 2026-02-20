# Scoring Test & Evaluation Guide

This document provides reproducible examples that exercise the AIC scoring system.
Each example lists a goal, the scoring categories it tests, the expected outcome, and
the exact commands to run in each terminal.

## Prerequisites

Every terminal needs the ROS 2 workspace and Zenoh middleware configured:

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/shared_memory/transport_optimization/pool_size=536870912'
```

Build the workspace (if not already built):

```bash
cd ~/ws_aic
GZ_BUILD_FROM_SOURCE=1 colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --merge-install --symlink-install \
  --packages-ignore lerobot_robot_aic
```

## Scoring Tiers Reference

| Tier | Category | Description |
|------|----------|-------------|
| 1 | Model validity | Pass/fail: policy responded to `/insert_cable` action within timeout |
| 2 | Trajectory jerk | Smoothness of arm motion (higher = smoother) |
| 2 | Insertion force | Penalty for excessive force at the F/T sensor |
| 2 | Trajectory efficiency | Reward for shorter end-effector path length (higher = more direct) |
| 2 | Off-limit contacts | Penalty for collisions with the enclosure or task board |
| 3 | Cable insertion | Score based on plug-port distance, task duration, and insertion success bonus |

Results are written to `$AIC_RESULTS_DIR/scoring.yaml` when using the engine.
The default directory is `~/aic_results`. Each engine run **overwrites** the previous
`scoring.yaml`, so set `AIC_RESULTS_DIR` to a unique path per run to preserve results.

---

## Example 1: Tier 1 Failure -- No Model Running

**Goal:** Start the engine without launching `aic_model`. The engine should
time out waiting for the policy to accept the `/insert_cable` action, resulting
in Tier 1 failure.

**Expected outcome:**
- The engine reports a timeout or failure for each trial.
- Tier 1 should **fail** for all trials.
- Tier 2 and Tier 3 are skipped (all scores are zero).

### Terminal 0 -- Zenoh Router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 1 -- Simulation + Engine (no model)

```bash
AIC_RESULTS_DIR=~/aic_results/no_model \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  start_aic_engine:=true
```

---

## Example 2: CheatCode Reference Solution

**Goal:** Run the CheatCode policy through the full engine pipeline as the reference
solution. Exercises Tier 1 (pass), Tier 2 (jerk, force), and Tier 3
(cable insertion).

**Expected outcome:**
- All 3 trials complete.
- Tier 1 should **pass** for all trials.
- Tier 2 should show high jerk scores, no force penalty, and no off-limit contacts.
- Tier 3 should report successful cable insertion for Trials 1 and 2.
- **Known issue:** Trial 3 uses an SC plug. CheatCode cannot resolve the SC
  cable tip TF frame, so it logs a transform timeout error and returns early.
  Tier 2 and Tier 3 scores for Trial 3 will be low or zero.

### Terminal 0 -- Zenoh Router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 1 -- AIC Model (CheatCode)

```bash
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.CheatCode
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/cheatcode \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

---

## Example 3: WaveArm Baseline

**Goal:** Run the WaveArm policy through the engine. The arm waves but never
inserts the cable. Exercises Tier 1 (pass) and Tier 2 (smooth jerk, poor distance).

**Expected outcome:**
- All 3 trials complete.
- Tier 1 should **pass** for all trials.
- Tier 2 should show high jerk scores (smooth waving motion), no force penalty,
  and no off-limit contacts.
- Tier 3 should report failed cable insertion for all trials (the arm waves but
  never approaches the port).

### Terminal 0 -- Zenoh Router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 1 -- AIC Model (WaveArm)

```bash
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.WaveArm
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/wavearm \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  start_aic_engine:=true
```

---

## Example 4: Off-Limit Contact

**Goal:** Run the `WallToucher` policy through the engine. This policy uses
joint-space control to extend the arm sideways, touching the forearm against an
enclosure wall panel. The off-limit contact penalty should appear in the
scoring output.

**Expected outcome:**
- All 3 trials complete.
- Tier 1 should **pass** for all trials.
- Tier 2 should show an off-limit contacts penalty for all trials where a
  robot link (e.g. `forearm_link`) collided with the enclosure wall.
- Tier 3 should report failed cable insertion for all trials.

> **Note — Off-limit contacts:** "Off-limit" models are surfaces the robot must
> not touch during the task. The `OffLimitContactsPlugin` monitors three models:
>
> | Model | What it includes |
> |-------|-----------------|
> | `enclosure` | Floor, corner posts, and ceiling (the structural frame) |
> | `enclosure walls` | Transparent acrylic panels surrounding the workspace |
> | `task_board` | The board and everything mounted on it (NIC card mounts, SC ports, etc.) |
>
> Only contacts where one side is a **robot link** are penalized. The cable is a
> separate Gazebo model and is not expected to trigger the penalty.

### Terminal 0 -- Zenoh Router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 1 -- AIC Model (WallToucher)

```bash
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.WallToucher
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/wall_toucher \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

---

## Example 5: Excessive Force

**Goal:** Run the `WallPresser` policy through the engine. This policy uses
joint-space control to press the forearm into an enclosure wall with high
stiffness, generating sustained contact forces that trigger the Tier 2
insertion force penalty.

**Expected outcome:**
- All 3 trials complete.
- Tier 1 should **pass** for all trials.
- Tier 2 should show an insertion force penalty for all trials. Off-limit
  contacts may also appear as a side effect of the wall contact.
- Tier 3 should report failed cable insertion for all trials.

### Terminal 0 -- Zenoh Router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 1 -- AIC Model (WallPresser)

```bash
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.WallPresser
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/wall_presser \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

---

## Example 6: Smooth Motion -- Low Jerk

**Goal:** Run the `GentleGiant` policy through the engine. This policy moves the
arm slowly between two joint configurations using low stiffness and high damping,
producing minimal jerk (high Tier 2 jerk score).

**Expected outcome:**
- All 3 trials complete.
- Tier 1 should **pass** for all trials.
- Tier 2 should show high jerk scores (slow, smooth motion), no force penalty,
  and no off-limit contacts. Compare with Example 7 (`SpeedDemon`) to see the
  difference in jerk.
- Tier 3 should report failed cable insertion for all trials.

### Terminal 0 -- Zenoh Router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 1 -- AIC Model (GentleGiant)

```bash
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.GentleGiant
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/gentle_giant \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

---

## Example 7: Aggressive Motion -- High Jerk

**Goal:** Run the `SpeedDemon` policy through the engine. This policy moves the
arm rapidly between two joint configurations using high stiffness and low damping,
producing aggressive motion that triggers the insertion force penalty.

**Expected outcome:**
- All 3 trials complete.
- Tier 1 should **pass** for all trials.
- Tier 2 should show lower jerk scores than GentleGiant (Example 6) due to
  aggressive motion, plus an insertion force penalty for all trials. The arm
  oscillates aggressively due to low damping, generating sustained force at
  the F/T sensor. The arm should visibly snap between positions.
- Tier 3 should report failed cable insertion for all trials.

### Terminal 0 -- Zenoh Router

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 1 -- AIC Model (SpeedDemon)

```bash
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.SpeedDemon
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/speed_demon \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

---
