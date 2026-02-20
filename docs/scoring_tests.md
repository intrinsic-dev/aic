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

| Tier | Category | Range | Description |
|------|----------|-------|-------------|
| 1 | Model validity | 0-1 | Pass/fail: policy responded to `/insert_cable` action within timeout |
| 2 | Trajectory smoothness | 0-5 | Smoothness of arm motion; inversely proportional to jerk (higher = smoother) |
| 2 | Task duration | 0-10 | Reward for faster completion; only awarded on successful insertion or plug is within close proximity to port |
| 2 | Trajectory efficiency | 0-5 | Reward for shorter end-effector path length (higher = more direct) |
| 2 | Insertion force | 0 to -10 | Penalty for force > 5 N sustained for > 1 second |
| 2 | Off-limit contacts | 0 to -20 | Penalty for collisions with the enclosure or task board |
| 3 | Cable insertion | -10 or 0 to 60 | -10 penalty for wrong-port insertion; 60 for correct-port insertion; 0-40 for partial insertion or close proximity |

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
solution. Exercises Tier 1 (pass), Tier 2 (smoothness, duration, efficiency, force),
and Tier 3 (cable insertion).

**Expected outcome:**
- All 3 trials complete.
- Tier 1 should **pass** for all trials.
- Tier 2 should show high smoothness scores (up to 5), a task duration bonus
  (up to 10) for successful trials, and no force penalty or off-limit contacts.
- Tier 3 should report successful cable insertion (60 points) for Trials 1 and 2.
- **Known issue:** Trial 3 uses an SC plug. CheatCode cannot resolve the SC
  cable tip TF frame, so it logs a transform timeout error and returns early.
  Tier 3 for Trial 3 will show a proximity score instead of an insertion bonus.

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
inserts the cable. Exercises Tier 1 (pass) and Tier 2 (smoothness, efficiency).

**Expected outcome:**
- All 3 trials complete.
- Tier 1 should **pass** for all trials.
- Tier 2 should show high smoothness scores (smooth waving motion), no task
  duration bonus (no successful insertion), no force penalty, and no off-limit
  contacts.
- Tier 3 should report 0 score for all trials (the arm waves but
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
- Tier 2 should show an off-limit contacts penalty (-20) for all trials where a
  robot link (e.g. `forearm_link`) collided with the enclosure wall.
- Tier 3 should report 0 score for all trials (no insertion and plug outside of
  port proximity).

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
- Tier 2 should show an insertion force penalty (-10) for all trials. Off-limit
  contacts penalty (-20) may also appear as a side effect of the wall contact.
- Tier 3 should report 0 score for all trials (no insertion and plug outside of
  port proximity).

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
- Tier 2 should show high smoothness scores (slow, smooth motion), no task
  duration bonus (plug not within close proximity to port), no force penalty, and no off-limit
  contacts. Compare with Example 7 (`SpeedDemon`) to see the difference in
  smoothness.
- Tier 3 should report 0 score for all trials (no insertion and plug outside of
  port proximity).

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
- Tier 2 should show lower smoothness scores than GentleGiant (Example 6) due to
  aggressive motion, plus an insertion force penalty (-10) for all trials. The arm
  oscillates aggressively due to low damping, generating sustained force at
  the F/T sensor. The arm should visibly snap between positions.
- Tier 3 should report 0 score for all trials (no insertion and plug outside of
  port proximity).

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
