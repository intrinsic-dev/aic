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
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
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
| 2 | Plug-port distance | Minimum distance between plug and port during trial |
| 2 | Insertion force | Penalty for excessive force at the F/T sensor |
| 2 | Off-limit contacts | Penalty for collisions with the enclosure or task board |
| 3 | Insertion success | Bonus for plug fully inserted into the port |

Results are written to `$AIC_RESULTS_DIR/scoring.yaml` when using the engine.
The default directory is `~/aic_results`. Each engine run **overwrites** the previous
`scoring.yaml`, so set `AIC_RESULTS_DIR` to a unique path per run to preserve results.

---

## Example 1: CheatCode Reference Solution (Full Engine)

**Goal:** Run the CheatCode policy through the full engine pipeline as the reference
solution. Exercises Tier 1 (pass), Tier 2 (jerk, distance, force), and Tier 3
(insertion success).

**Scoring categories exercised:** Tier 1, Tier 2 (jerk + distance), Tier 3.

**Expected outcome:** Tier 1 should pass for all trials. Tier 2 jerk and distance
scores should be moderate to high. Tier 3 cable insertion is expected to fail — the
CheatCode policy gets close to the port but does not fully insert.

### Terminal 0 -- Zenoh Router

```bash
/opt/ros/kilted/lib/rmw_zenoh_cpp/rmw_zenohd
```

### Terminal 1 -- AIC Model (CheatCode)

```bash
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.CheatCode
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/cheatcode \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

### What to verify

- All 3 trials complete without errors.
- `~/aic_results/cheatcode/scoring.yaml` exists and contains scores for each trial.
- Tier 1 should pass for all trials. Tier 3 insertion is expected to fail.

---

## Example 2: WaveArm Baseline (Full Engine)

**Goal:** Run the WaveArm policy through the engine. The arm waves but never
inserts the cable. Exercises Tier 1 (pass) and Tier 2 (smooth jerk, poor distance).

**Scoring categories exercised:** Tier 1, Tier 2 (jerk + distance), Tier 3 (fail).

**Expected outcome:** Tier 1 should pass for all trials. Tier 2 jerk scores should
be high (the arm moves smoothly) while distance scores should be low (the plug stays
far from the port). Tier 3 should fail for all trials since the arm never attempts insertion.

### Terminal 0 -- Zenoh Router

```bash
/opt/ros/kilted/lib/rmw_zenoh_cpp/rmw_zenohd
```

### Terminal 1 -- AIC Model (WaveArm)

```bash
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/wavearm \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  start_aic_engine:=true
```

### What to verify

- All 3 trials complete. Tier 1 should pass for all trials.
- `~/aic_results/wavearm/scoring.yaml` shows high jerk scores (smooth motion) and low distance scores (plug stays far from port).
- Tier 3 should fail for all trials (no insertion).

---

## Example 3: Off-Limit Contact (Full Engine)

**Goal:** Run the `BoardCrasher` policy through the engine. This policy uses
joint-space control to rotate the wrist (fingers pointing up) and lower the
shoulder, pressing the `wrist_1_link` / `forearm_link` into the task board.
The off-limit contact penalty should appear in the scoring output.

**Scoring categories exercised:** Tier 1, Tier 2 (off-limit contacts penalty).

**Expected outcome:** Tier 1 should pass. The `scoring.yaml` contacts category should
show a penalty for trials where a robot link collides with the task board.
Not every trial may trigger the contact — the board position varies per trial.

> **Note:** The `OffLimitContactsPlugin` only detects contacts between **robot links**
> and off-limit models (`enclosure`, `task_board`). The cable is a separate Gazebo
> model and does not trigger detection. The `enclosure_walls` (transparent panels) are
> also **not** in the off-limit list.

### Terminal 0 -- Zenoh Router

```bash
/opt/ros/kilted/lib/rmw_zenoh_cpp/rmw_zenohd
```

### Terminal 1 -- AIC Model (BoardCrasher)

```bash
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.BoardCrasher
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/board_crasher \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

### What to verify

- `~/aic_results/board_crasher/scoring.yaml` exists.
- The contacts category shows a penalty for at least one trial where a
  robot link (e.g. `forearm_link`) collided with the task board.
- Tier 1 should pass for all trials.

---

## Example 4: Excessive Force (Full Engine)

**Goal:** Run the `Jackhammer` policy through the engine. This policy repeatedly
hammers the gripper up and down against the task board using high stiffness,
triggering the Tier 2 insertion force penalty.

**Scoring categories exercised:** Tier 1, Tier 2 (insertion force penalty).

**Expected outcome:** Tier 1 should pass. The `scoring.yaml` insertion force
category should show a penalty for trials where the F/T sensor exceeded
the force threshold.

### Terminal 0 -- Zenoh Router

```bash
/opt/ros/kilted/lib/rmw_zenoh_cpp/rmw_zenohd
```

### Terminal 1 -- AIC Model (Jackhammer)

```bash
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.Jackhammer
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/jackhammer \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

### What to verify

- `~/aic_results/jackhammer/scoring.yaml` exists.
- The insertion force category shows a penalty for trials where force
  exceeded the threshold.
- Tier 1 should pass for all trials.

---

## Example 5: Smooth Motion -- Low Jerk (Full Engine)

**Goal:** Run the `GentleGiant` policy through the engine. This policy moves the
arm slowly between two joint configurations using low stiffness and high damping,
producing minimal jerk (high Tier 2 jerk score).

**Scoring categories exercised:** Tier 1, Tier 2 (trajectory jerk + insertion force).

**Expected outcome:** Tier 1 should pass. The jerk score should be high
because the arm moves slowly and smoothly. The insertion force penalty should **not**
trigger — the low stiffness keeps forces well below the threshold. Compare with
Example 6 to see the difference in force penalties.

### Terminal 0 -- Zenoh Router

```bash
/opt/ros/kilted/lib/rmw_zenoh_cpp/rmw_zenohd
```

### Terminal 1 -- AIC Model (GentleGiant)

```bash
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.GentleGiant
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/gentle_giant \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

### What to verify

- `~/aic_results/gentle_giant/scoring.yaml` exists.
- The trajectory jerk score should be high for all trials.
- No insertion force penalty (low stiffness keeps forces below the threshold).
- Compare with Example 6 (`SpeedDemon`) to see the force penalty difference.

---

## Example 6: Aggressive Motion -- High Jerk (Full Engine)

**Goal:** Run the `SpeedDemon` policy through the engine. This policy moves the
arm rapidly between two joint configurations using high stiffness and low damping,
producing aggressive motion that triggers the insertion force penalty.

**Scoring categories exercised:** Tier 1, Tier 2 (insertion force penalty + trajectory jerk).

**Expected outcome:** Tier 1 should pass. The high stiffness drives the joints
aggressively, generating forces above the F/T sensor threshold and triggering
a force penalty on each trial. The arm visibly snaps between positions and the
force penalty clearly differentiates this from Example 5 (`GentleGiant`).

### Terminal 0 -- Zenoh Router

```bash
/opt/ros/kilted/lib/rmw_zenoh_cpp/rmw_zenohd
```

### Terminal 1 -- AIC Model (SpeedDemon)

```bash
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.SpeedDemon
```

### Terminal 2 -- Simulation + Engine

```bash
AIC_RESULTS_DIR=~/aic_results/speed_demon \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true
```

### What to verify

- `~/aic_results/speed_demon/scoring.yaml` exists.
- The insertion force category shows a penalty for each trial.
- The arm should visibly snap to positions and may oscillate.
- Compare total score with Example 5 (`GentleGiant`) — the force penalty is the
  main differentiator.

---

## Example 7: Tier 1 Failure -- No Model Running

**Goal:** Start the engine without launching `aic_model`. The engine should
time out waiting for the policy to accept the `/insert_cable` action, resulting
in Tier 1 failure.

**Scoring categories exercised:** Tier 1 (failure).

**Expected outcome:** The engine reports model validation failure. All tiers score zero.

### Terminal 0 -- Zenoh Router

```bash
/opt/ros/kilted/lib/rmw_zenoh_cpp/rmw_zenohd
```

### Terminal 1 -- Simulation + Engine (no model)

```bash
AIC_RESULTS_DIR=~/aic_results/no_model \
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  start_aic_engine:=true
```

Do **not** start `aic_model` in any other terminal.

### What to verify

- The engine eventually reports a timeout or failure for each trial.
- `~/aic_results/no_model/scoring.yaml` shows Tier 1 failure for all trials.
- All other tier scores are zero (scoring is skipped when Tier 1 fails).

---
