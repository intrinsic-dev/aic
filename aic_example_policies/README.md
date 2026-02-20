# Example Policies

This package contains baseline policy implementations that demonstrate different approaches to the cable insertion task. These examples serve as reference implementations and starting points for developing your own policies.

> [!NOTE]
> **Prerequisites:** Before running these policies, ensure you have the evaluation environment running. See [Getting Started](../docs/getting_started.md) for setup instructions.
>
> **Command Format:**
> - If using the **container workflow** (recommended): Launch with `distrobox enter -r aic_eval -- /entrypoint.sh [parameters]`
> - If **built from source**: Launch with `ros2 launch aic_bringup aic_gz_bringup.launch.py [parameters]`
> - Run policies with `pixi run ros2 run` (Pixi workspace) or `ros2 run` (native ROS 2)

---

## Available Policies

### 1. WaveArm - Minimal Example

![Wave Arm Policy](../../media/wave_arm_policy.gif)

A minimal example showing how to implement the `insert_cable()` callback and issue motion commands to the arm. This policy simply moves the robot arm back and forth in a waving motion without attempting to solve the task.

**Purpose:** Demonstrates the basic Policy API structure.

**Launch the evaluation environment:**
```bash
/entrypoint.sh ground_truth:=false start_aic_engine:=true
```

**Run the policy:**
```bash
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.WaveArm
```

**Source:** [`WaveArm.py`](./aic_example_policies/ros/WaveArm.py)

---

### 2. CheatCode - Ground Truth Policy

A "cheating" solution that uses the TF transformation tree provided by the simulation when `ground_truth:=true` is set at launch time. This policy uses the poses of the plug and port to calculate target poses to send to `aic_controller`.

**Purpose:** Useful for training and debugging. Ground truth data will not be available during official evaluation.

**Launch simulation with ground truth:**
```bash
/entrypoint.sh \
  nic_card_mount_0_present:=true \
  sc_port_0_present:=true \
  ground_truth:=true \
  spawn_task_board:=true \
  spawn_cable:=true \
  attach_cable_to_gripper:=true \
  sfp_mount_rail_0_present:=true \
  start_aic_engine:=false
```

**Run the policy:**
```bash
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.CheatCode
```

**Trigger task execution:**
```bash
src/aic/aic_model/test/create_and_cancel_task.py
```

**Source:** [`CheatCode.py`](./aic_example_policies/ros/CheatCode.py)

---

### 3. RunACT - ACT Policy

An implementation of a [LeRobot ACT](https://huggingface.co/docs/lerobot/en/act) (Action Chunking with Transformers) policy trained with a small dataset available on HuggingFace (TODO add link).

**Purpose:** Demonstrates integration of a trained neural network policy for the cable insertion task.

**Launch the evaluation environment:**
```bash
/entrypoint.sh ground_truth:=false start_aic_engine:=true
```

**Run the policy:**
```bash
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.RunACT
```

**Source:** [`RunACT.py`](./aic_example_policies/ros/RunACT.py)

---

## Scoring Examples

For expected scoring results and reproducible test commands for each policy, see the [Scoring Test & Evaluation Guide](../../docs/scoring_tests.md).
