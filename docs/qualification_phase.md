# Qualification Phase: Technical Overview

The **Qualification Phase** is the entry point for participants to demonstrate their policy's ability to control the robot, converge on targets, and generalize across different plug types. This phase is conducted entirely in simulation and with participant policies evaluted within the provided [Gazebo simulation environment](./scene_description.md).
In collaboration with **NVIDIA** and **Google Deepmind**, the toolkit also includes mirror simulation environments for IsaacLab and MuJoCo respectively for participants to train robust policies.

## A note on simulation

No simulator perfectly mirrors reality.
While contact-rich processes (like insertion) often highlight the "Reality Gap," our goal isn't perfect physical symmetry—it's functional validation.
Here is how we are addressing physics discrepancies:
- **Signal over Precision**: We use Gazebo to ensure your policies are performing the intended tasks correctly, rather than over-indexing on hyper-specific insertion physics.
- **Tuned Environment**: We will provide a Gazebo environment specifically tuned to approximate cable physics and insertion dynamics as closely as possible.
- **Domain Randomization**: We actually encourage you to train across different simulators. These physical variations offer an excellent opportunity for domain randomization, better preparing your models for a "sim-to-sim-to-real" transfer.

## 1. Phase Setup & Constraints

* **Task Scope:** A single cable insertion is evaluated per trial. Only one plug on the cable is tested for insertion; the other end of the cable remains free and unconnected.
* **Environment:** Evaluated in Gazebo without Flowstate.
* **Robot State:** The robot starts with one plug already in-hand.
* **Proximity:** The robot starts within a few centimeters of the insertion target.
* **Randomization:** The [task board](./task_board_description.md) pose, orientation, and specific component pose on the rails are randomized for each trial.
* **Orchestration:** The `aic_engine` node manages the complete trial lifecycle, including spawning task boards, validating policy behavior, monitoring task execution, and collecting scoring data. For detailed information about the engine's operation and configuration, see the [AIC Engine README](../aic_engine/README.md).

## 2. Trial Descriptions

The qualification phase consists of **three specific trials** designed to test different aspects of the participant's policy.

### Trial 1 and 2: Policy validity and convergence

![TRIAL 1](../../media/aic_board_trial_1_sfp.png)

* **Objective:** Verify policy convergence and the ability to handle randomized NIC poses.

* **Start State:**
	* The robot is grasping the `SFP_MODULE` plug end of an [sfp_sc_cable](../aic_assets/models/sfp_sc_cable/).
	* The task board is spawned with a randomized pose (position and orientation).
	* One `NIC_CARD` is mounted on a randomly selected `NIC_RAIL` (one of 5 rails: `nic_rail_0` through `nic_rail_4`) with a random translation and orientation offset.
	* The opposite end of the cable (SC plug) remains free and unconnected.

* **Manipulation Task:** Insert the grasped `SFP_MODULE` plug into either `SFP_PORT_0` or `SFP_PORT_1` on the spawned NIC card.


### Trial 3: Generalization (SC)

![TRIAL 3](../../media/aic_board_trial_3_sc.png)

* **Objective:** Verify the policy's ability to generalize across different plug and port types.

* **Start State:**
	* The robot is grasping the `SC_PLUG` end of the same [sfp_sc_cable](../aic_assets/models/sfp_sc_cable/).
	* The task board is spawned with a randomized pose (position and orientation).
	* SC ports are mounted on the task board: `SC_PORT_0` on `SC_RAIL_0` and `SC_PORT_1` on `SC_RAIL_1`, each with random translation and orientation offsets.
	* The opposite end of the cable (SFP module) remains free and unconnected.

* **Manipulation Task:** Insert the grasped `SC_PLUG` into one of the SC ports (`SC_PORT_0` or `SC_PORT_1`), ensuring alignment with the task board's SC rails.


## 3. Evaluation Metrics & Scoring

Each trial is scored using a tiered system. Only submissions passing Tier 1 will proceed to quantitative scoring.

### Tier 1: Policy Validity (Prerequisite)

* **Check:** Submission must load and run without errors.
* **Requirement:** The policy must generate robot commands on the specified ROS topics from sensor inputs.
* **Conformance:** The policy must adhere to all requirements specified in the [Challenge Rules](./challenge_rules.md), including proper lifecycle node behavior and topic publishing constraints.

> **Note:** Policies that fail Tier 1 validation will not proceed to Tier 2 or Tier 3 evaluation. All requirements must be satisfied before quantitative scoring can begin.

### Tier 2: Performance & Convergence (Quantitative)

The following metrics are measured and scored for each trial:

#### Smoothness
* **Measurement:** Jerk is computed over the robot trajectory from joint accelerations.
* **Data Source:** ROS 2 topic `/joint_states` - jerk is calculated from changes in `joint.acceleration`.
* **Scoring:** Lower jerk values result in higher scores.

#### Convergence
* **Measurement:** Euclidean distance between the plug tip and port base over time.
* **Data Source:** ROS 2 topic `/ground_truth_poses` from PoseBroadcaster plugin (Gazebo topic also available).
* **Scoring:** A higher score is awarded for minimizing the distance and achieving a faster convergence rate.

#### Task Completion Time
* **Measurement:** Elapsed time from task start to completion.
* **Data Source:** ROS 2 topic from `aic_engine` publishing `TaskState`.
* **Scoring:** Faster completion times receive higher scores.

#### Command Safety
* **Measurement:** Wrenches and motion commands sent to the robot controller are monitored for excessive values.
* **Data Source:** ROS 2 topics from `aic_model` output - `MotionUpdate` and `JointMotionUpdate`.
* **Scoring:** Excessive command values result in score deductions.

### Tier 3: Task Success (Primary Objective)

* **Binary Success:** A significant bonus is awarded for successful insertion.
* **Measurement:** Error in position and orientation of the plug inside the port.
* **Data Source:** ROS 2 topic `/ground_truth_poses` from PoseBroadcaster plugin.
* **Verification:** Success is detected via contact sensors, force/torque feedback, and Gazebo plugins.
* **Criteria:** Correct alignment and full insertion into the target port within tolerance thresholds.

### Penalties

Scores will be deducted for the following:

#### Collisions
* **Measurement:** Unintended contact with the environment or task board.
* **Data Source:** Custom Gazebo plugin that monitors all contacts, filtering out cable-related contacts, published to a dedicated ROS 2 topic.
* **Penalty:** Score deductions based on collision severity and frequency.

#### Excessive Forces
* **Measurement:** Forces applied during insertion that exceed component safety limits.
* **Data Source:** Temporal filter applied to contact force data from the custom Gazebo plugin.
* **Penalty:** Score deductions proportional to force magnitude and duration beyond safe thresholds.

---

## Next Steps

For detailed instructions on implementing your policy and the submission workflow, see the [Key Steps for Participation](./phases.md#key-steps-for-participation) in the Competition Phases document.
