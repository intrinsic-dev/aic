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

The same policy submitted by the participant will be used for all three trials.

In each trial, the robot spawns at a pre-specified (not random) pose, and the cable plug is fixed in its gripper.

### Trial 1 and 2: Policy validity and convergence

![TRIAL 1](../../media/aic_board_trial_1_sfp.png)

* **Objective:** Verify policy convergence and the ability to handle randomized NIC poses. The only difference between these two trials is the randomness in 1) the pose of the task board, 2) which `NIC_RAIL` the `NIC_CARD` gets spawned on, and 3) the translation and orientation offset of the `NIC_CARD` on that `NIC_RAIL`.

* **Start State:**
	* The robot is grasping the `SFP_MODULE` plug end of an [sfp_sc_cable](../aic_assets/models/sfp_sc_cable/).
	* The task board is spawned with a randomized pose (position and orientation).
	* One `NIC_CARD` is mounted on a randomly selected `NIC_RAIL` (one of 5 rails: `nic_rail_0` through `nic_rail_4`) with a random translation and orientation offset.
	* The opposite end of the cable (SC plug) remains free and unconnected.

* **Manipulation Task:** Insert the grasped `SFP_MODULE` plug into either `SFP_PORT_0` or `SFP_PORT_1` on the spawned NIC card (the task config from `aic_engine` will specify which).

### Trial 3: Generalization (SC)

![TRIAL 3](../../media/aic_board_trial_3_sc.png)

* **Objective:** Verify the policy's ability to generalize across different plug and port types.

* **Start State:**
	* The robot is grasping the `SC_PLUG` end of the same [sfp_sc_cable](../aic_assets/models/sfp_sc_cable/).
	* The task board is spawned with a randomized pose (position and orientation).
	* SC ports are mounted on the task board: `SC_PORT_0` on `SC_RAIL_0` and `SC_PORT_1` on `SC_RAIL_1`, each with random translation and orientation offsets.
	* The opposite end of the cable (SFP module) remains free and unconnected.

* **Manipulation Task:** Insert the grasped `SC_PLUG` into one of the SC ports (`SC_PORT_0` or `SC_PORT_1`, as specified by `aic_engine`), ensuring alignment with the task board's SC rails.


## 3. Evaluation Metrics & Scoring

See [Scoring](scoring.md) for details.

---

## Next Steps

For detailed instructions on implementing your policy and the submission workflow, see the [Key Steps for Participation](./phases.md#key-steps-for-participation) in the Competition Phases document.
