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

* **Task Scope:** A single cable insertion is evaluated per trial. Only one plug on the cable is tested for insertion; the other end of the cable remains free and unconnected. During evaluation, the only plug-port insertions and cable involved will be `SFP_MODULE` to `SFP_PORT` and `SC_PLUG` to `SC_PORT`. The same flexible cable is used across trials. However, the general configuration of the task board will vary (e.g., the number and placement of NIC cards and SC ports), and the task definition will clearly specify which port in which component the grasped plug needs to be inserted into. This task definition is described by the [`aic_task_interfaces/msg/Task.msg`](../aic_interfaces/aic_task_interfaces/msg/Task.msg) message and is forwarded to the participant model via a ROS 2 action request. If you are using the provided Python template, this `Task` object is available directly as a parameter to the `Policy.insert_cable` method. 3D assets for all these components can be found in the [`aic_assets/models`](../aic_assets/models/) directory. No unseen plug or port types will be presented.
* **Environment:** Evaluated in Gazebo without Flowstate.
* **Robot State:** The robot starts with one plug already in-hand.
* **Grasp Pose:** While the goal is to have a consistent grasp between the plug and `gripper/tcp` frame with `gripper_offset` values seen in [sample_config.yaml](../aic_engine/config/sample_config.yaml) (e.g., `x: 0.0, y: 0.015385, z: 0.04245, roll: 0.4432, pitch: -0.4838, yaw: 1.3303` for SFP Module and `x: 0.0, y: 0.015385, z: 0.04045, roll: 0.4432, pitch: -0.4838, yaw: 1.3303` for SC plugs), in practice there will be small deviations (~2mm, ~0.04 rad) in the relative grasp pose. We encourage participants to develop policies that are robust against these minor differences.
* **Proximity:** The robot starts within a few centimeters of the insertion target.
* **Randomization:** The [task board](./task_board_description.md) pose, orientation, and specific component pose on the rails are randomized for each trial.
* **Orchestration:** The `aic_engine` node manages the complete trial lifecycle, including spawning task boards, validating policy behavior, monitoring task execution, and collecting scoring data. For detailed information about the engine's operation and configuration, see the [AIC Engine README](../aic_engine/README.md).

## 2. Trial Descriptions

The qualification phase consists of **three specific trials** designed to test different aspects of the participant's policy.

The same policy submitted by the participant will be used for all three trials.

In each trial, the robot spawns at a pre-specified (not random) pose, and the cable plug is fixed in its gripper.

> [!NOTE]
> The exact number and sequence of trials during final evaluation may be subject to change. However, they will always consist of some combination of the SFP and SC insertions described below.

### Trial 1 and 2: Policy validity and convergence

![TRIAL 1](../../media/aic_board_trial_1_sfp.png)

* **Objective:** Verify policy convergence and the ability to handle randomized NIC poses. The only difference between these two trials is the randomness in 1) the pose of the task board, 2) which `NIC_RAIL` the `NIC_CARD` gets spawned on, and 3) the translation and orientation offset of the `NIC_CARD` on that `NIC_RAIL`.

* **Start State:**
	* The robot is grasping the `SFP_MODULE` plug end of an [sfp_sc_cable](../aic_assets/models/sfp_sc_cable/).
	* The task board is spawned with a randomized pose (position and yaw angle). While multiple components and NIC cards may be present on the board, the specific target port of interest will always be within view of the robot cameras.
	* One or more `NIC_CARD`s are mounted on randomly selected `NIC_RAIL`s (there are 5 rails: `nic_rail_0` through `nic_rail_4`) each with a random translation (along its rail) and a random yaw offset.
	* The opposite end of the cable (SC plug) remains free and unconnected.

* **Manipulation Task:** Insert the grasped `SFP_MODULE` plug into either `SFP_PORT_0` or `SFP_PORT_1` on the spawned NIC card (the task config from `aic_engine` will specify which).

### Trial 3: Generalization (SC)

![TRIAL 3](../../media/aic_board_trial_3_sc.png)

* **Objective:** Verify the policy's ability to generalize across different plug and port types.

* **Start State:**
	* The robot is grasping the `SC_PLUG` end of the same [sfp_sc_cable](../aic_assets/models/sfp_sc_cable/).
	* The task board is spawned with a randomized pose (position and yaw angle). While multiple components and SC ports may be present on the board, the specific target port of interest will always be within view of the robot cameras.
	* One or both SC ports are mounted on the task board: `SC_PORT_0` on `SC_RAIL_0` and `SC_PORT_1` on `SC_RAIL_1`, each with a random translation along its rail. Only one SC port will be the target port.
	* The opposite end of the cable (SFP module) remains free and unconnected.

* **Manipulation Task:** Insert the grasped `SC_PLUG` into one of the SC ports (`SC_PORT_0` or `SC_PORT_1`, as specified by `aic_engine`), ensuring alignment with the task board's SC rails.


## 3. Evaluation Metrics & Scoring

See [Scoring](scoring.md) for details.

---

## Next Steps

For detailed instructions on implementing your policy and the submission workflow, see the [Key Steps for Participation](./phases.md#key-steps-for-participation) in the Competition Phases document.
