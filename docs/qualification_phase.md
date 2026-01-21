# Qualification Phase: Technical Overview

The **Qualification Phase** is the entry point for participants to demonstrate their model's ability to control the robot, converge on targets, and generalize across different connector types. This phase is conducted entirely in simulation using the provided **Gazebo simulation environment**.

## 1. Phase Setup & Constraints

* **Task Scope:** A single cable insertion is evaluated per trial.
* **Environment:** Evaluated in Gazebo without Flowstate.
* **Robot State:** The robot starts with one cable connector already in-hand.
* **Proximity:** The robot starts within a few centimeters of the insertion target.
* **Randomization:** The [task board](./task_board_description.md) pose, orientation, and specific component pose on the rails are randomized for each trial.

## 2. Trial Descriptions

The qualification phase consists of **three specific trials** designed to test different aspects of the participant's policy.

### Trial 1 and 2: Model validity and convergence

* **Objective:** Verify model convergence and the ability to handle randomized NIC poses.

* **Task Board Configuration:**
	* **Zone 1:** One `NIC_CARD` is installed on a randomized `NIC_RAIL` with a random translation and orientation offset.
	* **Zone 3/4:** One **SC-LC cable** is used. The SC end is connected to an SC port on the board. 

* **Manipulation Task:** The robot starts with an **SFP module** in-hand. The robot must insert the grasped SFP module into the target `SFP_PORT` on the specified NIC card.


### Trial 3: Generalization (SC)

* **Objective:** Verify the model's ability to generalize across different plug and port types.


* **Task Board Configuration:**
	* **Zone 2:** One `SC_PORT` is randomized along either `SC_RAIL_0` or `SC_RAIL_1` with a random translation and position offset.
	* **Manipulation Task:** The robot starts grasping the **SC plug** end of the cable. The model must align and insert the SC plug into the target `SC_PORT` along the SC rail.

Note: `SC_PORT_0` will alway be located on `SC_RAIL_0` and `SC_PORT_1` on `SC_RAIL_1`.

![AIC Task Board](../../media/aic_board_trial_3_sc.png)

## 3. Evaluation Metrics & Scoring

Each trial is scored using a tiered system. Only submissions passing Tier 1 will proceed to quantitative scoring.

### Tier 1: Model Validity (Prerequisite)

* **Check:** Submission must load and run without errors.
* **Requirement:** The model must generate robot commands on the specified ROS topics from sensor inputs.

### Tier 2: Performance & Convergence (Quantitative)

* **Distance:** Measured ability to minimize the distance between the cable connector and the target port over time.
* **Efficiency:** A higher score is awarded for a lower final distance and a faster rate of convergence.
* **Smoothness:** Jerk is measured in joint space to ensure stable movement.

### Tier 3: Task Success (Primary Objective)

* **Binary Success:** A significant bonus is awarded for successful insertion.
* **Verification:** Success is detected via contact sensors, force/torque feedback, and Gazebo plugins.
* **Criteria:** Correct alignment and full insertion into the target port.

### Penalties

Scores will be deducted for the following:

* **Collisions:** Any unintended contact with the environment or board.
* **Excessive Force:** Applying forces beyond the safety limits of the components.

