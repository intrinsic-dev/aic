# Competition Phases

## Qualification Phase: Train Your Model

During qualification, participants use their preferred tools—including open source software and simulators—alongside the Intrinsic challenge toolkit to train a model for the cable insertion task. All submitted models are evaluated using Gazebo.

![](../../media/qualification_overview.png)

### Technical Overview

Review the core technical requirements for this phase, including setup constraints, trial descriptions, and evaluation metrics. Refer to the [Qualification Phase: Technical Overview](./qualification_phase.md) document for full specifications.

### Implementation Workflow

To successfully qualify, participants must create a ROS 2 node that adheres to the behavioral requirements specified in the [Challenge Rules](./challenge_rules.md).

#### Recommended Approach: Using the `aic_model` Framework

For convenience, we provide an `aic_model` framework that handles all ROS 2 lifecycle management and boilerplate. You simply implement a Python policy class:

1.  **Create a Policy Class:** Define a Python class that derives from [`PolicyRos`](https://github.com/intrinsic-dev/aic/blob/main/aic_model/aic_model/policy_ros.py).
2.  **Implement `insert_cable()`:** This method is called when `aic_engine` requests a new task. It receives observation data and callable methods for robot control.
3.  **Load Your Model:** Initialize your trained policy (e.g., PyTorch checkpoint, ONNX model, or control algorithm) when your class is instantiated.
4.  **Process Observations:** Use the provided `get_observation()` callback to retrieve sensor data at up to 20 Hz.
5.  **Output Commands:** Use `set_pose_target()` and other provided methods to command the robot.
6.  **Return on Completion:** Your `insert_cable()` method should return when the task is complete.

> **Tutorial:** For a step-by-step guide, see [Creating a New Policy Node](./policy.md#tutorial-creating-a-new-policy-node).
>
> **Example:** Reference implementation: [`WaveArm.py`](../aic_example_policies/aic_example_policies/ros/WaveArm.py)

#### Alternative: Implement Your Own Node

You may also implement your own ROS 2 node from scratch, as long as it:
- Is named `aic_model` and implements the ROS 2 Lifecycle interface
- Responds to the `/insert_cable` action server
- Follows all requirements in the [Challenge Rules](./challenge_rules.md)

---

### Participation Guidelines

* **Policy Development:** Participants are free to use any approach to develop a policy, including:
    * Real-world teleoperation data.
    * Training in a simulator of choice (MuJoCo, Isaac Sim, O3DE, etc.).
    * Classical control algorithms.
* **Interface Requirements:** Policies (wrapped in the service described above) must consume world information and output actions using standard formats.
* **Evaluation:** The provided Evaluator Simulator (Gazebo) scores the performance of participant models.
    * During development, participants can run the Evaluator Simulator locally to test performance.
    * Upon submission, a cloud instance runs the same Evaluator Simulator to log official scores.

For more information, please refer to:
* [Scene Description](./scene_description.md)
* [AIC Interfaces](./aic_interfaces.md)

---

## Phase 1: Develop in Flowstate

Teams advancing to Phase 1 gain access to **Intrinsic Flowstate** (our development environment) and the **Intrinsic Vision Model**. Using these tools, teams will build a complete robotic cable handling solution incorporating their trained models.

*TODO*

## Phase 2: Run on Real Robots

Phase 2 participants deploy their solutions to a physical robotic workcell at Intrinsic’s HQ. This phase validates solutions in the real world and determines prize winners.

*TODO*
