# Competition Phases

## Qualification Phase: Train Your Model

During qualification, participants use their preferred tools—including open source software and simulators—alongside the Intrinsic challenge toolkit to train a model for the cable insertion task. All submitted models are evaluated using Gazebo.

![](../../media/qualification_overview.png)

### Technical Overview

Review the core technical requirements for this phase, including setup constraints, trial descriptions, and evaluation metrics. Refer to the [Qualification Phase: Technical Overview](./qualification_phase.md) document for full specifications.

### Implementation Workflow

To successfully qualify, participants must complete the implementation of the `aic_model` node. This involves modifying the provided template to integrate your custom policy.

1.  **Locate the Template:** Navigate to [`aic_model/aic_model/model_node.py`](../aic_model/aic_model/model_node.py) in the provided repository.
2.  **Load Your Model:** Implement the logic to initialize and load your trained policy (e.g., PyTorch checkpoint, ONNX model, or control policy) when the node starts.
3.  **Run Inference:** Configure the node to subscribe to observation topics. When an observation is received, pass the data through your model to generate the next action.
4.  **Output Commands:** Publish the generated actions to the appropriate command topics to move the robot.
5.  **Trigger Completion:** Once the cable insertion is successfully detected, your node must trigger the completion callback to signal the end of the task.

> **Note:** You can reference the template structure here: [aic_model/aic_model/model_node.py](../aic_model/aic_model/model_node.py)

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
