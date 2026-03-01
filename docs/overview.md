# Challenge Overview

![](../../media/aic_overview.png)

The **AI for Industry Challenge** targets a critical bottleneck in modern manufacturing: electronics assembly. Specifically, it focuses on dexterous cable management and insertion—a task that currently remains largely manual and repetitive.

From a robotics perspective, this task is notoriously difficult due to the complex physics involved in manipulating flexible cables and the extreme precision required to perceive, handle, and insert connectors.

**The Goal:**
Participants will train an AI model using open-source simulators (e.g., Isaac Sim, MuJoCo, Gazebo), leveraging ROS for communication. This is your opportunity to bridge the sim-to-real gap and make tangible progress on a significant real-world problem.

**The Reward:**
Finalists will deploy their models from simulation to a physical workcell hosted at Intrinsic’s HQ. The top five teams will share a **$180,000 prize pool**.

---

## Phases

The challenge officially begins on **March 2** and runs through **September 8, 2026**. It consists of three distinct phases:

* **Qualification (Mar 2 - May 15):** Participants train and test their cable assembly models in simulation. Evaluation Period: May 18 - 27. Top 30 announced May 28.
* **Phase 1 (May 28 - Jul 14):** Qualified teams advance and gain access to Intrinsic Flowstate to develop a complete cable handling solution. Evaluation Period: Jul 14 - 21. Top 10 announced Jul 22.
* **Phase 2 (Jul 27 - Aug 25):** Top teams move on to deploy and refine their solutions on a physical workcell provided by Intrinsic for real-world testing and evaluation. Evaluation Period: Aug 26 - Sep 4. Winner announced Sep 8.

For a detailed breakdown of expectations and deliverables, please refer to [Competition Phases](./phases.md).

## Evaluation

Scoring across all three phases is automated. Rankings are determined by a combination of the following criteria:

* **Model Validity:** Submissions must load without errors and generate valid robot commands on the required ROS topics. Invalid submissions are disqualified.
* **Task Success:** A binary metric applied to each successful cable insertion.
* **Precision:** Scores based on how closely the connectors are inserted relative to their target pose.
* **Safety:** Penalties applied for collisions or excessive force exerted on connectors or cables.
* **Efficiency:** Measurement of the overall cycle time to complete the assembly tasks; faster solutions are rewarded.

For details, see [Scoring](./scoring.md) and the [Scoring Test & Evaluation Guide](./scoring_tests.md).

## Submission

To advance in the challenge and remain eligible for prizes, teams must submit their models at the end of each phase.
* **Authentication:** Each team leader will receive a unique authentication token for uploads.
* **Frequency:** Teams may make multiple submissions prior to the deadline; the final submission will be used for scoring.

For detailed upload instructions, see [Submission Guidelines](./submission.md).

## Baseline Policies

We provide several baseline policy implementations to help you get started, including a minimal example, a ground truth-based policy for debugging, and an ACT (Action Chunking with Transformers) policy.

For details on running these policies, see the [Example Policies README](../aic_example_policies/README.md) and the [Policy Integration Guide](./policy.md#baseline-policies).

---

## Getting Started

Ready to begin? Please consult the [Getting Started Guide](./getting_started.md).
