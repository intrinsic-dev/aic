# Challenge Overview

The AI for Industry Challenge targets a high-value bottleneck in modern manufacturing: electronics assembly. More specifically dexterous cable management and insertion, which today is a manual, repetitive process.

In terms of robotics, this task is notoriously difficult due to the complex physics of flexible cables and the precision required to see, handle and carefully insert connectors.

Train an AI model using open-source simulators (e.g., Isaac Sim, MuJoCo, Gazebo), leveraging ROS for communication. This is your opportunity to bridge the sim-to-real gap and make real progress against a very real problem.

Finalists will deploy their models from simulation to a physical workcell which will be hosted at Intrinsic’s HQ. The top five solutions and teams win a share of the $180,000 prize pool.

## Phases

The challenge will officially begin on February 11 and run through July 2026. It has three phases:

- Qualification (~3 months): Participants will train and test their cable assembly models in simulation.
- Phase #1: (~1 month): Qualified teams will advance and gain access to Intrinsic Flowstate to develop a complete cable handling solution.
- Phase #2: (~1 month): Top teams will move on to deploy and refine their solutions on a physical workcell provided by Intrinsic for real-world testing and evaluation.

For more details on expectations and deliverables in each phase, click [here](./phases.md).

## Getting Started

To get started, click [here](./getting_started.md)

## Evaluation

Scoring across all three phases of the challenge will be automated and determined by a combination of the following evaluation criteria:

- Model validity: Submission must load without errors and generate valid robot commands on the required ROS topic. Invalid submissions will not be scored.
- Task success: A binary metric will be applied per successful cable insertion.
- Precision: Submissions will be scored based on how closely the connectors are inserted to their respective targets.‍
- Safety: Penalties will be applied for any collisions or for exerting excessive forces on connectors or cables.‍
- Efficiency: The overall cycle time to complete the entire set of assembly tasks will be measured, rewarding more efficient solutions.

## Submission

At each phase, teams must submit their models or solutions to be considered for advancement in the challenge and eligibility for prizes. Each team leader will receive a unique authentication token to upload submissions. Teams may make multiple submissions before the submission deadline.

For more details, click [here](./submission.md).

## Baseline Solution

TODO
