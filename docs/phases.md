# Competition Phases

## Qualification Phase: Train your model

During qualification, use your favorite tools, including open source tools and simulators, along with the Intrinsic challenge toolkit, to train your model and solve the cable insertion task. Submitted models will be evaluated using Gazebo.

![](../../media/qualification_overview.png)

- To develop a policy, participants are free to use any approach
  - Real world teleop data
  - Training in simulator of choice (MuJoCo, Isaac Sim, O3DE, etc)
  - Classical control algorithms.
- Policies (wrapped in a service) simply have to output actions in standard formats while consuming information about the world in standard formats.
- The Evaluator Simulator (Gazebo) that will be provided as part of the toolkit will score performance of participant models.
- During development participants can run the Evaluator Simulator locally to score performance.
  - When ready, participants will submit their services.
  - Cloud instance will run the same Evaluator Sim and log participant scores.

## Phase 1: Develop in Flowstate

For those who advance to Phase 1 - teams will gain access to Intrinsic Flowstate, our development environment, and the Intrinsic Vision Model to build a complete robotic cable handling solution using their trained models.

TODO

## Phase 2: Run on real robots

Phase 2 participants will get the chance to deploy their solution to a robotic workcell set up at Intrinsic’s HQ - for real world solution validation and a chance to win prizes.

TODO
