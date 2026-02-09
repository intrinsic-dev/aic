# AI for Industry Challenge Toolkit

[![build](https://github.com/intrinsic-dev/aic/actions/workflows/build.yml/badge.svg)](https://github.com/intrinsic-dev/aic/actions/workflows/build.yml)
[![style](https://github.com/intrinsic-dev/aic/actions/workflows/style.yml/badge.svg)](https://github.com/intrinsic-dev/aic/actions/workflows/style.yml)

> [!NOTE]
> This repository is under active development.

![](../media/aic_banner.png)

The **AI for Industry Challenge** is an open competition for developers and roboticists aimed at solving some of the hardest, high-impact problems in robotics and manufacturing.

This repository contains the official toolkit to help participants start developing their solutions. For registration details, official rules, and FAQs, please visit the [AI for Industry Challenge event page](https://www.intrinsic.ai/events/ai-for-industry-challenge).

---

## Quick Start Guide

New to the challenge? Follow these steps to get started:

1. **📖 Understand the Challenge**
   - Read the [Challenge Overview](./docs/overview.md) to understand the goals.
   - Review the [Qualification Phase](./docs/qualification_phase.md) to understand what you'll be building.

2. **🔧 Set Up Your Environment**
   - Follow the [Installation Guide](./docs/getting_started.md) to set up your development environment.
   - Choose between local setup or Docker containers.

3. **💻 Develop Your Policy**
   - Start with the [Policy Integration Guide](./docs/policy.md) to implement your solution.
   - Review [AIC Interfaces](./docs/aic_interfaces.md) to understand available sensors and actuators.
   - Consult the [Challenge Rules](./docs/challenge_rules.md) to ensure compliance.

4. **🧪 Test Your Solution**
   - Use the provided simulation environment to test your policy.
   - Refer to [Troubleshooting](./docs/troubleshooting.md) if you encounter issues.

5. **📦 Submit Your Entry**
   - Package your solution following the [Submission Guidelines](./docs/submission.md).
   - Submit through the official portal.

---

## Toolkit Architecture

The AI for Industry Challenge toolkit is divided into **two main components**:

### 1. Evaluation Component (Provided - Run by Organizers)

This component provides the complete evaluation infrastructure:
- **`aic_engine`** - Orchestrates trials and computes scores.
- **`aic_bringup`** - Launches simulation environment (Gazebo, robot, sensors).
- **`aic_controller`** - Low-level robot control with force management.
- **`aic_adapter`** - Sensor fusion and data synchronization.

**What you receive:** Standard ROS sensor topics providing camera images, joint states, force/torque measurements, and TF frames.

### 2. Participant Model Component (Your Implementation - What You Submit)

This is what you develop and submit:
- **A ROS 2 node** that follows the behavioral requirements defined in [Challenge Rules](./docs/challenge_rules.md).
- **Your custom logic** - Code to process sensor data and command the robot to insert cables.

**What you provide:** A container with a ROS 2 Lifecycle node named `aic_model` that responds to the `/insert_cable` action and outputs robot motion commands via standard ROS topics/services.

**Convenient Entry Point:** We provide an `aic_model` framework that handles all the ROS 2 boilerplate and lifecycle management. You simply implement a Python policy class that gets dynamically loaded at runtime. See the [Policy Integration Guide](./docs/policy.md) for details.

### Development and Submission Workflow

**Development Options:**
- Develop inside a container (recommended - matches evaluation environment).
- OR develop in native Ubuntu 24.04 environment (requires all dependencies).

**Submission Requirements:**
- Package your solution using the provided `aic_model` Dockerfile.
- Submit your container - it must respond to standard ROS inputs and command the robot to insert cables.
- Your container interfaces with the evaluation component via ROS topics.

---
## Repository Structure

```
aic/
├── aic_adapter/          # Adapter for interfacing between model and controller
├── aic_assets/           # 3D models and simulation assets
├── aic_bringup/          # Launch files for starting the challenge environment
├── aic_controller/       # Robot controller implementation
├── aic_description/      # Robot and environment URDF/SDF descriptions
├── aic_engine/           # Trial orchestration and validation engine
├── aic_example_policies/ # Example policy implementations
├── aic_gazebo/           # Gazebo-specific plugins and configurations
├── aic_interfaces/       # ROS 2 message, service, and action definitions
├── aic_model/            # Template for participant policy implementation
├── aic_scoring/          # Scoring system implementation
├── aic_utils/            # Utility packages and tools
├── docker/               # Docker container definitions
└── docs/                 # Comprehensive documentation
```

---

## Key Packages for Participants

### `aic_model` - Convenient Policy Framework (Recommended)
This package provides a ready-to-use ROS 2 Lifecycle node that dynamically loads and executes your Python policy implementation. It handles all ROS 2 boilerplate, lifecycle management, and challenge rule compliance, allowing you to focus on implementing your policy logic.
- **Location**: `aic_model/`.
- **Documentation**: [Policy Integration Guide](./docs/policy.md).
- **Tutorial**: [Creating a New Policy Node](./docs/policy.md#tutorial-creating-a-new-policy-node).

> **Note:** While we recommend using this framework, you may implement your own ROS 2 node from scratch as long as it adheres to the [Challenge Rules](./docs/challenge_rules.md).

### `aic_interfaces` - Communication Protocols
Defines all ROS 2 messages, services, and actions used in the challenge.
- **Location**: `aic_interfaces/`.
- **Documentation**: [AIC Interfaces](./docs/aic_interfaces.md).

### `aic_example_policies` - Reference Implementations
Example policies demonstrating different approaches and techniques.
- **Location**: `aic_example_policies/`.
- **README**: [aic_example_policies/README.md](./aic_example_policies/README.md).

### `aic_bringup` - Launch the Environment
Launch files to start the simulation, robot, and scoring systems.
- **Location**: `aic_bringup/`.
- **README**: [aic_bringup/README.md](./aic_bringup/README.md).

### `aic_engine` - Trial Orchestrator
Manages trial execution, validates participant models, and collects scoring data.
- **Location**: `aic_engine/`.
- **README**: [aic_engine/README.md](./aic_engine/README.md).

---

## Additional Documentation

### Challenge Information

* **[Challenge Overview](./docs/overview.md):** High-level summary of the competition goals and structure.
* **[Competition Phases](./docs/phases.md):** Details on Qualification, Phase 1, and Phase 2.
* **[Qualification Phase](./docs/qualification_phase.md):** Detailed technical overview of the qualification phase trials and scoring.
* **[Challenge Rules](./docs/challenge_rules.md):** Required behavior for participant models.
* **[Scoring](./docs/scoring.md):** Metrics and methods used to evaluate performance.

### Technical Documentation

* **[Getting Started](./docs/getting_started.md):** How to set up your local development environment.
* **[Policy Integration](./docs/policy.md):** Guide to implementing your policy in the `aic_model` framework.
* **[AIC Interfaces](./docs/aic_interfaces.md):** ROS 2 topics, services, and actions available to your policy.
* **[AIC Controller](./docs/aic_controller.md):** Understanding the robot controller and motion commands.
* **[Scene Description](./docs/scene_description.md):** Technical details of the simulation environment.
* **[Task Board Description](./docs/task_board_description.md):** Physical layout and specifications of the task board.
* **[Troubleshooting](./docs/troubleshooting.md):** Common issues and debugging strategies.

### Submission

* **[Submission Guidelines](./docs/submission.md):** How to package and submit your final model.

---


## Support and Resources

- **Issues**: Report bugs or request features via [GitHub Issues](https://github.com/intrinsic-dev/aic/issues).
- **Discussions**: Join the community at [Open Robotics Discourse](https://discourse.openrobotics.org/c/competitions/ai-for-industry-challenge/).
- **Event Page**: Visit the [AI for Industry Challenge](https://www.intrinsic.ai/events/ai-for-industry-challenge) for official updates.

---

## License

This project is licensed under the Apache License 2.0 - see the individual package files for details.
