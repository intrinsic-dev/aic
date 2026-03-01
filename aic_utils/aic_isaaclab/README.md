# AIC Isaac Lab Integration

This package provides documentation, scripts, and utilities for setting up AI for Industry Challenge (AIC) environment in Isaac Lab.


## Overview

[Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/index.html) is a unified and modular framework for robot learning that aims to simplify 
common workflows in robotics research (such as reinforcement learning, learning from demonstrations, and motion planning). In collaboration with 
**NVIDIA**, this integration enables participants to:

- Perform teleoperation in AIC environment 
- Record and replay episodes for Imitation Learning
- Use Reinforcement Learning with rsl-rl library for training policy


## Workflow

> [!TIP]
> If you run into issues that appear to be related to **Isaac Lab** (e.g. framework behavior, Docker setup, or Isaac Lab APIs), please open an issue on the [Isaac Lab GitHub repository](https://github.com/isaac-sim/IsaacLab). The maintainers there are best placed to help. For issues specific to the AIC integration or challenge assets, use this repo’s issue tracker.

**Recommended:** Use the assets prepared by the NVIDIA team. Download and place them as instructed, then start the container and run the task.

| Step | What you do | Section |
|------|-------------|---------|
| 1 | Install Docker and NVIDIA Container Toolkit | [Prerequisites](#prerequisites) |
| 2 | Clone and build Isaac Lab, then clone the AIC repo into `IsaacLab` | [Installation & Setup](#installation--setup) |
| 3 | Download the NVIDIA-prepared assets and place them in `Intrinsic_assets` | [Assets](#assets) |
| 4 | Start the Isaac Lab container and enter it | [Assets](#assets) |
| 5 | Run teleoperation or reinforcement learning from inside the container | [Usage](#usage) |


## Prerequisites

### Docker

1. Install [Docker Engine](https://docs.docker.com/engine/install/) for your platform.
2. Complete the [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/) to enable managing Docker as a non-root user.

### NVIDIA Container Toolkit (Optional)

1. Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to allow Docker Engine to access your NVIDIA GPU.

2. After installation, configure Docker to use the NVIDIA runtime:
    ```bash
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```


## Setup 

> [!NOTE]
> All commands in this section are to be executed on your **host machine** (not inside Docker).

> [!WARNING]
> The integration is tested with Isaac Lab version *2.3.2*.

Clone the Isaac Lab repository in your home directory:
```bash
cd ~
git clone git@github.com:isaac-sim/IsaacLab.git
```

Clone the AIC repository inside `IsaacLab` directory:
```bash
cd ~IsaacLab
git clone git@github.com:intrinsic-dev/aic.git
```

## Assets

The **NVIDIA team has prepared the assets** needed for the challenge. [Download the provided asset pack](https://developer.nvidia.com/downloads/Omniverse/learning/Events/Hackathons/Intrinsic_assets.zip), extract it, and place the files as follows.

Place the `Intrinsic_assets` directory inside `aic_task`:

```bash
~/IsaacLab/aic/aic_utils/aic_isaaclab/source/aic_task/aic_task/tasks/manager_based/aic_task/
```

**Files to place there** (from the downloaded pack):
- `nic_card.usd`
- `sc_port.usd`
- `robot_cable.usd`
- `aic.usd`
- `Task_board_rigid.usd`
- `cable.usd`

If the asset pack includes world, enclosure, or robot USDs and separate placement instructions, follow those. Otherwise the prepared pack is self-contained.


## Installation

Build the `base` profile (this creates the `isaac-lab-base` Docker image):
```bash
cd ~IsaacLab
./docker/container.py build base
```

Start the container and attach shell to it (from the Isaac Lab repo):
```bash
cd ~/IsaacLab
./docker/container.py start base
./docker/container.py enter base
```

Install `aic_task` in the Isaac Lab container with edit mode:
```
 python -m pip install -e aic/aic_utils/aic_isaaclab/source/aic_task
 ```


## Usage

> [!NOTE]
> The following commands are to be executed **inside the Isaac Lab container** after starting and entering it.

### Environment and Sensor Reading
List available environments (`AIC-Task-v0` RL Environment is provided as reference):
```bash
isaaclab -p aic/aic_utils/aic_isaaclab/scripts/list_envs.py
```

### Teleoperation and Imitation Learning
Teleoperate the robot with keyboard:
```bash
isaaclab -p aic/aic_utils/aic_isaaclab/scripts/teleop.py \
    --task AIC-Task-v0 --num_envs 1 --teleop_device keyboard --enable_cameras
```

For data collection:
```bash
isaaclab -p aic/aic_utils/aic_isaaclab/scripts/record_demos.py \
    --task AIC-Task-v0 --teleop_device keyboard --enable_cameras \
    --dataset_file ./datasets/dataset.hdf5 --num_demos 10
```

To replay collected episodes:
```bash
isaaclab -p aic/aic_utils/aic_isaaclab/scripts/replay_demos.py \
    --dataset_file ./datasets/dataset.hdf5
```


> [!NOTE]
> Users will have to connect the external environment with Isaac Lab for recording teleoperated data.

Additional resources:
1. [Teleoperation using Keyboard, Spacemouse and XR](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html#teleoperation)
2. [Recording Teleoperation data](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html#collecting-demonstrations)
3. [Imitation Learning in Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html#imitation-learning-with-isaac-lab-mimic)

### Reinforcement Learning
Run the training script from your terminal using the following command:
```bash
isaaclab -p aic/aic_utils/aic_isaaclab/scripts/rsl_rl/train.py \
    --task AIC-Task-v0 --num_envs 1 --enable_cameras
```

Other Resources:
1. [Gear Assembly Task](https://isaac-sim.github.io/IsaacLab/main/source/policy_deployment/02_gear_assembly/gear_assembly_policy.html)
2. [Creating a manager-based RL environment](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/create_manager_rl_env.html)
3. [Task Curation, VLA training and Policy Evaluation using Isaac Lab Arena](https://isaac-sim.github.io/IsaacLab-Arena/release/0.1.1/index.html)



Isaac Sim Documentation on Tuning and Importing Assets:
- [Tutorial: Import URDF](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/import_urdf.html)
- [Tuning Joint Drive Gains](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/joint_tuning.html)
- [Gain Tuner Extension](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html)
- [Physics Inspector](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/physics/joint_inspector.html)
- [Simulation Data Visualizer](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/physics/ext_isaacsim_inspect_physics.html)

### Directory Structure

```bash
aic_isaaclab/
├── README.md
├── pyproject.toml
├── scripts
│   ├── list_envs.py
│   ├── random_agent.py
│   ├── record_demos.py
│   ├── replay_demos.py
│   ├── rsl_rl
│   │   ├── cli_args.py
│   │   ├── play.py
│   │   └── train.py
│   ├── teleop.py
│   └── zero_agent.py
└── source
    └── aic_task
        ├── aic_task
        │   ├── __init__.py
        │   ├── extension.py
        │   └── tasks
        │       ├── __init__.py
        │       └── manager_based
        │           ├── __init__.py
        │           └── aic_task
        │               ├── __init__.py
        │               ├── agents
        │               │   ├── __init__.py
        │               │   └── rsl_rl_ppo_cfg.py
        │               ├── aic_task_env_cfg.py
        │               └── mdp
        │                   ├── __init__.py
        │                   ├── events.py
        │                   ├── observations.py
        │                   └── rewards.py
        ├── config
        │   └── extension.toml
        ├── docs
        │   └── CHANGELOG.rst
        ├── pyproject.toml
        └── setup.py
```



## Future Work

Planned improvements for the workflow:
- Add support to generate USD of Task Board


## Resources

- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/main/index.html)
- [AIC Getting Started Guide](../../docs/getting_started.md)
- [AIC Scene Description](../../docs/scene_description.md)