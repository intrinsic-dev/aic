# Getting Started

Welcome to the AI for Industry Challenge! This guide will help you set up your development environment and get started with building your solution.

## Architecture Overview

The challenge uses a two-component architecture:

1. **Evaluation Component** (provided) - Runs the simulation, robot, sensors, and scoring system
2. **Participant Model** (you implement) - Your ROS 2 node that processes sensor data and commands the robot

**Source code for both components is available in this toolkit.** Since the Evaluation Component will not change during the competition, we publish a Docker image (`aic_eval`) for convenience that you can reuse—this is the **recommended workflow**. Advanced users who prefer to build from source can follow the [Building from Source](./build_eval.md) guide.

For a detailed explanation of the architecture, packages, and interfaces, see the [Toolkit Architecture](../README.md#toolkit-architecture) section in the README.

---

## Setup

First, install the following tools:
* [Docker](#setup-docker) (required)
* [Distrobox](#setup-distrobox) (required)
* [Pixi](#setup-pixi) (required)
* [NVIDIA Container Toolkit](#setup-and-configure-nvidia-container-toolkit) (optional - for NVIDIA GPU users)

### Setup Docker

1. Install [Docker Engine](https://docs.docker.com/engine/install/) for your platform.
2. Complete the [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/) to enable managing Docker as a non-root user.

### Setup and Configure NVIDIA Container Toolkit (Optional)

> [!NOTE]
> This step is only required if you have an NVIDIA GPU and want to use GPU acceleration for optimal performance.

1. Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to allow Docker Engine to access your NVIDIA GPU.

2. After installation, configure Docker to use the NVIDIA runtime:
    ```bash
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```

### Setup Distrobox

We use [Distrobox](https://distrobox.it/) to tightly integrate the `aic_eval` container with your host system. We recommend installing Distrobox using your package manager. Check the [supported distros](https://distrobox.it/#installation) to see if your distribution supports Distrobox.

For Ubuntu, run:
```bash
sudo apt install distrobox
```

For other distributions, refer to the [Alternative methods](https://distrobox.it/#alternative-methods).

### Setup Pixi

We use [Pixi](https://pixi.prefix.dev/latest/) to manage packages and dependencies, including ROS 2.

For Ubuntu, run:
```bash
curl -fsSL https://pixi.sh/install.sh | sh
# Restart your terminal after installation
```

For other operating systems, refer to the [Alternative Installation Methods](https://pixi.prefix.dev/latest/installation/#alternative-installation-methods).


## Quick Start

This section will guide you through:

1. **Running the Evaluation Component** - Start the `aic_eval` container to bring up the simulation environment, robot, sensors, and scoring system
2. **Setting Up Your Workspace** - Clone the challenge repository and install dependencies using Pixi
3. **Running an Example Policy** - Execute a provided example policy from your local workspace against the evaluation container

Once you've completed these steps and want to prepare your submission, see the [Submission Guidelines](./submission.md) to learn how to containerize your participant workspace.

---

### Step 1: Start the Evaluation Container

```bash
# Indicate distrobox to use Docker as container manager
export DBX_CONTAINER_MANAGER=docker

# Create and enter the eval container
docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest
distrobox create -r -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
distrobox enter -r aic_eval

# Inside the container, start the environment
/entrypoint.sh ground_truth:=false start_aic_engine:=true
```

The [`entrypoint.sh`](../docker/aic_eval/Dockerfile) script runs a Zenoh router and the [`aic_gz_bringup.launch.py`](../aic_bringup/README.md#1-aic_gz_bringuplaunchpy) launch file.

**What you should see:**
- Two windows open: **Gazebo** (simulation) and **RViz** (visualization)
- In Gazebo: A workcell with a Universal Robots UR5e manipulator mounted on a table
- In the terminal: Log messages indicating the AIC engine has initialized and is waiting for the `aic_model` node
- No robot movement yet (the robot is waiting for your policy to connect)

![Evaluation Environment](../../media/eval_environment_waiting.png)

See [Scene Description](./scene_description.md) for more details about the simulation environment.

<!-- TODO: Update instruction to disable ACL after https://github.com/intrinsic-dev/aic/pull/190 or https://github.com/intrinsic-dev/aic/pull/171 is merged. -->

> [!TIP]
> If you have an NVIDIA GPU, create the distrobox container with GPU support for optimal performance:
> ```bash
> distrobox create -r --nvidia -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
> ```

> [!Note]
> If the `docker pull` command fails, you may need to [log in to ghcr.io](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-with-a-personal-access-token-classic).

<!-- TODO: Shouldn't need to login after we make it public -->

---

### Step 2: Set Up Your Workspace

```bash
# Clone this repo
mkdir -p ~/ws_aic/src
cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic

# Install and build dependencies
cd ~/ws_aic/src/aic
pixi install
```

**What you should see:**
- Pixi downloading and installing ROS 2 packages and dependencies
- A successful installation message when complete
- A `.pixi` directory created in your workspace with all dependencies

---

### Step 3: Run an Example Policy

```bash
pixi run ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
```

Once the `aic_model` node starts, the AIC engine spawns a task board and a gripper-attached cable in the Gazebo window. The eval container terminal will then track three successive trials and display their scores. See [Scoring](./scoring.md) for more details.

**Note:** The `WaveArm` policy is a dummy example that simply moves the robot arm back and forth in a waving motion. It does not attempt to solve the cable insertion task. The purpose of this example is to demonstrate how the [`aic_engine`](../aic_engine/README.md) orchestrates trials based on the [sample configuration](../aic_engine/config/sample_config.yaml) and scores the policy based on its performance (which will be poor in this case, as expected).

**What you should see:**
- **In Gazebo**: The task board and a cable attached to the gripper appear in the simulation
- **In the robot**: The arm moves back and forth in a waving motion
- **In the eval container terminal**:
  - Log messages showing trial progression (Trial 1/3, Trial 2/3, Trial 3/3)
  - Scoring information after each trial
  - Final summary with total scores across all trials
- The robot performing three successive trials automatically
- **Results saved to**: `$HOME/aic_results/` (or `$AIC_RESULTS_DIR` if set)

![Wave Arm Policy](../../media/wave_arm_policy.gif)

If the robot doesn't move or you don't see the expected behavior, check the [Troubleshooting](./troubleshooting.md) section.

---

## 🎉 Congratulations!

You've successfully completed the Quick Start guide! You now have:
- ✅ A running evaluation environment with Gazebo and RViz
- ✅ A local Pixi workspace with all dependencies installed
- ✅ Experience running an example policy and seeing how the AIC engine orchestrates trials

**Next:** When you're ready to submit your solution, you'll need to containerize your participant workspace. See the [Submission Guidelines](./submission.md) for detailed instructions on packaging and submitting your policy.

---

## Next Steps

Now that your environment is set up:

1. **💻 Start Developing**
   - Explore the [Scene Description](./scene_description.md) to learn how to customize and explore the environment
   - Read the [Policy Integration Guide](./policy.md) to understand how to create your own policy node
   - Check out [`aic_example_policies/`](../aic_example_policies/) for reference implementations
   - Review [AIC Interfaces](./aic_interfaces.md) to understand available sensors and actuators
   - Consult [AIC Controller](./aic_controller.md) to learn about motion commands

2. **🧪 Test and Iterate**
   - Use the example configurations in [`aic_engine/config/`](../aic_engine/config/) to test different scenarios
   - Create your own test scenarios by following the configuration examples in [`aic_engine/config/`](../aic_engine/config/)
   - Monitor your policy's behavior with ground truth data during development
   - See [Participant Utilities](./participant_utilities.md) for a list of helpful tools
   - Refer to [Troubleshooting](./troubleshooting.md) if you encounter issues

3. **📦 Prepare for Submission**
   - Package your solution following the [Submission Guidelines](./submission.md)
   - Test your container before submitting
   - Submit through the official portal

## Need Help?

- **Documentation**: Check the [main README](../README.md) for links to all documentation
- **Issues**: Report problems via [GitHub Issues](https://github.com/intrinsic-dev/aic/issues)
- **Community**: Join discussions at [Open Robotics Discourse](https://discourse.openrobotics.org/c/competitions/ai-for-industry-challenge/)
