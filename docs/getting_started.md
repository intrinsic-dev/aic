# Getting Started

Welcome to the AI for Industry Challenge! This guide will help you set up your development environment and get started with building your solution.

The challenge workflow relies on two distinct components:

### 1. Evaluation Container (`aic_eval`)
This container hosts:
- Gazebo simulation environment
- Robot arm and end-of-arm tooling
- Task board spawning and management
- Trial orchestration via `aic_engine`
- Scoring system

### 2. Participant Workspace
This is your development workspace where you implement your policy.

**What you'll do:**
- Implement your policy in the `aic_model` package
- Test locally against the evaluation container
- Build and submit your container image for official evaluation

> [!WARNING]
> Exploration of the aic_eval container logic is welcome. However, the evaluation 
> environment is read-only/stateless, meaning any manual changes made to the 
> container will not be reflected during the final evaluation.


## Prerequisite
* [Docker](#setup-docker)
* [Nvidia Container Toolkit](#setup-and-configure-nvidia-container-toolkit)
* [Distrobox](#setup-distrobox)
* [Pixi](#setup-pixi)

### Setup Docker

1. Install [Docker Engine](https://docs.docker.com/engine/install/) depending on your Platform.
2. Make sure to complete [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/). This will allow managing Docker as a non-root user.

### Setup and Configure Nvidia Container Toolkit

1. Install [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) which will allow Docker Engine to access Nvidia GPU.

2. After installing Nvidia Container Toolkit, configure docker to use nvidia runtime:
    ```bash
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```

### Setup Distrobox

We use [Distrobox](https://distrobox.it/) to tightly integrate aic_eval container to host. It is recommended to install Distrobox with the package manager. Check [supported distros](https://distrobox.it/#installation) to see if your distro already supports distrobox.
- For Ubuntu you can run following:
    ```bash
    sudo apt install distrobox
    ```

Refer to [Alternative methods](https://distrobox.it/#alternative-methods) if you are on different distro.

### Setup pixi

We use [Pixi](https://pixi.prefix.dev/latest/) to manage packages and dependencies including ROS2. 
- For Ubuntu you can run following:
    ```bash
    curl -fsSL https://pixi.sh/install.sh | sh
    # Restart the terminal
    ```
Refer to [Alternative Installation Methods](https://pixi.prefix.dev/latest/installation/#alternative-installation-methods) if you are on different OS.


## Quick Start (Eval container with pixi workspace)

1. **Start the evaluation container with distrobox:**
   ```bash
   # Indicate distrobox to use Docker as container manager
   export DBX_CONTAINER_MANAGER=docker

   # Create and enter the eval container
   docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest
   distrobox create -r -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
   distrobox enter -r aic_eval

   # Inside the container, start the environment
   /entrypoint.sh start_aic_engine:=true ground_truth:=false
   ```

   The ```entrypoint.sh``` script runs a Zenoh router and ```aic_gz_bringup.launch.py``` launch file. If everything launched successfully you should see Gazebo and RVIZ window showcasing a workcell with Universal Robots UR5e manipulator, cable and a task board. Check out [Scene Description](./scene_description.md) for more details.

   <!-- TODO: Update instruction to disable ACL after https://github.com/intrinsic-dev/aic/pull/190 or https://github.com/intrinsic-dev/aic/pull/171 is merged. -->

> [!Note]
> If your docker pull fails, you would need to [login to ghcr.io](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-with-a-personal-access-token-classic).

   <!-- TODO: Shouldn't need to login after we make it public -->


2. **Set up pixi workspace:**
   ```bash
   # Clone this repo
   mkdir -p ~/ws_aic/src
   cd ~/ws_aic/src
   git clone https://github.com/intrinsic-dev/aic

   # Install and build dependencies
   cd ~/ws_aic/src/arc
   pixi install
   ```

3. **Run an example policy:**
   ```bash
   pixi run ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
   ```

   After executing the sample policy, the arm should be waving to you. In the terminal If that's not the case checkout [Toubleshooting](./troubleshooting.md) section. 

> [!Tip]
> - Want to customize your training environment? See [Training Mode](#training-mode) to learn how to configure task board layouts and cable positions.
> - Learn where results are saved and how to monitor progress in [Monitoring and Results](#monitoring-and-results).



## Next Steps

Now that you have your environment set up:

1. **💻 Start Developing**
   - See [Explore Environment](./explore_environment.md) to understand available arguments and configurations.
   - Checkout `aic_example_policies/` for reference implementations
   - Review [AIC Interfaces](./aic_interfaces.md) to understand available sensors and actuators
   - Consult [AIC Controller](./aic_controller.md) to learn about motion commands
   - Follow the [Creating a new policy node tutorial](./policy_tutorial.md) to learn how to create your own policy node.

2. **🧪 Test and Iterate**
   - Use the example configurations in `aic_engine/config/` to test different scenarios
   - Monitor your policy's behavior with ground truth data during development
   - Refer to [Participant Utilities](./participant_utilities.md) for a list of helpful tools
   - Refer to [Troubleshooting](./troubleshooting.md) if you encounter issues

3. **📦 Prepare for Submission**
   - Package your solution following [Submission Guidelines](./submission.md)
   - Test your container before submitting
   - Submit through the official portal
