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

**How to use:**
- Pre-built images are available at `ghcr.io/intrinsic-dev/aic/aic_eval:latest`

### 2. Participant workspace
This is your development workspace where you implement your policy.

**What you'll do:**
- Implement your policy in the `aic_model` package
- Test locally against the evaluation container
- Build and submit your container image for official evaluation

**How to use:**
- See [Policy Integration Guide](./policy.md) for implementation details

### Requirements

* [docker](https://www.docker.com/)
* [nvidia-continer-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html)
* [distrobox](https://distrobox.it/)
* [pixi](https://prefix.dev/tools/pixi)

#### Install docker

Refer to [Docker Engine Installation](https://docs.docker.com/engine/install/).

#### Install and configure nvidia-container-toolkit

Follow https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html to install nvidia-container-toolkit.

After installing, be sure to enable it for docker:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

#### Install distrobox

If your distro already supports distrobox ([supported distros](https://distrobox.it/#installation)). It is recommended to install with the package manager.

For example, in ubuntu:

```bash
sudo apt install distrobox
```

If it is not supported, refer to [Alternative methods](https://distrobox.it/#alternative-methods) to install distrobox.

#### Install pixi

```bash
curl -fsSL https://pixi.sh/install.sh | sh
# Restart the terminal
```

Also see [Alternative Installation Methods](https://pixi.prefix.dev/latest/installation/#alternative-installation-methods).

### Quick Start

1. **Start the evaluation container:**
   ```bash
   # Set up Docker container manager
   export DBX_CONTAINER_MANAGER=docker

   # Create and enter the eval container
   distrobox create -r -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
   distrobox enter -r aic_eval

   # Inside the container, start the environment
   /entrypoint.sh
   ```

<!-- TODO: Update instruction to disable ACL after https://github.com/intrinsic-dev/aic/pull/190 or https://github.com/intrinsic-dev/aic/pull/171 is merged. -->

2. **Set up pixi workspace:**
   ```bash
   # Clone this repo
   mkdir -p ~/ws_aic/src
   cd ~/ws_aic/src
   git clone https://github.com/intrinsic-dev/aic

   # Install dependencies
   cd ~/ws_aic/src/arc
   pixi install
   ```

3. **Run an example policy:**
   ```bash
   pixi run ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
   ```

---

## Next Steps

Now that you have your environment set up:

1. **📚 Read the Documentation**
   - [Qualification Phase Details](./qualification_phase.md) - Understand the trials you'll be evaluated on
   - [Challenge Rules](./challenge_rules.md) - Ensure your policy complies with all requirements
   - [Policy Integration Guide](./policy.md) - Learn how to implement your policy

2. **💻 Start Developing**
   - Explore `aic_example_policies/` for reference implementations
   - Review [AIC Interfaces](./aic_interfaces.md) to understand available sensors and actuators
   - Consult [AIC Controller](./aic_controller.md) to learn about motion commands
   - Follow the [Creating a new policy node tutorial](./policy_tutorial.md) to learn how to create your own policy node.

3. **🧪 Test and Iterate**
   - Use the example configurations in `aic_engine/config/` to test different scenarios
   - Monitor your policy's behavior with ground truth data during development
   - Refer to [Troubleshooting](./troubleshooting.md) if you encounter issues

4. **📦 Prepare for Submission**
   - Package your solution following [Submission Guidelines](./submission.md)
   - Test your container before submitting
   - Submit through the official portal

---

## Advanced Topics

### LeRobot Support

A LeRobot interface is available to train a policy using [LeRobot](https://huggingface.co/lerobot). See [lerobot_robot_aic](../aic_utils/lerobot_robot_aic/README.md).

### Debugging Commands

**Send a reference wrench command (10N in the positive z-axis) to the controller:**
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup move_to_contact.launch.py contact_force_z:=10.0
```

**Control the gripper via ROS 2 Action (range: 0.0 to 0.025m):**
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup gripper_action.launch.py use_position:=true position:=0.024
```

**Send a joint-position command to the arm:**
```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
# Switch to joint target mode on the controller
ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode '{target_mode: 1}'
# Send joint target
ros2 topic pub /aic_controller/joint_commands aic_control_interfaces/msg/JointMotionUpdate '{target_state:
{positions: [0.0, -1.57, -1.57, -1.57, 1.57, 0] }, target_stiffness: [100.0, 100.0, 100.0, 50.0, 50.0, 50.0], target_damping: [40.0, 40.0, 40.0, 15.0, 15.0, 15.0], trajectory_generation_mode: {mode: 2}, time_to_target_seconds: 1.0 }' --once
```

---

## Need Help?

- **Documentation**: Check the [main README](../README.md) for links to all documentation
- **Issues**: Report problems via [GitHub Issues](https://github.com/intrinsic-dev/aic/issues)
- **Community**: Join discussions at [Open Robotics Discourse](https://discourse.openrobotics.org/c/competitions/ai-for-industry-challenge/)
