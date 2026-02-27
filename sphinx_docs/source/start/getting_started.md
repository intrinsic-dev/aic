# Overview

Welcome to the AI for Industry Challenge! Follow this guide to familarize yourself with the toolkit structure, prepare  your environment, and confirm your setup by running a quickstart example before developing your solution.

## Architecture Overview

The challenge uses a two-component architecture:

1. **Evaluation Component** (provided) - Runs the simulation, robot, sensors, and scoring system
2. **Participant Model** (you implement) - Your ROS 2 node that processes sensor data and commands the robot

**Source code for both components is available in this toolkit.** Since the Evaluation Component will not change during the competition, we publish a Docker image (`aic_eval`) for convenience that you can reuse—this is the **recommended workflow**. Advanced users who prefer to build from source can follow the [Building from Source](./build_eval.md) guide.

For a detailed explanation of the architecture, packages, and interfaces, see the [Toolkit Architecture](../README.md#toolkit-architecture) section in the README.

---

## Requirements

**Minimum Compute Specifications:**

- **OS:** Ubuntu 24.04
- **CPU:** 4-8 cores
- **RAM:** 32GB+
- **GPU:** NVIDIA RTX 2070+ or equivalent
- **VRAM:** 8GB+

```{note}
While the challenge can run on systems without a GPU, performance will be significantly reduced. See [Troubleshooting](./troubleshooting.md#no-gpu-available) for optimization tips for CPU-only systems.
```

**Cloud Evaluation Instance:**

For cloud evaluation, all participant submissions will be evaluated on the same instance type with the following specifications:

- **vCPU:** 64 cores
- **RAM:** 256 GiB
- **GPU:** 1 x NVIDIA L4 Tensor Core
- **VRAM:** 24 GiB

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

```{note}
This step is only required if you have an NVIDIA GPU and want to use GPU acceleration for optimal performance.
```

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
