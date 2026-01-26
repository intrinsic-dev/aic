# Developer Workflow

This describes various ways to setup a workspace. The workspace is separated into 2 main components.

* The `evaluator` runs a gazebo simulation. It spawns the task set and performs scoring.
* The `participant` which runs a policy to finish the task. You will need to prepare a docker image of this workspace for submission.

In a typical workflow, you will need to setup both the evaluator and participant workspace.

# Evaulator

## Docker

Runs the evaluator on docker.

### Requirements

- docker
- [nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/)

### Setup

Follow the [nvidia container toolkit instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to configure docker. Particularly, make sure to follow [this](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker) step.

### Build `aic_eval` (optional)

```bash
cd docker
docker compose build eval
```

### Start a `aic_eval` container

```bash
docker run -it --rm --gpus=all --network=host ghcr.io/intrinsic-dev/aic/aic_eval
```

## Native

The evaluator is ran natively on the host. There is no sandbox, container, isolation etc.

### Requirements

- [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
- [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

### Install

Add `packages.osrfoundation.org` to the apt sources list:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

Build the workspace
```bash
sudo apt update && sudo apt upgrade -y
mkdir ~/ws_aic/src -p
cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic
vcs import . < aic/aic.repos --recursive
# Install Gazebo dependencies.
sudo apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
cd ~/ws_aic
# Install ROS dependencies using rosdep.
rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev rosetta"
source /opt/ros/kilted/setup.bash
GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install --packages-ignore lerobot_robot_aic aic_lerobot_tools
```

### Launch

> [!NOTE]
> For detailed information about all available launch files and their configurable parameters, see the [aic_bringup README](../aic_bringup/README.md).

> [!NOTE]
> We rely on [rmw_zenoh](https://github.com/ros2/rmw_zenoh) as the ROS 2 middleware for this application. Please ensure the `RMW_IMPLEMENTATION` environment variable is set to `rmw_zenoh_cpp` in all terminals.

Start the Zenoh router.

```bash
source ~/ws_aic/install/setup.bash
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 run rmw_zenoh_cpp rmw_zenohd
```

#### Evaluator bringup

> [!NOTE]
> Update with launch commands to start Zenoh router with ACLs.

To launch the evaluator,

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 launch aic_bringup aic_gz_bringup.launch.py ground_truth:=false start_aic_engine:=true
```

This will launch Gazebo with the robot arm and end-of-arm tooling together with all required drivers.
The `TaskBoard` and `Cable` will be spawned by `aic_engine`, the orchestrator for the challenge.
Note that you will need to bring up your model for the `aic_engine` to work correctly, see [Submission Bringup](#submission-bringup).

# Participant

## pixi (Recommended)

### Requirements

- [pixi](https://pixi.prefix.dev/latest/) (Recommended)

### Quick Guide to pixi

Pixi is a fast, modern, and reproducible package management tool for developers of all backgrounds. A pixi workspace is provided which set ups all the required ROS and pypi dependencies.

To set up the workspace, simply run

```bash
pixi install
```

#### Creating a ROS package

The [official docs](https://pixi.prefix.dev/latest/tutorials/ros2/) provides an in depth tutorial to create a ROS package in a pixi workspace.

You may also look at other pixi packages in this repo for examples. E.g. [aic_control_interfaces](../aic_interfaces/aic_control_interfaces/pixi.toml), [lerobot_robot_aic](../aic_utils/lerobot_robot_aic/pixi.toml).

#### Adding dependencies

ROS and other "system" dependencies:

```bash
pixi add ros-kilted-my-pkg
```

pypi dependencies:

```bash
pixi add --pypi mydep
```

#### Local dependencies

If your ROS package requires a local dependency, those dependencies must be available on the [root package](../pixi.toml). They must be added to the `[dev]` section so that pixi knows where to look for them while building. They also need to be added to the `[dependencies]` so that they are available at run time.

#### Build-Run-Debug cycle (python)

pixi does not install your package in "editable" mode. Any changes you made will not be reflected until you reinstall the package.

```bash
pixi reinstall <package>
```

> [!TIP]
> You may enter the pixi environment with `pixi shell` and force an "editable" install with `pip install -e` or `colcon build --symlink-install`. But note that this circumvents pixi and may cause unintended side effects.

You can then run your policy node as per a ROS package:

```bash
pixi run ros2 run <package> <executable>
```

#### Preparing docker image for submission

See the `aic_model` [Dockerfile](../docker/aic_model/Dockerfile) for an example. You would mostly just `COPY` the local dependencies in, then run `pixi install --frozen`.

#### pixi and ROS package mapping (optional)

> [!NOTE]
> This describes advanced dependency handling which is not required unless you plan to release your pixi package as a ROS or conda package.

##### Mapping between ROS and conda dependencies

pixi automatically converts dependencies it finds in `package.xml` to conda packages. There is a builtin mapping which works in most cases, but if your package requires a uncommon package, then pixi will fail to find a mapping.

In those cases, you can use `package.build.config.extra-package-mappings` config in your pixi manifest to specify a mapping. See [lerobot_robot_aic](../aic_utils/lerobot_robot_aic/pixi.toml) for an example.

##### Mapping between pypi and conda dependencies

pixi automatically converts dependencies it finds in a python package to conda dependencies. There is a builtin mapping which works in most cases, but if your package requires a uncommon package, then pixi will fail to find a mapping.

In those cases, you can provide a custom set of mapping. See the [conda-pypi-map](https://pixi.prefix.dev/latest/reference/pixi_manifest/#conda-pypi-map-optional) config. You may also check this project's [pixi.toml](../pixi.toml) and [mappings](../robostack_mapping.json) for examples.

## Native

### Requirements

- [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
- [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

### pypi

The biggest challenge with running the participant workspace natively is that AI tools are often released in pypi and ROS/python/ubuntu does not support mixing pypi and system packages.

There are several ways around it:

- Use `--break-system-packages` (not recommended).
- Create a virtual environment (`python3 -m venv`). The `--system-site-packages` flag must be used.
- Vendor the packages that you need.
- Release the packages you need as ROS packages.
- etc

All of these solutions are "try at your own risk". They either require a lot of work or does not guarantee a stable environment. ROS packages requires deprecated features in setuptools which may not be compatible with features required by newer pypi packages.

## devcontainer

TODO: This would be mostly just a container with pixi and other dev tools.
