## Creating a new policy node

A policy node is essentially a ROS2 node that subscribes to observations and publishes actions to be executed.

For this tutorial, we will be using [aic_model](../aic_model/README.md) to implement a policy node.

> [!Important]
> Take note of the prompt in the bash examples. If it starts with `(aic) $`, then it should be run from inside the pixi environment.
>
> Example:
> ```bash
> $ pixi shell # This is outside pixi environment.
> (aic) $ ros2 pkg list # This is inside the environment.

**Create a new ROS2 package:**

```bash
(aic) $ ros2 pkg create my_policy_node --build-type ament_python
```

**Add AIC dependencies**

Add the following to `package.xml`:
```xml
  <depend>aic_control_interfaces</depend>
  <depend>aic_model</depend>
  <depend>aic_model_interfaces</depend>
  <depend>aic_task_interfaces</depend>
  <depend>geometry_msgs</depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <depend>trajectory_msgs</depend>
```  

**Create a pixi package**

Create a `pixi.toml` file with the following contents:

```toml
[package.build.backend]
name = "pixi-build-ros"
version = "==0.3.3.20260113.c8b6a54"
channels = [
  "https://prefix.dev/pixi-build-backends",
  "robostack-kilted",
  "conda-forge",
]

[package.host-dependencies]
ros-kilted-aic-control-interfaces = { path = "../aic_interfaces/aic_control_interfaces" }
ros-kilted-aic-model = { path = "../aic_model" }
ros-kilted-aic-model-interfaces = { path = "../aic_interfaces/aic_model_interfaces" }
ros-kilted-aic-task-interfaces = { path = "../aic_interfaces/aic_task_interfaces" }

[package.build-dependencies]
ros-kilted-aic-control-interfaces = { path = "../aic_interfaces/aic_control_interfaces" }
ros-kilted-aic-model = { path = "../aic_model" }
ros-kilted-aic-model-interfaces = { path = "../aic_interfaces/aic_model_interfaces" }
ros-kilted-aic-task-interfaces = { path = "../aic_interfaces/aic_task_interfaces" }
```

> [!Tip]
> Normally, pixi will automatically discover the dependencies from `package.xml`. But because we are building the aic interfaces from source, we need to tell pixi where to find them.

**Add the pixi package to the workspace**

In the root `pixi.toml`, add the new package to `[dependencies]`.

It should look like this

```toml
[dependencies]
# ...
ros-kilted-my-policy-node = { path = "my_policy_node" }
```

**Implement `PolicyRos`**

For brevity, we will reuse the code from `aic_example_policies`.

```bash
(aic) $ cp aic_example_policies/aic_example_policies/ros/WaveArm.py my_policy_node/my_policy_node/WaveArm.py
```

**Run the node**

```bash
$ pixi reinstall ros-kilted-my-policy-node
$ pixi run ros2 run aic_model aic_model --ros-args -p policy:=my_policy_node.WaveArm
```

> [!Note]
> The command above runs the `aic_model` node, which then dynamically loads and runs your specific policy implementation (`my_policy_node.WaveArm`).

## Dependency Management

This is a quick guide on how to manage dependencies under a pixi workspace. Unlike a typical ROS workspace, you will not be using any system dependencies; all dependencies must come from conda or pypi.

### Adding dependencies

#### ROS dependencies

The pixi workspace is set up with the robostack-kilted channel. You can install most ROS packages with pixi.

```bash
$ pixi add ros-kilted-ros-core
```

#### pypi dependencies

Unlike a native ROS workspace, a pixi workspace can mix ROS and pypi dependencies.

```bash
$ pixi add --pypi pandas
```

#### Local dependencies

If your package requires a local dependency in the same workspace, those dependencies must be declared on both the package's `pixi.toml` and the root `pixi.toml`.

For example, if `my_policy_node` requires `my_local_dep`, 

`my_policy_node/pixi.toml`:
```toml
[package.host-dependencies]
# ...
ros-kilted-my-local-dep = { path = "../my_local_dep" }

[package.build-dependencies]
# ...
ros-kilted-my-local-dep = { path = "../my_local_dep" }
```

`pixi.toml`:
```toml
[dependencies]
# ...
ros-kilted-my-policy-node = { path = "my_policy_node" }
ros-kilted-my-local-dep = { path = "my_local_dep" }
```

> [!Tip]
> pixi automatically prefixes a ROS package with `ros-<distro>-` and converts underscores to hyphens.

## Build-Run-Debug cycle (python)

pixi does not install your package in "editable" mode. Any changes you made will not be reflected until you reinstall the package.

```bash
$ pixi reinstall <package>
```

> [!Tip]
> You may enter the pixi environment with `pixi shell` and force an "editable" install with `pip install -e`. But note that this circumvents pixi and may cause unintended side effects.

## Preparing for submission

After you are satisfied with your policy, you will need to prepare a Docker image for submission.

```bash
$ mkdir -p docker/my_policy_node
$ cp docker/aic_model/Dockerfile docker/my_policy_node/
```

Open the Dockerfile and add your policy node to the build instructions.

Update the Dockerfile with the following:

```dockerfile
# Add other local dependencies
COPY my_policy_node /ws_aic/src/aic/my_policy_node # <-- Add this line
```

Open `docker/docker-compose.yaml`, replace `docker/aic_model/Dockerfile` with the path to your Dockerfile. Also be sure to update the command to use your policy node.

`docker/docker-compose.yaml`:

```yaml
  model:
    image: localhost/aic/aic_model
    build:
      dockerfile: docker/my_policy_node/Dockerfile # <-- replace this line
      context: ..
    command: --ros-args -p policy:=my_policy_node.WaveArm # <-- and this line
```

Build the image

```bash
$ docker compose build model
```

Test that everything works

```bash
$ docker compose up
```

This will run your model and the evaluator in an environment that replicates the submission portal.

- External network access is restricted.
- Zenoh ACLs will be employed to restrict what the policy node can interact with.
- Shared memory is disabled.

Make sure that your policy works, then export a tarball of your image.

```bash
$ docker save localhost/aic/my_policy_node | gzip > my_policy_node.tar.gz
```

## Conclusion

Congratulations! You have successfully created, tested, and packaged a policy node.

In this tutorial, you have learned how to:
- Create and set up a new ROS2 package for a policy node.
- Manage Python and ROS dependencies within a `pixi` workspace.
- Understand the build, run, and debug cycle for developing your policy.
- Prepare a Docker image for your policy for submission.

You are now equipped with the fundamental skills to develop, test, and submit your own policies for the AI for Industry Challenge. Feel free to explore the provided example policies and other documentation for more advanced concepts and inspiration.
