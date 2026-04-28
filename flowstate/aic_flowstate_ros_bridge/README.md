# robot_control_bridge

A Flowstate-ROS bridge plugin that bridges the real-time controller service to ROS.

## Quickstart (Sideload as Flowstate Service)

### Build the robot_control_bridge bundle (Docker)

Set up the Docker engine:

```bash
cd ~/ws_aic
./src/sdk-ros/scripts/setup_docker.sh
```

Then, create the service bundle using the `build_service_bundle.sh` script.

```bash
cd ~/ws_aic
./src/aic/flowstate/scripts/build_flowstate_ros_bridge.sh --ros_distro kilted
```

### Sideload the flowstate_ros_bridge service into Flowstate

```bash
# 1. Export path to side-loaded service bundle
export SERVICE_BUNDLE=~/ws_aic/images/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster Endpoint
export INTRINSIC_CLUSTER="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CLUSTER \
  $SERVICE_BUNDLE
```

## ROS 2 Interfaces

### Services

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `/aic_controller/change_target_mode` | `aic_control_interfaces/srv/ChangeTargetMode` | Select the target mode (Cartesian or joint) to define the expected input. The controller will subscribe to either `/aic_controller/pose_commands` or `/aic_controller/joint_commands` accordingly. |
| `/aic_controller/restart_bridge` | `std_srvs/srv/Trigger` | Triggers the restart of the controller bridge. To be called after robot faults occur and are cleared. |

### Published Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/aic_controller/controller_state` | `aic_control_interfaces/msg/ControllerState` | Data on current TCP pose and velocity, reference TCP pose, TCP tracking error and reference joint efforts. |

### Command Topics
The bridge listens to the following topics as motion commands for the controller.

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/aic_controller/joint_commands` | `aic_control_interfaces/msg/JointMotionUpdate` | Target configurations for joint-space control. |
| `/aic_controller/pose_commands` | `aic_control_interfaces/msg/MotionUpdate` | Target poses for Cartesian-space control. |
