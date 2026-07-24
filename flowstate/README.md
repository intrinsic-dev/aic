# Flowstate Service: `aic_model`

Package and inject the `aic_model` container  as a service in Flowstate.
---

## 📂 Directory Structure

*   `services/aic_model/`
    *   `Dockerfile.service`: Update AIC_ROUTER_ADDR which allows talking to Zenoh router in Flowstate.
    *   `aic_model.manifest.textproto`: Manifest for `aic_model` service for Flowstate.
*   `scripts/`
    *   `build_aic_model.sh`: Automates building the base image, service image, and bundling using `inbuild`.

---

## 🏗️ Workspace Setup (Phase 1)

For Phase 1, you should set up a new workspace to avoid dependency conflicts with the qualification workspace.

```bash
# Create the workspace directory
mkdir -p ~/ws_aic_phase1/src
cd ~/ws_aic_phase1/src

# Clone the aic repository
git clone https://github.com/intrinsic-dev/aic -b phase_1

# Import Flowstate-specific repositories
vcs import . < aic/flowstate/flowstate.repos
```

---

## 🚀 Automated Building and Bundling with Pixi

You can build and bundle both the services and skills automatically inside the Pixi environment. Run these commands from the **workspace root** (e.g. `~/ws_aic_phase1`):

```bash
# 1. Use the help to get the list of all available skills
pixi run --manifest-path src/aic/pixi.toml bash src/aic/flowstate/scripts/build_aic_flowstate_skills.sh --help

# 2. Build and bundle skills in aic_flowstate_skills (builds all by default, or specify a single skill via --skill_name)
pixi run --manifest-path src/aic/pixi.toml bash src/aic/flowstate/scripts/build_aic_flowstate_skills.sh [--skill_name <insert_cable_skill|tare_force_torque_sensor_skill|switch_to_aic_controller_skill>]

# 3. Build and bundle the flowstate ROS bridge service
pixi run --manifest-path src/aic/pixi.toml bash src/aic/flowstate/scripts/build_flowstate_ros_bridge.sh

# 4. Build and bundle the AIC adapter service
pixi run --manifest-path src/aic/pixi.toml bash src/aic/flowstate/scripts/build_aic_adapter.sh

# 5. Build and bundle the AIC logger service
pixi run --manifest-path src/aic/pixi.toml bash src/aic/flowstate/scripts/build_aic_logger.sh
```

---

## 🔧 Prerequisites

Before components can be built or uploaded:
1.  **Solution Cluster ID**: Retrieve this from your Flowstate solution URL.
    *   *Example*: `https://flowstate.intrinsic.ai/.../vmp-xxxx-xxxxxxx` -> Cluster ID `vmp-xxxx-xxxxxxx`
2.  **Required Tools**:
    *   `docker buildx` support.
    *   `inctl` tool

- Download the 'inctl' tool if it doesn't exist in `ws_aic_phase1`:
  ```bash
  cd ~/ws_aic_phase1
  wget "https://github.com/intrinsic-ai/sdk/releases/download/v1.31.20260427.1/inctl-linux-amd64" -O inctl \
  && chmod +x inctl
  ```

---

## 🛠️ Building the Service

Use the `build_aic_model.sh` script to build and pack the service bundle.

```bash
cd ~/ws_aic_phase1
./src/aic/flowstate/scripts/build_aic_model.sh --container_image <NAME_OF_AIC_MODEL_IMAGE>
```

> [!IMPORTANT]
> Replace `<NAME_OF_AIC_MODEL_IMAGE>` with your actual your image name (e.g., `my-solution:v1`).

### Build Stages
1.  **Service Image**: Extends base container image with `Dockerfile.service` to AIC_ROUTER_ADDR.
2.  **Bundle**: Runs `inbuild` to package layout requirements with manifest files into `aic_model.bundle.tar`.

The final output is saved to: `./images/aic_model/aic_model.bundle.tar`.

---

## 📥 Installing to Flowstate

Once built, upload and install the service into your solution context.

```bash
# 1. Export path to side-loaded service bundle
export SERVICE_BUNDLE=~/ws_aic_phase1/images/aic_model/aic_model.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster Endpoint
export INTRINSIC_CLUSTER="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CLUSTER \
  $SERVICE_BUNDLE
```

> [!NOTE]
> It is possible that first time you run `inctl asset install` it fails with error. In that case, please try again.

---

## 🛠️ Building the insert_cable_skill

We can use the `build_container.sh` script to build and package the skill bundle using the following instructions.

---

```bash
cd ~/ws_aic_phase1

# This command builds the insert_cable_skill, the Intrinsic SDK, and the necessary ROS dependencies into a tar image.
./src/aic/flowstate/scripts/build_container.sh \
  --skill_name insert_cable_skill \
  --skill_package aic_flowstate_skills \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/insert_cable_skill/src/insert_cable_skill.manifest.textproto \
  --dockerfile ./src/aic/flowstate/resources/Dockerfile.skill

# This command bundles the skill into a deployable tarball
./inbuild skill bundle \
  --file_descriptor_set images/insert_cable_skill/insert_cable_skill_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_skills/insert_cable_skill/src/insert_cable_skill.manifest.textproto \
  --oci_image images/insert_cable_skill/insert_cable_skill.tar \
  --output images/insert_cable_skill/insert_cable_skill.bundle.tar
```

---

## 📥 Installing insert_cable_skill to Flowstate

After building, upload and install the skill into your solution context.

---

```bash

# 1. Export path to side-loaded service bundle
export SKILL_BUNDLE=~/ws_aic_phase1/images/insert_cable_skill/insert_cable_skill.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster Endpoint
export INTRINSIC_CLUSTER="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CLUSTER \
  $SKILL_BUNDLE

```

---

## 🛠️ Building the tare_force_torque_sensor_skill

We can use the `build_container.sh` script to build and package the skill bundle using the following instructions.

---

```bash
cd ~/ws_aic_phase1

# This command builds the insert_cable_skill, the Intrinsic SDK, and the necessary ROS dependencies into a tar image.
./src/aic/flowstate/scripts/build_container.sh \
  --skill_name tare_force_torque_sensor_skill \
  --skill_package aic_flowstate_skills \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/tare_force_torque_sensor_skill/src/tare_force_torque_sensor_skill.manifest.textproto \
  --dockerfile ./src/aic/flowstate/resources/Dockerfile.skill

# This command bundles the skill into a deployable tarball
./inbuild skill bundle \
  --file_descriptor_set images/tare_force_torque_sensor_skill/tare_force_torque_sensor_skill_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_skills/tare_force_torque_sensor_skill/src/tare_force_torque_sensor_skill.manifest.textproto \
  --oci_image images/tare_force_torque_sensor_skill/tare_force_torque_sensor_skill.tar \
  --output images/tare_force_torque_sensor_skill/tare_force_torque_sensor_skill.bundle.tar
```

---

## 📥 Installing tare_force_torque_sensor_skill to Flowstate

After building, upload and install the skill into your solution context.

---

```bash

# 1. Export path to side-loaded service bundle
export SKILL_BUNDLE=~/ws_aic_phase1/images/tare_force_torque_sensor_skill/tare_force_torque_sensor_skill.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster Endpoint
export INTRINSIC_CLUSTER="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CLUSTER \
  $SKILL_BUNDLE

```

---

## 🛠️ Building the switch_to_aic_controller_skill

We can use the `build_container.sh` script to build and package the skill bundle using the following instructions.

---

```bash
cd ~/ws_aic_phase1

# This command builds the switch_to_aic_controller_skill, the Intrinsic SDK, and the necessary ROS dependencies into a tar image.
./src/aic/flowstate/scripts/build_container.sh \
  --skill_name switch_to_aic_controller_skill \
  --skill_package aic_flowstate_skills \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/switch_to_aic_controller_skill/src/switch_to_aic_controller_skill.manifest.textproto \
  --dockerfile ./src/aic/flowstate/resources/Dockerfile.skill

# This command bundles the skill into a deployable tarball
./inbuild skill bundle \
  --file_descriptor_set images/switch_to_aic_controller_skill/switch_to_aic_controller_skill_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_skills/switch_to_aic_controller_skill/src/switch_to_aic_controller_skill.manifest.textproto \
  --oci_image images/switch_to_aic_controller_skill/switch_to_aic_controller_skill.tar \
  --output images/switch_to_aic_controller_skill/switch_to_aic_controller_skill.bundle.tar
```

---

## 📥 Installing switch_to_aic_controller_skill to Flowstate

After building, upload and install the skill into your solution context.

---

```bash

# 1. Export path to side-loaded service bundle
export SKILL_BUNDLE=~/ws_aic_phase1/images/switch_to_aic_controller_skill/switch_to_aic_controller_skill.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster Endpoint
export INTRINSIC_CLUSTER="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CLUSTER \
  $SKILL_BUNDLE

```

---

## 🛠️ Building the start_recording_skill

```bash
./src/aic/flowstate/scripts/build_container.sh \
  --skill_name start_recording_skill \
  --skill_package aic_flowstate_skills \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/start_recording_skill/src/start_recording_skill.manifest.textproto \
  --dockerfile ./src/aic/flowstate/resources/Dockerfile.skill

./inbuild skill bundle \
  --file_descriptor_set images/start_recording_skill/start_recording_skill_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_skills/start_recording_skill/src/start_recording_skill.manifest.textproto \
  --oci_image images/start_recording_skill/start_recording_skill.tar \
  --output images/start_recording_skill/start_recording_skill.bundle.tar
```

---

## 📥 Installing start_recording_skill to Flowstate

After building, upload and install the skill into your solution context.

---

```bash

# 1. Export path to side-loaded skill bundle
export SKILL_BUNDLE=~/ws_aic_phase1/images/start_recording_skill/start_recording_skill.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster Endpoint
export INTRINSIC_CLUSTER="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CLUSTER \
  $SKILL_BUNDLE

```

---

## 🛠️ Building the stop_recording_skill

```bash
./src/aic/flowstate/scripts/build_container.sh \
  --skill_name stop_recording_skill \
  --skill_package aic_flowstate_skills \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/stop_recording_skill/src/stop_recording_skill.manifest.textproto \
  --dockerfile ./src/aic/flowstate/resources/Dockerfile.skill

./inbuild skill bundle \
  --file_descriptor_set images/stop_recording_skill/stop_recording_skill_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_skills/stop_recording_skill/src/stop_recording_skill.manifest.textproto \
  --oci_image images/stop_recording_skill/stop_recording_skill.tar \
  --output images/stop_recording_skill/stop_recording_skill.bundle.tar
```

---

## 📥 Installing stop_recording_skill to Flowstate

After building, upload and install the skill into your solution context.

---

```bash

# 1. Export path to side-loaded skill bundle
export SKILL_BUNDLE=~/ws_aic_phase1/images/stop_recording_skill/stop_recording_skill.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster Endpoint
export INTRINSIC_CLUSTER="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CLUSTER \
  $SKILL_BUNDLE

```
