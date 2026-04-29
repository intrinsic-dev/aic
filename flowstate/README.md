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
  wget "https://github.com/intrinsic-ai/sdk/releases/download/v1.28.20260223/inctl-linux-amd64" -O inctl \
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

## 🛠️ Building the Skill

We can use the `build_container.sh` and `build_bundle.sh` script from the intrinsic-ai/sdk-ros repository to build and package the skill bundle. The instructions below build and bundle the `insert_cable_skill`.

---

```bash
cd ~/ws_aic_phase1

# This command builds the insert_cable_skill, the Intrinsic SDK, and the necessary ROS dependencies into a tar image.
./src/sdk-ros/scripts/build_container.sh \
  --ros_distro "kilted" \
  --skill_package aic_flowstate_skills \
  --skill_name insert_cable_skill

# This command bundles the skill into a deployable tarball
./src/sdk-ros/scripts/build_bundle.sh \
  --skill_package aic_flowstate_skills \
  --skill_name insert_cable_skill \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/insert_cable_skill/src/insert_cable_skill.manifest.textproto
```

---

## 📥 Installing skills to Flowstate

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