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