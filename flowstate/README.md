# Flowstate Service: `aic_model`

Package and inject the `aic_model` container  as a service in Flowstate.

---

## 📂 Directory Structure

*   `services/aic_model/`
    *   `Dockerfile.service`: Update Zenoh config to talk to Zenoh router in Flowstate.
    *   `aic_model.manifest.textproto`: Manifest for `aic_model` service for Flowstate.
*   `scripts/`
    *   `build_aic_model.sh`: Automates building the base image, service image, and bundling using `inbuild`.

---

## 🔧 Prerequisites

Before components can be built or uploaded:
1.  **Solution Context ID**: Retrieve this from your Flowstate solution URL.
    *   *Example*: `https://flowstate.intrinsic.ai/.../vmp-xxxx-xxxxxxx` -> Context ID `vmp-xxxx-xxxxxxx`
2.  **Required Tools**:
    *   `docker buildx` support.

---

## 🛠️ Building the Service

Use the `build_aic_model.sh` script to build and pack the service bundle.

```bash
cd ~/ws_aic
./src/aic/flowstate/scripts/build_aic_model.sh --dockerfile <PATH_TO_AIC_MODEL_DOCKERFILE>
```

> [!IMPORTANT]
> Replace `<PATH_TO_AIC_MODEL_DOCKERFILE>` with your actual `aic_model` Dockerfile path (e.g., `./src/aic/docker/aic_model/Dockerfile`).

### Build Stages
1.  **Base Image**: Builds `aic_model:latest` using the provided Dockerfile.
2.  **Service Image**: Extends with `Dockerfile.service` to add Zenoh configuration and entrypoint setting.
3.  **Bundle**: Runs `inbuild` to package layout requirements with manifest files into `aic_model.bundle.tar`.

The final output is saved to: `./images/aic_model/aic_model.bundle.tar`.

---

## 📥 Installing to Flowstate

Once built, upload and install the service into your solution context.

```bash
# 1. Export path to side-loaded service bundle
export SERVICE_BUNDLE=~/ws_aic/images/aic_model/aic_model.bundle.tar

# 2. Add Organization
export INTRINSIC_ORGANIZATION="<ORG_NAME>"

# 3. Add Cluster/Context Endpoint
export INTRINSIC_CONTEXT="vmp-xxxx-xxxxxxx"

./inctl asset install \
  --org $INTRINSIC_ORGANIZATION \
  --cluster $INTRINSIC_CONTEXT \
  $SERVICE_BUNDLE
```

---

## ⚙️ Configuration & Execution Notes

*   **Startup Default Argument**: Inside `Dockerfile.service`, the entrypoint loads:
    `pixi run --as-is ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.CheatCode`
