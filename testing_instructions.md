# Testing Instructions - Taskboard Spawning Skill

These instructions describe how to build, bundle, and install the query skill for the taskboard spawning feature.

## 1. Prerequisites

Ensure you are at the root of the `aic_ws` workspace.

## 2. Build Container Image

Run the following command to build the Docker container and extract the file descriptor set. We must use `--ros_distro kilted` and include `python3-toml` as a dependency.

```bash
./src/sdk-ros/scripts/build_container.sh \
  --skill_name get_taskboard_state_skill \
  --skill_package aic_flowstate_skills \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/get_taskboard_state_skill/src/get_taskboard_state_skill.manifest.textproto \
  --ros_distro kilted \
  --dependencies "python3-toml"
```

*Output*: 
- A Docker image tarball at `images/get_taskboard_state_skill/get_taskboard_state_skill.tar`.
- A file descriptor set at `images/get_taskboard_state_skill/get_taskboard_state_skill_protos.desc`.

## 3. Generate Skill Bundle

Use `inbuild` to create the `.bundle.tar` file:

```bash
inbuild skill bundle \
  --file_descriptor_set images/get_taskboard_state_skill/get_taskboard_state_skill_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_skills/get_taskboard_state_skill/src/get_taskboard_state_skill.manifest.textproto \
  --oci_image images/get_taskboard_state_skill/get_taskboard_state_skill.tar \
  --output images/get_taskboard_state_skill/get_taskboard_state_skill.bundle.tar
```

## 4. Install on Cluster

Install the generated skill bundle on the cluster using `inctl`:

```bash
inctl asset install images/get_taskboard_state_skill/get_taskboard_state_skill.bundle.tar \
  --cluster=vmp-ca4d-6iue3lp2 \
  --org=aic-dev@xfa-prod-aic-us
```

---
*Note: Instructions for the service (`taskboard_state_server`) will be added once we confirm the correct bundling procedure for services, as `inbuild` seems to be tailored for skills.*
