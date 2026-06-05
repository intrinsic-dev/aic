#!/usr/bin/env bash

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC skills (insert_cable_skill, tare_force_torque_sensor_skill) for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --ros_distro DISTRO  Name of the ROS distro (default: kilted)"
  echo ""
}

ROS_DISTRO="kilted"

while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      show_help
      exit 0
      ;;
    --ros_distro)
      ROS_DISTRO="$2"
      shift
      shift
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit 1
fi

set -o errexit
set -o verbose

# Build and bundle insert_cable_skill
src/aic/flowstate/scripts/build_container.sh \
  --ros_distro "$ROS_DISTRO" \
  --skill_name insert_cable_skill \
  --skill_package aic_flowstate_skills \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/insert_cable_skill/src/insert_cable_skill.manifest.textproto \
  --dockerfile src/aic/flowstate/resources/Dockerfile.skill

./inbuild skill bundle \
  --file_descriptor_set images/insert_cable_skill/insert_cable_skill_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_skills/insert_cable_skill/src/insert_cable_skill.manifest.textproto \
  --oci_image images/insert_cable_skill/insert_cable_skill.tar \
  --output images/insert_cable_skill/insert_cable_skill.bundle.tar

# Build and bundle tare_force_torque_sensor_skill
src/aic/flowstate/scripts/build_container.sh \
  --ros_distro "$ROS_DISTRO" \
  --skill_name tare_force_torque_sensor_skill \
  --skill_package aic_flowstate_skills \
  --manifest_path src/aic/flowstate/aic_flowstate_skills/tare_force_torque_sensor_skill/src/tare_force_torque_sensor_skill.manifest.textproto \
  --dockerfile src/aic/flowstate/resources/Dockerfile.skill

./inbuild skill bundle \
  --file_descriptor_set images/tare_force_torque_sensor_skill/tare_force_torque_sensor_skill_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_skills/tare_force_torque_sensor_skill/src/tare_force_torque_sensor_skill.manifest.textproto \
  --oci_image images/tare_force_torque_sensor_skill/tare_force_torque_sensor_skill.tar \
  --output images/tare_force_torque_sensor_skill/tare_force_torque_sensor_skill.bundle.tar
