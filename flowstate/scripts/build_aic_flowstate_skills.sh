#!/usr/bin/env bash

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC skills (insert_cable_skill, tare_force_torque_sensor_skill, switch_to_aic_controller_skill) for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --ros_distro DISTRO  Name of the ROS distro (default: kilted)"
  echo "  --skill_name NAME    Build only the specified skill (default: build all)"
  echo ""
}

ROS_DISTRO="kilted"
SKILL_NAME=""

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
    --skill_name)
      SKILL_NAME="$2"
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

build_and_bundle_skill() {
  local name="$1"
  src/aic/flowstate/scripts/build_container.sh \
    --ros_distro "$ROS_DISTRO" \
    --skill_name "$name" \
    --skill_package aic_flowstate_skills \
    --manifest_path src/aic/flowstate/aic_flowstate_skills/"$name"/src/"$name".manifest.textproto \
    --dockerfile src/aic/flowstate/resources/Dockerfile.skill

  ./inbuild skill bundle \
    --file_descriptor_set images/"$name"/"$name"_protos.desc \
    --manifest src/aic/flowstate/aic_flowstate_skills/"$name"/src/"$name".manifest.textproto \
    --oci_image images/"$name"/"$name".tar \
    --output images/"$name"/"$name".bundle.tar
}

if [ -n "$SKILL_NAME" ]; then
  case "$SKILL_NAME" in
    insert_cable_skill|tare_force_torque_sensor_skill|switch_to_aic_controller_skill)
      build_and_bundle_skill "$SKILL_NAME"
      ;;
    *)
      echo "Unknown skill name: $SKILL_NAME"
      echo "Available skills: insert_cable_skill, tare_force_torque_sensor_skill, switch_to_aic_controller_skill"
      exit 1
      ;;
  esac
else
  build_and_bundle_skill "insert_cable_skill"
  build_and_bundle_skill "tare_force_torque_sensor_skill"
  build_and_bundle_skill "switch_to_aic_controller_skill"
fi
