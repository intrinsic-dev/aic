#!/bin/bash
if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit
fi

ROS_DISTRO="kilted"

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC model container image for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --ros_distro ROS_DISTRO  Name of the ROS distro (default: kilted)"
  echo ""
}

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

set -o errexit
set -o verbose
src/sdk-ros/scripts/build_container.sh \
  --ros_distro "$ROS_DISTRO" \
  --service_name aic_flowstate_ros_bridge \
  --service_package aic_flowstate_ros_bridge \
  --dependencies nlohmann-json3-dev
src/sdk-ros/scripts/build_bundle.sh \
  --service_name aic_flowstate_ros_bridge \
  --service_package aic_flowstate_ros_bridge \
  --manifest_path src/aic/flowstate/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge.manifest.textproto \
  --default_config src/aic/flowstate/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge_default_config.pbtxt
