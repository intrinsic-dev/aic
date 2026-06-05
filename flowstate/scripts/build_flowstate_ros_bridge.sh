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

SDK_VERSION="v1.31.20260427.1"
if [ ! -f ./inbuild ]; then
  echo "INFO: Downloading inbuild tool version ${SDK_VERSION}..."
  wget "https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inbuild-linux-amd64" -O inbuild \
    && chmod +x inbuild
fi

src/aic/flowstate/scripts/build_container.sh \
  --ros_distro "$ROS_DISTRO" \
  --service_name aic_flowstate_ros_bridge \
  --service_package aic_flowstate_ros_bridge \
  --dependencies nlohmann-json3-dev \
  --dockerfile src/aic/flowstate/resources/Dockerfile.service

echo "INFO: Loading newly built service image into local daemon..."
docker load -i "images/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge.tar"

echo "INFO: Extracting config and descriptor set from container..."
docker create --name temp_container_service aic_flowstate_ros_bridge:aic_flowstate_ros_bridge
docker cp "temp_container_service:/opt/ros/overlay/install/share/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge_protos.desc" \
  "images/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge_protos.desc"
docker rm -f temp_container_service

echo "INFO: Building the service bundle..."
./inbuild service bundle \
  --file_descriptor_set images/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge_protos.desc \
  --manifest src/aic/flowstate/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge.manifest.textproto \
  --oci_image images/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge.tar \
  --output images/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge.bundle.tar \
  --default_config src/aic/flowstate/aic_flowstate_ros_bridge/aic_flowstate_ros_bridge_default_config.pbtxt
