#!/usr/bin/env bash
if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit 1
fi

ROS_DISTRO="kilted"
IMAGES_DIR="./images"
BUILDER_NAME="container-builder"

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC logger container image for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help               Show this help message and exit"
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

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
AIC_TOP_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WORKSPACE_ROOT=$(cd "$AIC_TOP_DIR/../.." && pwd)
DOCKERFILE_SERVICE="$AIC_TOP_DIR/flowstate/resources/Dockerfile.service"
SERVICE_DIR="$AIC_TOP_DIR/flowstate/aic_logger"

# 1. Prepare output directory
if [ -d "$IMAGES_DIR/aic_logger" ]; then
  echo "INFO: Deleting existing $IMAGES_DIR/aic_logger directory..."
  rm -rf "$IMAGES_DIR/aic_logger"
fi
mkdir -p "$IMAGES_DIR/aic_logger"

# 2. Build and export service image to tar file
echo "INFO: Building and exporting image to compressed tar file..."
docker buildx build -t aic_logger:aic_logger \
  --builder="$BUILDER_NAME" \
  --output="type=docker,dest=$IMAGES_DIR/aic_logger/aic_logger.tar,push=false,name=aic_logger:aic_logger" \
  --file "$DOCKERFILE_SERVICE" \
  --build-arg="SERVICE_PACKAGE=aic_logger" \
  --build-arg="SERVICE_NAME=aic_logger" \
  --build-arg="SERVICE_EXECUTABLE_NAME=aic_logger_main" \
  --build-arg="DEPENDENCIES=ros-kilted-rosbag2,ros-kilted-rosbag2-storage-mcap" \
  --build-arg="ROS_DISTRO=$ROS_DISTRO" \
  "$WORKSPACE_ROOT"

chmod 644 "$IMAGES_DIR/aic_logger/aic_logger.tar"

# 3. Load newly built image into local daemon and extract descriptor set
echo "INFO: Loading newly built service image into local daemon..."
docker load -i "$IMAGES_DIR/aic_logger/aic_logger.tar"

echo "INFO: Extracting config and descriptor set from container..."
TEMP_CONTAINER_NAME="temp_container_aic_logger_$$"
docker rm -f "$TEMP_CONTAINER_NAME" &>/dev/null || true
docker create --name "$TEMP_CONTAINER_NAME" aic_logger:aic_logger
docker cp "$TEMP_CONTAINER_NAME:/opt/ros/overlay/install/share/aic_logger/aic_logger_protos.desc" \
  "$IMAGES_DIR/aic_logger/aic_logger_protos.desc"
docker rm -f "$TEMP_CONTAINER_NAME" &>/dev/null || true

# 4. Download inbuild tool if needed
SDK_VERSION="v1.31.20260427.1"
if [ ! -f ./inbuild ]; then
  echo "INFO: Downloading inbuild tool version ${SDK_VERSION}..."
  wget "https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inbuild-linux-amd64" -O inbuild \
    && chmod +x inbuild
fi

if [ -f ./inbuild ] && [ ! -x ./inbuild ]; then
  echo "INFO: Making inbuild tool executable..."
  chmod +x ./inbuild
fi

# 5. Build the service bundle using inbuild
echo "INFO: Building the service bundle using inbuild..."
./inbuild service bundle \
  --file_descriptor_set "$IMAGES_DIR/aic_logger/aic_logger_protos.desc" \
  --manifest "$SERVICE_DIR/aic_logger.manifest.textproto" \
  --oci_image "$IMAGES_DIR/aic_logger/aic_logger.tar" \
  --output "$IMAGES_DIR/aic_logger/aic_logger.bundle.tar" \
  --default_config "$SERVICE_DIR/aic_logger_default_config.pbtxt"
