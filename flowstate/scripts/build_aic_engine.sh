#!/usr/bin/env bash
set -e

IMAGES_DIR=./images
BUILDER_NAME=container-builder

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC engine container image for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --images_dir DIR     Directory to save output images (default: ./images)"
  echo "  --builder_name NAME  Name of the container builder (default: container-builder)"
  echo ""
}

while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      show_help
      exit 0
      ;;
    --images_dir)
      IMAGES_DIR="$2"
      shift
      shift
      ;;
    --builder_name)
      BUILDER_NAME="$2"
      shift
      shift
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Compute absolute path to top-level aic directory (two levels up from flowstate/scripts)
AIC_TOP_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
SERVICE_DIR="$AIC_TOP_DIR/flowstate/services/aic_engine"
DOCKERFILE_SERVICE="$SERVICE_DIR/Dockerfile.service"

# 1. Build and export the service image to a compressed .tar bundle
# This builds the image and exports it directly to a tar archive with ZSTD compression,
# which is much smaller and can be subsequently bundled by the inbuild tool.
echo "INFO: Building and exporting image to compressed tar file..."
if [ -d "$IMAGES_DIR/aic_engine" ]; then
  echo "INFO: Deleting existing $IMAGES_DIR/aic_engine directory..."
  rm -rf "$IMAGES_DIR/aic_engine"
fi
mkdir -p "$IMAGES_DIR/aic_engine"

docker buildx build -t flowstate:aic_engine \
  --no-cache \
  --builder="$BUILDER_NAME" \
  --output="type=docker,dest=$IMAGES_DIR/aic_engine/aic_engine.tar,compression=zstd,push=false,name=flowstate:aic_engine" \
  --file "$DOCKERFILE_SERVICE" \
  "$AIC_TOP_DIR/../.."

# TODO: enable compression with dedicated builder
#  --builder="$BUILDER_NAME" \
#  --output="type=docker,dest=$IMAGES_DIR/aic_engine/aic_engine.tar,compression=zstd,push=false,name=flowstate:aic_engine" \
#
chmod 644 "$IMAGES_DIR/aic_engine/aic_engine.tar"

# 3. Bundle the service using inbuild
# Packages the service using Intrinsic's 'inbuild' tool. It retrieves the
# corresponding SDK version, downloads the tool if necessary, and generates
# a .bundle.tar file using the service manifest and exported image.
SDK_VERSION=v1.28.20260223

# Download the 'inbuild' tool if it doesn't exist
if [ ! -f ./inbuild ]; then
  echo "INFO: Downloading inbuild tool version ${SDK_VERSION}..."
  wget "https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inbuild-linux-amd64" -O inbuild \
  && chmod +x inbuild
fi

# Ensure the 'inbuild' tool is executable
if [ -f ./inbuild ] && [ ! -x ./inbuild ]; then
  echo "INFO: Making inbuild tool executable..."
  chmod +x ./inbuild
fi

echo "INFO: Loading newly built service image into local daemon..."
docker load -i "$IMAGES_DIR/aic_engine/aic_engine.tar"

echo "INFO: Extracting descriptor set from container..."
docker create --name temp_container_service flowstate:aic_engine
docker cp "temp_container_service:/workspace/install/share/aic_engine/aic_engine_protos.desc" \
  "$IMAGES_DIR/aic_engine/aic_engine_protos.desc"
docker rm -f temp_container_service

echo "INFO: Bundling service using inbuild..."
./inbuild service bundle \
  --file_descriptor_set "$IMAGES_DIR/aic_engine/aic_engine_protos.desc" \
  --manifest "$AIC_TOP_DIR/aic_engine/aic_engine.manifest.textproto" \
  --oci_image "$IMAGES_DIR/aic_engine/aic_engine.tar" \
  --output "$IMAGES_DIR/aic_engine/aic_engine.bundle.tar" \
  --default_config "$AIC_TOP_DIR/aic_engine/aic_engine_default_config.pbtxt"
