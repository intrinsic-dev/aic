#!/usr/bin/env bash

IMAGES_DIR=./images
BUILDER_NAME=container-builder

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC model container image for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --images_dir DIR     Directory to save output images (default: ./images)"
  echo "  --builder_name NAME  Name of the container builder (default: container-builder)"
  echo "  --container_image IMAGE  Name of the base container image (default: my-solution:v1)"
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
    --container_image)
      CONTAINER_IMAGE="$2"
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

if [[ -z "$CONTAINER_IMAGE" ]]; then
  CONTAINER_IMAGE="my-solution:v1"
fi

if ! docker image inspect "$CONTAINER_IMAGE" &> /dev/null; then
  echo "ERROR: Base image '$CONTAINER_IMAGE' not found. Please build it first or provide a valid --container_image."
  exit 1
fi

# 1. Build the Flowstate service image on top of my-solution:v1
# This step adds the necessary configurations to the base image,
# allowing the service to communicate with Flowstate Zenoh router.
SERVICE_DIR="$AIC_TOP_DIR/flowstate/services/aic_model"
DOCKERFILE_SERVICE="$SERVICE_DIR/Dockerfile.service"

docker buildx build -t flowstate:aic_model \
  --no-cache \
  --load \
  --build-arg CONTAINER_IMAGE="$CONTAINER_IMAGE" \
  --file "$DOCKERFILE_SERVICE" \
  "$SERVICE_DIR"

# 2. Export the service image to a .tar bundle
# This saves the docker image as a tar archive to the file system, which
# can be subsequently bundled by the inbuild tool.
echo "INFO: Exporting image to tar file..."
if [ -d "$IMAGES_DIR/aic_model" ]; then
  echo "INFO: Deleting existing $IMAGES_DIR/aic_model directory..."
  rm -rf "$IMAGES_DIR/aic_model"
fi
mkdir -p "$IMAGES_DIR/aic_model"
docker save -o "$IMAGES_DIR/aic_model/aic_model.tar" flowstate:aic_model
chmod 644 "$IMAGES_DIR/aic_model/aic_model.tar"

# 3. Bundle the service using inbuild
# Packages the service using Intrinsic's 'inbuild' tool. It retrieves the
# corresponding SDK version, downloads the tool if necessary, and generates
# a .bundle.tar file using the service manifest and exported image.
SDK_VERSION="v1.28.20260223"

# Download the 'inbuild' tool if it doesn't exist
if [ ! -f ./inbuild ]; then
  echo "INFO: Downloading inbuild tool version ${SDK_VERSION}..."
  wget "https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inbuild-linux-amd64" -O inbuild \
  && chmod +x inbuild
fi

echo "INFO: Bundling service using inbuild..."
./inbuild service bundle \
  --manifest "$SERVICE_DIR/aic_model.manifest.textproto" \
  --oci_image "$IMAGES_DIR/aic_model/aic_model.tar" \
  --output "$IMAGES_DIR/aic_model/aic_model.bundle.tar"

