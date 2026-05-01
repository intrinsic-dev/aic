#!/usr/bin/env bash

IMAGES_DIR=./images
BUILDER_NAME=container-builder
ROS_DISTRO=jazzy

while [[ $# -gt 0 ]]; do
  case $1 in
    --images_dir)
      IMAGES_DIR="$2"
      shift # past argument
      shift # past value
      ;;
    --builder_name)
      BUILDER_NAME="$2"
      shift # past argument
      shift # past value
      ;;
    --service_name)
      SERVICE_NAME="$2"
      shift # past argument
      shift # past value
      ;;
    --service_package)
      SERVICE_PACKAGE="$2"
      shift # past argument
      shift # past value
      ;;
    --skill_name)
      SKILL_NAME="$2"
      shift # past argument
      shift # past value
      ;;
    --skill_package)
      SKILL_PACKAGE="$2"
      shift # past argument
      shift # past value
      ;;
    --dockerfile)
      CUSTOM_DOCKERFILE="$2"
      shift # past argument
      shift # past value
      ;;
    --manifest_path)
      MANIFEST_PATH="$2"
      shift # past argument
      shift # past value
      ;;
    --dependencies)
      DEPENDENCIES="$2"
      shift # past argument
      shift # past value
      ;;
    --ros_distro)
      ROS_DISTRO="$2"
      shift # past argument
      shift # past value
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if [[ -n "$SERVICE_NAME" && -n "$SERVICE_PACKAGE" ]]; then
  mkdir -p $IMAGES_DIR/$SERVICE_NAME

  if [[ -n "$CUSTOM_DOCKERFILE" ]]; then
    DOCKERFILE="$CUSTOM_DOCKERFILE"
  else
    DOCKERFILE="$SCRIPT_DIR/../resources/Dockerfile.service"
  fi

  docker buildx build -t $SERVICE_PACKAGE:$SERVICE_NAME \
      --builder="$BUILDER_NAME" \
      --output="\
        type=docker,\
        dest=$IMAGES_DIR/$SERVICE_NAME/$SERVICE_NAME.tar,\
        compression=zstd,\
        push=false,\
        name=$SERVICE_PACKAGE:$SERVICE_NAME" \
      --file $DOCKERFILE \
      --build-arg="SERVICE_PACKAGE=$SERVICE_PACKAGE" \
      --build-arg="SERVICE_NAME=$SERVICE_NAME" \
      --build-arg="DEPENDENCIES=$DEPENDENCIES" \
      --build-arg="SERVICE_EXECUTABLE_NAME=${SERVICE_NAME}_main" \
      --build-arg="ROS_DISTRO=$ROS_DISTRO" \
      .
elif [[ -n "$SKILL_NAME" && -n "$SKILL_PACKAGE" ]]; then
  mkdir -p $IMAGES_DIR/$SKILL_NAME

  if [[ -n "$CUSTOM_DOCKERFILE" ]]; then
    DOCKERFILE="$CUSTOM_DOCKERFILE"
  else
    DOCKERFILE="$SCRIPT_DIR/../resources/Dockerfile.skill"
  fi

  docker buildx build -t $SKILL_PACKAGE:$SKILL_NAME \
      --builder="$BUILDER_NAME" \
      --output="\
        type=docker,\
        dest=$IMAGES_DIR/$SKILL_NAME/$SKILL_NAME.tar,\
        compression=zstd,\
        push=false,\
        name=$SKILL_PACKAGE:$SKILL_NAME" \
      --file $DOCKERFILE \
      --build-arg="SKILL_PACKAGE=$SKILL_PACKAGE" \
      --build-arg="SKILL_NAME=$SKILL_NAME" \
      --build-arg="SKILL_EXECUTABLE_NAME=${SKILL_NAME}_main" \
      --build-arg="ROS_DISTRO=$ROS_DISTRO" \
      .

  if [[ -n "$MANIFEST_PATH" ]]; then
    # Parse the SDK_VERSION from sdk_version.json
    SDK_VERSION_FILE="$SCRIPT_DIR/../intrinsic_sdk_cmake/cmake/sdk_version.json"
    SDK_VERSION=$(grep -oP '"sdk_version": "\K[^"]+' "$SDK_VERSION_FILE")

    # Download the 'inbuild' tool if it doesn't exist
    if [ ! -f ./inbuild ]; then
        echo "INFO: Downloading inbuild tool version ${SDK_VERSION}..."
        wget "https://github.com/intrinsic-ai/sdk/releases/download/${SDK_VERSION}/inbuild-linux-amd64" -O inbuild \
          && chmod +x inbuild
    fi

    echo "INFO: Loading newly built image into local daemon..."
    docker load -i "images/${SKILL_NAME}/${SKILL_NAME}.tar"

    echo "INFO: Extracting descriptor set from container..."
    docker create --name temp_container "$SKILL_PACKAGE:$SKILL_NAME"
    docker cp "temp_container:/opt/ros/overlay/install/share/${SKILL_PACKAGE}/${SKILL_NAME}_protos.desc" \
     "images/${SKILL_NAME}/${SKILL_NAME}_protos.desc"
    docker rm -f temp_container

  fi
fi