#!/usr/bin/env bash

SKILLS_DIR="src/aic/flowstate/aic_flowstate_skills"

# Function to list all valid skills dynamically
list_available_skills() {
  local skills=()
  if [ -d "$SKILLS_DIR" ]; then
    for dir in "$SKILLS_DIR"/*; do
      if [ -d "$dir" ]; then
        local name
        name=$(basename "$dir")
        # Check if the manifest file exists for this skill
        if [ -f "$dir/src/${name}.manifest.textproto" ]; then
          skills+=("$name")
        fi
      fi
    done
  fi
  echo "${skills[@]}"
}

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the AIC skills for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --ros_distro DISTRO  Name of the ROS distro (default: kilted)"
  echo "  --skill_name NAME    Build only the specified skill (default: build all)"
  echo ""
  
  local skills
  skills=$(list_available_skills)
  if [ -n "$skills" ]; then
    echo "Available skills:"
    for skill in $skills; do
      echo "  - $skill"
    done
    echo ""
  fi
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

# Get the list of all available skills dynamically
AVAILABLE_SKILLS=($(list_available_skills))

if [ ${#AVAILABLE_SKILLS[@]} -eq 0 ]; then
  echo "No skills found in $SKILLS_DIR"
  exit 1
fi

if [ -n "$SKILL_NAME" ]; then
  # Check if the requested skill is in the list of available skills
  is_valid=false
  for skill in "${AVAILABLE_SKILLS[@]}"; do
    if [ "$skill" = "$SKILL_NAME" ]; then
      is_valid=true
      break
    fi
  done

  if [ "$is_valid" = true ]; then
    build_and_bundle_skill "$SKILL_NAME"
  else
    echo "Unknown skill name: $SKILL_NAME"
    echo "Available skills: ${AVAILABLE_SKILLS[*]}"
    exit 1
  fi
else
  # Build all available skills
  for skill in "${AVAILABLE_SKILLS[@]}"; do
    build_and_bundle_skill "$skill"
  done
fi
