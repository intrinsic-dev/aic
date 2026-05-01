#!/usr/bin/env bash
# Start the AIC eval environment in a Docker container.
# Usage: ./scripts/start_eval.sh [extra ros args...]
#
# Examples:
#   ./scripts/start_eval.sh
#   ./scripts/start_eval.sh ground_truth:=true start_aic_engine:=false
#   ./scripts/start_eval.sh gazebo_gui:=false launch_rviz:=false ground_truth:=false start_aic_engine:=true
set -euo pipefail

IMAGE="ghcr.io/intrinsic-dev/aic/aic_eval:latest"
CONTAINER_NAME="aic_eval"

DEFAULT_ARGS=(
    "spawn_task_board:=true"
    "nic_card_mount_0_present:=true"
    "spawn_cable:=true"
    "cable_type:=sfp_sc_cable"
    "attach_cable_to_gripper:=true"
    "ground_truth:=true"
    "start_aic_engine:=false"
    "launch_rviz:=false"
    "gazebo_gui:=false"
)

LAUNCH_ARGS=("${@:-${DEFAULT_ARGS[@]}}")

# Remove any existing container with the same name
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Detect NVIDIA GPU
GPU_FLAGS=""
if docker info 2>/dev/null | grep -q "Runtimes.*nvidia"; then
    GPU_FLAGS="--gpus all"
fi

echo "[eval] Starting $CONTAINER_NAME with args: ${LAUNCH_ARGS[*]}"

# shellcheck disable=SC2086
docker run \
    --rm \
    --name "$CONTAINER_NAME" \
    --platform linux/amd64 \
    -p 7447:7447 \
    $GPU_FLAGS \
    "$IMAGE" \
    "${LAUNCH_ARGS[@]}"
