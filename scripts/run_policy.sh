#!/usr/bin/env bash
# Run a policy against the running eval environment.
# Usage: ./scripts/run_policy.sh [policy_class]
#
# Examples:
#   ./scripts/run_policy.sh
#   ./scripts/run_policy.sh aic_example_policies.ros.WaveArm
#   ./scripts/run_policy.sh aic_example_policies.ros.SACPolicy
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

POLICY="${1:-aic_example_policies.ros.WaveArm}"

cd "$REPO_ROOT"

# Common env for all ros2 commands
export ZENOH_ROUTER_CONFIG_URI="$REPO_ROOT/docker/aic_eval/aic_zenoh_config.json5"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1

echo ""
echo "=== Topic List ==="
pixi run ros2 topic list

echo ""
echo "=== Node List ==="
pixi run ros2 node list

echo ""
echo "=== Topic Hz (joint_states, 3s sample) ==="
timeout 3 pixi run ros2 topic hz /joint_states 2>/dev/null || true

echo ""
echo "=== Running policy: $POLICY ==="
pixi run ros2 run aic_model aic_model \
    --ros-args \
    -p use_sim_time:=true \
    -p policy:="$POLICY"
