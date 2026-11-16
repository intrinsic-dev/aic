#!/usr/bin/env bash
set -euo pipefail

print_usage() {
  cat <<'EOF'
Usage:
  scripts/submit_to_challenge.sh -t <team_name> -v <version_tag> [options]

Required:
  -t, --team            Name of the team
  -v, --version         Immutable image tag to publish (example: v7 or 92f45e6)

Options:
  -r, --region          AWS region (default: us-east-1)
  --registry            ECR registry host (default: 973918476471.dkr.ecr.us-east-1.amazonaws.com)
  -h, --help            Show this help

Examples:
  scripts/submit_to_challenge.sh -t teamname -v v7
  scripts/submit_to_challenge.sh -t teamname -v 92f45e6

Notes:
  - This script only builds, logs in, tags, and pushes to ECR.
  - Final submission is done in the challenge portal by pasting the printed OCI URI.
EOF
}

TEAM_NAME=""
VERSION_TAG=""
AWS_REGION="us-east-1"
REGISTRY="973918476471.dkr.ecr.us-east-1.amazonaws.com"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -t|--team)
      TEAM_NAME="$2"
      shift 2
      ;;
    -v|--version)
      VERSION_TAG="$2"
      shift 2
      ;;
    -r|--region)
      AWS_REGION="$2"
      shift 2
      ;;
    --registry)
      REGISTRY="$2"
      shift 2
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      print_usage
      exit 1
      ;;
  esac
done

if [[ -z "$TEAM_NAME" ]]; then
  echo "Error: --team is required."
  print_usage
  exit 1
fi

if [[ -z "$VERSION_TAG" ]]; then
  echo "Error: --version is required."
  print_usage
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker not found. Install Docker first."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

REMOTE_URI="$REGISTRY/aic-team/$TEAM_NAME:$VERSION_TAG"

LOCAL_IMAGE="$(docker compose -f "$REPO_ROOT/docker/docker-compose.yaml" config --format json | python3 -c 'import json,sys; print(json.load(sys.stdin)["services"]["model"]["image"])')"

if [[ -z "$LOCAL_IMAGE" ]]; then
  echo "Error: could not resolve model image from docker/docker-compose.yaml"
  exit 1
fi

echo "==> Team: $TEAM_NAME"
echo "==> Local image: $LOCAL_IMAGE"
echo "==> Remote image: $REMOTE_URI"

echo "==> Building model image"
docker compose -f "$REPO_ROOT/docker/docker-compose.yaml" build model

echo "==> Running local evaluation (this may take a while)"
docker compose -f "$REPO_ROOT/docker/docker-compose.yaml" up -d --force-recreate eval model
bash "$REPO_ROOT/docker/wait_for_eval2.sh"
SCORE_FILE="$REPO_ROOT/docker/aic_results/scoring_submission_precheck_$VERSION_TAG.yaml"
docker cp aic-eval-1:/root/aic_results/scoring.yaml "$SCORE_FILE"
echo "==> Local scoring saved to docker/aic_results/scoring_submission_precheck_$VERSION_TAG.yaml"

echo "==> Local validation summary"
python3 - "$SCORE_FILE" <<'PY'
import sys
from pathlib import Path

score_file = Path(sys.argv[1])
lines = score_file.read_text().splitlines()

total = None
trial_totals = {}
current_trial = None
for raw in lines:
    line = raw.rstrip()
    if line.startswith("total:"):
        total = float(line.split(":", 1)[1].strip())
    elif line.startswith("trial_") and line.endswith(":"):
        current_trial = line[:-1]
        trial_totals[current_trial] = 0.0
    elif line.strip().startswith("score:") and current_trial is not None:
        try:
            value = float(line.split(":", 1)[1].strip())
            trial_totals[current_trial] += value
        except ValueError:
            pass

if total is not None:
    print(f"  total: {total:.6f}")
for trial in sorted(trial_totals.keys()):
    print(f"  {trial}: {trial_totals[trial]:.6f}")
PY

echo
echo "Ready to push:"
echo "  local:  $LOCAL_IMAGE"
echo "  remote: $REMOTE_URI"
read -r -p "Push this image now? [y/N]: " PUSH_CONFIRM
case "$PUSH_CONFIRM" in
  y|Y|yes|YES)
    ;;
  *)
    echo "Push skipped by user."
    echo "You can later push manually with:"
    echo "  docker tag $LOCAL_IMAGE $REMOTE_URI"
    echo "  docker push $REMOTE_URI"
    exit 0
    ;;
esac

echo "==> Authenticating to ECR"
if ! command -v aws >/dev/null 2>&1; then
  echo "Error: aws CLI not found. Install AWS CLI first."
  exit 1
fi
export AWS_PROFILE="$TEAM_NAME"
aws ecr get-login-password --region "$AWS_REGION" | docker login --username AWS --password-stdin "$REGISTRY"

echo "==> Tagging image"
docker tag "$LOCAL_IMAGE" "$REMOTE_URI"

echo "==> Pushing image"
docker push "$REMOTE_URI"

echo
echo "Push completed. Submit this OCI image URI in the challenge portal:"
echo "$REMOTE_URI"
