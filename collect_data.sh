#!/usr/bin/env bash
# collect_data.sh — Full AIC perception data collection pipeline
#
# What this does:
#   1. Generates a randomized sample_config.yaml (N trials)
#   2. Starts the eval container (Terminal 1 equivalent)
#   3. Runs DataCollector policy (Terminal 2 equivalent)
#   4. Repeats until you have enough images
#   5. Trains YOLOv8 when done
#
# Usage:
#   ./collect_data.sh                    # Run with defaults (3 trials, collect once)
#   ./collect_data.sh --runs 10          # 10 separate runs = ~1350 images
#   ./collect_data.sh --trials 6        # 6 trials per run
#   ./collect_data.sh --target 2000     # Keep running until 2000 images collected
#   ./collect_data.sh --train-only      # Skip collection, just train YOLO on existing data
#
# Prerequisites:
#   pip install pyyaml ultralytics
#   The eval container must already exist: sudo docker ps -a | grep aic_eval

set -euo pipefail

# ── Config ────────────────────────────────────────────────────────────────────
TRIALS_PER_RUN=3
MAX_RUNS=999
TARGET_IMAGES=0          # 0 = run exactly MAX_RUNS times; >0 = run until target reached
OUTPUT_DIR="$HOME/aic_perception_data"
WS_DIR="$HOME/ws_aic/src/aic"
CONFIG_PATH="$WS_DIR/aic_engine/config/sample_config.yaml"
DATACOLLECTOR_PATH="$WS_DIR/aic_example_policies/aic_example_policies/ros/DataCollector.py"
GENERATE_CONFIG="$HOME/generate_config.py"
TRAIN_ONLY=false
SKIP_TRAIN=false

# ── Colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'
log()  { echo -e "${BLUE}[collect]${NC} $*"; }
ok()   { echo -e "${GREEN}[collect]${NC} $*"; }
warn() { echo -e "${YELLOW}[collect]${NC} $*"; }
err()  { echo -e "${RED}[collect]${NC} $*"; }

# ── Arg parsing ───────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --trials)   TRIALS_PER_RUN=$2; shift 2 ;;
        --runs)     MAX_RUNS=$2; shift 2 ;;
        --target)   TARGET_IMAGES=$2; shift 2 ;;
        --train-only) TRAIN_ONLY=true; shift ;;
        --no-train) SKIP_TRAIN=true; shift ;;
        --output)   OUTPUT_DIR=$2; shift 2 ;;
        *) err "Unknown arg: $1"; exit 1 ;;
    esac
done

# ── Helpers ───────────────────────────────────────────────────────────────────
count_images() {
    find "$OUTPUT_DIR/images" -name "*.png" 2>/dev/null | wc -l
}

check_container() {
    sudo docker ps -a --format '{{.Names}}' | grep -q "^aic_eval$"
}

container_running() {
    sudo docker ps --format '{{.Names}}' | grep -q "^aic_eval$"
}

ensure_container_up() {
    if ! container_running; then
        log "Starting aic_eval container..."
        # Fix mount propagation (needed on native Ubuntu Docker)
        sudo mount --make-rshared / 2>/dev/null || true
        sudo docker start aic_eval
        log "Waiting for container to be ready..."
        sleep 3
    fi
}

wait_for_collection_done() {
    local timeout=$1
    local elapsed=0
    local check_interval=5

    log "Waiting for DataCollector to finish (timeout: ${timeout}s)..."
    while [[ $elapsed -lt $timeout ]]; do
        # Check if aic_model process is still running in background
        if ! jobs %2 &>/dev/null 2>&1; then
            # Job finished
            return 0
        fi
        sleep $check_interval
        elapsed=$((elapsed + check_interval))
        local imgs
        imgs=$(count_images)
        echo -ne "\r  Images so far: $imgs (${elapsed}s elapsed)   "
    done
    echo ""
    warn "Timeout reached after ${timeout}s"
    return 1
}

# ── TRAIN ONLY mode ───────────────────────────────────────────────────────────
if [[ "$TRAIN_ONLY" == "true" ]]; then
    IMGS=$(count_images)
    log "Train-only mode. Found $IMGS images in $OUTPUT_DIR"
    if [[ $IMGS -eq 0 ]]; then
        err "No images found. Run collection first."
        exit 1
    fi
    # Write YOLO config
    YAML_PATH="$OUTPUT_DIR/aic_ports.yaml"
    cat > "$YAML_PATH" << EOF
path: $OUTPUT_DIR
train: images/train
val: images/val
nc: 3
names: ['sfp_port', 'sc_port', 'nic_card']
EOF
    log "Starting YOLOv8 training on $IMGS images..."
    yolo detect train \
        model=yolov8m.pt \
        data="$YAML_PATH" \
        epochs=100 \
        imgsz=1024 \
        batch=8 \
        project="$OUTPUT_DIR/runs" \
        name=aic_ports_v1
    ok "Training complete! Model at: $OUTPUT_DIR/runs/aic_ports_v1/weights/best.pt"
    exit 0
fi

# ── Preflight checks ──────────────────────────────────────────────────────────
log "Preflight checks..."

if ! check_container; then
    err "aic_eval container not found. Create it first with distrobox."
    exit 1
fi

if [[ ! -f "$GENERATE_CONFIG" ]]; then
    err "generate_config.py not found at $GENERATE_CONFIG"
    exit 1
fi

if [[ ! -f "$DATACOLLECTOR_PATH" ]]; then
    err "DataCollector.py not found at $DATACOLLECTOR_PATH"
    err "Expected: $DATACOLLECTOR_PATH"
    exit 1
fi

if ! python3 -c "import yaml" 2>/dev/null; then
    err "pyyaml not installed: pip install pyyaml"
    exit 1
fi

mkdir -p "$OUTPUT_DIR"/{images/train,images/val,labels/train,labels/val,metadata}

ok "Preflight passed."
log "Output dir:     $OUTPUT_DIR"
log "Trials per run: $TRIALS_PER_RUN"
log "Max runs:       $MAX_RUNS"
[[ $TARGET_IMAGES -gt 0 ]] && log "Target images:  $TARGET_IMAGES"

# ── Main collection loop ──────────────────────────────────────────────────────
RUN=0
SEED=$RANDOM

while [[ $RUN -lt $MAX_RUNS ]]; do
    RUN=$((RUN + 1))
    IMGS=$(count_images)

    # Check if target reached
    if [[ $TARGET_IMAGES -gt 0 && $IMGS -ge $TARGET_IMAGES ]]; then
        ok "Target of $TARGET_IMAGES images reached ($IMGS collected). Stopping collection."
        break
    fi

    log "═══════════════════════════════════════"
    log "RUN $RUN / $MAX_RUNS  |  Images so far: $IMGS"
    log "═══════════════════════════════════════"

    # 1. Generate randomized config
    SEED=$((SEED + RUN * 7))
    log "Generating config (seed=$SEED, trials=$TRIALS_PER_RUN)..."
    python3 "$GENERATE_CONFIG" \
        --trials "$TRIALS_PER_RUN" \
        --output "$CONFIG_PATH" \
        --seed "$SEED"

    # 2. Make sure the eval container is running
    ensure_container_up

    # 3. Start the eval container entrypoint in background (inside tmux if available)
    log "Starting eval container entrypoint..."

    # Kill any existing aic sessions
    sudo docker exec aic_eval bash -c "pkill -f entrypoint.sh || true" 2>/dev/null || true
    sleep 1

    # Run entrypoint detached inside the container
    sudo docker exec -d aic_eval bash -c \
        "/entrypoint.sh ground_truth:=true start_aic_engine:=true > /tmp/aic_eval.log 2>&1"

    log "Waiting for Gazebo and aic_engine to initialize (20s)..."
    sleep 20

    # Verify the engine started
    if ! sudo docker exec aic_eval bash -c "pgrep -f aic_engine > /dev/null 2>&1"; then
        warn "aic_engine may not have started yet, waiting more..."
        sleep 10
    fi

    # 4. Run the DataCollector policy from host via pixi
    log "Running DataCollector policy..."
    TIMEOUT_PER_TRIAL=300  # 5 min per trial (plenty of buffer)
    TOTAL_TIMEOUT=$(( TRIALS_PER_RUN * TIMEOUT_PER_TRIAL ))

    cd "$WS_DIR"
    timeout "$TOTAL_TIMEOUT" pixi run ros2 run aic_model aic_model \
        --ros-args \
        -p use_sim_time:=true \
        -p policy:=aic_example_policies.ros.DataCollector \
        2>&1 | tee "/tmp/datacollector_run${RUN}.log" || {
        warn "DataCollector exited (possibly finished normally or timed out)"
    }

    IMGS_AFTER=$(count_images)
    NEW_IMGS=$((IMGS_AFTER - IMGS))
    ok "Run $RUN complete. Collected $NEW_IMGS new images (total: $IMGS_AFTER)"

    # 5. Clean up the container entrypoint for next run
    log "Stopping eval container processes for clean restart..."
    sudo docker exec aic_eval bash -c "pkill -f entrypoint.sh || true; pkill -f gz || true; pkill -f ros2 || true" 2>/dev/null || true
    sleep 3

    # Small pause between runs
    [[ $RUN -lt $MAX_RUNS ]] && sleep 2
done

# ── Final summary ─────────────────────────────────────────────────────────────
FINAL_IMGS=$(count_images)
TRAIN_IMGS=$(find "$OUTPUT_DIR/images/train" -name "*.png" 2>/dev/null | wc -l)
VAL_IMGS=$(find "$OUTPUT_DIR/images/val" -name "*.png" 2>/dev/null | wc -l)

ok "═══════════════════════════════════════"
ok "Collection complete!"
ok "  Total images: $FINAL_IMGS"
ok "  Train:        $TRAIN_IMGS"
ok "  Val:          $VAL_IMGS"
ok "  Output:       $OUTPUT_DIR"
ok "═══════════════════════════════════════"

# ── Auto-train if we have enough data ────────────────────────────────────────
if [[ "$SKIP_TRAIN" == "true" ]]; then
    log "Skipping training (--no-train). Run manually:"
    log "  ./collect_data.sh --train-only"
    exit 0
fi

if [[ $FINAL_IMGS -lt 200 ]]; then
    warn "Only $FINAL_IMGS images — recommend at least 500 for training."
    warn "Run more collection: ./collect_data.sh --runs 5"
    warn "Or train anyway:     ./collect_data.sh --train-only"
    exit 0
fi

# Write YOLO dataset config
YAML_PATH="$OUTPUT_DIR/aic_ports.yaml"
cat > "$YAML_PATH" << EOF
# AIC Port Detection Dataset
# Generated by collect_data.sh
path: $OUTPUT_DIR
train: images/train
val: images/val
nc: 3
names: ['sfp_port', 'sc_port', 'nic_card']
EOF

log "Starting YOLOv8 training on $FINAL_IMGS images..."
if ! command -v yolo &>/dev/null; then
    warn "ultralytics not installed. Install: pip install ultralytics"
    warn "Then train: yolo detect train model=yolov8m.pt data=$YAML_PATH epochs=100 imgsz=1024 batch=8"
    exit 0
fi

yolo detect train \
    model=yolov8m.pt \
    data="$YAML_PATH" \
    epochs=100 \
    imgsz=1024 \
    batch=8 \
    project="$OUTPUT_DIR/runs" \
    name=aic_ports_v1

ok "Training complete!"
ok "Best model: $OUTPUT_DIR/runs/aic_ports_v1/weights/best.pt"