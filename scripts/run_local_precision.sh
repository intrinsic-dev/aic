#!/usr/bin/env bash
# Unified runner for LocalPrecisionPolicy: train, test, and deploy.
#
# Subcommands:
#   train-p1   Train Phase 1 (XY centering) SAC policy in MuJoCo sim
#   train-p2   Train Phase 2 (F/T insertion) SAC policy in MuJoCo sim
#   test       Run a quick headless rollout and print per-step stats
#   deploy     Run LocalPrecisionPolicy against the live eval environment
#
# Usage:
#   ./scripts/run_local_precision.sh train-p1 --scene /path/to/scene.xml [opts]
#   ./scripts/run_local_precision.sh train-p2 --scene /path/to/scene.xml [opts]
#   ./scripts/run_local_precision.sh test  --scene /path/to/scene.xml [--phase 1|2] [--ckpt /path/to/ckpt.pt]
#   ./scripts/run_local_precision.sh deploy [--p1-ckpt /path] [--p2-ckpt /path]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_P1_CKPT="$REPO_ROOT/checkpoints/phase1/final.pt"
DEFAULT_P2_CKPT="$REPO_ROOT/checkpoints/phase2/final.pt"
DEFAULT_PORT_TYPE="sfp"
DEFAULT_TOTAL_STEPS=500000
DEFAULT_BUFFER_SIZE_P1=50000
DEFAULT_BUFFER_SIZE_P2=20000
DEFAULT_TEST_EPISODES=3

# Correct body/sensor names for aic_utils/aic_mujoco/mjcf/scene.xml
DEFAULT_SFP_BODY="sfp_port_0_link"
DEFAULT_SC_BODY="sc_port_0::sc_port_link"
DEFAULT_FT_SENSOR="AtiForceTorqueSensor_force"

# ── Platform runner ───────────────────────────────────────────────────────────
# On linux-64 / osx-arm64: uses pixi (default).
# On linux-aarch64 or any unsupported platform: set USE_SYSTEM_PYTHON=1 to
# bypass pixi and use the system Python directly (after pip-installing deps).
#   export USE_SYSTEM_PYTHON=1
#   pip install "mujoco==3.5.0" torch gymnasium opencv-python "numpy<2.3" transforms3d
if [[ "${USE_SYSTEM_PYTHON:-0}" == "1" ]]; then
    RUNNER="python3"
else
    RUNNER="pixi run python"
fi

# ── Zenoh / ROS env (mirrors run_policy.sh) ───────────────────────────────────
export ZENOH_ROUTER_CONFIG_URI="$REPO_ROOT/docker/aic_eval/aic_zenoh_config.json5"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1

# ── Helpers ───────────────────────────────────────────────────────────────────

usage() {
    sed -n '2,15p' "$0" | sed 's/^# \?//'
    exit 1
}

die() { echo "[error] $*" >&2; exit 1; }

require_scene() {
    [[ -n "${SCENE:-}" ]] || die "--scene is required for this subcommand."
    [[ -f "$SCENE" ]] || die "Scene file not found: $SCENE"
}

# ── Subcommands ───────────────────────────────────────────────────────────────

cmd_train_p1() {
    SCENE=""
    PORT_TYPE="$DEFAULT_PORT_TYPE"
    TOTAL_STEPS="$DEFAULT_TOTAL_STEPS"
    BUFFER_SIZE="$DEFAULT_BUFFER_SIZE_P1"
    SAVE_DIR="$REPO_ROOT/checkpoints/phase1"
    LOAD=""
    DET_WEIGHT=0.1
    SFP_BODY="$DEFAULT_SFP_BODY"
    SC_BODY="$DEFAULT_SC_BODY"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --scene)          SCENE="$2";       shift 2 ;;
            --port_type)      PORT_TYPE="$2";   shift 2 ;;
            --total_steps)    TOTAL_STEPS="$2"; shift 2 ;;
            --buffer_size)    BUFFER_SIZE="$2"; shift 2 ;;
            --save_dir)       SAVE_DIR="$2";    shift 2 ;;
            --load)           LOAD="$2";        shift 2 ;;
            --det_loss_weight) DET_WEIGHT="$2"; shift 2 ;;
            --sfp_port_body)  SFP_BODY="$2";    shift 2 ;;
            --sc_port_body)   SC_BODY="$2";     shift 2 ;;
            *) shift ;;
        esac
    done

    require_scene
    mkdir -p "$SAVE_DIR"

    echo "[train-p1] Scene:         $SCENE"
    echo "[train-p1] Port type:     $PORT_TYPE"
    echo "[train-p1] Steps:         $TOTAL_STEPS"
    echo "[train-p1] Save dir:      $SAVE_DIR"
    echo "[train-p1] SFP body:      $SFP_BODY"
    echo "[train-p1] SC body:       $SC_BODY"
    [[ -n "$LOAD" ]] && echo "[train-p1] Resuming:      $LOAD"

    cd "$REPO_ROOT"
    $RUNNER - <<PYEOF
import sys
import aic_example_policies.training.centering_env as _ce

# Inject correct body names without touching source files
_ce.CenteringConfig.__dataclass_fields__['sfp_port_body'].default = "$SFP_BODY"
_ce.CenteringConfig.__dataclass_fields__['sc_port_body'].default  = "$SC_BODY"

sys.argv = [
    'train_centering',
    '--scene',        '$SCENE',
    '--port_type',    '$PORT_TYPE',
    '--total_steps',  '$TOTAL_STEPS',
    '--buffer_size',  '$BUFFER_SIZE',
    '--save_dir',     '$SAVE_DIR',
    '--det_loss_weight', '$DET_WEIGHT',
] + (['--load', '$LOAD'] if '$LOAD' else [])

from aic_example_policies.training.train_centering import main
main()
PYEOF
}

cmd_train_p2() {
    SCENE=""
    PORT_TYPE="$DEFAULT_PORT_TYPE"
    TOTAL_STEPS="$DEFAULT_TOTAL_STEPS"
    BUFFER_SIZE="$DEFAULT_BUFFER_SIZE_P2"
    SAVE_DIR="$REPO_ROOT/checkpoints/phase2"
    LOAD=""
    DET_WEIGHT=0.1
    SFP_BODY="$DEFAULT_SFP_BODY"
    SC_BODY="$DEFAULT_SC_BODY"
    FT_SENSOR="$DEFAULT_FT_SENSOR"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --scene)          SCENE="$2";       shift 2 ;;
            --port_type)      PORT_TYPE="$2";   shift 2 ;;
            --total_steps)    TOTAL_STEPS="$2"; shift 2 ;;
            --buffer_size)    BUFFER_SIZE="$2"; shift 2 ;;
            --save_dir)       SAVE_DIR="$2";    shift 2 ;;
            --load)           LOAD="$2";        shift 2 ;;
            --det_loss_weight) DET_WEIGHT="$2"; shift 2 ;;
            --sfp_port_body)  SFP_BODY="$2";    shift 2 ;;
            --sc_port_body)   SC_BODY="$2";     shift 2 ;;
            --ft_sensor_name) FT_SENSOR="$2";   shift 2 ;;
            *) shift ;;
        esac
    done

    require_scene
    mkdir -p "$SAVE_DIR"

    echo "[train-p2] Scene:         $SCENE"
    echo "[train-p2] Port type:     $PORT_TYPE"
    echo "[train-p2] Steps:         $TOTAL_STEPS"
    echo "[train-p2] Save dir:      $SAVE_DIR"
    echo "[train-p2] SFP body:      $SFP_BODY"
    echo "[train-p2] SC body:       $SC_BODY"
    echo "[train-p2] F/T sensor:    $FT_SENSOR"
    [[ -n "$LOAD" ]] && echo "[train-p2] Resuming:      $LOAD"

    cd "$REPO_ROOT"
    $RUNNER - <<PYEOF
import sys
import aic_example_policies.training.insertion_env as _ie

# Inject correct body/sensor names without touching source files
_ie.InsertionConfig.__dataclass_fields__['sfp_port_body'].default = "$SFP_BODY"
_ie.InsertionConfig.__dataclass_fields__['sc_port_body'].default  = "$SC_BODY"
_ie.InsertionConfig.__dataclass_fields__['ft_sensor_name'].default = "$FT_SENSOR"

sys.argv = [
    'train_insertion',
    '--scene',        '$SCENE',
    '--port_type',    '$PORT_TYPE',
    '--total_steps',  '$TOTAL_STEPS',
    '--buffer_size',  '$BUFFER_SIZE',
    '--save_dir',     '$SAVE_DIR',
    '--det_loss_weight', '$DET_WEIGHT',
] + (['--load', '$LOAD'] if '$LOAD' else [])

from aic_example_policies.training.train_insertion import main
main()
PYEOF
}

cmd_test() {
    SCENE=""
    PHASE=1
    CKPT=""
    PORT_TYPE="$DEFAULT_PORT_TYPE"
    N_EPISODES="$DEFAULT_TEST_EPISODES"
    SFP_BODY="$DEFAULT_SFP_BODY"
    SC_BODY="$DEFAULT_SC_BODY"
    FT_SENSOR="$DEFAULT_FT_SENSOR"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --scene)          SCENE="$2";      shift 2 ;;
            --phase)          PHASE="$2";      shift 2 ;;
            --ckpt)           CKPT="$2";       shift 2 ;;
            --port_type)      PORT_TYPE="$2";  shift 2 ;;
            --episodes)       N_EPISODES="$2"; shift 2 ;;
            --sfp_port_body)  SFP_BODY="$2";   shift 2 ;;
            --sc_port_body)   SC_BODY="$2";    shift 2 ;;
            --ft_sensor_name) FT_SENSOR="$2";  shift 2 ;;
            *) shift ;;
        esac
    done

    require_scene

    if [[ -z "$CKPT" ]]; then
        [[ "$PHASE" == "1" ]] && CKPT="$DEFAULT_P1_CKPT" || CKPT="$DEFAULT_P2_CKPT"
    fi

    echo "[test] Scene:      $SCENE"
    echo "[test] Phase:      $PHASE"
    echo "[test] Checkpoint: $CKPT"
    echo "[test] Episodes:   $N_EPISODES"
    echo ""

    cd "$REPO_ROOT"
    $RUNNER - <<PYEOF
import numpy as np
import torch, os

scene     = "$SCENE"
phase     = $PHASE
ckpt      = "$CKPT"
n_ep      = $N_EPISODES
port_type = "$PORT_TYPE"

if phase == 1:
    from aic_example_policies.training.centering_env import CenteringConfig, CenteringEnv
    cfg = CenteringConfig(
        scene_path=scene, port_type=port_type,
        sfp_port_body="$SFP_BODY", sc_port_body="$SC_BODY",
    )
    env = CenteringEnv(cfg)
else:
    from aic_example_policies.training.insertion_env import InsertionConfig, InsertionEnv
    cfg = InsertionConfig(
        scene_path=scene, port_type=port_type,
        sfp_port_body="$SFP_BODY", sc_port_body="$SC_BODY",
        ft_sensor_name="$FT_SENSOR",
    )
    env = InsertionEnv(cfg)

from aic_example_policies.training.networks import SACActor
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
actor  = SACActor(phase).to(device)

if os.path.exists(ckpt):
    state = torch.load(ckpt, map_location=device)
    actor.load_state_dict(state["actor"])
    actor.eval()
    print(f"Loaded checkpoint: {ckpt}")
else:
    print(f"[warn] Checkpoint not found ({ckpt}), running random policy.")
    actor = None

def to_tensor(obs, device):
    out = {}
    for k, v in obs.items():
        if k == "proprio":
            out[k] = torch.from_numpy(v).unsqueeze(0).float().to(device)
        else:
            out[k] = torch.from_numpy(v.transpose(2,0,1)).unsqueeze(0).float().to(device) / 255.0
    return out

results = []
for ep in range(n_ep):
    obs, _ = env.reset()
    ep_reward, steps, final_info, done = 0.0, 0, {}, False
    while not done:
        if actor is not None:
            with torch.no_grad():
                action = actor.get_action(to_tensor(obs, device)).squeeze(0).cpu().numpy()
        else:
            action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        ep_reward += reward; steps += 1; final_info = info
        done = terminated or truncated
    results.append((ep_reward, steps, final_info))
    xy_err = final_info.get("xy_error", float("nan"))
    depth  = final_info.get("depth",  "n/a")
    ft_mag = final_info.get("ft_mag", "n/a")
    print(f"  ep {ep+1:2d}: reward={ep_reward:+7.2f}  steps={steps:3d}  xy_err={xy_err:.4f}m"
          + (f"  depth={depth:.4f}m  ft_mag={ft_mag:.1f}N" if phase == 2 else ""))

print(f"\nMean reward over {n_ep} episodes: {np.mean([r[0] for r in results]):+.2f}")
env.close()
PYEOF
}

cmd_deploy() {
    P1_CKPT="$DEFAULT_P1_CKPT"
    P2_CKPT="$DEFAULT_P2_CKPT"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --p1-ckpt) P1_CKPT="$2"; shift 2 ;;
            --p2-ckpt) P2_CKPT="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    [[ -f "$P1_CKPT" ]] || echo "[warn] Phase 1 checkpoint not found: $P1_CKPT"
    [[ -f "$P2_CKPT" ]] || echo "[warn] Phase 2 checkpoint not found: $P2_CKPT"

    export AIC_PHASE1_CKPT="$P1_CKPT"
    export AIC_PHASE2_CKPT="$P2_CKPT"

    echo "[deploy] Phase 1 ckpt: $P1_CKPT"
    echo "[deploy] Phase 2 ckpt: $P2_CKPT"
    echo "[deploy] Starting LocalPrecisionPolicy..."
    echo ""

    cd "$REPO_ROOT"
    pixi run ros2 run aic_model aic_model \
        --ros-args \
        -p use_sim_time:=true \
        -p policy:="aic_example_policies.ros.LocalPrecisionPolicy"
}

# ── Dispatch ──────────────────────────────────────────────────────────────────

[[ $# -lt 1 ]] && usage

SUBCMD="$1"; shift

case "$SUBCMD" in
    train-p1) cmd_train_p1 "$@" ;;
    train-p2) cmd_train_p2 "$@" ;;
    test)     cmd_test     "$@" ;;
    deploy)   cmd_deploy   "$@" ;;
    *)        die "Unknown subcommand: $SUBCMD. Expected: train-p1 | train-p2 | test | deploy" ;;
esac
