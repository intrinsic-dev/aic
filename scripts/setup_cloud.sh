#!/usr/bin/env bash
# One-time setup for AIC eval on a fresh Ubuntu 24.04 cloud instance with NVIDIA GPU.
# Runs the eval via plain Docker (no distrobox) with port 7447 exposed for Zenoh.
# Run as a regular user with sudo access.
set -euo pipefail

# --- Docker ---
if ! command -v docker &>/dev/null; then
    echo "[setup] Installing Docker..."
    curl -fsSL https://get.docker.com | sh
    sudo usermod -aG docker "$USER"
    echo "[setup] Docker installed. You may need to log out and back in for group changes to take effect."
else
    echo "[setup] Docker already installed."
fi

# --- NVIDIA Container Toolkit ---
if command -v nvidia-smi &>/dev/null; then
    if ! dpkg -l | grep -q nvidia-container-toolkit; then
        echo "[setup] Installing NVIDIA Container Toolkit..."
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
        curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
            | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
            | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
        sudo apt-get update -qq
        sudo apt-get install -y nvidia-container-toolkit
        sudo nvidia-ctk runtime configure --runtime=docker
        sudo systemctl restart docker
        echo "[setup] NVIDIA Container Toolkit installed."
    else
        echo "[setup] NVIDIA Container Toolkit already installed."
    fi
else
    echo "[setup] No NVIDIA GPU detected, skipping NVIDIA Container Toolkit."
fi

# --- Pixi ---
if ! command -v pixi &>/dev/null; then
    echo "[setup] Installing Pixi..."
    curl -fsSL https://pixi.sh/install.sh | sh
    export PATH="$HOME/.pixi/bin:$PATH"
    echo "[setup] Pixi installed."
else
    echo "[setup] Pixi already installed."
fi

# --- Shell config (pixi PATH + Zenoh env vars) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SHELL_BLOCK="
# --- AIC eval (added by setup_cloud.sh) ---
export PATH=\"\$HOME/.pixi/bin:\$PATH\"
export ZENOH_ROUTER_CONFIG_URI=\"$REPO_ROOT/docker/aic_eval/aic_zenoh_config.json5\"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1"

for RC in "$HOME/.bashrc" "$HOME/.zshrc"; do
    if [ -f "$RC" ] && ! grep -q "AIC eval (added by setup_cloud.sh)" "$RC"; then
        echo "$SHELL_BLOCK" >> "$RC"
        echo "[setup] Added pixi PATH and Zenoh env vars to $RC."
    fi
done

# --- Pull aic_eval image ---
echo "[setup] Pulling aic_eval Docker image..."
docker pull --platform linux/amd64 ghcr.io/intrinsic-dev/aic/aic_eval:latest

# --- Install pixi dependencies ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
echo "[setup] Installing pixi dependencies in $REPO_ROOT..."
cd "$REPO_ROOT"
pixi install

echo ""
echo "[setup] Done. Reload your shell, then open two terminals and run:"
echo "  source ~/.bashrc  (or ~/.zshrc)"
echo "  Terminal 1: ./scripts/start_eval.sh"
echo "  Terminal 2: ./scripts/run_policy.sh aic_example_policies.ros.WaveArm"
