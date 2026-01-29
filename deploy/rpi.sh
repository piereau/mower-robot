#!/usr/bin/env bash
set -euo pipefail

# Deploy code to the Raspberry Pi and restart services.
#
# Modes:
#   rsync (default) - Push local code directly (fast iteration)
#   git             - Pull from remote on RPi (production-like)
#
# Usage:
#   RPI_HOST=herbobot.local ./deploy/rpi.sh              # rsync mode
#   RPI_HOST=herbobot.local ./deploy/rpi.sh git          # git pull mode
#   RPI_HOST=herbobot.local ./deploy/rpi.sh --install-deps
#   RPI_HOST=herbobot.local ./deploy/rpi.sh --ros2
#
# Tip: Set up SSH keys to avoid password prompts:
#   ssh-keygen -t ed25519
#   ssh-copy-id pi@herbobot.local

RPI_HOST="${RPI_HOST:?RPI_HOST is required}"
RPI_USER="${RPI_USER:-pi}"
RPI_PATH="${RPI_PATH:-/home/pi/mower-roboto}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MODE="${1:-rsync}"
INSTALL_DEPS=false
DEPLOY_ROS2=false

# Parse args
for arg in "$@"; do
  case $arg in
    --install-deps) INSTALL_DEPS=true ;;
    --ros2) DEPLOY_ROS2=true ;;
  esac
done

# =============================================================================
# SSH Connection Multiplexing - reuse single connection for all commands
# This means you only enter password ONCE (or zero times with SSH keys)
# =============================================================================
SSH_CONTROL_PATH="/tmp/ssh-mower-$$"
SSH_OPTS="-o ControlMaster=auto -o ControlPath=${SSH_CONTROL_PATH} -o ControlPersist=10"

cleanup() {
  # Close the SSH master connection on exit
  ssh -O exit -o ControlPath="${SSH_CONTROL_PATH}" "${RPI_USER}@${RPI_HOST}" 2>/dev/null || true
}
trap cleanup EXIT

# Helper functions using multiplexed connection
remote() {
  ssh ${SSH_OPTS} "${RPI_USER}@${RPI_HOST}" "$@"
}

rsync_to() {
  rsync -avz --delete -e "ssh ${SSH_OPTS}" "$@"
}

# =============================================================================
# Start deployment
# =============================================================================
echo "🤖 Deploying mower-roboto to ${RPI_USER}@${RPI_HOST}:${RPI_PATH}"
echo "   Mode: ${MODE}"
echo ""

# Establish master connection (prompts for password once if needed)
echo "🔌 Connecting..."
ssh ${SSH_OPTS} -fN "${RPI_USER}@${RPI_HOST}"

if [ "$MODE" = "git" ]; then
  echo "📥 Pulling latest code from git..."
  remote "cd ${RPI_PATH} && git pull"
else
  # Ensure target directories exist
  remote "mkdir -p ${RPI_PATH}/backend ${RPI_PATH}/shared"

  echo "📤 Syncing backend..."
  rsync_to \
    --exclude 'venv' \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.env' \
    "${REPO_ROOT}/apps/backend/" \
    "${RPI_USER}@${RPI_HOST}:${RPI_PATH}/backend/"

  echo "📤 Syncing shared types..."
  rsync_to \
    "${REPO_ROOT}/shared/" \
    "${RPI_USER}@${RPI_HOST}:${RPI_PATH}/shared/"

  if [ "$DEPLOY_ROS2" = true ]; then
    echo "📤 Syncing ROS 2 workspace..."
    remote "mkdir -p ${RPI_PATH}/ros2"
    rsync_to \
      --exclude 'build' \
      --exclude 'install' \
      --exclude 'log' \
      "${REPO_ROOT}/apps/ros2/" \
      "${RPI_USER}@${RPI_HOST}:${RPI_PATH}/ros2/"
  fi
fi

if [ "$INSTALL_DEPS" = true ]; then
  echo "📦 Installing Python dependencies..."
  remote "cd ${RPI_PATH}/backend && ./venv/bin/pip install -r requirements.txt"
fi

echo "🔄 Restarting mower-backend service..."
remote "sudo systemctl restart mower-backend.service"

echo ""
echo "✅ Deployment complete!"
echo ""
echo "📊 Service status:"
remote "sudo systemctl status mower-backend.service --no-pager | head -10"
