#!/usr/bin/env bash
set -euo pipefail

# Deploy code to the Raspberry Pi and restart services.
#
# Modes:
#   rsync (default) - Push local code directly (fast iteration)
#   git             - Pull from remote on RPi (production-like)
#
# Usage:
#   # rsync mode (default)
#   RPI_HOST=192.168.1.28 ./deploy/rpi.sh
#
#   # git mode
#   RPI_HOST=192.168.1.28 ./deploy/rpi.sh git
#
#   # install deps after deploy
#   RPI_HOST=192.168.1.28 ./deploy/rpi.sh rsync --install-deps
#
#   # deploy ROS 2 as well (when ready)
#   RPI_HOST=192.168.1.28 ./deploy/rpi.sh rsync --ros2

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

echo "🤖 Deploying mower-roboto to ${RPI_USER}@${RPI_HOST}:${RPI_PATH}"
echo "   Mode: ${MODE}"
echo ""

if [ "$MODE" = "git" ]; then
  echo "📥 Pulling latest code from git..."
  ssh "${RPI_USER}@${RPI_HOST}" "cd ${RPI_PATH} && git pull"
else
  # Ensure target directories exist
  ssh "${RPI_USER}@${RPI_HOST}" "mkdir -p ${RPI_PATH}/backend ${RPI_PATH}/shared"

  echo "📤 Syncing backend..."
  rsync -avz --delete \
    --exclude 'venv' \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.env' \
    "${REPO_ROOT}/apps/backend/" \
    "${RPI_USER}@${RPI_HOST}:${RPI_PATH}/backend/"

  echo "📤 Syncing shared types..."
  rsync -avz --delete \
    "${REPO_ROOT}/shared/" \
    "${RPI_USER}@${RPI_HOST}:${RPI_PATH}/shared/"

  if [ "$DEPLOY_ROS2" = true ]; then
    echo "📤 Syncing ROS 2 workspace..."
    ssh "${RPI_USER}@${RPI_HOST}" "mkdir -p ${RPI_PATH}/ros2"
    rsync -avz --delete \
      --exclude 'build' \
      --exclude 'install' \
      --exclude 'log' \
      "${REPO_ROOT}/apps/ros2/" \
      "${RPI_USER}@${RPI_HOST}:${RPI_PATH}/ros2/"
  fi
fi

if [ "$INSTALL_DEPS" = true ]; then
  echo "📦 Installing Python dependencies..."
  ssh "${RPI_USER}@${RPI_HOST}" "cd ${RPI_PATH}/backend && ./venv/bin/pip install -r requirements.txt"
fi

echo "🔄 Restarting mower-backend service..."
ssh "${RPI_USER}@${RPI_HOST}" "sudo systemctl restart mower-backend.service"

echo ""
echo "✅ Deployment complete!"
echo ""
echo "📊 Service status:"
ssh "${RPI_USER}@${RPI_HOST}" "sudo systemctl status mower-backend.service --no-pager | head -10"
