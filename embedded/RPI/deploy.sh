#!/usr/bin/env bash
set -euo pipefail

# Deploy backend code to the RPI and restart the service.
# Usage:
#   RPI_HOST=192.168.1.28 RPI_USER=pi ./embedded/RPI/deploy.sh



RPI_HOST="${RPI_HOST:?RPI_HOST is required}"
RPI_USER="${RPI_USER:-pi}"
RPI_PATH="${RPI_PATH:-/home/pi/mower-roboto}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

rsync -avz --delete \
  --exclude 'node_modules' \
  --exclude 'dist' \
  --exclude 'backend/venv' \
  "${REPO_ROOT}/backend/" \
  "${RPI_USER}@${RPI_HOST}:${RPI_PATH}/backend/"

ssh "${RPI_USER}@${RPI_HOST}" "sudo systemctl restart mower-backend.service"
