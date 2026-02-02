# Mower-Roboto Project Context

> **IMPORTANT FOR AI AGENTS:** Read this file first when starting any development session!

## Deployment

### 🚀 RPi Deploy Script

**Always use the deploy script to push code to the Raspberry Pi:**

```bash
# Deploy backend only
RPI_HOST=herbobot.local ./deploy/rpi.sh

# Deploy backend + ROS 2 packages (builds on RPi)
RPI_HOST=herbobot.local ./deploy/rpi.sh --ros2

# Install Python dependencies after adding new ones
RPI_HOST=herbobot.local ./deploy/rpi.sh --install-deps

# Git pull mode (for production-like deploys)
RPI_HOST=herbobot.local ./deploy/rpi.sh git
```

The script handles:
- SSH connection multiplexing (single password prompt)
- rsync with proper excludes
- ROS 2 workspace build with colcon
- systemd service installation and restart
- Status reporting

### Systemd Services

| Service | Description | Location |
|---------|-------------|----------|
| `mower-backend.service` | FastAPI backend | `deploy/services/` |
| `mower-ros2-bridge.service` | Serial bridge node | `deploy/services/` |
| `mower-teleop-bridge.service` | Teleop WebSocket bridge | `deploy/services/` |

## Architecture Overview

### Communication Flow (Teleoperation)

```
Dashboard (Browser)
    ↓ WebSocket
FastAPI Backend (RPi)
    ↓ Unix Socket (/tmp/mower_ros_bridge.sock)
ROS 2 ws_bridge node (RPi)
    ↓ /cmd_vel topic
ROS 2 serial_bridge node (RPi)
    ↓ USB Serial
Arduino Mega
    ↓ Motor commands
Motors
```

## Key Paths

| Component | Path |
|-----------|------|
| Backend | `apps/backend/` |
| Frontend | `apps/frontend/` |
| ROS 2 packages | `apps/ros2/src/` |
| Deploy script | `deploy/rpi.sh` |
| Systemd services | `deploy/services/` |
| Stories | `_bmad-output/implementation-artifacts/` |

## Testing Commands

```bash
# Monitor ROS 2 topics (on RPi)
ros2 topic echo /cmd_vel
ros2 topic echo /mower/status

# Check service status
sudo systemctl status mower-backend
sudo systemctl status mower-ros2-bridge
sudo systemctl status mower-teleop-bridge

# View logs
sudo journalctl -u mower-backend -f
sudo journalctl -u mower-ros2-bridge -f
```
