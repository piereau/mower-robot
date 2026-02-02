# Deployment

Scripts and configuration for deploying mower-roboto to the Raspberry Pi.

## Quick Start

```bash
# Set your RPi IP address
export RPI_HOST=herbobot.local # 192.168.1.28

# Deploy backend (rsync mode - fast)
./deploy/rpi.sh

# Deploy with dependency installation
./deploy/rpi.sh rsync --install-deps

# Deploy via git pull (production-like)
./deploy/rpi.sh git
```

## Initial RPi Setup

### 1. Clone the repo (one-time)

```bash
# On the RPi
git clone https://github.com/YOUR_USER/mower-roboto.git /home/pi/mower-roboto
cd /home/pi/mower-roboto
```

### 2. Setup Python backend

```bash
cd /home/pi/mower-roboto/backend
python3 -m venv venv
./venv/bin/pip install -r requirements.txt

# Create .env file
cat > .env << EOF
USE_MOCK_CAMERA=false
USE_MOCK_SERIAL=false
SERIAL_PORT=/dev/ttyUSB0
EOF
```

### 3. Install systemd service

```bash
sudo cp /home/pi/mower-roboto/deploy/services/mower-backend.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable mower-backend.service
sudo systemctl start mower-backend.service
```

### 4. Verify

```bash
# Check service status
sudo systemctl status mower-backend.service

# View logs
sudo journalctl -u mower-backend.service -f
```

## RPi Directory Structure

After deployment, the RPi will have:

```
/home/pi/mower-roboto/
├── backend/          # FastAPI app (from apps/backend/)
│   ├── app/
│   ├── venv/         # Python virtualenv (created on RPi)
│   ├── requirements.txt
│   └── .env          # RPi-specific config (not synced)
├── shared/           # Shared types (from shared/)
└── ros2/             # ROS 2 workspace (future, from apps/ros2/)
```

## Useful Commands

```bash
# Restart service
ssh pi@$RPI_HOST "sudo systemctl restart mower-backend.service"

# View live logs
ssh pi@$RPI_HOST "sudo journalctl -u mower-backend.service -f"

# Check service status
ssh pi@$RPI_HOST "sudo systemctl status mower-backend.service"

# Stop service
ssh pi@$RPI_HOST "sudo systemctl stop mower-backend.service"
```

## Camera Setup (RPi Cam 2.1)

```bash
# Install camera tools
sudo apt update && sudo apt install -y libcamera-apps

# Test camera
rpicam-hello

# Enable in backend
# Edit /home/pi/mower-roboto/backend/.env:
# USE_MOCK_CAMERA=false
```
