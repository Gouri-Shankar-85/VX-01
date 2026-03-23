#!/bin/bash
# =============================================================================
# VX-01 Raspberry Pi 5 Deployment — run this ONCE on the Pi 5
# Usage: ./deploy-rdk.sh YOUR_DOCKERHUB_USERNAME
#
# CHANGED FROM RDK X5 VERSION:
#   - Works on Ubuntu 24.04 (Pi 5 default OS)
#   - Home directory is /home/ubuntu instead of /home/sunrise
#   - Docker install command updated for 24.04
#   - Added cgroup v2 check (Pi 5 needs it for Docker)
# =============================================================================

set -e
REGISTRY="${1:?Usage: ./deploy-rdk.sh YOUR_DOCKERHUB_USERNAME}"
HOME_DIR="/home/ubuntu"    # Pi 5 default user — change if yours is different

echo "VX-01 Raspberry Pi 5 Setup"
echo "Registry : ${REGISTRY}"
echo "Home dir : ${HOME_DIR}"
echo ""

# ── Verify we are on a 64-bit ARM system ────────────────────────────────────
ARCH=$(uname -m)
if [ "${ARCH}" != "aarch64" ]; then
    echo "ERROR: This script is for ARM64. Detected: ${ARCH}"
    exit 1
fi

# ── Install Docker if missing ────────────────────────────────────────────────
if ! command -v docker &>/dev/null; then
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com | sudo sh
    sudo usermod -aG docker "${USER}"
    echo ""
    echo "Docker installed. Log out and back in, then run this script again."
    exit 0
fi

# ── Install docker compose plugin if missing ─────────────────────────────────
if ! docker compose version &>/dev/null; then
    sudo apt-get update && sudo apt-get install -y docker-compose-plugin
fi

# ── cgroup v2 check (required for Docker on Pi 5 / Ubuntu 24.04) ─────────────
# Pi 5 with Ubuntu 24.04 uses cgroup v2 by default — Docker works fine with it.
# If you see "cgroup: No such file" errors, uncomment the lines below to add
# the kernel parameters:
#   sudo sed -i 's/$/ cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1/' \
#       /boot/firmware/cmdline.txt
#   echo "Reboot required — run script again after reboot."
#   sudo reboot
echo "cgroup check: $(cat /sys/fs/cgroup/cgroup.controllers 2>/dev/null || echo 'cgroup v1 — OK')"

# ── Install udev rules ───────────────────────────────────────────────────────
echo "Installing udev rules..."
sudo tee /etc/udev/rules.d/99-vx01-devices.rules > /dev/null << 'RULES'
# Pololu Mini Maestro
SUBSYSTEM=="usb",  ATTR{idVendor}=="1ffb", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="tty",  ATTRS{idVendor}=="1ffb", MODE="0666", GROUP="plugdev", SYMLINK+="ttyMAESTRO"
# Benewake TFmini-S (CP2102)
SUBSYSTEM=="tty",  ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="ttyTFMINI"
# Benewake TFmini-S (CH340)
SUBSYSTEM=="tty",  ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout", SYMLINK+="ttyTFMINI"
# Radilink PIX6 / ArduPilot
SUBSYSTEM=="tty",  ATTRS{idVendor}=="26ac", MODE="0666", GROUP="dialout", SYMLINK+="ttyPIXHAWK"
SUBSYSTEM=="tty",  ATTRS{idVendor}=="1209", MODE="0666", GROUP="dialout", SYMLINK+="ttyPIXHAWK"
# Hiwonder IM10A IMU
SUBSYSTEM=="tty",  ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE="0666", GROUP="dialout", SYMLINK+="ttyIMU"
SUBSYSTEM=="tty",  KERNEL=="ttyUSB1", MODE="0666", GROUP="dialout", SYMLINK+="ttyIMU"
# Generic fallbacks
SUBSYSTEM=="tty",  KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty",  KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
SUBSYSTEM=="video4linux", MODE="0666", GROUP="video"
RULES

sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,plugdev,video "${USER}" 2>/dev/null || true
echo "udev rules installed"

# ── Create workspace and docker-compose file ─────────────────────────────────
mkdir -p "${HOME_DIR}/vx01"

cat > "${HOME_DIR}/vx01/docker-compose.rdk.yml" << EOF
name: vx01-rdk

services:
  vx01-robot:
    # This image is built FROM ubuntu:22.04 inside the Dockerfile.
    # The Pi 5 host runs Ubuntu 24.04 but the container is 22.04 — this is
    # intentional so ROS2 Humble works correctly.
    image: ${REGISTRY}/vx01-base:humble-arm64
    container_name: vx01-robot
    network_mode: host
    privileged: true
    ipc: host
    restart: unless-stopped
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - /dev:/dev
      - ${HOME_DIR}/vx01/vx01_ws:/vx01_ws
      - vx01_logs:/vx01_ws/log
      - /root/.ssh:/root/.ssh:ro
      - /root/.gitconfig:/root/.gitconfig:ro
    devices:
      - /dev/ttyPIXHAWK:/dev/ttyPIXHAWK
      - /dev/ttyMAESTRO:/dev/ttyMAESTRO
      - /dev/ttyTFMINI:/dev/ttyTFMINI
      - /dev/ttyIMU:/dev/ttyIMU
      - /dev/video0:/dev/video0
      - /dev/video1:/dev/video1
    command: ros2 launch vx01_hardware hardware.launch.py

  vx01-mavros:
    image: ${REGISTRY}/vx01-base:humble-arm64
    container_name: vx01-mavros
    network_mode: host
    privileged: true
    ipc: host
    restart: unless-stopped
    environment:
      - ROS_DOMAIN_ID=0
    devices:
      - /dev/ttyPIXHAWK:/dev/ttyPIXHAWK
    command: >
      ros2 launch mavros apm.launch
      fcu_url:=serial:///dev/ttyPIXHAWK:921600
      gcs_url:=udp://:14550@0.0.0.0:14555

volumes:
  vx01_logs:
EOF

# ── Pull image and start ──────────────────────────────────────────────────────
echo "Pulling ARM64 image (this can take a while on first run)..."
docker pull ${REGISTRY}/vx01-base:humble-arm64

echo "Starting containers..."
docker compose -f "${HOME_DIR}/vx01/docker-compose.rdk.yml" up -d

echo ""
echo "Done. Robot is running."
echo ""
echo "  Logs:  docker logs -f vx01-robot"
echo "  Shell: docker exec -it vx01-robot bash"
echo "  Stop:  docker compose -f ${HOME_DIR}/vx01/docker-compose.rdk.yml down"
echo ""
echo "NOTE: The container runs Ubuntu 22.04 + ROS2 Humble inside."
echo "      The Pi 5 host is Ubuntu 24.04 — this is expected and correct."