#!/bin/bash
# =============================================================================
# VX-01 RDK X5 Deployment — run this ONCE on the RDK X5
# Usage: ./deploy-rdk.sh YOUR_DOCKERHUB_USERNAME
# =============================================================================

set -e
REGISTRY="${1:?Usage: ./deploy-rdk.sh YOUR_DOCKERHUB_USERNAME}"

echo "VX-01 RDK X5 Setup"
echo "Registry: ${REGISTRY}"

# Install Docker if missing
if ! command -v docker &>/dev/null; then
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com | sudo sh
    sudo usermod -aG docker "${USER}"
    echo "Docker installed. Run this script again after logging out and back in."
    exit 0
fi

# Install docker compose plugin if missing
docker compose version &>/dev/null || \
    sudo apt-get update && sudo apt-get install -y docker-compose-plugin

# Install udev rules
echo "Installing udev rules..."
sudo tee /etc/udev/rules.d/99-vx01-devices.rules > /dev/null << 'RULES'
SUBSYSTEM=="usb",  ATTR{idVendor}=="1ffb", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="tty",  ATTRS{idVendor}=="1ffb", MODE="0666", GROUP="plugdev", SYMLINK+="ttyMAESTRO"
SUBSYSTEM=="tty",  ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="ttyTFMINI"
SUBSYSTEM=="tty",  ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout", SYMLINK+="ttyTFMINI"
SUBSYSTEM=="tty",  ATTRS{idVendor}=="26ac", MODE="0666", GROUP="dialout", SYMLINK+="ttyPIXHAWK"
SUBSYSTEM=="tty",  ATTRS{idVendor}=="1209", MODE="0666", GROUP="dialout", SYMLINK+="ttyPIXHAWK"
SUBSYSTEM=="tty",  KERNEL=="ttyUSB1", MODE="0666", GROUP="dialout", SYMLINK+="ttyIMU"
SUBSYSTEM=="tty",  KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty",  KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
SUBSYSTEM=="video4linux", MODE="0666", GROUP="video"
RULES
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,plugdev,video "${USER}" 2>/dev/null || true
echo "udev rules installed"

# Create docker-compose.rdk.yml on the RDK
mkdir -p /home/root/vx01
cat > /home/root/vx01/docker-compose.rdk.yml << EOF
name: vx01-rdk
services:
  vx01-robot:
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
      - /root/.ssh:/root/.ssh:ro
      - /root/.gitconfig:/root/.gitconfig:ro
    devices:
      - /dev/ttyPIXHAWK:/dev/ttyPIXHAWK
      - /dev/ttyMAESTRO:/dev/ttyMAESTRO
      - /dev/ttyTFMINI:/dev/ttyTFMINI
      - /dev/ttyIMU:/dev/ttyIMU
      - /dev/video0:/dev/video0
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
EOF

# Pull and start
echo "Pulling image..."
docker pull ${REGISTRY}/vx01-base:humble-arm64

echo "Starting containers..."
docker compose -f /home/root/vx01/docker-compose.rdk.yml up -d

echo ""
echo "Done. Robot is running."
echo "  Logs:  docker logs -f vx01-robot"
echo "  Shell: docker exec -it vx01-robot bash"
echo "  Stop:  docker compose -f /home/root/vx01/docker-compose.rdk.yml down"