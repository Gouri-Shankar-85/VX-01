#!/bin/bash
set -e
REGISTRY="${1:?Usage: ./deploy-rdk.sh YOUR_DOCKERHUB_USERNAME}"
HOME_DIR="/home/pi"

echo "VX-01 Raspberry Pi 5 Setup"
echo "Registry : ${REGISTRY}"

ARCH=$(uname -m)
if [ "${ARCH}" != "aarch64" ]; then
    echo "ERROR: This script is for ARM64. Detected: ${ARCH}"
    exit 1
fi

if ! command -v docker &>/dev/null; then
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com | sudo sh
    sudo usermod -aG docker "${USER}"
    echo "Docker installed. Log out and back in, then run this script again."
    exit 0
fi

if ! docker compose version &>/dev/null; then
    sudo apt-get update && sudo apt-get install -y docker-compose-plugin
fi

echo "cgroup check: $(cat /sys/fs/cgroup/cgroup.controllers 2>/dev/null || echo 'cgroup v1')"

echo "Installing udev rules..."
sudo cp "$(dirname "$0")/docker/udev/99-vx01-devices.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,plugdev,video "${USER}" 2>/dev/null || true
echo "udev rules installed"

mkdir -p "${HOME_DIR}/vx01/vx01_ws"

cat > "${HOME_DIR}/vx01/.env" << EOF
DOCKER_REGISTRY=${REGISTRY}
EOF

cat > "${HOME_DIR}/vx01/docker-compose.rdk.yml" << EOF
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
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    volumes:
      - /dev:/dev
      - ${HOME_DIR}/vx01/vx01_ws:/vx01_ws
      - vx01_logs:/vx01_ws/log
      - /home/pi/.ssh:/root/.ssh:ro
      - /home/pi/.gitconfig:/root/.gitconfig:ro
    devices:
      - /dev/ttyPIXHAWK:/dev/ttyPIXHAWK
      - /dev/ttyMAESTRO:/dev/ttyMAESTRO
      - /dev/ttyTFMINI:/dev/ttyTFMINI
      - /dev/ttyIMU:/dev/ttyIMU
      - /dev/video0:/dev/video0
      - /dev/video1:/dev/video1
    command: tail -f /dev/null

  vx01-mavros:
    image: ${REGISTRY}/vx01-base:humble-arm64
    container_name: vx01-mavros
    network_mode: host
    privileged: true
    ipc: host
    restart: unless-stopped
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    devices:
      - /dev/ttyPIXHAWK:/dev/ttyPIXHAWK
    command: tail -f /dev/null

  vx01-rosbridge:
    image: ${REGISTRY}/vx01-base:humble-arm64
    container_name: vx01-rosbridge
    network_mode: host
    ipc: host
    restart: unless-stopped
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    command: >
      bash -c "source /opt/ros/humble/setup.bash &&
               ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090"

  vx01-dashboard:
    image: ${REGISTRY}/vx01-dashboard:latest
    container_name: vx01-dashboard
    restart: unless-stopped
    ports:
      - "5173:5173"

  vx01-dashboard-backend:
    image: ${REGISTRY}/vx01-dashboard-backend:latest
    container_name: vx01-dashboard-backend
    restart: unless-stopped
    network_mode: host
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
    ports:
      - "3001:3001"

volumes:
  vx01_logs:
EOF

echo "Pulling images..."
docker pull ${REGISTRY}/vx01-base:humble-arm64
docker pull ${REGISTRY}/vx01-dashboard:latest
docker pull ${REGISTRY}/vx01-dashboard-backend:latest

echo "Starting containers..."
cd "${HOME_DIR}/vx01"
docker compose -f docker-compose.rdk.yml up -d

echo ""
echo "Done. All containers are running."
echo ""
echo "  Dashboard:   http://$(hostname -I | awk '{print $1}'):5173"
echo "  Rosbridge:   ws://$(hostname -I | awk '{print $1}'):9090"
echo "  Robot shell: docker exec -it vx01-robot bash"
echo "  Logs:        docker logs -f vx01-robot"
echo "  Stop:        docker compose -f ${HOME_DIR}/vx01/docker-compose.rdk.yml down"