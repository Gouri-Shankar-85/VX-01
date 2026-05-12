# VX-01 Docker Infrastructure — User Manual

**Hybrid Amphibious Search & Rescue Robot**  
ROS 2 Humble · Ignition Fortress 6 · RDK X5 / Raspberry Pi 5 ARM64 · Laptop AMD64

---

## Part 1 — Project Structure

Your repository lives at `~/vx-01/` on your laptop. This is the full layout after all files are in place:

```text
~/vx-01/
├── .env                     # Contains DOCKER_REGISTRY=gourishankar85
├── build.sh                 # Centralized script for all build commands
├── deploy-rdk.sh            # Raspberry Pi 5 / RDK X5 first-time setup script
├── docker-compose.yml       # Laptop compose file (dev, sim, hardware profiles)
├── docker-compose.rdk.yml   # Robot-side compose file (runs without profiles)
├── docker/                  # Dockerfiles and entrypoints
│   ├── Dockerfile.base      # ROS 2 Humble hardware image
│   ├── Dockerfile.sim       # Extends base + Gazebo/RViz
│   ├── Dockerfile.dashboard # React dashboard image
│   ├── entrypoint.sh        # Container startup script
│   └── udev/                # Hardware symlink rules
│       └── 99-vx01-devices.rules
├── vx01_dashboard/          # React + Vite + Tailwind frontend
└── vx01_ws/
    └── src/                 # Your ROS 2 packages
```

---

## Part 2 — Containers Overview

### Laptop (`docker-compose.yml`)
Containers are started using specific profiles:

| Container | Profile | Image | Purpose |
|-----------|---------|-------|---------|
| `vx01-dev` | `dev` | `vx01-sim:humble` | Interactive shell with all tools + simulation |
| `vx01-hardware` | `hardware` | `vx01-base:humble` | Connect to real sensors on laptop |
| `vx01-sim` | `sim` | `vx01-sim:humble` | Gazebo Fortress simulation |
| `vx01-rosbridge` | `dev`/`hw`/`sim` | `vx01-base:humble` | WebSocket bridge for dashboard (port 9090) |
| `vx01-dashboard` | `dev`/`hw`/`sim` | `vx01-dashboard:latest` | React dashboard served on port 5173 |

### Robot / RDK X5 (`docker-compose.rdk.yml`)
All containers run automatically (restart unless-stopped) without profiles:

| Container | Image | Purpose |
|-----------|-------|---------|
| `vx01-robot` | `vx01-base:humble-arm64` | Main robot container — idle, enter with `exec` |
| `vx01-mavros` | `vx01-base:humble-arm64` | ArduPilot bridge — idle, start MAVROS inside |
| `vx01-rosbridge` | `vx01-base:humble-arm64` | rosbridge WebSocket on port 9090 |
| `vx01-dashboard`| `vx01-dashboard:latest` | Dashboard served on port 5173 |

---

## Part 3 — One-Time Laptop Setup

### 3.1 Requirements
Ubuntu 22.04, Docker Engine 24+, `docker buildx`. Check with:
```bash
docker --version
docker compose version
docker buildx version
```
If Docker is missing:
```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
# Log out and back in after this
```

### 3.2 Install udev Device Rules
Creates stable symlinks for all VX-01 devices:
```bash
cd ~/vx-01
chmod +x build.sh
./build.sh udev-install
# Log out and back in after this
```

### 3.3 Allow Docker to Open GUI Windows
Add this to your `~/.bashrc` so it runs on every login:
```bash
echo 'xhost +local:docker' >> ~/.bashrc
source ~/.bashrc
```

### 3.4 SSH for GitHub
No setup needed. The compose files mount your `~/.ssh` and `~/.gitconfig` read-only into every container. `git push` works inside any container automatically using your host SSH key.

---

## Part 4 — Building Images

### 4.1 Build the Hardware Image (AMD64)
```bash
cd ~/vx-01
./build.sh build-base
```
*First build: 20-40 min. Subsequent builds are fast (Docker cache).*

### 4.2 Build the Simulation Image (AMD64 only)
```bash
./build.sh build-sim
```

### 4.3 Build the Dashboard Image (AMD64 → Docker Hub)
```bash
./build.sh build-dashboard
```
*Builds the React frontend and pushes to Docker Hub. The robot pulls this image natively.*

### 4.4 Build ARM64 Image for Robot
Recommended — uses GitHub Actions (native ARM64 runner):
```bash
./build.sh build-arm-ci
```
*Monitor: https://github.com/Gouri-Shankar-85/VX-01/actions. Wait for green before deploying.*

Alternative — local cross-compile via QEMU (may be slow):
```bash
./build.sh build-arm
```

### 4.5 Push All AMD64 Images to Docker Hub
```bash
./build.sh push
```

### 4.6 When to Rebuild?
| Situation | Rebuild? | Command |
|-----------|----------|---------|
| Changed `.cpp` or `.py` in `vx01_ws/src/` | No | `colcon build` inside container |
| Added a new ROS 2 package to `src/` | No | `colcon build` inside container |
| Added an `apt` package to `Dockerfile.base` | Yes | `./build.sh build-base` then `build-arm-ci` |
| Changed `entrypoint.sh` | Yes | `./build.sh build-base` then `build-arm-ci` |
| Changed the dashboard (React files) | Yes | `./build.sh build-dashboard` |
| Nothing changed, starting work today | No | Just start containers |

---

## Part 5 — Daily Laptop Workflow

### 5.1 Start Dev Container
```bash
cd ~/vx-01
docker compose --profile dev up -d
docker exec -it vx01-dev bash
```
*Your `src/` folder is mounted live. Edit on laptop, build inside container.*

### 5.2 Build ROS 2 Workspace (inside container)
```bash
cd /vx01_ws
colcon build --symlink-install
source install/setup.bash
```

### 5.3 Run Simulation
```bash
# Start sim profile (includes rosbridge + dashboard):
docker compose --profile sim up -d

# Or launch Gazebo from inside a running dev container:
ros2 launch vx01_simulation sim.launch.py
```

### 5.4 Run Real Hardware on Laptop
```bash
docker compose --profile hardware up -d
docker exec -it vx01-hardware bash
```

### 5.5 Git Push from Inside Container
```bash
cd /vx01_ws
git add src/your_package/
git commit -m "your message"
git push
```

### 5.6 Stop Everything
```bash
docker compose --profile dev down
docker compose --profile sim down
docker compose --profile hardware down
```

---

## Part 6 — Raspberry Pi 5 / RDK X5 Setup

### 6.1 First-Time Setup (run once)
Copy the deploy script to the robot and run it (assuming user `pi`, IP `<robot-ip>`):
```bash
# On your laptop:
scp deploy-rdk.sh pi@<robot-ip>:/home/pi/
scp -r docker/udev pi@<robot-ip>:/home/pi/docker/

# SSH into the robot:
ssh pi@<robot-ip>

# On the robot:
chmod +x deploy-rdk.sh
./deploy-rdk.sh gourishankar85
```
*This script installs Docker, udev rules, pulls the base and dashboard images, and starts all containers automatically on every reboot.*

### 6.2 Access Dashboard
Open in a browser on any device on the same network:
```text
http://<robot-ip>:5173
```

### 6.3 Daily Robot Commands
| Goal | Command |
|------|---------|
| Check all containers | `docker ps` |
| Open robot shell | `docker exec -it vx01-robot bash` |
| See robot logs | `docker logs -f vx01-robot` |
| Check devices | `ls -la /dev/ttyPIXHAWK /dev/ttyMAESTRO /dev/video0` |
| Restart container | `docker restart vx01-robot` |
| Stop all containers | `docker compose -f ~/vx01/docker-compose.rdk.yml down` |
| Start all containers| `docker compose -f ~/vx01/docker-compose.rdk.yml up -d` |

### 6.4 Update Robot After Image Change
```bash
# Base image update:
docker pull gourishankar85/vx01-base:humble-arm64
docker compose -f ~/vx01/docker-compose.rdk.yml down
docker compose -f ~/vx01/docker-compose.rdk.yml up -d

# Dashboard update only:
docker pull gourishankar85/vx01-dashboard:latest
docker restart vx01-dashboard
```

---

## Part 7 — Devices and Stable Port Names

udev rules create fixed symlinks based on USB vendor/product ID. Plug order does not matter.

| Device | Symlink | Protocol | Baud |
|--------|---------|----------|------|
| Radiolink PIX6 (ArduPilot) | `/dev/ttyPIXHAWK` | USB ACM | 921600 |
| Pololu Maestro 18ch | `/dev/ttyMAESTRO` | USB HID + Serial | — |
| Benewake TFmini-S LiDAR | `/dev/ttyTFMINI` | UART | 115200 |
| Hiwonder IM10A IMU | `/dev/ttyIMU` | UART | 9600 |
| YDLIDAR HP60C Camera | `/dev/video0` | USB UVC | — |

---

## Part 8 — Git Workflow

### 8.1 Normal Code Change (No Rebuild)
Edit files in `~/vx-01/vx01_ws/src/`, jump into the `vx01-dev` container, run `colcon build`, and test.

### 8.2 Dockerfile Changed
```bash
# On laptop:
git commit -am "update dockerfile" && git push
./build.sh build-base
./build.sh build-arm-ci

# On robot:
docker pull gourishankar85/vx01-base:humble-arm64
docker compose -f ~/vx01/docker-compose.rdk.yml down
docker compose -f ~/vx01/docker-compose.rdk.yml up -d
```

### 8.3 Dashboard Changed
```bash
# On laptop:
git commit -am "update dashboard" && git push
./build.sh build-dashboard

# On robot:
docker pull gourishankar85/vx01-dashboard:latest
docker restart vx01-dashboard
```

---

## Part 9 — Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `exec format error` on robot | Wrong arch image (AMD64 on ARM64) | Wait for `build-arm-ci` to finish, pull again |
| Container keeps restarting | Windows line endings (CRLF) in entrypoint | `dos2unix docker/entrypoint.sh`, rebuild |
| `/dev/ttyTFMINI` not found | udev rules not installed | Run `./build.sh udev-install` on host |
| Permission denied on `/dev/tty*` | Not in dialout group | `sudo usermod -aG dialout $USER`, log out |
| MAVROS connected: False | Wrong device or baud | Check `/dev/ttyPIXHAWK` exists, baud 921600 |
| Dashboard shows ROS Disconnected | rosbridge not running | `docker logs -f vx01-rosbridge` |
| Gazebo/RViz not opening | X11 not allowed | `xhost +local:docker` on host |
| colcon build: missing package | rosdep not run | `rosdep install --from-paths src --ignore-src -r -y` |

---

## Part 10 — Quick Reference

**One-Time Laptop Setup**
```bash
cd ~/vx-01
./build.sh udev-install
echo 'xhost +local:docker' >> ~/.bashrc && source ~/.bashrc
./build.sh build-base
./build.sh build-sim
./build.sh build-dashboard
```

**Every Day — Laptop**
```bash
cd ~/vx-01
docker compose --profile dev up -d
docker exec -it vx01-dev bash
# inside container: cd /vx01_ws && colcon build --symlink-install
```
