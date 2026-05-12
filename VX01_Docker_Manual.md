# VX-01 Docker & Deployment Manual

This manual provides the exact commands needed to develop, simulate, and deploy code for the VX-01 robot using Docker. By using Docker, you avoid installing ROS 2 directly on your host machine and ensure your code runs identically on your laptop and the robot's hardware.

---

## 1. Initial Host Setup (Run Once)

Before using the Docker containers, you must configure your host machine to allow hardware access and GUI windows (for RViz/Gazebo).

### Install udev Device Rules
This creates stable symlinks for all VX-01 hardware devices.
```bash
cd ~/vx-01
chmod +x build.sh
./build.sh udev-install
# Log out and log back into your Ubuntu account for changes to take effect
```

### Allow Docker to Open GUI Windows
Add this to your `~/.bashrc` to allow Gazebo and RViz to render on your screen.
```bash
echo 'xhost +local:docker' >> ~/.bashrc
source ~/.bashrc
```

---

## 2. Daily Development Workflow

When writing code on your laptop, your `vx01_ws/src/` folder is mounted live into the container. Any code changes you make in your host editor are instantly available inside the container to build.

### Start the Dev Container
```bash
cd ~/vx-01
docker compose --profile dev up -d
docker exec -it vx01-dev bash
```

### Build the Workspace
Once inside the container shell:
```bash
cd /vx01_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3. Running the Simulation

You can test your code in Gazebo without the physical robot. The `sim` profile automatically launches the simulation, the ROS bridge, and the web dashboard.

```bash
cd ~/vx-01
docker compose --profile sim up -d

# Drop into the sim container to launch the Gazebo world:
docker exec -it vx01-sim bash
ros2 launch vx01_simulation sim.launch.py
```
**Access the Dashboard:** Open `http://localhost:5173` in your web browser.

---

## 4. Deploying to Real Hardware (RDK X5 / Raspberry Pi)

When your code works in simulation and you are ready to test it on the physical robot.

### Step 1: Build the ARM64 Image (On Laptop)
Push your code to GitHub, then trigger the cross-compilation pipeline:
```bash
git push origin main
./build.sh build-arm-ci
```
*(Wait for the GitHub Actions pipeline to finish successfully).*

### Step 2: Pull and Run (On Robot)
SSH into the robot and pull the updated image from the registry:
```bash
docker pull gourishankar85/vx01-base:humble-arm64
docker compose -f ~/vx01/docker-compose.rdk.yml down
docker compose -f ~/vx01/docker-compose.rdk.yml up -d
```

---

## 5. Hardware Device Symlink Reference

The `udev` rules automatically map hardware to the following stable paths, regardless of which USB port they are plugged into:

| Device | Symlink | Protocol |
|--------|---------|----------|
| Radiolink PIX6 | `/dev/ttyPIXHAWK` | USB ACM (921600 baud) |
| Pololu Maestro Mini | `/dev/ttyMAESTRO` | USB Serial |
| Benewake TFmini-S | `/dev/ttyTFMINI` | UART (115200 baud) |
| Hiwonder IM10A | `/dev/ttyIMU` | UART (9600 baud) |
| YDLIDAR HP60C | `/dev/video0` | USB UVC |

---

## 6. Useful Docker Commands

| Command | Purpose |
|---------|---------|
| `docker compose --profile dev down` | Stop all development containers |
| `docker logs -f vx01-dev` | View real-time container logs |
| `docker restart vx01-dashboard` | Restart the web dashboard |
