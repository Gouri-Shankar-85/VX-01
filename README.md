# VX-01 Multi-Terrain Robot

A modular multi-terrain robotic platform combining the agility of a hexapod crawler with the aerial capabilities of a multirotor drone. The system is designed for autonomous and manual navigation across land and air, powered by a distributed compute architecture using ROS 2 and ArduPilot.

<p align="center">
  <img src="vx01_robot.jpg" alt="VX-01 Multi-Terrain Robot" width="800"/>
</p>

---

## 🌟 Key Features

- **Hexapod Locomotion:** Real-time 18-DOF inverse kinematics (IK) executing synchronized Tripod and Wave gaits.
- **Aerial Flight:** Fully integrated drone flight (STABILIZE, GUIDED, AUTO) utilizing a MAVROS bridge.
- **Hybrid Mode Management:** Safe state machine transitioning the robot from ground crawling to aerodynamic flight postures.
- **Visual SLAM & Odometry:** RTAB-Map integration using depth camera data for 3D point cloud generation and obstacle avoidance.
- **Computer Vision:** Real-time object detection, victim identification, and terrain walkability classification using YOLOv4-tiny.
- **Mission Control Dashboard:** A sleek, React-based web UI communicating over `rosbridge_server` for live telemetry, manual teleop, and mapping visualization.

---

## 🛠️ Hardware Stack

| Component | Description |
|-----------|-------------|
| **RDK X5 (ARM64)** | High-level compute board running Ubuntu 22.04 + ROS 2 Humble (Dockerized). Handles IK, SLAM, Vision, and the Web Dashboard. |
| **Radiolink PIX6 (ArduPilot)** | Flight controller connected via USB (MAVLink). Manages aerial stabilization, IMU processing, and GPS. |
| **Pololu Maestro Mini** | Dedicated serial servo controller actuating the 18 high-torque hexapod leg servos. |
| **YDLIDAR HP60C RGBD** | Depth Camera mounted on the front chassis for perception and mapping. |
| **Benewake TFmini-S** | Downward-facing LiDAR distance sensor for precise altitude holding. |
| **Hiwonder IM10A** | External IMU for redundant odometry. |

---

## 📦 Workspace Architecture

The software is strictly modularized into distinct ROS 2 packages within the `vx01_ws/src` directory:

```
vx01_ws/src
├── vx01_bringup          # Master hardware/sim launch files and QoS relays
├── vx01_camera           # Driver for the YDLIDAR depth camera
├── vx01_control          # Locomotion meta-package (Aerial, Aquatic, Hexapod IK, Mode Manager)
├── vx01_description      # Robot URDF, Xacro kinematics, and STL 3D meshes
├── vx01_hardware         # ros2_control interfaces for the Pololu Maestro and Drone GPIOs
├── vx01_imu              # External I2C/Serial IMU drivers
├── vx01_mapping          # RTAB-Map visual SLAM integration
├── vx01_mavros_bridge    # C++ nodes bridging ArduPilot/MAVROS telemetry to custom topics
├── vx01_msgs             # Custom ROS 2 interfaces (DroneState, HexapodState, Thrust)
├── vx01_perception       # OpenCV + YOLOv4-tiny stack for victim and terrain detection
└── vx01_simulation       # Gazebo SITL physics environments
```
*(Refer to the individual `README.md` inside each package for detailed node, topic, and build instructions.)*

---

## 🚀 Quick Start Guide

This project is fully containerized. Please refer to the [VX01 Docker & Deployment Manual](VX01_Docker_Manual.md) in the root directory for complete, in-depth documentation.

### 1. Simulation on Laptop
Run the simulation stack (Gazebo + ROS 2 Bridge + Dashboard) on your computer without needing the physical hardware:

```bash
cd ~/vx-01
# Start the simulation profile containers in the background
docker compose --profile sim up -d

# Open the Gazebo simulation window from inside the container
docker exec -it vx01-sim bash
ros2 launch vx01_simulation sim.launch.py
```
Open a browser and go to `http://localhost:5173` to control the simulated robot!

### 2. Daily Code Development
To edit code and test it without affecting the real robot:

```bash
cd ~/vx-01
# Start the development container
docker compose --profile dev up -d
docker exec -it vx01-dev bash

# Once inside the container, build the workspace:
cd /vx01_ws
colcon build --symlink-install
source install/setup.bash
```
*Note: Your `~/vx-01/vx01_ws/src/` folder is mounted live into the container. Any code changes you make on your laptop instantly appear inside!*

### 3. Deploying to Real Hardware (RDK X5 / Raspberry Pi)
To update the physical robot with your latest code:

```bash
# On your laptop, trigger the ARM64 cloud build:
./build.sh build-arm-ci
# (Wait for the GitHub Action to finish building the image)

# SSH into your robot and pull the latest image:
docker pull gourishankar85/vx01-base:humble-arm64
docker compose -f ~/vx01/docker-compose.rdk.yml down
docker compose -f ~/vx01/docker-compose.rdk.yml up -d
```

### Accessing the Dashboard
Whenever the `sim` or hardware containers are running, you can access the Mission Control Dashboard:
- **Locally (Simulation):** `http://localhost:5173`
- **Real Robot:** `http://<ROBOT_IP>:5173`

---

## 🤝 Contributing
Pull requests are welcome. When adding new ROS 2 nodes, ensure they are properly integrated into the `vx01_bringup` launch files and that relevant `ros2_control` hardware parameters are updated.
