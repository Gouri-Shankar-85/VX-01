# VX-01 Multi-Terrain Robot

A highly advanced, modular, multi-terrain robotic platform that seamlessly combines the agility of a hexapod crawler with the aerial capabilities of a multirotor drone. The system is designed for autonomous and manual navigation across land and air, powered by a distributed compute architecture using ROS 2 and ArduPilot.

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
| **Pixhawk (ArduPilot)** | Flight controller connected via USB (MAVLink). Manages aerial stabilization, IMU processing, and GPS. |
| **Pololu Maestro Mini** | Dedicated serial servo controller actuating the 18 high-torque hexapod leg servos. |
| **Orbbec Astra** | RGB-D Depth Camera mounted on the front chassis for perception and mapping. |
| **TF-Mini LiDAR** | Downward-facing distance sensor for precise altitude holding. |

---

## 📦 Workspace Architecture

The software is strictly modularized into distinct ROS 2 packages within the `vx01_ws/src` directory:

```
vx01_ws/src
├── vx01_bringup          # Master hardware/sim launch files and QoS relays
├── vx01_camera           # Driver for the Orbbec Astra depth camera
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

## 🚀 Installation & Deployment

The entire software stack is heavily containerized for the RDK X5 ARM64 architecture, ensuring no host OS dependency conflicts.

### 1. Build the Docker Stack
Use the provided build script to generate the ROS 2 Base, Bridge, and Dashboard images:
```bash
./build.sh build-base
./build.sh build-dashboard
```

### 2. Launch the Hardware Stack
Bring up the background containers (rosbridge, MAVROS, web dashboard) via Docker Compose:
```bash
docker compose -f docker-compose.rdk.yml up -d
```

Attach to the active ROS container and launch the hardware interfaces:
```bash
docker exec -it vx01-robot bash
colcon build --symlink-install
ros2 launch vx01_bringup vx01_hw_launch.py
```

### 3. Access the Mission Control Dashboard
Open a browser on any device on the same network:
`http://<robot-ip>:5173`

---

## 🤝 Contributing
Pull requests are welcome. When adding new ROS 2 nodes, ensure they are properly integrated into the `vx01_bringup` launch files and that relevant `ros2_control` hardware parameters are updated.
