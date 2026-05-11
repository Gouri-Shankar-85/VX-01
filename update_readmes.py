#!/usr/bin/env python3

import os

readmes = {
    "/home/devan/vx-01/README.md": """# VX-01 Multi-Terrain Robot

![VX-01 Architecture](https://img.shields.io/badge/Platform-Multi--Terrain-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg?logo=ros)
![Status](https://img.shields.io/badge/Status-Active_Development-brightgreen.svg)

The **VX-01** is a highly advanced, modular, multi-terrain robotic platform designed for autonomous and manual navigation across land, air, and water. By combining the agility of a hexapod with the flight capabilities of a drone, the VX-01 adapts to virtually any environment.

## 🌟 System Architecture

The core architecture operates on a distributed compute model:
- **High-Level Compute:** **RDK X5** (ARM64) running Ubuntu 22.04 + ROS 2 Humble inside Docker containers. Handles SLAM, inverse kinematics, vision, web dashboard, and high-level mission planning.
- **Flight Controller:** **Pixhawk (ArduPilot)** connected via USB (MAVLink/MAVROS). Manages aerial stabilization, GPS, IMU, and autonomous flight modes.
- **Servo Controller:** **Pololu Maestro Mini**. Receives commands from the RDK X5 via `ros2_control` to actuate the 18 hexapod leg servos.
- **Perception:** **Orbbec Astra** Depth Camera for 3D mapping and obstacle avoidance (RTAB-Map).

## 🚀 Key Features

* **Terrestrial Locomotion:** Hexapod gait generation (Tripod/Wave) with real-time inverse kinematics.
* **Aerial Flight:** Full MAVROS integration for STABILIZE and GUIDED autonomous flight.
* **Web Dashboard:** A sleek, React-based UI communicating via `rosbridge_server` for manual teleop, mapping visualization, and telemetry monitoring.
* **Containerized Deployment:** Entire software stack is split into robust Docker containers (Base, ROS Bridge, MAVROS, Dashboard) managed by `docker-compose`.

## 📦 Workspace Structure

The ROS 2 workspace (`vx01_ws/src`) is heavily modularized:
* `vx01_bringup`: Core launch files and system initialization.
* `vx01_control`: Gait generation, mode management, and locomotion scripts.
* `vx01_description`: URDF, Xacro, and 3D meshes.
* `vx01_hardware`: `ros2_control` hardware interfaces for the Maestro controller.
* `vx01_mapping`: RTAB-Map SLAM integration for the depth camera.
* `vx01_mavros_bridge`: Custom C++ bridging for Pixhawk telemetry and command handling.

## 🛠️ Quick Start

### 1. Build the Docker Images
Use the included cross-compilation and build scripts:
```bash
# Build the base and ROS bridge images
./build.sh build-base

# Build the Web Dashboard
./build.sh build-dashboard
```

### 2. Launch the Hardware Stack
```bash
# Start all background containers (ROS, MAVROS, Dashboard)
docker compose -f docker-compose.rdk.yml up -d

# Drop into the ROS container to launch the hardware
docker exec -it vx01-robot bash
colcon build --symlink-install
ros2 launch vx01_bringup vx01_hw_launch.py
```

### 3. Access the Dashboard
Open a browser and navigate to `http://<robot-ip>:5173` to access the Mission Control Dashboard.

## 🤝 Contributing
Contributions are welcome. Please ensure that all new ROS 2 nodes include proper launch file integration and parameter definitions.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_bringup/README.md": """# vx01_bringup

The `vx01_bringup` package is the entry point for the VX-01 robot. It contains the primary launch files required to start the hardware interfaces, spawn controllers, and initialize the MAVROS bridge.

## Launch Files
- `vx01_hw_launch.py`: The master launch file for real hardware. It launches the `ros2_control` node, the hexapod controllers, MAVROS, QoS relays, and the state publisher.

## Scripts
- `qos_relay.py`: Converts MAVROS Best-Effort QoS topics to Reliable QoS, ensuring compatibility with `rosbridge_server` and the web dashboard.
- `mission_coordinator.py`: High-level state machine handling the drone's autonomous flight sequence.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_control/README.md": """# vx01_control

Meta-package containing the various control systems for the VX-01's multi-modal locomotion.

## Sub-Packages
- **`vx01_hexapod_locomotion`**: Contains the inverse kinematics (IK) engine and gait generators (Tripod, Ripple, Wave). Maps desired twist commands to joint angles.
- **`vx01_aerial_control`**: High-level control loops for drone flight maneuvers.
- **`vx01_aquatic_control`**: Future aquatic propulsion mapping.
- **`vx01_mode_manager`**: Handles the transition logic between crawling, flying, and swimming states.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_description/README.md": """# vx01_description

Contains the visual and physical properties of the VX-01 robot.

## Contents
- **URDF / Xacro**: Defines the kinematic tree, joints (coxa, femur, tibia), and sensor frames (camera, IMU, tfmini).
- **Meshes**: STL/DAE files for visual rendering in RViz and Gazebo.
- **SRDF**: Semantic descriptions for MoveIt integration (if applicable).
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_hardware/README.md": """# vx01_hardware

Contains the `ros2_control` hardware interfaces bridging ROS 2 to the physical actuators.

## `vx01_hexapod_hardware`
Implements a `SystemInterface` for the Pololu Maestro Mini servo controller.
- Communicates via `/dev/ttyMAESTRO` (serial).
- Translates position commands from the ROS 2 Joint Trajectory Controllers into Maestro-specific PWM serial packets.
- Handles smooth interpolation and serial port recovery.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_mapping/README.md": """# vx01_mapping

Handles 3D Environment Mapping and SLAM (Simultaneous Localization and Mapping).

## Core Integration
- **RTAB-Map**: Uses RGB-D data from the Orbbec Astra camera combined with Pixhawk odometry to generate dense 3D point clouds and 2D occupancy grids.
- Exposes map data over `/map` for the Web Dashboard visualization.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_mavros_bridge/README.md": """# vx01_mavros_bridge

A custom C++ bridge package that interfaces deeply with MAVROS to expose simplified topics for the rest of the ROS 2 ecosystem.

## Features
- Aggregates MAVROS state, battery, and GPS into a single `vx01_msgs/DroneState`.
- Safely forwards normalized thrust and attitude commands from the control nodes to MAVROS override topics.
- Handles coordinate frame transformations between ROS (ENU) and ArduPilot.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_msgs/README.md": """# vx01_msgs

Defines all custom ROS 2 message and service types used internally by the VX-01 ecosystem.

## Messages
- `DroneState.msg`: Aggregated telemetry (Altitude, Battery, Mode, GPS Lock).
- `DroneThrust.msg`: Four-axis unified thrust commands.
- `HexapodState.msg`: Current gait, stance height, and leg contact states.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_camera/README.md": """# vx01_camera

Handles driver initialization for the robot's vision sensors.

## Nodes
- **`ascamera_node`**: Driver for the Orbbec Astra Depth Camera. Publishes aligned depth images, RGB streams, and point clouds used by the mapping and perception stacks.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_perception/README.md": """# vx01_perception

Vision processing and object detection algorithms. Consumes raw RGB/Depth streams from `vx01_camera` and outputs actionable bounding boxes or spatial markers for the locomotion and mapping nodes.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_simulation/README.md": """# vx01_simulation

Gazebo Classic / Gazebo Ignition simulation environments for the VX-01.

- Includes plugins for simulated Pololu actuators and MAVROS SITL.
- Allows full software-in-the-loop (SITL) testing of gaits and flight modes before hardware deployment.
""",

    "/home/devan/vx-01/vx01_ws/src/vx01_imu/README.md": """# vx01_imu

Drivers and filtering nodes for external IMUs. (Note: Primary flight IMU data is handled internally by the Pixhawk via MAVROS, this package is for auxiliary sensors like the BNO085 if used for redundant odometry).
"""
}

def main():
    for filepath, content in readmes.items():
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'w') as f:
            f.write(content)
        print(f"Updated {filepath}")

if __name__ == "__main__":
    main()
