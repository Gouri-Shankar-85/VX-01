# VX-01 Perception

ROS 2 perception package for the VX-01 robot. Handles victim detection, terrain classification, environmental mapping, and mission and mode management.

---

## Package Structure

```
vx01_perception/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── topics.yaml
├── models/
│   ├── coco.names
│   ├── yolov4-tiny.cfg
│   └── yolov4-tiny.weights
└── scripts/
    ├── victim_detector_node.py
    ├── terrain_classifier_node.py
    ├── mapping_node.py
    ├── mission_manager_node.py
    └── mode_manager_node.py
```

---

## Nodes

### victim_detector_node

Detects persons in the scene using YOLOv4-Tiny and estimates their 3D position using the depth image.

**Subscribes to:**
| Topic | Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB camera stream |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth camera stream |
| `/mission_state` | `vx01_msgs/MissionState` | Current mission phase |

**Publishes to:**
| Topic | Type | Description |
|---|---|---|
| `/victim_detections` | `vx01_msgs/VictimArray` | Detected victims with position estimates |

**Notes:**
- Detection is only active when `mission_phase` is `SEARCHING` or `VICTIM_FOUND`.
- Position is estimated in the `camera_depth_optical_frame` using the depth value at the bounding box centre.
- Requires `yolov4-tiny.weights`, `yolov4-tiny.cfg`, and `coco.names` in the `models/` directory.

---

### terrain_classifier_node

Analyses the depth image to classify terrain type and compute a walkability score.

**Subscribes to:**
| Topic | Type | Description |
|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth camera stream |
| `/mission_state` | `vx01_msgs/MissionState` | Current mission phase |

**Publishes to:**
| Topic | Type | Description |
|---|---|---|
| `/terrain_type` | `vx01_msgs/Terrain` | Classified terrain type and slope |
| `/walkability_score` | `vx01_msgs/Walkability` | Walkability score and boolean flag |

**Terrain types:** `WALKABLE`, `OBSTACLE`, `DEBRIS`, `STAIRS`, `GAP`

---

### mapping_node

Passes through point cloud data and subscribes to camera and IMU streams for future mapping integration.

**Subscribes to:**
| Topic | Type | Description |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB camera stream |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth camera stream |
| `/camera/points` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/imu` | `sensor_msgs/Imu` | IMU data |

**Publishes to:**
| Topic | Type | Description |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid map |
| `/odom` | `nav_msgs/Odometry` | Odometry estimate |
| `/point_cloud_map` | `sensor_msgs/PointCloud2` | Accumulated point cloud |

---

### mission_manager_node

Manages the overall mission state machine, responding to operator commands, victim detections, and battery status.

**Subscribes to:**
| Topic | Type | Description |
|---|---|---|
| `/system_ready` | `std_msgs/Bool` | System readiness flag |
| `/operator_command` | `std_msgs/String` | Operator commands (`START`, `ABORT`, `RESET`) |
| `/victim_detections` | `vx01_msgs/VictimArray` | Incoming victim detections |
| `/battery_state` | `sensor_msgs/BatteryState` | Battery level |
| `/navigation_status` | `std_msgs/String` | Navigation status |

**Publishes to:**
| Topic | Type | Description |
|---|---|---|
| `/mission_state` | `vx01_msgs/MissionState` | Current mission phase (1 Hz) |

**Mission phases:** `IDLE` → `SEARCHING` → `VICTIM_FOUND` → `RETURNING` → `COMPLETE`

---

### mode_manager_node

Decides the robot's locomotion mode based on terrain walkability and mission state.

**Subscribes to:**
| Topic | Type | Description |
|---|---|---|
| `/terrain_type` | `vx01_msgs/Terrain` | Current terrain classification |
| `/walkability_score` | `vx01_msgs/Walkability` | Terrain walkability |
| `/mission_state` | `vx01_msgs/MissionState` | Current mission phase |

**Publishes to:**
| Topic | Type | Description |
|---|---|---|
| `/robot_mode` | `vx01_msgs/RobotMode` | Selected locomotion mode (2 Hz) |

**Modes:** `HEXAPOD` (walkable terrain), `DRONE` (non-walkable terrain)

---

## Dependencies

- ROS 2 Humble
- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `std_msgs`
- `cv_bridge`
- `opencv-python` (system install via ROS 2)
- `vx01_msgs`
- `numpy < 2.0` (required for compatibility with ROS 2 Humble's `cv_bridge`)

---

## Building

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_perception
source install/setup.bash
```

---

## Running

Each node is launched individually via `ros2 run`:

```bash
ros2 run vx01_perception victim_detector_node
ros2 run vx01_perception terrain_classifier_node
ros2 run vx01_perception mapping_node
ros2 run vx01_perception mission_manager_node
ros2 run vx01_perception mode_manager_node
```

All nodes load topic names from `config/topics.yaml` at startup.

---

## Configuration

Topic names are centralised in `config/topics.yaml`. Edit this file to remap any topic without modifying node source code.

---

## Notes

- YOLOv4-Tiny model files must be present in the `models/` directory before building. The build system installs them alongside the node executables.

