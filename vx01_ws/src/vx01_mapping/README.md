# vx01_mapping

Visual SLAM and dense 3D map generation using RTAB-Map and depth data.

---

## Package Structure

```
vx01_mapping/
├── CMakeLists.txt
├── launch
│   └── vx01_mapping.launch.py
├── package.xml
└── README.md

1 directory, 4 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_mapping
source install/setup.bash
```

---

## Launch

```bash
ros2 launch vx01_mapping vx01_mapping.launch.py
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 2D map for navigation |
| `/odom` | `nav_msgs/Odometry` | Visual odometry estimate |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 3D point cloud of environment |
