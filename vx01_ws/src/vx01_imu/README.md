# vx01_imu

Driver and integration for auxiliary I2C/Serial IMU sensors for RTAB-Map odometry.

---

## Package Structure

```
vx01_imu/
├── launch
│   └── vx01_imu.launch.py
├── package.xml
├── README.md
├── resource
│   └── vx01_imu
├── setup.cfg
├── setup.py
└── vx01_imu
    ├── __init__.py
    └── vx01_imu_node.py

3 directories, 8 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_imu
source install/setup.bash
```

---

## Launch

```bash
ros2 launch vx01_imu vx01_imu.launch.py
```
