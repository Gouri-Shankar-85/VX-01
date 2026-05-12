# vx01_bringup

Master initialization package for the VX-01 hardware and simulation.

---

## Package Structure

```
vx01_bringup/
├── CMakeLists.txt
├── config
│   ├── ardupilot_sim_bypass.param
│   ├── drone
│   │   ├── drone_controller_manager.yaml
│   │   ├── drone_initial_positions.yaml
│   │   └── drone_joint_limits.yaml
│   ├── hexapod
│   │   ├── hexapod_controller_manager_hw.yaml
│   │   ├── hexapod_initial_positions.yaml
│   │   ├── hexapod_joint_limits.yaml
│   │   └── servo_mapping.yaml
│   ├── mavros
│   │   ├── apm_pluginlists.yaml
│   │   └── mavros_params.yaml
│   ├── mavros_sim_config.yaml
│   ├── rtabmap_sim_params.yaml
│   ├── sensors
│   │   ├── camera_params.yaml
│   │   ├── imu_params.yaml
│   │   ├── lidar_params.yaml
│   │   ├── nav2_costmap_params.yaml
│   │   ├── object_detection.yaml
│   │   ├── rtabmap_params.yaml
│   │   └── sensor_fusion.yaml
│   └── vx01_controller_manager.yaml
├── launch
│   ├── vx01_hw_launch.py
│   ├── vx01_hybrid_sim.launch.py
│   └── vx01.launch.py
├── package.xml
├── README.md
└── scripts
    ├── hexapod_teleop_key.py
    ├── mission_coordinator.py
    ├── qos_relay.py
    ├── test_single_servo.py
    └── walk_test.cpp

7 directories, 31 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_bringup
source install/setup.bash
```

---

## Launch

```bash
ros2 launch vx01_bringup vx01_hw_launch.py
```
