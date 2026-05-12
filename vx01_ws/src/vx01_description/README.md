# vx01_description

Robot modeling, containing URDF, Xacro, and STL meshes for the entire VX-01 structure.

---

## Package Structure

```
vx01_description/
├── CMakeLists.txt
├── config
│   └── rviz
├── launch
│   └── vx01_rviz.launch.py
├── meshes
│   ├── base
│   │   └── base_link.STL
│   ├── drone
│   │   ├── arm_link_0.STL
│   │   ├── arm_link_1.STL
│   │   ├── arm_link_2.STL
│   │   ├── arm_link_3.STL
│   │   ├── motor_link_0.STL
│   │   ├── motor_link_1.STL
│   │   ├── motor_link_2.STL
│   │   ├── motor_link_3.STL
│   │   ├── prop_link_0.STL
│   │   ├── prop_link_1.STL
│   │   ├── prop_link_2.STL
│   │   └── prop_link_3.STL
│   ├── hexapod
│   │   ├── coxa_link_0.STL
│   │   ├── coxa_link_1.STL
│   │   ├── coxa_link_2.STL
│   │   ├── coxa_link_3.STL
│   │   ├── coxa_link_4.STL
│   │   ├── coxa_link_5.STL
│   │   ├── femur_link_0.STL
│   │   ├── femur_link_1.STL
│   │   ├── femur_link_2.STL
│   │   ├── femur_link_3.STL
│   │   ├── femur_link_4.STL
│   │   ├── femur_link_5.STL
│   │   ├── tibia_link_0.STL
│   │   ├── tibia_link_1.STL
│   │   ├── tibia_link_2.STL
│   │   ├── tibia_link_3.STL
│   │   ├── tibia_link_4.STL
│   │   └── tibia_link_5.STL
│   └── sensors
│       ├── depth_camera_link.STL
│       ├── imu_link.STL
│       └── lidar_link.STL
├── package.xml
├── README.md
└── urdf
    ├── base
    │   └── base.xacro
    ├── control
    │   └── vx01.ros2_control.xacro
    ├── drone
    │   ├── arm_0.xacro
    │   ├── arm_1.xacro
    │   ├── arm_2.xacro
    │   ├── arm_3.xacro
    │   └── control
    │       └── drone.ros2_control.xacro
    ├── gazebo
    │   └── vx01_sim.xacro
    ├── hexapod
    │   ├── control
    │   │   └── hexapod.ros2_control.xacro
    │   ├── leg_0.xacro
    │   ├── leg_1.xacro
    │   ├── leg_2.xacro
    │   ├── leg_3.xacro
    │   ├── leg_4.xacro
    │   └── leg_5.xacro
    ├── sensors
    │   ├── depth_camera.xacro
    │   ├── imu.xacro
    │   ├── tf_lidar.xacro
    │   └── tf_mini.xacro
    └── vx01.urdf.xacro

17 directories, 58 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_description
source install/setup.bash
```

---

## Launch

```bash
ros2 launch vx01_description display.launch.py
```
