# vx01_simulation

Gazebo environments, physics plugins, and SITL launch configurations for testing without hardware.

---

## Package Structure

```
vx01_simulation/
├── CMakeLists.txt
├── config
│   └── bridge.yaml
├── launch
│   ├── vx01_mapping.launch.py
│   ├── vx01_sim.launch.py
│   └── vx01_world.launch.py
├── package.xml
├── README.md
└── worlds
    └── empty_world.sdf

3 directories, 8 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_simulation
source install/setup.bash
```

---

## Launch

```bash
ros2 launch vx01_simulation vx01_sim.launch.py
```
