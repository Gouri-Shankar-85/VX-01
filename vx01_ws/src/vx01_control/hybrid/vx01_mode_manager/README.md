# vx01_mode_manager

Safe state machine for transitioning the robot between terrestrial walking and aerial flight modes.

---

## Package Structure

```
vx01_mode_manager/
├── CMakeLists.txt
├── include
│   └── vx01_mode_manager
├── package.xml
├── README.md
└── src
    └── mode_manager_node.cpp

3 directories, 4 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_mode_manager
source install/setup.bash
```
