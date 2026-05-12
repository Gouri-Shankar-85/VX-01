# vx01_aerial_control

Aerial flight sequence control loop via MAVROS.

---

## Package Structure

```
vx01_aerial_control/
├── include
│   └── vx01_aerial_control
├── package.xml
├── README.md
├── resource
│   └── vx01_aerial_control
├── setup.cfg
├── setup.py
├── src
│   └── aerial_controller_node.cpp
└── vx01_aerial_control
    ├── aerial_controller_node.py
    ├── drone_flight_manager.py
    ├── drone_teleop.py
    └── __init__.py

5 directories, 10 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_aerial_control
source install/setup.bash
```
