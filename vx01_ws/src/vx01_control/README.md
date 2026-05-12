# vx01_control

Meta-package containing aerial, aquatic, terrestrial locomotion, and mode management nodes.

---

## Package Structure

```
vx01_control/
├── aerial
│   ├── README.md
│   └── vx01_aerial_control
│       ├── include
│       │   └── vx01_aerial_control
│       ├── package.xml
│       ├── README.md
│       ├── resource
│       │   └── vx01_aerial_control
│       ├── setup.cfg
│       ├── setup.py
│       ├── src
│       │   └── aerial_controller_node.cpp
│       └── vx01_aerial_control
│           ├── aerial_controller_node.py
│           ├── drone_flight_manager.py
│           ├── drone_teleop.py
│           └── __init__.py
├── aquatic
│   ├── README.md
│   └── vx01_aquatic_control
│       ├── CMakeLists.txt
│       ├── include
│       │   └── vx01_aquatic_control
│       ├── package.xml
│       ├── README.md
│       └── src
├── hybrid
│   ├── README.md
│   └── vx01_mode_manager
│       ├── CMakeLists.txt
│       ├── include
│       │   └── vx01_mode_manager
│       ├── package.xml
│       ├── README.md
│       └── src
│           └── mode_manager_node.cpp
├── locomotion
│   ├── kinematics_reference
│   │   ├── bezier_curve.JPG
│   │   ├── hexapod.JPG
│   │   └── leg.JPG
│   ├── README.md
│   └── vx01_hexapod_locomotion
│       ├── config
│       │   └── hexapod.yaml
│       ├── launch
│       │   ├── hexapod.launch.py
│       │   └── hexapod_standalone.launch.py
│       ├── package.xml
│       ├── README.md
│       ├── resource
│       │   └── vx01_hexapod_locomotion
│       ├── setup.cfg
│       ├── setup.py
│       └── vx01_hexapod_locomotion
│           ├── gait.py
│           ├── hexapod_node.py
│           ├── __init__.py
│           ├── kinematics.py
│           └── teleop_node.py
└── README.md

24 directories, 38 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_control
source install/setup.bash
```
