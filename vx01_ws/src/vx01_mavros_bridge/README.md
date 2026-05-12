# vx01_mavros_bridge

C++ integration bridge to safely route VX-01 custom topics into native MAVROS protocols.

---

## Package Structure

```
vx01_mavros_bridge/
├── CMakeLists.txt
├── include
│   └── vx01_mavros_bridge
│       ├── flight_mode_manager.hpp
│       ├── mavros_bridge.hpp
│       ├── motor_commander.hpp
│       └── telemetry_handler.hpp
├── package.xml
├── README.md
└── src
    ├── flight_mode_manager.cpp
    ├── mavros_bridge.cpp
    ├── motor_commander.cpp
    └── telemetry_handler.cpp

3 directories, 11 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_mavros_bridge
source install/setup.bash
```

---

## Launch

```bash
ros2 run vx01_mavros_bridge mavros_bridge_node
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vx01/drone/thrust_command` | `vx01_msgs/DroneThrust` | Input custom thrust |
| `/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | Output MAVROS throttle overrides |
| `/vx01/drone/state` | `vx01_msgs/DroneState` | Output aggregated telemetry |
