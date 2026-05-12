# vx01_msgs

Defines custom ROS 2 messages, services, and actions for the VX-01 platform.

---

## Package Structure

```
vx01_msgs/
├── action
│   ├── Dive.action
│   └── Navigate.action
├── CMakeLists.txt
├── msg
│   ├── DroneState.msg
│   ├── Exploration.msg
│   ├── FlightStatus.msg
│   ├── HexapodState.msg
│   ├── HexapodStatus.msg
│   ├── HybridMode.msg
│   ├── MissionState.msg
│   ├── MotorThrust.msg
│   ├── RobotMode.msg
│   ├── Terrain.msg
│   ├── UnderwaterState.msg
│   ├── VictimArray.msg
│   ├── Victim.msg
│   └── Walkability.msg
├── package.xml
├── README.md
└── srv
    ├── Calibrate.srv
    └── SwitchMode.srv

3 directories, 21 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_msgs
source install/setup.bash
```
