# vx01_hexapod_locomotion

Computes inverse kinematics (IK) and gait sequences to drive the 18-servo hexapod chassis.

---

## Package Structure

```
vx01_hexapod_locomotion/
├── config
│   └── hexapod.yaml
├── launch
│   ├── hexapod.launch.py
│   └── hexapod_standalone.launch.py
├── package.xml
├── README.md
├── resource
│   └── vx01_hexapod_locomotion
├── setup.cfg
├── setup.py
└── vx01_hexapod_locomotion
    ├── gait.py
    ├── hexapod_node.py
    ├── __init__.py
    ├── kinematics.py
    └── teleop_node.py

4 directories, 13 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_hexapod_locomotion
source install/setup.bash
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Target twist command |
| `/leg_X_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Trajectory command to specific leg |
