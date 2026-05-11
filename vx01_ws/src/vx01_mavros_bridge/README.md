# vx01_mavros_bridge

A custom C++ bridge package that interfaces deeply with MAVROS to expose simplified topics for the rest of the ROS 2 ecosystem.

## Features
- Aggregates MAVROS state, battery, and GPS into a single `vx01_msgs/DroneState`.
- Safely forwards normalized thrust and attitude commands from the control nodes to MAVROS override topics.
- Handles coordinate frame transformations between ROS (ENU) and ArduPilot.
