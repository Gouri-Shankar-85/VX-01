# vx01_msgs

Defines all custom ROS 2 message and service types used internally by the VX-01 ecosystem.

## Messages
- `DroneState.msg`: Aggregated telemetry (Altitude, Battery, Mode, GPS Lock).
- `DroneThrust.msg`: Four-axis unified thrust commands.
- `HexapodState.msg`: Current gait, stance height, and leg contact states.
