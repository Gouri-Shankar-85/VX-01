# vx01_hardware

Contains the `ros2_control` hardware interfaces bridging ROS 2 to the physical actuators.

## `vx01_hexapod_hardware`
Implements a `SystemInterface` for the Pololu Maestro Mini servo controller.
- Communicates via `/dev/ttyMAESTRO` (serial).
- Translates position commands from the ROS 2 Joint Trajectory Controllers into Maestro-specific PWM serial packets.
- Handles smooth interpolation and serial port recovery.
