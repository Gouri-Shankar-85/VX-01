# vx01_bringup

The `vx01_bringup` package is the entry point for the VX-01 robot. It contains the primary launch files required to start the hardware interfaces, spawn controllers, and initialize the MAVROS bridge.

## Launch Files
- `vx01_hw_launch.py`: The master launch file for real hardware. It launches the `ros2_control` node, the hexapod controllers, MAVROS, QoS relays, and the state publisher.

## Scripts
- `qos_relay.py`: Converts MAVROS Best-Effort QoS topics to Reliable QoS, ensuring compatibility with `rosbridge_server` and the web dashboard.
- `mission_coordinator.py`: High-level state machine handling the drone's autonomous flight sequence.
