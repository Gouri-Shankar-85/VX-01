# vx01_hexapod_hardware

Custom `ros2_control` hardware interface connecting the Pololu Maestro Mini servo controller over serial UART.

---

## Package Structure

```
vx01_hexapod_hardware/
├── CMakeLists.txt
├── hexapod_hardware_plugin.xml
├── include
│   └── vx01_hexapod_hardware
│       ├── communication
│       │   ├── maestro_protocol.hpp
│       │   └── serial_interface.hpp
│       ├── hexapod_hardware_interface.hpp
│       └── servo
│           ├── servo_config.hpp
│           └── servo_controller.hpp
├── package.xml
├── README.md
└── src
    ├── communication
    │   ├── maestro_protocol.cpp
    │   └── serial_interface.cpp
    ├── hexapod_hardware_interface.cpp
    └── servo
        ├── servo_config.cpp
        └── servo_controller.cpp

7 directories, 14 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_hexapod_hardware
source install/setup.bash
```
