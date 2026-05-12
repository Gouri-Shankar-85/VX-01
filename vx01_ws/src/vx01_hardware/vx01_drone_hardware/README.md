# vx01_drone_hardware

Custom `ros2_control` hardware interface for handling GPIO and PWM signals for drone components.

---

## Package Structure

```
vx01_drone_hardware/
├── CMakeLists.txt
├── drone_hardware_plugin.xml
├── include
│   └── vx01_drone_hardware
│       ├── drone_hardware_interface.hpp
│       ├── gpio
│       │   ├── gpio_interface.hpp
│       │   └── pwm_controller.hpp
│       └── servo
│           ├── arm_servo_config.hpp
│           └── arm_servo_controller.hpp
├── package.xml
├── README.md
└── src
    ├── drone_hardware_interface.cpp
    ├── gpio
    │   ├── gpio_interface.cpp
    │   └── pwm_controller.cpp
    └── servo
        ├── arm_servo_config.cpp
        └── arm_servo_controller.cpp

7 directories, 14 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_drone_hardware
source install/setup.bash
```
