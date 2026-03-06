# vx01_camera

ROS2 driver package for the YDLidar HP60C RGBD depth camera.

**Platform:** RDK X5 · Ubuntu 22.04 · ROS2 Humble  
**Part of:** VX-01 Hybrid Search and Rescue Robot

---

## Package Location

```
VX-01/
└── vx01_ws/
    └── src/
        └── vx01_camera/
```

---

## Build

From the VX-01 workspace root:

```bash
colcon build --packages-select vx01_camera
source install/setup.bash
```

---

## udev Rules (one-time, on host)

Required to access the camera without root permissions.

```bash
./build.sh udev-install
```

---

## Launch

```bash
ros2 launch vx01_camera vx01_camera.launch.py
```

---

## Published Topics

| Topic | Type |
|-------|------|
| `/vx01_camera/depth/image_raw` | sensor_msgs/Image |
| `/vx01_camera/color/image_raw` | sensor_msgs/Image |
| `/vx01_camera/depth/points` | sensor_msgs/PointCloud2 |
| `/vx01_camera/camera_info` | sensor_msgs/CameraInfo |

---

## Camera Specs

| Parameter | Value |
|-----------|-------|
| Depth resolution | 640 x 480 |
| RGB resolution | 640 x 480 |
| Frame rate | 15 fps |
| Depth range | 0.2 - 4m |
| Interface | USB 2.0 Type-C |

---

## Configuration

Config file used: `configurationfiles/hp60c_v2_00_20230704_configEncrypt.json`

Do not modify or replace this file.

---

## Troubleshooting

**Camera not detected:**
```bash
lsusb
v4l2-ctl --list-devices
```

**Node crashes on launch - library not found:**  
Rebuild the Docker image. The SDK libraries are baked in during the Docker build.

**Permission denied on /dev/video0:**  
Run `./build.sh udev-install` on the host and replug the camera.