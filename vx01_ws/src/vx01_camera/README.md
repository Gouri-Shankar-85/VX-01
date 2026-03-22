# VX-01 Camera

ROS2 driver package for the YDLidar HP60C RGBD depth camera.

---

## Package Structure

```
vx01_camera/
├── CMakeLists.txt
├── configurationfiles
│   ├── hp60cn_v2_00_20230704_configEncrypt.json
│   ├── hp60c_v2_00_20230704_configEncrypt.json
│   └── readme.md
├── include
│   ├── ascamera_node.h
│   ├── Camera.h
│   ├── CameraPublisher.h
│   ├── CameraSrv.h
│   ├── LogRedirectBuffer.h
│   └── TfTreeFrameIdInfo.h
├── launch
│   ├── ascamera.launch.py
│   ├── hp60cn.launch.py
│   └── vx01_camera.launch.py
├── libs
│   ├── include
│   │   ├── as_camera_sdk_api.h
│   │   ├── as_camera_sdk_def.h
│   │   ├── common.h
│   │   └── Logger.h
│   └── lib
│       ├── aarch64-linux-gnu
│       ├── arm-linux-gnueabihf
│       └── x86_64-linux-gnu
├── package.xml
├── README.md
├── scripts
│   ├── angstrong-camera.rules
│   ├── create_udev_rules.sh
│   └── gettarget.sh
└── src
    ├── ascamera_node.cpp
    ├── Camera.cpp
    ├── CameraPublisher.cpp
    ├── CameraSrv.cpp
    └── TfTreeFrameIdInfo.cpp

```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
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
| `/vx01_camera/camera_publisher/depth0/image_raw` | `sensor_msgs/Image` |
| `/vx01_camera/camera_publisher/depth0/camera_info` | `sensor_msgs/CameraInfo` |
| `/vx01_camera/camera_publisher/depth0/points` | `sensor_msgs/PointCloud2` |
| `/vx01_camera/camera_publisher/rgb0/image` | `sensor_msgs/Image` |
| `/vx01_camera/camera_publisher/rgb0/camera_info` | `sensor_msgs/CameraInfo` |

---

## Camera Specs

| Parameter | Value |
|-----------|-------|
| Depth resolution | 640 x 480 |
| RGB resolution | 640 x 480 |
| Frame rate | 30 fps |
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