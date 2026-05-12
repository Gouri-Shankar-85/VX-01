# vx01_perception

Computer vision stack utilizing YOLOv4-tiny for target/victim detection and terrain classification.

---

## Package Structure

```
vx01_perception/
├── CMakeLists.txt
├── config
│   └── topics.yaml
├── models
│   ├── coco.names
│   ├── yolov4-tiny.cfg
│   └── yolov4-tiny.weights
├── package.xml
├── README.md
├── scripts
│   ├── mapping_node.py
│   ├── mission_manager_node.py
│   ├── mode_manager_node.py
│   ├── terrain_classifier_node.py
│   └── victim_detector_node.py
└── vx01_perception
    └── __init__.py

4 directories, 13 files
```

---

## Build

From the VX-01 workspace root:

```bash
cd ~/vx01_ws
colcon build --packages-select vx01_perception
source install/setup.bash
```
