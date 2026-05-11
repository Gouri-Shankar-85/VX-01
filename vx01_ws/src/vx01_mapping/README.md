# vx01_mapping

Handles 3D Environment Mapping and SLAM (Simultaneous Localization and Mapping).

## Core Integration
- **RTAB-Map**: Uses RGB-D data from the Orbbec Astra camera combined with Pixhawk odometry to generate dense 3D point clouds and 2D occupancy grids.
- Exposes map data over `/map` for the Web Dashboard visualization.
