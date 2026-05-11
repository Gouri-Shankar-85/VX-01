# vx01_control

Meta-package containing the various control systems for the VX-01's multi-modal locomotion.

## Sub-Packages
- **`vx01_hexapod_locomotion`**: Contains the inverse kinematics (IK) engine and gait generators (Tripod, Ripple, Wave). Maps desired twist commands to joint angles.
- **`vx01_aerial_control`**: High-level control loops for drone flight maneuvers.
- **`vx01_aquatic_control`**: Future aquatic propulsion mapping.
- **`vx01_mode_manager`**: Handles the transition logic between crawling, flying, and swimming states.
