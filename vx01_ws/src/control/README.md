# VX-01 Control Domain

## Overview

The **control** domain contains all motion execution modules of the VX-01 hybrid robot.

It is responsible for generating movement commands for different mobility modes — land, air, and water — and managing transitions between them.

This layer does not directly control hardware. Instead, it outputs motion commands that are executed by hardware interface drivers.

---

## Mobility Modes

VX-01 supports tri-modal mobility:

- **Land** → Hexapod walking
- **Air** → Quadcopter flight
- **Water** → Aquatic propulsion
- **Hybrid** → Mode transitions & coordination

---

## Directory Structure

```bash
control/
├── locomotion/
│   └── vx01_hexapod_locomotion
│
├── aerial/
│   └── vx01_aerial_control
│
├── aquatic/
│   └── vx01_aquatic_control
│
└── hybrid/
    └── vx01_mode_manager
