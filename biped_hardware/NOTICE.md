# Third-party code

The following files are adapted from **AsterisCrack/BipedRobotJetson** (MIT License,
Copyright (c) 2026 Asteris — full text in `LICENSE`):

| file here | original |
|---|---|
| `biped_hardware/st3215/registers.py` | `hardware/st3215/registers.py` (unchanged) |
| `biped_hardware/st3215/protocol.py`  | `hardware/st3215/protocol.py` (import path changed) |
| `biped_hardware/serial_bus.py`       | `hardware/serial_bus.py` (unchanged) |
| `biped_hardware/bno055.py`           | `hardware/imu/bno055.py` (reads accelerometer incl. gravity, selectable fusion mode) |

The servo IDs, direction signs and PID defaults in `config/servos.yaml` come from that repo's
`config/robot.yaml`.  Everything else in this package was written for this project.
