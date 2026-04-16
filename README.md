# Black Robot — Vehicle Controller

Skid-steer robot control system with a 6-stage steering algorithm, CAN motor control, and UDP joystick input.

## Project Structure

```
black_robot/
├── README.md                  ← This file
├── docs/                      ← Specifications & documentation
│   ├── CAN Command V0.1.pdf   ← Motor controller CAN protocol
│   ├── skid_steer_spec.docx.pdf ← 6-stage steering algorithm spec
│   ├── ARCHITECTURE.md        ← System architecture & design
│   └── MEASUREMENTS.md        ← Physical robot measurements
├── reference/                 ← Legacy / reference code
│   └── vehicle_controller.py  ← Original monolithic controller (archived)
├── jetson/                    ← Code that runs on the Jetson (robot)
│   ├── main.py                ← Entry point — orchestrates all subsystems
│   ├── steering.py            ← 6-stage steering algorithm (pure math)
│   ├── can_transmitter.py     ← Sends DriveCommand CAN frames
│   ├── can_telemetry.py       ← Reads speed/temp/power CAN telemetry
│   ├── udp_receiver.py        ← Receives joystick commands via UDP
│   └── shared_state.py        ← Thread-safe state container
├── laptop/                    ← Code that runs on the operator laptop
│   └── joystick_sim.py        ← Keyboard → UDP joystick simulator
└── tools/                     ← Measurement & calibration scripts
    └── measure_vmax.py        ← V_max measurement (run on robot, lifted)
```

## Quick Start

### On the Jetson (Robot)
```bash
# Copy jetson/ files to ~/gg_vehicle/ on the robot
scp jetson/* nvidia@<robot_ip>:~/gg_vehicle/

# SSH into the robot and run
ssh nvidia@<robot_ip>
cd ~/gg_vehicle
python3 main.py
```

### On the Laptop (Operator)
```bash
python3 laptop/joystick_sim.py <robot_ip>
```

### Joystick Simulator Modes
| Mode | Name | Description |
|------|------|-------------|
| 1 | Drive Mode | W/S = drive, A/D = steer (normal driving) |
| 2 | Debug Mode | Independent L/R motor control (W/S + E/D) |
| 3 | Hold Mode | Momentary keys at configurable speed % |

## Communication

```
[Laptop]                    [Jetson/Robot]
joystick_sim.py  ─── UDP 8888 ───►  udp_receiver.py
                                         │
                                    shared_state.py
                                         │
                                    steering.py (6-stage algorithm)
                                         │
                                    can_transmitter.py ─── CAN bus ───► Motor Controllers
                                         │
                                    can_telemetry.py  ◄── CAN bus ──── Motor Controllers
```

## CAN Protocol
- **DriveCommand**: `0x300 + NodeID` — send throttle L/R (Speed Mode)
- **SpeedTelemetry**: `0x310 + NodeID` — read RPM L/R
- **Temperature**: `0x2D0 + NodeID` — motor/MOSFET/CPU temps
- **Power**: `0x320 + NodeID` — voltage, current, faults
- **Byte order**: Big-Endian (Motorola)
- **Front driver NodeID**: 2 (0x302, 0x312, 0x2D2, 0x322)
- **Rear driver NodeID**: 4 (0x304, 0x314, 0x2D4, 0x324)

## Troubleshooting & Hardware Quirks
- **Shattered Gears / Free-spinning Wheel**: If a motor makes a rattling noise and suddenly has ZERO physical resistance when spun by hand (while powered on), a physical connection has failed. Most likely, one of the three thick phase wires is disconnected/loose (causing the controller to lose magnetic holding torque and grind out of sync), or the wheel's internal gears/axle have shattered under load.
- **70% Software Speed Limit**: To prevent hardware breakage caused by huge momentary torques from the 1000/1000 CAN throttle output, `steering.py` implements a `GLOBAL_LIMIT_PCT = 0.70`. This gracefully caps outputs to `700` while preserving turning math.
- **R: 0 RPM CAN Glitch**: The dual-channel motor controllers will occasionally drop the Right-side telemetry for one frame (printing `0 RPM`). This naturally corrects almost immediately and is not a mechanical failure.
- **MotorL 102.0°C**: If `[CAN FRONT TEMP]` constantly reports `MotorL: 102.0°C` the temperature sensor thermistor inside the left motor is an open circuit (unplugged or broken).

## Services

### Jetson — `robot-logic.service`
Runs `main.py` automatically on the Jetson on boot.

**Install (one-time):**
```bash
# Allow nvidia user to run ip link commands without password (needed by main.py)
sudo visudo -f /etc/sudoers.d/nvidia-can
# Add this line:
#   nvidia ALL=(ALL) NOPASSWD: /sbin/ip link set can0 *

# Install the service
sudo cp ~/gg_vehicle/robot-logic.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot-logic.service
sudo systemctl start robot-logic.service
```

**Daily commands:**
```bash
sudo systemctl status robot-logic     # Is it running?
sudo journalctl -u robot-logic -f     # Live log output
sudo systemctl restart robot-logic    # Restart after code changes
sudo systemctl stop robot-logic       # Stop it manually
sudo systemctl disable robot-logic    # Stop auto-starting on boot
```

---

### Steam Deck — `onboard-optimus.service`
Legacy service from the previous developer (Optimus project). Runs a Docker-based vehicle GUI from `/home/deck/optimus_ws/`. Auto-starts on boot and **listens on UDP port 5005** for telemetry from the robot.

> ⚠️ **Conflict:** This service holds port 5005. If you try to run `vehicle_gui.py` manually at the same time, you will get `address already in use`. Stop the service first:
> ```bash
> sudo systemctl stop onboard-optimus.service
> ```

**Current status:** Left enabled (auto-starts on boot). Do NOT run `vehicle_gui.py` while this service is running.

To permanently replace it with the new GUI:
```bash
sudo systemctl disable onboard-optimus.service
sudo systemctl stop onboard-optimus.service
# Then launch vehicle_gui.py manually or create a new service for it
```
