# Black Robot — Teleoperation System

Skid-steer UGV (Unmanned Ground Vehicle) controlled via the Optimus GCS platform.
Uses a 6-stage steering algorithm, CAN motor control, and the Optimus ROS2 teleoperation stack.

---

## Network Topology

| Device | Role | IP | SSH credentials |
|---|---|---|---|
| **Jetson AGX Orin** | Robot onboard computer | `192.168.120.20` | `nvidia` / `Q1w2as34` |
| **Optimus GCS** | Operator control station | `192.168.120.169` | `oper` / `Q1w2as34` |

> Connect your laptop to the same network subnet (`192.168.120.x`).

---

## System Architecture (Current)

```
┌────────────────────────────────────────────────────────────────────────┐
│  Optimus GCS  (192.168.120.169)                                         │
│                                                                         │
│  Native Optimus Software                                                │
│    Operator: D-Pad = N/F/R gear   Joystick = throttle/steer            │
│    ↓  ROS2  (domain=1,  /vehicle_120/drive_control)                    │
└─────────────────────────────────────────┬──────────────────────────────┘
                                          │ ROS2 (DDS, domain 1)
┌─────────────────────────────────────────▼──────────────────────────────┐
│  Jetson AGX Orin  (192.168.120.20)                                      │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  Docker: onboard_optimus  (ProntoController + ROS2 onboard)     │   │
│  │    ↓ subscribes to drive_control, gear_control                  │   │
│  │    ↓                                                            │   │
│  │  ros2_drive_bridge.py  ────── UDP port 8888 ──────────────────► │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  main.py  (host, ~/gg_vehicle/)                                 │   │
│  │    udp_receiver.py  ◄── port 8888                               │   │
│  │    steering.py      (6-stage skid-steer algorithm)              │   │
│  │    can_transmitter.py ──── CAN bus (can0, 500kbps) ──────────── ┤   │
│  │    can_telemetry.py   ◄─── CAN bus                              │   │
│  │    heartbeat_emitter.py ── UDP port 5005 ──────────────────────►│   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  Motor Controllers ◄──── CAN (0x302 front, 0x304 rear)                 │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Startup Sequence

Three processes must run, in this order:

### 1. Jetson — main.py (host)

```bash
ssh nvidia@192.168.120.20
# password: Q1w2as34

cd ~/gg_vehicle
python3 main.py
```

Expected output:
```
[UDP] Listening for Steam Deck on port 8888...
[CAN] Bus initialised: can0 @ 500000 bps
[HB EMIT] Heartbeat → 192.168.120.169:5005 at 5 Hz
[READY] All systems go.
```

### 2. Jetson — ROS2 Drive Bridge (inside Docker container)

Open a second SSH session to the Jetson:

```bash
ssh nvidia@192.168.120.20
# password: Q1w2as34

docker exec -it onboard_optimus bash

# Inside the container:
export ROS_DOMAIN_ID=1
python3 /ros2_drive_bridge.py
```

Expected output:
```
Bridge started — /vehicle_120/drive_control + /vehicle_120/gear_control → UDP 127.0.0.1:8888  speed=30%
```

Optional — adjust speed limit (D-pad also adjusts live):
```bash
python3 /ros2_drive_bridge.py --speed 0.50   # 50% max speed
```

### 3. Optimus GCS — already running

The Optimus GCS software starts automatically. The `onboard_optimus` Docker container also starts automatically on the Jetson at boot.

---

## Controls (Gamepad on Optimus GCS)

| Input | Action |
|---|---|
| **D-Pad Up** | Gear → **Forward** |
| **D-Pad Down** | Gear → **Reverse** |
| **D-Pad Centre** | Gear → **Neutral** (stops) |
| **Throttle axis** | Speed (proportional) |
| **Steering axis** | Left / Right |
| **D-Pad Left/Right** | Speed cap −5% / +5% (default 30%) |

> ⚠️ Robot will **not move** while in Neutral gear regardless of throttle input.

---

## Telemetry (Heartbeat → GCS)

`main.py` sends a heartbeat packet at 5 Hz to the GCS on port `5005`.
The Optimus `vehicle_gui.py` (if running) will show:

| HUD field | Source |
|---|---|
| CONNECTED (green dot) | Heartbeat received |
| Voltage | CAN power telemetry (`shared_state.battery_voltage`) |
| GEAR: LOW/HIGH | `shared_state.gear_low` |
| THR / FNT bars | Echo of last received PWM values |

---

## Project Structure

```
black_robot/
├── README.md                       ← This file
├── docs/
│   ├── ARCHITECTURE.md             ← System design & 6-stage steering spec
│   ├── MEASUREMENTS.md             ← Physical robot measurements
│   ├── CAN Command V0.1.pdf        ← Motor controller CAN protocol
│   └── skid_steer_spec.docx.pdf    ← Steering algorithm specification
├── jetson/                         ← Runs on the Jetson (host, not Docker)
│   ├── main.py                     ← Entry point — orchestrates all subsystems
│   ├── heartbeat_emitter.py        ← Sends status packets to Optimus GCS (port 5005)
│   ├── ros2_drive_bridge.py        ← Runs INSIDE onboard_optimus container
│   ├── steering.py                 ← 6-stage skid-steer algorithm
│   ├── can_transmitter.py          ← Sends DriveCommand CAN frames
│   ├── can_telemetry.py            ← Reads speed/temp/power CAN telemetry
│   ├── udp_receiver.py             ← Receives joystick commands via UDP :8888
│   ├── shared_state.py             ← Thread-safe state container
│   ├── robot_config.py             ← All tunable parameters & network config
│   └── robot-logic.service         ← Systemd service file for main.py
├── from_steam_deck/gg_vehicle/     ← Alternative: standalone PyQt5 GUI controller
│   ├── vehicle_gui.py              ← Full camera + HUD dashboard (PyQt5)
│   ├── vehicle_logic.py            ← VehicleController, GamepadManager
│   └── setup_vehicle_gui.sh        ← Dependency installer
├── reference/
│   └── vehicle_controller.py       ← Legacy monolithic controller (archived)
└── tools/
    ├── measure_vmax.py             ← V_max measurement (robot lifted)
    ├── can_diag.py                 ← CAN bus diagnostics
    └── steering_test.py            ← Automated steering test
```

---

## Key Configuration (robot_config.py)

```python
OPTIMUS_GCS_IP  = "192.168.120.169"   # Heartbeat destination
HEARTBEAT_PORT  = 5005                 # GCS listens here
HEARTBEAT_HZ    = 5

V_MAX           = 2.0                  # m/s at full throttle
TRACK_WIDTH_M   = 0.50                 # wheel-centre to wheel-centre
WHEEL_RADIUS_M  = 0.125
```

To change the speed limit permanently, edit `ros2_drive_bridge.py`:
```python
_DEFAULT_SPEED = 0.30   # 30% → change to 0.50 for 50%
```
Or pass `--speed 0.50` on launch.

---

## Deployment (copy files to Jetson)

```bash
# From your laptop, inside this repo:
sshpass -p 'Q1w2as34' scp jetson/main.py jetson/heartbeat_emitter.py \
    jetson/robot_config.py jetson/ros2_drive_bridge.py \
    nvidia@192.168.120.20:/home/nvidia/gg_vehicle/

# Copy bridge into the running Docker container:
sshpass -p 'Q1w2as34' ssh nvidia@192.168.120.20 \
    "docker cp /home/nvidia/gg_vehicle/ros2_drive_bridge.py onboard_optimus:/"
```

---

## CAN Bus

| Parameter | Value |
|---|---|
| Interface | `can0` |
| Bitrate | `500 kbps` |
| Byte order | **Big-Endian** (Motorola) |
| Front driver NodeID | `2` → CAN IDs: `0x302`, `0x312`, `0x2D2`, `0x322` |
| Rear driver NodeID | `4` → CAN IDs: `0x304`, `0x314`, `0x2D4`, `0x324` |
| DriveCommand frame | `0x300 + NodeID` — throttle L/R ±1000, mode byte, limit |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Robot doesn't move, throttle=0 in log | Gear is Neutral | Switch to Forward (D-Pad Up) |
| `[STEER] Input D:+0.00` — no UDP | Bridge not running | Start `ros2_drive_bridge.py` inside container |
| main.py CAN error on start | `can0` not up | `sudo ip link set can0 up type can bitrate 500000` |
| GCS shows OFFLINE | Heartbeat not reaching GCS | Check firewall, confirm main.py running |
| Motors stutter or lock | Fault level ≥ 3 | Check `[CAN FRONT POWER]` Fault field; power-cycle motors |
| `MotorL: 102.0°C` | Thermistor open circuit | Hardware fault; sensor disconnected inside motor |

---

## Systemd Service (Jetson)

`main.py` can run automatically on boot:

```bash
sudo cp ~/gg_vehicle/robot-logic.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot-logic.service
sudo systemctl start robot-logic.service
```

Daily commands:
```bash
sudo systemctl status robot-logic
sudo journalctl -u robot-logic -f      # live logs
sudo systemctl restart robot-logic
```

> ⚠️ The `ros2_drive_bridge.py` inside the Docker container is **not** auto-started yet.
> Add it to the `onboard_optimus` Docker entrypoint or create a separate systemd service if needed.
