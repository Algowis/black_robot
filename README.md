# Black Robot — Teleoperation System

Skid-steer UGV (Unmanned Ground Vehicle) controlled via the Optimus GCS platform.
Uses a developer-specified differential steering algorithm, CAN motor control, and the Optimus ROS2 teleoperation stack.

---

## Network Topology

| Device | Role | IP | SSH credentials |
|---|---|---|---|
| **Jetson AGX Orin** | Robot onboard computer | `192.168.120.20` | `nvidia` / `Q1w2as34` |
| **Optimus GCS** | Operator control station | `192.168.120.169` | `oper` / `Q1w2as34` |

> Connect your laptop to the same network subnet (`192.168.120.x`).

---

## System Architecture

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
│  │    ros2_drive_bridge.py  ── UDP port 8888 ──────────────────►   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  main.py  (host, ~/gg_vehicle/)                                 │   │
│  │    udp_receiver.py  ◄── port 8888                               │   │
│  │    steering.py      (differential steering algorithm)           │   │
│  │    can_transmitter.py ──── CAN bus (can0, 500kbps) ──────────── ┤   │
│  │    can_telemetry.py   ◄─── CAN bus                              │   │
│  │    heartbeat_emitter.py ── UDP port 5005 ──────────────────────►│   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  Motor Controllers ◄──── CAN (0x302 front, 0x304 rear)                 │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Auto-Start on Boot

All three processes start **automatically on Jetson boot** via systemd. No manual intervention is needed.

| Service | What it runs | Auto-starts |
|---|---|---|
| `can0-up.service` | Brings `can0` up at 500kbps | ✅ Boot |
| `robot-logic.service` | Runs `main.py` on host (35s delay — waits for container CAN init) | ✅ After `can0-up` |
| `bridge-startup.service` | Copies + runs `ros2_drive_bridge.py --speed 0.20` inside `onboard_optimus` | ✅ After `onboard-optimus` |

> ⚠️ **Important:** `robot-logic` has a 35-second startup delay. This is intentional — the `onboard_optimus` container's startup script (`onboard_startup.sh`) reconfigures `can0` ~20s after boot. Without the delay, `main.py` opens the CAN socket before the container resets it, causing `[Errno 100] Network is down` errors.

### Service Management

```bash
# Check status
sudo systemctl status robot-logic
sudo systemctl status bridge-startup
sudo systemctl status can0-up

# Live logs
sudo journalctl -u robot-logic -f
sudo journalctl -u bridge-startup -f

# Restart manually (e.g. after a code update)
sudo systemctl restart robot-logic
sudo systemctl restart bridge-startup

# Check bridge is running inside container
docker exec onboard_optimus ps aux | grep ros2_drive
docker exec onboard_optimus cat /tmp/bridge_output.txt
```

---

## Manual Startup (if needed)

### 1. Bring up CAN bus

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 2. Start main.py on Jetson host

```bash
ssh nvidia@192.168.120.20   # password: Q1w2as34
cd ~/gg_vehicle
python3 main.py
```

Expected output:
```
[UDP] Listening for Steam Deck on port 8888...
[INIT] can0 up at 500000 bps
[HB EMIT] Heartbeat → 192.168.120.169:5005 at 5 Hz
[READY] All systems go.
```

### 3. Start bridge inside Docker container

```bash
docker exec -it onboard_optimus bash
source /opt/ros/humble/setup.bash
source /backend/install/setup.bash
ROS_DOMAIN_ID=1 python3 /ros2_drive_bridge.py --speed 0.20
```

Expected output:
```
Bridge started — /vehicle_120/drive_control + /vehicle_120/gear_control → UDP 127.0.0.1:8888  speed=20%
```

---

## Deployment (copy files to Jetson)

```bash
# From your laptop, inside this repo:

# Deploy Jetson host files
sshpass -p 'Q1w2as34' scp \
  jetson/main.py jetson/steering.py jetson/can_transmitter.py \
  jetson/can_telemetry.py jetson/udp_receiver.py jetson/shared_state.py \
  jetson/robot_config.py jetson/heartbeat_emitter.py \
  nvidia@192.168.120.20:/home/nvidia/gg_vehicle/

# Deploy bridge (host copy — bridge-startup.service copies it into container on restart)
sshpass -p 'Q1w2as34' scp \
  jetson/ros2_drive_bridge.py \
  nvidia@192.168.120.20:/home/nvidia/gg_vehicle/ros2_drive_bridge.py

# Restart main.py and bridge to pick up changes
sshpass -p 'Q1w2as34' ssh nvidia@192.168.120.20 "
  sudo systemctl restart robot-logic &&
  sudo systemctl restart bridge-startup
"
```

> **Note:** The bridge script is automatically copied from `/home/nvidia/gg_vehicle/ros2_drive_bridge.py` into the container each time `bridge-startup.service` starts. You only need to SCP to the host.

---

## Controls (Gamepad on Optimus GCS)

| Input | Action |
|---|---|
| **D-Pad Up** | Gear → **Forward** |
| **D-Pad Down** | Gear → **Reverse** |
| **D-Pad Centre** | Gear → **Neutral** (stops) |
| **Throttle axis** | Speed (proportional, capped by `--speed`) |
| **Steering axis** | Left / Right differential |
| **D-Pad Left/Right** | Speed cap −5% / +5% (default 20%) |

> ⚠️ Robot will **not move** while in Neutral gear regardless of throttle input.

---

## Steering Algorithm

### Pivot (no throttle)

When throttle = 0 and steer stick is deflected:
- Both tracks spin in opposite directions
- Power = **90%** (`_PIVOT_SCALE = 0.90`), independent of `--speed`

### Arc Turn (throttle + steer)

Developer-specified differential model:

| Parameter | Value |
|---|---|
| Outer wheel | Stays at current drive speed |
| Minimum differential | **200 CAN units** (applied as soon as steer is deflected) |
| Maximum differential | **600 CAN units** (at full stick) |
| Inner wheel floor | Max `−50%` of outer speed (prevents reversal) |

**Example at 20% forward speed (outer = +200 CAN):**
- Slight steer → inner = `0`
- Full steer → inner = `−100` (capped at −50% of +200)

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
| Motor mode | `0x05` = Speed mode, **braking OFF** |
| Torque limit | `700 nM` (Limit field in Speed mode = max torque) |

### DriveCommand Frame Format

```
Byte 0-1: ThrottleLeft   (int16, big-endian)  -1000..+1000
Byte 2-3: ThrottleRight  (int16, big-endian)  -1000..+1000
Byte 4:   MotorMode      (uint8)              0x05 = Speed, brake OFF
Byte 5-6: Limit          (uint16, big-endian) 700 nM max torque
Byte 7:   Reserved       (uint8)              0x00
```

**Example — 20% forward speed, brake OFF:**
```
00 C8 00 C8 05 02 BC 00
│         │         │  └─ Reserved
│         │         └──── 0x02BC = 700 nM
│         └────────────── 0x05 = Speed + Brake OFF
│    00 C8 = Right +200
└──────── 00 C8 = Left +200
```

---

## Key Configuration

### `robot_config.py`

```python
TORQUE_LIMIT = 700        # nM — max torque in Speed mode (Limit field)
V_MAX        = 2.0        # m/s at full throttle (±1000 CAN units)
TRACK_WIDTH  = 0.585      # m — wheel centre to centre
WHEEL_RADIUS = 0.1825     # m — radius
```

### `ros2_drive_bridge.py`

```python
_DEFAULT_SPEED = 0.20     # 20% max drive speed (override with --speed flag)
_PIVOT_SCALE   = 0.90     # 90% power for pivot turns (independent of --speed)
```

### `steering.py`

```python
STEER_MIN_DIFF_CAN = 200  # Min differential to overcome track friction
STEER_MAX_DIFF_CAN = 600  # Max differential at full stick
DEADZONE           = 0.005 # Near-zero threshold for ROS2 inputs
```

---

## Project Structure

```
black_robot/
├── README.md
├── docs/
│   ├── ARCHITECTURE.md             ← System design & steering spec
│   ├── MEASUREMENTS.md             ← Physical robot measurements
│   ├── CAN Command V0.1.pdf        ← Motor controller CAN protocol
│   └── skid_steer_spec.docx.pdf    ← Steering algorithm specification
├── jetson/                         ← Runs on the Jetson
│   ├── main.py                     ← Entry point — orchestrates all subsystems
│   ├── steering.py                 ← Differential steering algorithm
│   ├── can_transmitter.py          ← Sends DriveCommand CAN frames
│   ├── can_telemetry.py            ← Reads speed/temp/power CAN telemetry
│   ├── udp_receiver.py             ← Receives joystick commands via UDP :8888
│   ├── shared_state.py             ← Thread-safe state container
│   ├── robot_config.py             ← All tunable parameters & network config
│   ├── heartbeat_emitter.py        ← Sends status packets to Optimus GCS
│   ├── ros2_drive_bridge.py        ← Runs INSIDE onboard_optimus container
│   ├── robot-logic.service         ← Systemd: auto-starts main.py (35s delay)
│   ├── bridge-startup.service      ← Systemd: auto-starts bridge in container
│   └── can0-up.service             ← Systemd: brings up can0 at boot
├── laptop/
│   └── joystick_ros_bridge.py      ← Alternative: laptop joystick → UDP bridge
├── from_steam_deck/gg_vehicle/     ← Standalone PyQt5 GUI controller
│   ├── vehicle_gui.py
│   └── vehicle_logic.py
├── reference/
│   └── vehicle_controller.py       ← Legacy monolithic controller (archived)
└── tools/
    ├── can_diag.py                 ← CAN bus diagnostics
    └── steering_test.py            ← Automated steering test
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Robot doesn't move, throttle=0 in log | Gear is Neutral | Switch to Forward (D-Pad Up) |
| `[STEER] Input D:+0.00` — no UDP | Bridge not running | `sudo systemctl restart bridge-startup` |
| `[CAN TX ERROR] Network is down` | `can0` reset by container | Wait — 35s delay handles this. If persists: `sudo systemctl restart can0-up robot-logic` |
| `ModuleNotFoundError: optimus_interfaces` | Bridge missing workspace source | Check `bridge-startup.service` sources `/backend/install/setup.bash` |
| Bridge not in container after reboot | Container restarted, file lost | `bridge-startup.service` auto-copies from host. Check service status |
| GCS shows OFFLINE | Heartbeat not reaching GCS | Check firewall, confirm `robot-logic` running |
| Goes backward when steering | Inner wheel diff too large | Reduce `STEER_MAX_DIFF_CAN` in `steering.py` |
| Robot won't pivot on ground | Torque too low | Increase `TORQUE_LIMIT` in `robot_config.py` (currently 700 nM) |
| `MotorL: 102.0°C` | Thermistor open circuit | Hardware fault — launch with `--ignore-thermal` flag |
| CAN TX buffer full spam | Too many frames with no ACK | Motor controllers off or CAN wiring issue |
