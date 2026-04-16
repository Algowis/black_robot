# Architecture & Design

## System Overview

The Black Robot vehicle controller runs on a Jetson (Tegra Ubuntu) connected to two dual-motor controllers via CAN bus. An operator controls the robot from a laptop using a joystick simulator over UDP.

## Thread Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                      Jetson (main.py)                         │
│                                                              │
│  ┌─────────────────┐  ┌──────────────────┐                  │
│  │  UDP Receiver    │  │  CAN Telemetry    │   Background    │
│  │  (thread)        │  │  (thread)         │   threads       │
│  │  port 8888       │  │  reads CAN frames │                │
│  └────────┬─────────┘  └────────┬──────────┘                │
│           │                     │                            │
│           ▼                     ▼                            │
│  ┌──────────────────────────────────────────┐                │
│  │           SharedState (thread-safe)       │                │
│  │  throttle_pwm, front_pwm, gear_low       │                │
│  │  front_speed_left_rpm, ...               │                │
│  │  front_fault_level, battery_voltage      │                │
│  │  last_udp_time, last_telem_time          │                │
│  └────────────────────┬─────────────────────┘                │
│                       │                                      │
│                       ▼                                      │
│  ┌────────────────────────────────────────┐                  │
│  │         Control Loop (main thread)      │  50 Hz          │
│  │                                         │                 │
│  │  1. Safety checks (fault/timeout)       │                 │
│  │  2. steering.compute()                  │                 │
│  │     → 6-stage algorithm                 │                 │
│  │  3. transmitter.send_drive_command()    │                 │
│  └─────────────────────┬───────────────────┘                 │
│                        │                                     │
│                        ▼                                     │
│  ┌──────────────────────────────────────┐                    │
│  │          CAN Transmitter              │                    │
│  │  DriveCommand → 0x302 (front)         │                    │
│  │  DriveCommand → 0x304 (rear)          │                    │
│  └──────────────────────────────────────┘                    │
└──────────────────────────────────────────────────────────────┘
```

## 6-Stage Steering Algorithm

Implemented in `steering.py`. Each stage runs every control loop cycle (50 Hz).

```
Input: throttle_pwm, front_pwm, speed_left_rpm, speed_right_rpm

Stage 1 — Reference Speed
│ V_robot_actual = avg(left_telem, right_telem)
│ V_target_drive = DriveCmd × V_max
│ V_for_omega = max(|actual|, |target|)  ← conservative for safety
▼
Stage 2 — TurnAmount by Speed Zone
│ Zone A (V ≤ 0.2 m/s):  Pivot — full differential allowed
│ Zone B (V ≥ 1.0 m/s):  Dynamic — ω limited by a_lat_max / V
│ Zone C (between):       Blend — linear interpolation A↔B
▼
Stage 3 — Conflict Resolution
│ If |drive| + |turn| > V_max → reduce drive, keep turn
▼
Stage 4 — Per-Motor Targets
│ Target_left  = V_target_drive + TurnAmount
│ Target_right = V_target_drive − TurnAmount
▼
Stage 5 — Slew Rate Limiter
│ max_delta = MaxAccel × dt  (2.0 m/s² × 0.02s = 0.04 m/s per cycle)
│ Applied independently to each side
▼
Stage 6 — CAN Throttle Output
│ Throttle = round((velocity / V_max) × 1000)
│ Clamped to [-1000, +1000]

Output: throttle_left, throttle_right
```

## Safety System

Three checks run every cycle BEFORE the steering algorithm:

| Priority | Check | Trigger | Action |
|----------|-------|---------|--------|
| 1 | Motor fault | `fault_level > 0` from CAN | Stop + reset slew |
| 2 | UDP timeout | No joystick packet for 500ms | Stop + reset slew |
| 3 | Telemetry timeout | No CAN data for 1s | Stop + reset slew |

- Warnings print **once** per event (no console spam)
- Recovery is automatic with `✓ Recovered` message
- `steering.reset()` zeros the slew limiter to prevent jump on recovery

## CAN Protocol Summary

### DriveCommand (Master → Motor Controller)
- CAN ID: `0x300 + NodeID`
- Byte order: **Big-Endian (Motorola)**

| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| 0-1 | ThrottleLeft | int16 | -1000..+1000 |
| 2-3 | ThrottleRight | int16 | -1000..+1000 |
| 4 | MotorMode | uint8 | 0x01 = Speed Mode |
| 5-6 | MaxTorque | uint16 | nM (default: 20) |
| 7 | Reserved | uint8 | 0x00 |

### SpeedTelemetry (Motor Controller → Master)
- CAN ID: `0x310 + NodeID`

| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| 0-1 | SpeedLeft | int16 | RPM |
| 2-3 | SpeedRight | int16 | RPM |
| 4-5 | TorqueLeft | int16 | nM |
| 6-7 | TorqueRight | int16 | nM |

## Key Design Decisions

1. **Big-Endian byte order** — confirmed by testing. The motor controllers use Motorola byte order, not Intel.

2. **V_max = 13.6 m/s** — extrapolated from 571 RPM at throttle 800 (max tested before mechanical noise).

3. **Telemetry timeout = 1.0s** — spec says 200ms, but CAN messages arrive in bursts with natural pauses. 200ms caused false alarms.

4. **Slew limiter in m/s** — all internal calculations use physical units (m/s) for correctness. Conversion to CAN throttle happens only at Stage 6.

5. **Anti-rollover uses "Option Y"** — `V_for_omega = max(|actual|, |commanded|)` for conservative turn limiting during deceleration.
