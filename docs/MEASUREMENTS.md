# Physical Measurements

All measurements taken on 2026-04-14.

## Wheel Geometry

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| **Wheel diameter** | 36.5 | cm | Including tire |
| **Wheel radius (r)** | 0.1825 | m | = 36.5 / 2 |
| **Wheelbase** | 89 | cm | Center-to-center, front-to-rear (same side) |
| **Track width (W)** | 58.5 | cm | Center-to-center, left-to-right |

## Motor Performance

Measured with `tools/measure_vmax.py` — robot lifted, wheels spinning freely.

| Throttle | Left RPM | Right RPM | Speed (m/s) |
|----------|----------|-----------|-------------|
| 100 | 0 | 0 | 0.00 |
| 200 | 0 | 0 | 0.00 |
| 300 | 0 | 0 | 0.00 |
| 400 | 164 | 169 | 3.18 |
| 500 | 0* | 0* | — |
| 600 | 0* | 0* | — |
| 700 | 473 | 478 | 9.09 |
| 800 | 571 | 571 | 10.91 |
| 900+ | — | — | Stopped: mechanical noise |

\* Zero readings at 500-600 are a measurement script timing issue, not actual zero speed.

### Derived Parameters

| Parameter | Value | Unit | How |
|-----------|-------|------|-----|
| **Max RPM at throttle 800** | 571 | RPM | Measured |
| **Estimated max RPM at throttle 1000** | ~714 | RPM | Linear extrapolation |
| **V_max** | 13.6 | m/s (49 km/h) | 714 × 2π × 0.1825 / 60 |

## Electrical

| Parameter | Value | Notes |
|-----------|-------|-------|
| Battery voltage | ~52.2 V | Measured from CAN telemetry |
| CAN bus bitrate | 500,000 bps | — |

## Algorithm Parameters (from spec)

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| W | 0.585 | m | Track width |
| r | 0.1825 | m | Wheel radius |
| V_max | 13.6 | m/s | Max robot speed |
| a_lat_max | 3.0 | m/s² | Anti-rollover lateral accel limit |
| MaxAccel | 2.0 | m/s² | Slew rate limiter |
| V_pivot | 0.2 | m/s | Below → pivot mode |
| V_blend_end | 1.0 | m/s | Above → full dynamic anti-rollover |
| deadzone | 0.05 | — | Joystick dead-band |
| telem_timeout | 1.0 | s | CAN telemetry watchdog |
| UDP_timeout | 0.5 | s | Joystick connection watchdog |
