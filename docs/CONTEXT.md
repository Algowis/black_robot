# Session Context: Black Robot Vehicle Controller

**Date**: April 14, 2026  
**Goal**: Implement a robust 6-stage skid-steer steering algorithm with active anti-rollover protection and CAN bus motor control for a differential drive robot (Black Robot).

---

## 1. Project Background and Architecture

The Black Robot uses a Jetson (Tegra Ubuntu) as the main vehicle controller, which talks to two dual-motor controllers via CAN bus (500,000 bps). The operator utilizes a Steam Deck (or laptop simulator) sending joystick inputs via a custom UDP protocol.

### Core Modules:
- **`main.py`**: The 50 Hz control loop. Enforces safety watchdogs (UDP timeouts, CAN telemetry timeouts) and invokes the steering algorithm.
- **`steering.py`**: The 6-Stage Algorithm pure-math implementation. Converts arbitrary joystick inputs into physically safe, hardware-scaled CAN outputs.
- **`can_transmitter.py`**: Formats and sends `DriveCommand` (0x302, 0x304). *Key learning: Hardware expects Big-Endian (Motorola) byte order.*
- **`can_telemetry.py`**: Parses `Speed`, `Power`, and `Temperature` from motor controllers. Monitors CAN fault codes.
- **`udp_receiver.py`**: Receives structural Steam Deck payload over UDP Port 8888.
- **`shared_state.py`**: Thread-safe state container passing telemetry up to the control loop.

---

## 2. Physical & Derived Parameters

*   **Wheelbase**: 89 cm
*   **Track Width (W)**: 58.5 cm (0.585 m)
*   **Wheel Diameter**: 36.5 cm (Radius `r` = 0.1825 m)
*   **V_max**: 13.6 m/s (approx. 49 km/h, extrapolated from `measure_vmax.py` returning 571 RPM at 80% throttle).
*   **Anti-Rollover Limit (`a_lat_max`)**: 3.0 m/s² (Configurable in `steering.py`).
*   **Slew Rate Limit (`MaxAccel`)**: 2.0 m/s² (Configurable in `steering.py`).

---

## 3. The 6-Stage Skid Steer Algorithm

The most significant achievement of the session is the realization of the `skid_steer_spec.docx` into `steering.py`.

1.  **Reference Speed Formulation**: `V_for_omega = max(|actual_speed|, |commanded_speed|)`. Captures true speed using CAN telemetry to prevent rollover during heavy braking. 
2.  **Turn Amount Calculation (Anti-Rollover)**:
    *   **Zone A (Pivot)**: `V < 0.2 m/s`. Allows full turning (robot rotates in place).
    *   **Zone B (Dynamic)**: `V > 1.0 m/s`. Restricts angular velocity based on lateral acceleration: `TurnAmount_max = (a_lat_max / V_for_omega) * (W / 2)`.
    *   **Zone C (Blend)**: `0.2 <= V <= 1.0 m/s`. Smoothly interpolates limits between Pivot and Dynamic modes.
3.  **Conflict Resolution**: If Drive + Steer > `V_max`, Drive is reduced to preserve turning authority entirely.
4.  **Mixing**: Calculates raw `Target_Left` and `Target_Right` in m/s.
5.  **Slew Rate Limiting**: Enforces `MaxAccel`. Prevents huge delta-v spikes (which blow fuses or gears) and organically handles smooth direction transitions.
6.  **CAN Scaling**: Converts physical `m/s` to `[-1000, 1000]` throttle values for the CAN commands.

---

## 4. Key Debugging & Discoveries

*   **Big-Endian vs Little-Endian**: `vehicle_controller.py` originally used Little-Endian struct formats, which resulted in garbled voltage/RPM telemetry and erratic motor movement. Reversing to Big-Endian (`>`) matched the motor controller expectations.
*   **Safety Watchdog Tweaks**: The telemetry watchdog was triggering false positives at `0.2s` because CAN frames (especially speed) naturally burst. Increased the timeout threshold to `1.0s`. 
*   **Simulator Dead-zones**: Modified `joystick_sim.py` to stop auto-centering strings so we could confidently send fixed off-center values (Mode 3: Hold Mode for momentary keystrokes).
*   **Missing UDP Packets**: Our `steering_test.py` initially failed to control the robot because the custom Steam Deck protocol required a specific byte header (`0xAA, 0x55`, lengths, IDs) wrapped around the payload. Updating the `build_packet` method fixed this.
*   **Steam Deck UDP Payload Format Mismatch**: The real Steam Deck GUI dynamically added a 4th `back_pwm` parameter, creating a 7-byte struct (`<BHHH`). The Jetson `udp_receiver.py` was hardcoded to expect the older 5-byte (`<BHH`) format used by laptop testing tools, which caused standard Steam Deck commands to throw an internal `struct.error` and be silently dropped. The receiver has been updated to dynamically unpack both structs based on payload length (`len == 5` vs `len == 7`) to ensure future compatibility.

---

## 5. Summary of Automated Testing

Using `tools/steering_test.py` running on `laptop`, we verified the math of `steering.py`:

*   **Test 1 (Drive 25% + Turn 100%)**: Output narrowed aggressively to 231 vs 269. The anti-rollover recognized 4.2 m/s was too fast for a sharp turn and squeezed a 100% turn command down to a tiny, safe arc.
*   **Test 2 (Rapid Direction Change)**: Instantly swapping from +25% to -25% drove output down slowly and linearly (`+112 → -35 → -109...`) proving the Stage 5 Slew Limiter works.
*   **Test 3 (Pivot)**: Releasing throttle and holding full turn at speed smoothly braked the robot into Zone A, then executed a `+202 / -202` tank spin— proving 0-speed anti-rollover is fully bounded and safe.

---

## 6. Hardware & Safety Limitations

*   **70% Global Speed Limit (Anti-Rattle)**: During testing, pushing the motors to `1000` (100% CAN output) caused a violent gear rattling noise inside the hubs, causing them to shut down or slip. To prevent this, `steering.py` now enforces `GLOBAL_LIMIT_PCT = 0.70`. This gracefully caps the maximum physical output at `±700` while preserving turning ratios mathematically prior to the Slew Rate limiter.
*   **Intermittent Right Back Motor Failure**: If the motor makes a grinding noise and spins with zero resistance (no cogging torque) under power, but occasionally recovers, the issue is physical:
    *   **Phase Disconnect**: One of the thick 3-Phase power cables (Yellow/Green/Blue) has a loose connection or bullet connector. When it shakes loose, the motor loses phase sync, spins freely, and makes grinding noises.
    *   **Controller Over-Current Shutoff**: The controller's internal protection tripped during the high-torque spike and disabled the MOSFETs for that channel without sending a CAN fault code. Power-cycling resets it.
*   **Temperature Sensor Malfunction**: The `MotorL` reading is fixed at `102.0°C` / `103.0°C`. This indicates an open-circuit (unplugged or broken) thermistor wire inside the Left Front motor axis. 
*   **Battery Depletion Safety Shutoff**: If the Jetson control loop processes perfectly taking valid joystick bounds to proper scaled commands (e.g. `Out L: +231`) but the CAN telemetry reads `0A` output and the wheels refuse to spin, the 48V battery has likely bled down to a minimum voltage threshold, engaging the motor-driver safety cutoff without severing the telemetry feed.

---

## 7. Next Steps / Tuning Guide

When deploying onto the floor/pavement:
1.  **If the robot feels unresponsive or sluggish to accelerate/brake**: Open `steering.py` and increase `MAX_ACCEL = 2.0` up to `3.0` or `4.0`.
2.  **If the robot turns too wide at moderate speeds**: The limit is too conservative. Increase `A_LAT_MAX = 3.0` to `4.0` or `5.0`.
3.  **If pivoting in place is too slow**: Decrease `V_PIVOT` or loosen `A_LAT_MAX_PIVOT` specifically. Currently, pivot math piggybacks slightly on general lateral limits for smoothness.
4.  **Watchdog monitoring**: Keep an eye on the console `[SAFETY]` prints during outdoor test drives. If UDP drops out frequently due to bad Wi-Fi range, the robot will emergency brake. Consider increasing the UDP heartbeat tolerance if this becomes an issue.
