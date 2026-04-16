# Bug Analysis: Steam Deck UDP vs Robot Movement

**Date:** April 15, 2026

There are two primary issues preventing the robot from moving via the Steam Deck. The first is a network protocol mismatch blocking the joystick inputs, and the second is a hardware/power restriction blocking the CAN commands from physically turning the motors.

## 1. UDP Payload Format Mismatch (The Steam Deck Bug)

**Symptom:** The Steam Deck connects and sends data, but the Jetson's `shared_state` is never updated. 

**Root Cause:**
- In `vehicle_logic.py` (Steam Deck), the `pack_direct_control` function packs **4 fields**: `flag`, `throttle_pwm`, `front_pwm`, and `back_pwm`. This creates a **7-byte** struct (`<BHHH`).
- In `udp_receiver.py` (Jetson), the `_parse_packet` function is hardcoded to unpack a **5-byte** struct (`<BHH`), which lacks the `back_pwm` field.
- **The Crash:** When the 7-byte payload arrives, `struct.unpack("<BHH", payload)` throws a `struct.error` because the payload length doesn't match the format string. The Jetson silently catches the error, prints `[UDP PARSE ERROR]` (or in our debug, shows the payload size mismatch), and drops the packet. 
*(Note: Because your laptop tools like `steering_test.py` use the old 5-byte format, they don't trigger this crash).*

## 2. CAN Torque Restriction (The Motor Issue)

**Symptom:** When using `tools/steering_test.py` (which correctly uses the 5-byte format), the Jetson mathematics and logic run perfectly. The output log shows:
`[STEER] Input D:+0.25 S:-1.00 → Out L: +231 R: +269 | RPM L: 0 R: 0`
Despite accurately calculating and transmitting the positive `+231` (23%) and `+269` (26%) throttle targets to the CAN bus, the motor RPM remains `0` and current draw rests at `0A`.

**Root Cause Analysis:**
The CAN connection is fully alive (the `46.0V` telemetry confirms this). The failure is physical or limit-based:
1. **Low Torque Limit:** In `can_transmitter.py`, the drive command specifies a `TORQUE_LIMIT = 20`. Depending on the motor controller's scale (e.g. 0.1 N·m per unit = 2.0 N·m), `20` may simply not be enough torque to break the static friction of the robot's heavy gears.
2. **Motor Deadband:** 23% power (`231/1000`) might be within the motor controller's hardcoded "deadzone", meaning the controller refuses to apply physical power until the target breaks a higher threshold.

## Proposed Action Plan & Resolutions

1. **Protocol Fix (✅ RESOLVED):** Updated `udp_receiver.py` on the Jetson to conditionally unpack based on the `len(payload)`. If `len == 5`, use `<BHH`. If `len == 7`, use `<BHHH`. This makes the Jetson fully backward-compatible with both the Steam Deck and your laptop testing tools.
2. **Motor Hardware Check (✅ RESOLVED):** The secondary issue preventing the wheels from spinning was confirmed to be a depleted/dead robot battery. Once fully charged, the new UDP protocol fix and the original `20` Torque limit will be tested together.
