# Motor Controller "Thermal Shutdown" Diagnosis

## Date: April 16, 2026

### The Problem
The Black Robot was experiencing intermittent "stuck" states where it would refuse to drive despite receiving valid joystick commands. The steering output showed commands being sent to the motors (`Out L: +500 R: +500`), but the motor controller reported `0 RPM` and the telemetry reader frequently logged:
`[SAFETY] ⚠ Motor controller in protection mode! Front: 0x51 Rear: 0x05 → STOPPING`

This was initially suspected to be a thermal shutdown caused by a known hardware issue: a broken thermistor on the left front motor that constantly reads `102.0°C`.

### The Investigation
We added tracking for the `MotorMode` byte (Byte 6 of CAN Frame `0x320`) and observed the following:
1. When powered on, the front controller idled in mode `0x51`.
2. When a drive command was sent, the controller briefly transitioned to `0x55`, then immediately fell back to `0x51`.
3. The Jetson control software was intercepting these modes and triggering a safety shutdown, thinking `0x51` and `0x55` were thermal protection states. This caused the slew-rate limiter to reset continuously, preventing the robot from accumulating any speed.
4. Sometimes the controller stopped sending telemetry frames entirely, leaving the robot dead in the water.

### The Breakthrough (Reading the Specs)
We extracted the text from the project's CAN specification document (`docs/CAN Command V0.1.pdf`) and discovered a critical misunderstanding of the telemetry data:

**1. MotorMode Byte (Byte 6) is NOT a fault indicator**
The `MotorMode` byte is actually a status bitfield indicating operational state, not thermal protection.
*   **0x51 (`0b01010001`)**: Speed Mode (`01`), Braking Enabled, Left Motor Standing (RPM < MIN), Right Motor Standing (RPM < MIN). This is the **perfectly normal** idle state.
*   **0x55 (`0b01010101`)**: Same as above, but with Braking *Disabled*. Also a normal state during transitions.
*   **0x29** is Regen Braking active on both sides.

**2. FaultLevel Byte (Byte 5) is the actual fault indicator**
The true thermal protection status is reported in `FaultLevel` (Byte 5). The spec defines it as:
*   `0`: None (no fault)
*   `1`: Warning (starting to get hot)
*   `2`: Degraded (performance reduced)
*   `3`: Stopping
*   `4`: Stopped

During all our "stuck" moments, `FaultLevel` was completely clear (`0`). The motor controller firmware entirely ignores the broken `102°C` left motor thermistor and does not raise a fault.

### The True Cause
The robot wasn't entering thermal shutdown at all. The Jetson control software was artificially blocking commands because it misinterpreted the normal idle state (`0x51`) as a thermal error.

The secondary issue—where the controller stopped communicating entirely (no telemetry)—was likely a hardware-level lockup or CAN bus timeout needing a physical power cycle (turning the 65V battery off and on).

### The Fix
1.  **Corrected Safety Logic:** Modified `jetson/main.py`. The control loop no longer inspects `MotorMode` for thermal faults. Instead, it correctly checks `FaultLevel`.
2.  **Appropriate Thresholds:** The code now allows operation at `FaultLevel 0` and `FaultLevel 1` (Warning), only triggering a safety stop if `FaultLevel >= 2` (Degraded or worse).
3.  **Diagnostic Tooling:** Created a standalone script, `tools/can_diag.py`, capable of decoding the full bitfields (standing flags, regen flags) and sending raw test commands to verify controller health independent of the main control loop.

### Result
After removing the flawed `MotorMode` constraint and power-cycling the 65V battery, the `can_diag.py` script successfully drove the motors to 325 RPM. The motors transitioned cleanly through idle (`0x51`), driving (`0x05`), and regen braking (`0x29`). The false `102°C` reading persists but has been proven to not trigger a controller-level fault. The robot is now operational again.
