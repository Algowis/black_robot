# Skid-Steer Specification Compliance Report

This document outlines the current implementation status of the 6-stage skid-steer steering algorithm (`skid_steer_spec.docx.pdf`) within the `jetson/steering.py` and `jetson/main.py` files.

Overall, the core mathematical models (Stages 1 through 6, including the 3 speed zones algorithm, conflict resolution, and anti-rollover mechanics) have been implemented robustly in `steering.py`. However, there are a few discrepancies related to safety safeguards, handling of timeouts, and module responsibility boundaries. 

## Discrepancies and Missing Implementations

### 1. Initialisation from Telemetry (Spec Section 5.2)
* **Spec requires:** `Cmd_left_prev` and `Cmd_right_prev` must be initialized to the *first telemetry reading received*, rather than `0`.
* **Current status (Not Implemented):** `steering.py` hardcodes these to `0.0` in both `__init__()` and `reset()`. If the robot is already moving when the script starts or recovers from a fault, this causes a large velocity jump that the slew limit calculation cannot smooth out in the first cycle.

### 2. Telemetry Timeout Behavior (Spec Sections 5.1 & 2.2)
* **Spec requires:** If telemetry times out (`telem_timeout` parameter = `0.2s` default), the controller must set `DriveCmd = 0` and `SteerCmd = 0` but **leave the Slew Rate Limiter active** so the robot decelerates smoothly to an eventual stop.
* **Current status (Violates Spec):** In `main.py`, `TELEM_TIMEOUT_S` is set to `1.0` seconds instead of `0.2` seconds. Furthermore, when a timeout occurs, a hard emergency stop is immediately invoked via `transmitter.send_stop()` alongside `steering.reset()`. Because this skips the `steering.compute()` path entirely, the slew rate limiter is turned off, meaning smooth deceleration is completely skipped.

### 3. Out of Scope Conversions (Spec Section 6)
* **Spec requires:** "RPM-to-m/s conversion (provide already-converted values as inputs)" should be handled separate from the core steering component.
* **Current status (Violates Spec):** `SteeringController.compute()` inside `steering.py` accepts parameter inputs natively in RPM (`speed_left_rpm`), then manually converts them to m/s internally using `_rpm_to_ms()`, rather than receiving standardized `m/s` values from its upstream caller.

### 4. Outdated Code Comments in `main.py`
* **Current status:** The file header of `main.py` includes the following comment:
  > `TODO: Stages 1-3 (anti-rollover, needs physical measurements)` 
  This header is outdated and misleading, as Stages 1-3 are fully implemented and functioning properly inside `steering.py`.

## Action Items
To achieve full compliance with the specification document, the following refactoring work remains:
- Move RPM-to-m/s conversions out of `steering.py`, optionally into `can_telemetry.py` or the `main.py` bridge.
- Alter the fallback conditions on telemetry timeout (or faults) in `main.py` so the loop commands `DriveCmd = 0`, `SteerCmd = 0` into `steering.compute()` instead of invoking `send_stop()` and `reset()`. Update timeout threshold to `0.2s`.
- Implement dynamic synchronization logic inside `steering.py`'s `reset()` / `__init__()` to pair `Cmd_left_prev` and `Cmd_right_prev` with the actual initial ground-speed read via telemetry.
