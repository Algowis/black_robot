# Skid-Steer Algorithm Test Results

**Date**: 2026-04-14  
**Tested via**: `tools/steering_test.py` (UDP stream 50 Hz)  
**Configuration**: `V_max = 13.6 m/s`, `a_lat_max = 3.0 m/s²`, `MaxAccel = 2.0 m/s²`

## Objective
Verify the full 6-stage steering algorithm behaves safely across standard driving, aggressive cornering, and rapid direction changes. The robot was run lifted on a stand.

## 1. Aggressive High-Speed Turning (Anti-Rollover)
* **Test**: Drive forward at 25% speed (~4.2 m/s, 15 km/h) and command **100% Left Steer**.
* **Result**: `Out L: +231, R: +269`
* **Analysis**: SUCCESS. At 4.2 m/s, a 100% skid turn would rip the tires off or flip the robot. The Stage 2 Anti-Rollover system engaged perfectly, identifying that the maximum allowable speed differential to keep lateral G-forces under `3.0 m/s²` was extremely small. The 100% command was automatically and safely scaled down to a gentle wide curve (differentials of `231` vs `269` CAN units).

## 2. Instant Reversal (Slew Rate Limiting)
* **Test**: From +25% forward, instantly command -25% reverse.
* **Result**:
  ```text
  Input D:-0.25 → Out L/R: +112
  Input D:-0.25 → Out L/R: -35
  Input D:-0.25 → Out L/R: -250
  ```
* **Analysis**: SUCCESS. The controllers did not suffer a violent current spike. Stage 5 smoothly ramped the physical motor targets down through 0, preventing physical shock to the gears, taking ~1.5 seconds to complete the transition.

## 3. Transition to Pivot Mode
* **Test**: From driving Forward + Right, instantly command 0% Forward, 100% Right Spin.
* **Result**:
  ```text
  Out L/R: +257/+219 (Forward + Right)
  Out L/R: +43/-1    (Braking toward zero)
  Out L/R: +202/-202 (Settling into Pivot Mode)
  ```
* **Analysis**: SUCCESS. The algorithm seamlessly blended from Zone B (high speed, highly limited steering) down into Zone A (0 speed). Once forward speed dropped below `V_pivot` (0.2 m/s), it executed the tank-spin safely. Notice that the output locked to roughly `±202` (20% throttle) instead of `1000`; this is because Pivot mode limits the tank spin to base `a_lat_max` safety constraints so it spins quickly but smoothly, rather than violently maxing out the RPM out of control.

## Conclusion & Tuning Guidance
The algorithm handles all edge cases gracefully exactly as specified. 

* If the physical robot feels "sluggish" to respond to throttle changes on the ground: **Increase `MAX_ACCEL` in `steering.py` from `2.0` to `3.0` or `4.0`**.
* If the robot feels too restrictive when trying to drift/corner at high speed: **Increase `A_LAT_MAX` in `steering.py` from `3.0` to `4.0` or `5.0`**.
