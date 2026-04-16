#!/usr/bin/env python3
"""
Black Robot — Main Controller

Orchestrates the three subsystems:
  1. UDP Receiver    — background thread, listens for joystick commands
  2. CAN Telemetry   — background thread, reads motor controller feedback
  3. Control Loop     — main thread, runs the drive logic at a fixed rate

Steering algorithm (partial):
  - Input normalization with deadband
  - Stage 4: Drive + Steer mixing
  - Stage 5: Slew rate limiter
  - Stage 6: CAN throttle output
  TODO: Stages 1-3 (anti-rollover, needs physical measurements)
"""

import argparse
import os
import sys
import time
import subprocess

# --- Auto-install python-can if missing ---
try:
    import can
except ImportError:
    print("Module 'python-can' not found. Installing...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "python-can"])
    import can

from shared_state import SharedState
from udp_receiver import UDPReceiver
from can_telemetry import CANTelemetry
from can_transmitter import CANTransmitter
from steering import SteeringController


# ================================================================== #
#  Configuration                                                       #
# ================================================================== #

CAN_CHANNEL   = "can0"
CAN_BITRATE   = 500000
CONTROL_HZ    = 100      # Control loop frequency (Hz)
SEND_TO_REAR  = True     # Send commands to rear driver (0x304) too


# ================================================================== #
#  CAN Bus Initialization                                              #
# ================================================================== #

def init_can_interface():
    """Bring up the CAN0 network interface."""
    print("[INIT] Configuring CAN interface...")
    os.system(f"sudo ip link set {CAN_CHANNEL} down 2>/dev/null")
    os.system(f"sudo ip link set {CAN_CHANNEL} type can bitrate {CAN_BITRATE}")
    os.system(f"sudo ip link set {CAN_CHANNEL} up")
    time.sleep(1)
    print(f"[INIT] {CAN_CHANNEL} up at {CAN_BITRATE} bps")


# ================================================================== #
#  Safety Timeouts                                                     #
# ================================================================== #

UDP_TIMEOUT_S    = 0.5       # Joystick connection safety timeout (seconds)
TELEM_TIMEOUT_S  = 0.2       # CAN telemetry safety timeout (per skid-steer spec)
NORMAL_MODES     = {0x01, 0x05, 0x51}  # Motor modes considered healthy
                                       # 0x01 = speed mode
                                       # 0x05 = speed + active/running
                                       # 0x51 = speed + both motors standing
# NOTE: MotorMode byte is STATUS (standing/braking flags), NOT protection.
# Thermal protection is indicated by FaultLevel field:
#   0=None, 1=Warning, 2=Degraded, 3=Stopping, 4=Stopped
FAULT_LEVEL_STOP = 2  # Stop driving at FaultLevel >= 2 (Degraded or worse)


# ================================================================== #
#  Control Loop                                                        #
# ================================================================== #

def control_loop(transmitter, state, steering, ignore_thermal=False):
    """
    Fixed-rate control loop. Reads shared state, computes motor commands
    via the steering controller, and sends them over CAN.

    Safety checks (each cycle):
      - UDP timeout:   no joystick packet for 500ms → stop
      - Telem timeout: no CAN feedback for 200ms → stop
      - Fault level:   motor controller reports fault → stop
    """
    dt = 1.0 / CONTROL_HZ
    print(f"[CTRL] Control loop running at {CONTROL_HZ} Hz (dt={dt*1000:.0f} ms)")

    # Track whether we've already printed the safety warning
    # (so we don't spam the console every cycle)
    safety_state = "OK"  # "OK" | "UDP_TIMEOUT" | "TELEM_TIMEOUT" | "FAULT" | "THERMAL"

    while True:
        t_start = time.monotonic()
        now = t_start

        # --- Read shared state (snapshot) ---
        snap = state.snapshot()
        throttle_pwm = snap["throttle_pwm"]
        front_pwm    = snap["front_pwm"]
        last_udp     = snap["last_udp_time"]
        last_telem   = snap["last_telem_time"]
        front_fault  = snap["front_fault_level"]
        rear_fault   = snap["rear_fault_level"]
        front_mode   = snap["front_motor_mode"]
        rear_mode    = snap["rear_motor_mode"]

        # --- Determine stop reason (checked in priority order) ---
        stop_reason = None

        # Safety check 1: Motor controller fault — FaultLevel from CAN spec
        # 0=None, 1=Warning(hot), 2=Degraded, 3=Stopping, 4=Stopped
        max_fault = max(front_fault, rear_fault)
        if max_fault >= FAULT_LEVEL_STOP:
            # Serious fault — degraded/stopping/stopped
            if ignore_thermal:
                if not hasattr(control_loop, '_thermal_warned'):
                    fault_names = {2: "DEGRADED", 3: "STOPPING", 4: "STOPPED"}
                    print(f"[SAFETY] ⚠ Motor fault: {fault_names.get(max_fault, max_fault)}! "
                          f"Front: {front_fault}  Rear: {rear_fault}"
                          f"  (--ignore-thermal: continuing anyway)")
                    control_loop._thermal_warned = True
            else:
                stop_reason = "FAULT"
                if safety_state != "FAULT":
                    fault_names = {2: "DEGRADED", 3: "STOPPING", 4: "STOPPED"}
                    print(f"[SAFETY] ⚠ Motor fault: {fault_names.get(max_fault, max_fault)}! "
                          f"Front: {front_fault}  Rear: {rear_fault}  → STOPPING")
        elif max_fault == 1:
            # Warning only — log once but keep driving
            if not hasattr(control_loop, '_fault_warn_printed'):
                print(f"[SAFETY] ⚡ Motor temp WARNING (still driving) "
                      f"Front: {front_fault}  Rear: {rear_fault}")
                control_loop._fault_warn_printed = True

        # Safety check 3: UDP joystick timeout
        if stop_reason is None and last_udp > 0 and (now - last_udp) > UDP_TIMEOUT_S:
            stop_reason = "UDP_TIMEOUT"
            if safety_state != "UDP_TIMEOUT":
                print(f"[SAFETY] ⚠ No joystick for {UDP_TIMEOUT_S*1000:.0f}ms → STOPPING")

        # Safety check 4: CAN telemetry timeout
        if stop_reason is None and last_telem > 0 and (now - last_telem) > TELEM_TIMEOUT_S:
            stop_reason = "TELEM_TIMEOUT"
            if safety_state != "TELEM_TIMEOUT":
                print(f"[SAFETY] ⚠ No CAN telemetry for {TELEM_TIMEOUT_S*1000:.0f}ms → STOPPING")

        # --- Act on result ---
        if stop_reason is not None:
            safety_state = stop_reason
            transmitter.send_stop()
            steering.reset()
        else:
            if safety_state != "OK" and last_udp > 0:
                print(f"[SAFETY] ✓ Recovered → resuming control")
            safety_state = "OK"
            # Full 6-stage algorithm with telemetry feedback
            throttle_left, throttle_right = steering.compute(
                throttle_pwm, front_pwm,
                speed_left_rpm=snap["front_speed_left_rpm"],
                speed_right_rpm=snap["front_speed_right_rpm"]
            )
            torque_mode = snap.get("torque_mode", False)
            transmitter.send_drive_command(throttle_left, throttle_right, torque_mode=torque_mode)

            # --- Debug: show raw input vs slew-limited output (2 Hz) ---
            if not hasattr(control_loop, '_last_dbg'):
                control_loop._last_dbg = 0.0
            if now - control_loop._last_dbg >= 0.5:
                control_loop._last_dbg = now
                raw_drive = (throttle_pwm - 1500) / 500.0
                raw_steer = (front_pwm - 1500) / 500.0
                actual_l = snap["front_speed_left_rpm"]
                actual_r = snap["front_speed_right_rpm"]
                print(f"[STEER] Input D:{raw_drive:+.2f} S:{raw_steer:+.2f} "
                      f"→ Out L:{throttle_left:+5d} R:{throttle_right:+5d}  "
                      f"| RPM L:{actual_l:+4d} R:{actual_r:+4d}")

        # --- Sleep for remainder of cycle ---
        elapsed = time.monotonic() - t_start
        sleep_time = max(0, dt - elapsed)
        time.sleep(sleep_time)


# ================================================================== #
#  Main                                                                #
# ================================================================== #

def main():
    parser = argparse.ArgumentParser(description="Black Robot Vehicle Controller")
    parser.add_argument("--ignore-thermal", action="store_true",
                        help="Bypass motor thermal protection check "
                             "(use when thermistor is broken)")
    args = parser.parse_args()

    print("=" * 60)
    print("  BLACK ROBOT — Vehicle Controller")
    print("  Steering: Full 6-Stage Skid-Steer Algorithm")
    print("  Anti-rollover | Slew limiter | Telemetry feedback")
    if args.ignore_thermal:
        print("  ⚠ THERMAL CHECK BYPASSED (--ignore-thermal)")
    print("=" * 60)

    # 1. Initialize CAN hardware
    init_can_interface()

    bus = None
    try:
        # 2. Open CAN bus
        print("[INIT] Opening CAN bus...")
        bus = can.interface.Bus(channel=CAN_CHANNEL, bustype="socketcan")

        # 3. Create shared state
        state = SharedState()

        # 4. Start subsystems
        udp = UDPReceiver(state)
        udp.start()

        telem = CANTelemetry(bus, state)
        telem.start()

        transmitter = CANTransmitter(bus, send_to_rear=SEND_TO_REAR)

        steering = SteeringController(loop_hz=CONTROL_HZ)
        steering.reset()

        print("[READY] All systems go. Press Ctrl+C to stop.")
        print("-" * 60)

        # 6. Run control loop on main thread (blocks forever)
        control_loop(transmitter, state, steering,
                     ignore_thermal=args.ignore_thermal)

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Ctrl+C received...")
    except Exception as e:
        print(f"\n[FATAL] {e}")
    finally:
        # Send stop command before exiting
        if bus is not None:
            try:
                print("[SHUTDOWN] Sending stop command...")
                stop_tx = CANTransmitter(bus, send_to_rear=SEND_TO_REAR)
                stop_tx.send_stop()
                time.sleep(0.1)
            except Exception:
                pass
            bus.shutdown()
        print("[SHUTDOWN] Done.")


if __name__ == "__main__":
    main()
