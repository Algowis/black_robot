#!/usr/bin/env python3
"""
V_max Measurement Script — run on the ROBOT with wheels OFF THE GROUND.

Ramps throttle from 0 → 1000 in safe steps, holds for 3 seconds at full
throttle to measure steady-state RPM, then ramps back down.

Calculates V_max from the peak RPM observed:
  V_max = RPM × 2π × r / 60

Usage:
  1. LIFT THE ROBOT — wheels must spin freely!
  2. python3 measure_vmax.py
  3. Wait for the test to complete (~15 seconds)
"""

import os
import sys
import time
import struct
import math
import subprocess

try:
    import can
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "python-can"])
    import can


# ================================================================== #
#  Configuration                                                       #
# ================================================================== #

CAN_CHANNEL = "can0"
CAN_BITRATE = 500000

WHEEL_RADIUS = 0.1825       # meters (36.5 cm diameter / 2)
TRACK_WIDTH  = 0.585        # meters (center-to-center left-right)

FRONT_DRIVE_ID = 0x302      # Front driver DriveCommand
FRONT_SPEED_ID = 0x312      # Front driver SpeedTelemetry

# Test parameters
RAMP_STEPS   = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]
HOLD_TIME    = 3.0          # seconds at each step to measure RPM
RAMP_PAUSE   = 1.0          # seconds pause between ramp steps
SEND_HZ      = 50


# ================================================================== #
#  CAN Helpers                                                         #
# ================================================================== #

def send_drive_command(bus, throttle_left, throttle_right):
    """Send DriveCommand frame (big-endian)."""
    payload = struct.pack(">hhBHB",
        int(throttle_left),
        int(throttle_right),
        0x01,       # MotorMode = Speed
        20,         # MaxTorque = 20 nM
        0x00        # Reserved
    )
    msg = can.Message(arbitration_id=FRONT_DRIVE_ID, data=payload, is_extended_id=False)
    bus.send(msg)


def send_stop(bus):
    """Send zero throttle."""
    send_drive_command(bus, 0, 0)


def read_speed(bus, timeout=0.05):
    """Try to read a speed telemetry frame. Returns (left_rpm, right_rpm) or None."""
    msg = bus.recv(timeout)
    if msg is None:
        return None

    base_id = msg.arbitration_id & 0xFF0
    if base_id == 0x310 and len(msg.data) >= 8:
        speed_L, speed_R, _, _ = struct.unpack(">hhhh", msg.data[:8])
        return speed_L, speed_R
    return None


# ================================================================== #
#  Main Test                                                           #
# ================================================================== #

def main():
    print("=" * 60)
    print("  V_max MEASUREMENT SCRIPT")
    print("  ⚠  WHEELS MUST BE OFF THE GROUND!")
    print("=" * 60)
    print()

    input("  Press ENTER when robot is LIFTED and safe to spin wheels...")
    print()

    # Init CAN
    os.system(f"sudo ip link set {CAN_CHANNEL} down 2>/dev/null")
    os.system(f"sudo ip link set {CAN_CHANNEL} type can bitrate {CAN_BITRATE}")
    os.system(f"sudo ip link set {CAN_CHANNEL} up")
    time.sleep(1)

    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype="socketcan")

    max_rpm_left  = 0
    max_rpm_right = 0
    results = []

    try:
        print(f"  {'Throttle':>10} | {'Left RPM':>10} | {'Right RPM':>10} | {'V (m/s)':>10}")
        print(f"  {'-'*10}-+-{'-'*10}-+-{'-'*10}-+-{'-'*10}")

        for throttle in RAMP_STEPS:
            print(f"\n  Ramping to throttle = {throttle}...")

            # Send commands at 50Hz for HOLD_TIME seconds
            t_start = time.monotonic()
            rpm_samples_left = []
            rpm_samples_right = []

            while time.monotonic() - t_start < HOLD_TIME:
                send_drive_command(bus, throttle, throttle)

                # Read any available speed telemetry
                speed = read_speed(bus, timeout=0.015)
                if speed is not None:
                    rpm_samples_left.append(speed[0])
                    rpm_samples_right.append(speed[1])

                time.sleep(1.0 / SEND_HZ)

            # Calculate average RPM from the last half of samples (steady state)
            if rpm_samples_left:
                half = len(rpm_samples_left) // 2
                avg_left  = sum(rpm_samples_left[half:]) / len(rpm_samples_left[half:])
                avg_right = sum(rpm_samples_right[half:]) / len(rpm_samples_right[half:])
            else:
                avg_left = avg_right = 0

            # Convert to m/s
            avg_rpm = (abs(avg_left) + abs(avg_right)) / 2
            v_ms = avg_rpm * 2 * math.pi * WHEEL_RADIUS / 60

            results.append((throttle, avg_left, avg_right, v_ms))

            # Track max
            max_rpm_left  = max(max_rpm_left, abs(avg_left))
            max_rpm_right = max(max_rpm_right, abs(avg_right))

            print(f"  {throttle:>10} | {avg_left:>10.1f} | {avg_right:>10.1f} | {v_ms:>10.2f}")

        # Ramp down
        print("\n  Ramping down...")
        for throttle in [500, 250, 0]:
            send_drive_command(bus, throttle, throttle)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n  ⚠ INTERRUPTED!")
    finally:
        send_stop(bus)
        time.sleep(0.2)
        send_stop(bus)
        bus.shutdown()

    # Final results
    max_rpm = max(max_rpm_left, max_rpm_right)
    v_max = max_rpm * 2 * math.pi * WHEEL_RADIUS / 60

    print()
    print("=" * 60)
    print("  RESULTS")
    print("=" * 60)
    print(f"  Wheel radius:     {WHEEL_RADIUS} m")
    print(f"  Track width:      {TRACK_WIDTH} m")
    print(f"  Max RPM (left):   {max_rpm_left:.1f}")
    print(f"  Max RPM (right):  {max_rpm_right:.1f}")
    print()
    print(f"  ★ V_max = {v_max:.2f} m/s  ({v_max * 3.6:.1f} km/h)")
    print()
    print("  Copy these into steering.py:")
    print(f"    WHEEL_RADIUS = {WHEEL_RADIUS}")
    print(f"    TRACK_WIDTH  = {TRACK_WIDTH}")
    print(f"    V_MAX        = {v_max:.2f}")
    print("=" * 60)


if __name__ == "__main__":
    main()
