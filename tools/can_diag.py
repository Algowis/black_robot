#!/usr/bin/env python3
"""
Black Robot — CAN Bus Diagnostic Tool

Comprehensive diagnostic that reads ALL CAN telemetry, decodes every field
per "CAN Command V0.1" spec, and optionally sends test drive commands.

Usage:
    python3 can_diag.py                  # Listen-only mode (safe)
    python3 can_diag.py --drive 200      # Send throttle=200 to both sides
    python3 can_diag.py --drive 200 -t 5 # Drive for 5 seconds then stop
    python3 can_diag.py --raw            # Show raw hex for every CAN frame

Requires: python-can (pip install python-can)
"""

import argparse
import os
import struct
import sys
import time
import subprocess

try:
    import can
except ImportError:
    print("Installing python-can...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "python-can"])
    import can


# ================================================================== #
#  CAN IDs                                                             #
# ================================================================== #

# Telemetry (Motor Driver → Master)
TEMP_BASE       = 0x2D0   # + NodeID
SPEED_BASE      = 0x310   # + NodeID
POWER_BASE      = 0x320   # + NodeID
ALARM_BASE      = 0x500   # + NodeID

# Commands (Master → Motor Driver)
DRIVE_BASE      = 0x300   # + NodeID
CURRENT_LIMIT   = 0x400   # + NodeID

NODE_NAMES = {2: "FRONT", 4: "REAR"}


# ================================================================== #
#  Bitfield Decoders                                                   #
# ================================================================== #

def decode_motor_mode(mode_byte):
    """Decode MotorMode byte per CAN Command V0.1 spec (telemetry version)."""
    # Bits 0-1: Motor mode
    mode_bits = mode_byte & 0x03
    mode_names = {0: "NoMode", 1: "Speed", 2: "Torque", 3: "Reserved"}
    mode_name = mode_names.get(mode_bits, "?")

    # Bit 2: DisableBraking
    braking_disabled = bool(mode_byte & 0x04)

    # Bit 3: brakingMotorLeft (regen braking active)
    left_braking = bool(mode_byte & 0x08)

    # Bit 4: StandingMotorLeft (RPM < MIN_RPM_STANDING)
    left_standing = bool(mode_byte & 0x10)

    # Bit 5: BrakingMotorRight (regen braking active)
    right_braking = bool(mode_byte & 0x20)

    # Bit 6: StandingMotorRight (RPM < MIN_RPM_STANDING)
    right_standing = bool(mode_byte & 0x40)

    flags = []
    flags.append(f"mode={mode_name}")
    if braking_disabled:
        flags.append("BRAKE_OFF")
    else:
        flags.append("brake_on")
    if left_standing:
        flags.append("L_STANDING")
    if right_standing:
        flags.append("R_STANDING")
    if left_braking:
        flags.append("L_REGEN")
    if right_braking:
        flags.append("R_REGEN")

    return " | ".join(flags)


def decode_fault_level(fault):
    """Decode FaultLevel per CAN Command V0.1 spec."""
    fault_names = {
        0: "None",
        1: "WARNING (getting hot)",
        2: "DEGRADED (performance reduced)",
        3: "STOPPING (initiating stop)",
        4: "STOPPED (critical condition)",
    }
    return fault_names.get(fault, f"UNKNOWN({fault})")


def decode_drive_mode(mode_byte):
    """Decode MotorMode byte for DriveCommand (command version)."""
    mode_bits = mode_byte & 0x03
    mode_names = {0: "NoMode", 1: "Speed", 2: "Torque", 3: "Reserved"}
    braking_disabled = bool(mode_byte & 0x04)
    return f"{mode_names.get(mode_bits, '?')} | {'BRAKE_OFF' if braking_disabled else 'brake_on'}"


# ================================================================== #
#  Frame Parsers                                                       #
# ================================================================== #

def parse_temperature(data, name):
    """0x2D0 — Temperature telemetry."""
    if len(data) < 7:
        return f"[{name} TEMP] SHORT FRAME ({len(data)} bytes)"
    motor_L, motor_R, mosfet_L, mosfet_R, cpu_temp = struct.unpack(">hhbbb", data[:7])
    return (
        f"[{name} TEMP] "
        f"MotorL: {motor_L/10.0:5.1f}°C  MotorR: {motor_R/10.0:5.1f}°C  "
        f"MosfetL: {mosfet_L}°C  MosfetR: {mosfet_R}°C  CPU: {cpu_temp}°C"
    )


def parse_speed(data, name):
    """0x310 — Speed + Torque telemetry."""
    if len(data) < 8:
        return f"[{name} SPEED] SHORT FRAME ({len(data)} bytes)"
    speed_L, speed_R, torque_L, torque_R = struct.unpack(">hhhh", data[:8])
    return (
        f"[{name} SPEED] "
        f"L: {speed_L:+5d} RPM  R: {speed_R:+5d} RPM  "
        f"TorqueL: {torque_L:+4d} nM  TorqueR: {torque_R:+4d} nM"
    )


def parse_power(data, name):
    """0x320 — Power telemetry with full bitfield decode."""
    if len(data) < 7:
        return f"[{name} POWER] SHORT FRAME ({len(data)} bytes)"
    total_amps, total_volts, right_amps, left_amps, fault_level, motor_mode = (
        struct.unpack(">bHbbbB", data[:7])
    )
    mode_str = decode_motor_mode(motor_mode)
    fault_str = decode_fault_level(fault_level)
    return (
        f"[{name} POWER] "
        f"{total_volts/10.0:.1f}V  Total: {total_amps}A  "
        f"L: {left_amps}A  R: {right_amps}A\n"
        f"             Fault: {fault_str}\n"
        f"             Mode:  0x{motor_mode:02x} = {mode_str}"
    )


def parse_alarm(data, name):
    """0x500 — Alarm/fault codes."""
    if len(data) < 6:
        return f"[{name} ALARM] SHORT FRAME ({len(data)} bytes)"
    cpu_fault, left_fault, right_fault = struct.unpack(">HHH", data[:6])
    if cpu_fault == 0 and left_fault == 0 and right_fault == 0:
        return None  # No alarms — don't print
    return (
        f"[{name} ⚠ ALARM] "
        f"CPU: 0x{cpu_fault:04x}  LeftMotor: 0x{left_fault:04x}  "
        f"RightMotor: 0x{right_fault:04x}"
    )


# ================================================================== #
#  CAN Interface                                                       #
# ================================================================== #

def init_can():
    """Bring up CAN0 and return a bus object."""
    print("[INIT] Configuring CAN interface...")
    os.system("sudo ip link set can0 down 2>/dev/null")
    os.system("sudo ip link set can0 type can bitrate 500000")
    os.system("sudo ip link set can0 up")
    time.sleep(0.5)
    bus = can.interface.Bus(channel="can0", bustype="socketcan")
    print("[INIT] CAN0 ready at 500kbps")
    return bus


def send_drive(bus, throttle_left, throttle_right, node_id=2,
               motor_mode=0x01, torque_limit=20):
    """Send a DriveCommand frame."""
    throttle_left = int(max(-1000, min(1000, throttle_left)))
    throttle_right = int(max(-1000, min(1000, throttle_right)))
    payload = struct.pack(">hhBHB",
        throttle_left, throttle_right,
        motor_mode, torque_limit, 0
    )
    msg = can.Message(
        arbitration_id=DRIVE_BASE + node_id,
        data=payload,
        is_extended_id=False,
    )
    bus.send(msg)
    return throttle_left, throttle_right


def send_stop(bus, node_id=2):
    """Send zero throttle."""
    return send_drive(bus, 0, 0, node_id)


# ================================================================== #
#  Main Diagnostic Loop                                                #
# ================================================================== #

def run_diagnostic(bus, args):
    """Listen to all CAN frames. Optionally send drive commands."""
    print("=" * 70)
    print("  BLACK ROBOT — CAN Diagnostic Tool")
    print("  Listening for all CAN telemetry frames...")
    if args.drive is not None:
        print(f"  ⚠ DRIVE MODE: sending throttle {args.drive} to both motors")
        print(f"  Duration: {args.duration}s (then auto-stop)")
    if args.raw:
        print("  RAW MODE: showing hex dump of every frame")
    print("=" * 70)
    print()

    drive_start = None
    drive_active = False
    frame_count = 0
    last_telem = {}

    # Track mode for change detection
    prev_modes = {}

    try:
        while True:
            # --- Send drive command if requested ---
            if args.drive is not None and not drive_active:
                print(f"\n>>> SENDING DRIVE COMMAND: L={args.drive} R={args.drive} "
                      f"Mode=Speed TorqueLimit=20 nM")
                send_drive(bus, args.drive, args.drive, node_id=2)
                if not args.front_only:
                    send_drive(bus, args.drive, args.drive, node_id=4)
                drive_start = time.monotonic()
                drive_active = True

            # Keep sending drive commands (controller needs periodic updates)
            if drive_active:
                elapsed = time.monotonic() - drive_start
                if elapsed > args.duration:
                    print(f"\n>>> DRIVE DONE ({args.duration}s) — sending STOP")
                    send_stop(bus, 2)
                    send_stop(bus, 4)
                    drive_active = False
                    args.drive = None  # Don't restart
                else:
                    # Re-send every 20ms (50Hz)
                    send_drive(bus, args.drive, args.drive, node_id=2)
                    if not args.front_only:
                        send_drive(bus, args.drive, args.drive, node_id=4)

            # --- Read CAN frame ---
            msg = bus.recv(timeout=0.02)
            if msg is None:
                continue

            frame_count += 1
            base_id = msg.arbitration_id & 0xFF0
            node_id = msg.arbitration_id & 0x00F
            name = NODE_NAMES.get(node_id, f"NODE_{node_id}")

            # --- Raw hex dump ---
            if args.raw:
                hex_data = " ".join(f"{b:02x}" for b in msg.data)
                print(f"  [RAW] 0x{msg.arbitration_id:03x}  [{len(msg.data)}]  {hex_data}")

            # --- Decode known frames ---
            output = None
            try:
                if base_id == 0x2D0:
                    output = parse_temperature(msg.data, name)
                elif base_id == 0x310:
                    output = parse_speed(msg.data, name)
                elif base_id == 0x320:
                    output = parse_power(msg.data, name)
                    # Mode change detection
                    motor_mode = struct.unpack(">bHbbbB", msg.data[:7])[5]
                    key = f"mode_{node_id}"
                    if key in prev_modes and prev_modes[key] != motor_mode:
                        old = prev_modes[key]
                        print(f"  >>> {name} MODE CHANGE: "
                              f"0x{old:02x} ({decode_motor_mode(old)}) → "
                              f"0x{motor_mode:02x} ({decode_motor_mode(motor_mode)})")
                    prev_modes[key] = motor_mode
                elif base_id == 0x500:
                    output = parse_alarm(msg.data, name)
                else:
                    if args.raw:
                        output = f"[{name} UNKNOWN 0x{msg.arbitration_id:03x}]"
            except struct.error as e:
                output = f"[{name} PARSE ERROR] {e} data={msg.data.hex()}"

            if output:
                print(output)

    except KeyboardInterrupt:
        print(f"\n\n--- Received {frame_count} CAN frames ---")
        if drive_active:
            print(">>> Sending STOP before exit...")
            send_stop(bus, 2)
            send_stop(bus, 4)
            time.sleep(0.1)
        print("Done.")


# ================================================================== #
#  Entry Point                                                         #
# ================================================================== #

def main():
    parser = argparse.ArgumentParser(
        description="Black Robot CAN Diagnostic Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 can_diag.py                # Listen only (safe)
  python3 can_diag.py --drive 200    # Send throttle 200 for 3s
  python3 can_diag.py --drive 500 -t 5  # Throttle 500 for 5s
  python3 can_diag.py --raw          # Show raw hex of all frames
        """
    )
    parser.add_argument("--drive", type=int, default=None,
                        help="Send this throttle value to both motors (-1000..+1000)")
    parser.add_argument("-t", "--duration", type=float, default=3.0,
                        help="Drive duration in seconds (default: 3)")
    parser.add_argument("--raw", action="store_true",
                        help="Show raw hex dump of every CAN frame")
    parser.add_argument("--front-only", action="store_true",
                        help="Only send commands to front driver (NodeID 2)")
    args = parser.parse_args()

    bus = init_can()
    try:
        run_diagnostic(bus, args)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
