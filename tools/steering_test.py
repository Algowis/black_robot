#!/usr/bin/env python3
"""
Automated Steering Test — sends a predefined sequence of drive/steer
commands via UDP to test the 6-stage algorithm behavior.

Runs on the LAPTOP. Robot must be running main.py.

Usage:
  python3 steering_test.py <robot_ip>
"""

import socket
import struct
import time
import sys


ROBOT_PORT = 8888
SEND_HZ   = 50
PWM_CENTER = 1500
SPEED_PCT  = 25     # 25% speed


def build_packet(throttle_pwm, front_pwm, gear_low=False):
    """
    Build a Steam Deck-compatible UDP packet.
    [STX1][STX2][PayloadLen][SenderID][MsgType=3][Flag][Throttle_u16][Front_u16][CRC16]
    """
    control_flag = 0x01 if gear_low else 0x00
    payload = struct.pack("<BHH", control_flag, int(throttle_pwm), int(front_pwm))
    payload_plus_crc_len = len(payload) + 2
    header = struct.pack("<BBBBB", 0xAA, 0x55, payload_plus_crc_len, 0x01, 0x03)
    crc = struct.pack("<H", 0x0000)
    return header + payload + crc


def run_test(sock, robot_ip):
    pwm_offset = int((SPEED_PCT / 100.0) * 500)  # 25% → 125

    max_turn = 500  # 100% steering offset

    # Define the test sequence: (name, drive_offset, steer_offset, duration_sec)
    # Positive drive = forward, Positive steer = right, Negative steer = left
    sequence = [
        ("FORWARD ONLY",       +pwm_offset,           0,  5.0),
        ("FORWARD + LEFT",     +pwm_offset,   -max_turn,  5.0),
        ("FORWARD + RIGHT",    +pwm_offset,   +max_turn,  5.0),
        ("FORWARD ONLY",       +pwm_offset,           0,  5.0),
        ("BACKWARD ONLY",      -pwm_offset,           0,  5.0),
        ("STOP",                         0,           0,  3.0),
        ("BACKWARD ONLY",      -pwm_offset,           0,  5.0),
        ("BACKWARD + LEFT",    -pwm_offset,   -max_turn,  5.0),
        ("BACKWARD + RIGHT",   -pwm_offset,   +max_turn,  5.0),
        ("STOP",                         0,           0,  2.0),
        ("FORWARD ONLY",       +pwm_offset,           0,  5.0),
        ("FORWARD + RIGHT",    +pwm_offset,   +max_turn,  5.0),
        ("RIGHT ONLY (PIVOT)",           0,   +max_turn,  5.0),
        ("STOP",                         0,           0,  3.0),
        ("FORWARD ONLY",       +pwm_offset,           0,  5.0),
        ("FORWARD + LEFT",     +pwm_offset,   -max_turn,  5.0),
        ("LEFT ONLY (PIVOT)",            0,   -max_turn,  5.0),
    ]

    total_time = sum(s[3] for s in sequence)
    print(f"  Speed: {SPEED_PCT}% (PWM offset: ±{pwm_offset})")
    print(f"  Total test duration: {total_time:.0f} seconds")
    print()
    input("  Press ENTER to start the test...")
    print()

    dt = 1.0 / SEND_HZ
    elapsed_total = 0.0

    for step_idx, (name, drive_off, steer_off, duration) in enumerate(sequence):
        throttle = PWM_CENTER + drive_off
        steer    = PWM_CENTER + steer_off

        drive_pct = drive_off / 500.0 * 100
        steer_pct = steer_off / 500.0 * 100

        print(f"  ┌─ Step {step_idx + 1}/{len(sequence)}: {name}")
        print(f"  │  Drive: {drive_pct:+.0f}%  Steer: {steer_pct:+.0f}%  "
              f"(Throttle: {throttle}  Steer: {steer})")
        print(f"  │  Duration: {duration:.0f}s")

        t_start = time.monotonic()
        while True:
            now = time.monotonic()
            step_elapsed = now - t_start
            if step_elapsed >= duration:
                break

            # Send packet at 50 Hz
            packet = build_packet(throttle, steer)
            sock.sendto(packet, (robot_ip, ROBOT_PORT))

            # Progress bar
            bar_len = 30
            filled = int(bar_len * step_elapsed / duration)
            bar = "█" * filled + "░" * (bar_len - filled)
            sys.stdout.write(
                f"\r  │  [{bar}] {step_elapsed:.1f}/{duration:.0f}s  "
            )
            sys.stdout.flush()

            time.sleep(dt)

        elapsed_total += duration
        print(f"\r  │  [{'█' * 30}] {duration:.0f}/{duration:.0f}s  ✓")
        print(f"  └─ Done ({elapsed_total:.0f}s total)")
        print()

    # Final stop
    for _ in range(25):
        packet = build_packet(PWM_CENTER, PWM_CENTER)
        sock.sendto(packet, (robot_ip, ROBOT_PORT))
        time.sleep(dt)

    print("  ════════════════════════════════════")
    print("  TEST COMPLETE")
    print("  ════════════════════════════════════")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 steering_test.py <robot_ip>")
        sys.exit(1)

    robot_ip = sys.argv[1]

    print("=" * 50)
    print("  AUTOMATED STEERING TEST")
    print(f"  Target: {robot_ip}:{ROBOT_PORT}")
    print("=" * 50)
    print()
    print("  ⚠  ENSURE ROBOT IS ON A STAND OR IN SAFE AREA!")
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        run_test(sock, robot_ip)
    except KeyboardInterrupt:
        print("\n\n  ⚠ INTERRUPTED — sending stop...")
        for _ in range(50):
            packet = build_packet(PWM_CENTER, PWM_CENTER)
            sock.sendto(packet, (robot_ip, ROBOT_PORT))
            time.sleep(0.02)
        print("  Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
