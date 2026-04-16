#!/usr/bin/env python3
"""
Joystick Simulator — run on laptop to send commands to the robot.

Two modes:
  [1] DRIVE MODE  — W/S = drive, A/D = steer (normal driving)
  [2] DEBUG MODE  — independent left/right motor control

DEBUG MODE Controls:
  W / S           — Left motor speed up / down   (4 steps, max 50%)
  Arrow Up / Down — Right motor speed up / down   (4 steps, max 50%)
  SPACE           — Stop both motors
  Q / ESC         — Quit

Usage:
  python3 joystick_sim.py <robot_ip>
"""

import socket
import struct
import time
import sys
import select
import tty
import termios


# ================================================================== #
#  Configuration                                                       #
# ================================================================== #

ROBOT_IP   = "192.168.1.100"   # override with argv[1]
ROBOT_PORT = 8888
SEND_HZ    = 50

# PWM
PWM_CENTER = 1500
PWM_MIN    = 1000
PWM_MAX    = 2000

# Debug mode: 4 speed steps, max 50% (throttle 500 out of 1000)
MAX_THROTTLE = 500            # 50% of full range (1000)
NUM_STEPS    = 4
STEP_SIZE    = MAX_THROTTLE // NUM_STEPS   # 125 per step


# ================================================================== #
#  Packet Builder                                                      #
# ================================================================== #

def build_packet(throttle_pwm, front_pwm, gear_low=False, torque_mode=False):
    """
    Build a Steam Deck-compatible UDP packet.

    In the old logic mapping:
      front_pwm    → left motor   (left_cmd  = (front_pwm - 1500) * 2)
      throttle_pwm → right motor  (right_cmd = (throttle_pwm - 1500) * 2)
    """
    control_flag = 0x00
    if gear_low:
        control_flag |= 0x01
    if torque_mode:
        control_flag |= 0x02
        
    payload = struct.pack("<BHH", control_flag, throttle_pwm, front_pwm)
    payload_plus_crc_len = len(payload) + 2
    header = struct.pack("<BBBBB", 0xAA, 0x55, payload_plus_crc_len, 0x01, 0x03)
    crc = struct.pack("<H", 0x0000)
    return header + payload + crc


# ================================================================== #
#  Terminal Raw Input                                                  #
# ================================================================== #

def is_key_available():
    return select.select([sys.stdin], [], [], 0)[0] != []


def read_key():
    """Read a single keypress."""
    return sys.stdin.read(1)


# ================================================================== #
#  Display                                                             #
# ================================================================== #

def draw_motor_bar(label, throttle, step_num):
    """Draw a visual bar for a motor's speed level."""
    pct = (throttle / 1000.0) * 100
    direction = "FWD" if throttle > 0 else "REV" if throttle < 0 else "---"

    # Build step indicator:  [■ ■ ■ □]
    blocks = ""
    for i in range(1, NUM_STEPS + 1):
        if i <= abs(step_num):
            blocks += "■ "
        else:
            blocks += "□ "

    return f"  {label}: [{blocks.rstrip()}]  {direction} {abs(throttle):4d}/1000 ({abs(pct):5.1f}%)"


# ================================================================== #
#  Debug Mode — Independent Motor Control                              #
# ================================================================== #

def run_debug_mode(sock, robot_ip):
    print()
    print("  ┌─────────────────────────────────────┐")
    print("  │   DEBUG: Independent Motor Control   │")
    print("  ├─────────────────────────────────────┤")
    print("  │  W / S  = Left motor  ▲ / ▼         │")
    print("  │  E / D  = Right motor ▲ / ▼         │")
    print("  │  SPACE  = Stop both                  │")
    print("  │  T      = Toggle Speed/Torque Mode   │")
    print("  │  Q      = Quit                       │")
    print("  ├─────────────────────────────────────┤")
    print(f"  │  {NUM_STEPS} steps, max {MAX_THROTTLE}/1000 ({MAX_THROTTLE/10}%)        │")
    print("  └─────────────────────────────────────┘")
    print()

    left_step  = 0   # -4..+4
    right_step = 0   # -4..+4
    torque_mode = False
    dt = 1.0 / SEND_HZ
    last_send = 0

    last_display = 0
    DISPLAY_HZ = 2  # refresh display 2x per second

    while True:
        # --- Handle key input ---
        if is_key_available():
            key = read_key()

            if key in ('q', 'Q', '\x1b'):
                _send_direct(sock, robot_ip, 0, 0)
                break

            elif key == 'w':
                left_step = min(NUM_STEPS, left_step + 1)
            elif key == 's':
                left_step = max(-NUM_STEPS, left_step - 1)

            elif key == 'e':
                right_step = min(NUM_STEPS, right_step + 1)
            elif key == 'd':
                right_step = max(-NUM_STEPS, right_step - 1)

            elif key == 't':
                torque_mode = not torque_mode
                
            elif key == ' ':
                left_step = 0
                right_step = 0

        # --- Compute throttle values ---
        left_throttle  = left_step * STEP_SIZE
        right_throttle = right_step * STEP_SIZE

        # --- Send at 50 Hz ---
        now = time.monotonic()
        if now - last_send >= dt:
            _send_direct(sock, robot_ip, left_throttle, right_throttle, torque_mode)
            last_send = now

        # --- Display at 2 Hz (avoid scroll spam) ---
        if now - last_display >= 1.0 / DISPLAY_HZ:
            last_display = now
            l_pct = left_throttle / 10
            r_pct = right_throttle / 10
            mode_str = "TRQ" if torque_mode else "SPD"
            sys.stdout.write(
                f"\r  [{mode_str}] L: {left_step:+d} ({l_pct:+5.1f}%)  "
                f"R: {right_step:+d} ({r_pct:+5.1f}%)   "
            )
            sys.stdout.flush()

        time.sleep(0.01)


def _send_direct(sock, robot_ip, left_throttle, right_throttle, torque_mode=False):
    """
    Send independent motor commands via the UDP protocol.

    Old logic mapping in vehicle_controller:
      left_cmd  = (front_pwm - 1500) * 2   → front_pwm  = left_throttle/2 + 1500
      right_cmd = (throttle_pwm - 1500) * 2 → throttle_pwm = right_throttle/2 + 1500
    """
    front_pwm    = int(left_throttle / 2 + PWM_CENTER)
    throttle_pwm = int(right_throttle / 2 + PWM_CENTER)

    # Clamp to valid PWM range
    front_pwm    = max(PWM_MIN, min(PWM_MAX, front_pwm))
    throttle_pwm = max(PWM_MIN, min(PWM_MAX, throttle_pwm))

    packet = build_packet(throttle_pwm, front_pwm, gear_low=False, torque_mode=torque_mode)
    sock.sendto(packet, (robot_ip, ROBOT_PORT))


# ================================================================== #
#  Drive Mode — Normal Driving (for later)                             #
# ================================================================== #

def run_drive_mode(sock, robot_ip):
    print()
    print("  ┌──────────────────────────────────┐")
    print("  │   DRIVE: Normal Steering Mode     │")
    print("  ├──────────────────────────────────┤")
    print("  │  W / S  = Drive forward / reverse │")
    print("  │  A / D  = Steer left / right      │")
    print("  │  SPACE  = Emergency stop           │")
    print("  │  T      = Toggle Speed/Torque Mode│")
    print("  │  Q      = Quit                     │")
    print("  └──────────────────────────────────┘")
    print()

    throttle = PWM_CENTER
    steer    = PWM_CENTER
    torque_mode = False
    dt = 1.0 / SEND_HZ
    last_send = 0
    PWM_STEP = 25

    while True:
        if is_key_available():
            key = read_key()

            if key in ('q', 'Q', 'ESC'):
                packet = build_packet(PWM_CENTER, PWM_CENTER)
                sock.sendto(packet, (robot_ip, ROBOT_PORT))
                break
            elif key == 'w':
                throttle = min(PWM_MAX, throttle + PWM_STEP)
            elif key == 's':
                throttle = max(PWM_MIN, throttle - PWM_STEP)
            elif key == 'a':
                steer = max(PWM_MIN, steer - PWM_STEP)
            elif key == 'd':
                steer = min(PWM_MAX, steer + PWM_STEP)
            elif key == 't':
                torque_mode = not torque_mode
            elif key == ' ':
                throttle = PWM_CENTER
                steer = PWM_CENTER


        now = time.monotonic()
        if now - last_send >= dt:
            packet = build_packet(throttle, steer, gear_low=False, torque_mode=torque_mode)
            sock.sendto(packet, (robot_ip, ROBOT_PORT))
            last_send = now

            drive_pct = (throttle - PWM_CENTER) / 500.0 * 100
            steer_pct = (steer - PWM_CENTER) / 500.0 * 100
            mode_str = "TRQ" if torque_mode else "SPD"
            sys.stdout.write(
                f"\r  [{mode_str}] Drive: {drive_pct:+6.1f}%  |  Steer: {steer_pct:+6.1f}%  "
            )
            sys.stdout.flush()

        time.sleep(0.01)


# ================================================================== #
#  Main                                                                #
# ================================================================== #

# ================================================================== #
#  Hold Mode — Momentary key press at fixed speed                      #
# ================================================================== #

def run_hold_mode(sock, robot_ip):
    print()
    speed_str = input("  Enter speed % (e.g. 30): ").strip()
    try:
        speed_pct = float(speed_str)
    except ValueError:
        speed_pct = 30.0
    speed_pct = max(1, min(100, speed_pct))

    # Convert percentage to PWM offset from center
    pwm_offset = int((speed_pct / 100.0) * 500)  # 30% → 150

    print()
    print("  ┌─────────────────────────────────────┐")
    print("  │   HOLD MODE — Momentary Controls     │")
    print("  ├─────────────────────────────────────┤")
    print(f"  │  Speed: {speed_pct:.0f}% (PWM offset: ±{pwm_offset})     │")
    print("  ├─────────────────────────────────────┤")
    print("  │  HOLD W = Forward                    │")
    print("  │  HOLD S = Reverse                    │")
    print("  │  HOLD A = Steer Left                 │")
    print("  │  HOLD D = Steer Right                │")
    print("  │  W+D = Forward + Right, etc.          │")
    print("  │  Release = Stop                       │")
    print("  │  Q = Quit                             │")
    print("  └─────────────────────────────────────┘")
    print()
    print("  Hold keys to send commands...")
    print()

    dt = 1.0 / SEND_HZ
    last_send = 0
    last_display = 0

    # Track when each key was last seen — hold for 150ms after last press
    KEY_HOLD_TIME = 0.15  # seconds
    key_last_seen = {}    # key → monotonic time

    while True:
        now = time.monotonic()

        # --- Drain ALL available keys from buffer ---
        while is_key_available():
            key = read_key()
            if key in ('q', 'Q', '\x1b'):
                # Stop and quit
                packet = build_packet(PWM_CENTER, PWM_CENTER)
                sock.sendto(packet, (robot_ip, ROBOT_PORT))
                return
            key_last_seen[key.lower()] = now

        # --- Determine which keys are "held" (seen within KEY_HOLD_TIME) ---
        active_keys = {k for k, t in key_last_seen.items()
                       if (now - t) < KEY_HOLD_TIME}

        # --- Map active keys to drive/steer ---
        throttle = PWM_CENTER
        steer = PWM_CENTER

        if 'w' in active_keys:
            throttle = PWM_CENTER + pwm_offset    # forward
        elif 's' in active_keys:
            throttle = PWM_CENTER - pwm_offset    # reverse

        if 'd' in active_keys:
            steer = PWM_CENTER + pwm_offset       # right
        elif 'a' in active_keys:
            steer = PWM_CENTER - pwm_offset       # left

        # --- Send at fixed rate ---
        if now - last_send >= dt:
            packet = build_packet(throttle, steer)
            sock.sendto(packet, (robot_ip, ROBOT_PORT))
            last_send = now

        # --- Display at 5 Hz ---
        if now - last_display >= 0.2:
            last_display = now
            drive_pct = (throttle - PWM_CENTER) / 500.0 * 100
            steer_pct = (steer - PWM_CENTER) / 500.0 * 100

            keys_str = ",".join(sorted(active_keys)) if active_keys else "---"
            sys.stdout.write(
                f"\r  Keys: {keys_str:<10}  "
                f"Drive: {drive_pct:+6.1f}%  "
                f"Steer: {steer_pct:+6.1f}%   "
            )
            sys.stdout.flush()

        time.sleep(0.01)


# ================================================================== #
#  Main                                                                #
# ================================================================== #

def main():
    robot_ip = ROBOT_IP
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]

    print("=" * 50)
    print("  JOYSTICK SIMULATOR")
    print(f"  Target: {robot_ip}:{ROBOT_PORT}")
    print("=" * 50)
    print()
    print("  Select mode:")
    print("    [1] Drive Mode   (normal steering)")
    print("    [2] Debug Mode   (independent L/R motors)")
    print("    [3] Hold Mode    (momentary keys at fixed speed)")
    print()

    mode = input("  Mode (1/2/3): ").strip()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        if mode == "1":
            run_drive_mode(sock, robot_ip)
        elif mode == "3":
            run_hold_mode(sock, robot_ip)
        else:
            run_debug_mode(sock, robot_ip)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        sock.close()
        print("\n\n  Simulator stopped.")


if __name__ == "__main__":
    main()

