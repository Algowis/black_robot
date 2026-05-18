"""
ros2_drive_bridge.py — Optimus ROS2 → Black Robot UDP bridge.

Runs INSIDE the onboard_optimus Docker container where ROS2 is available.
Subscribes to the Optimus drive_control and gear_control ROS2 topics,
converts them to the Black Robot UDP protocol, and sends to main.py on
the host at port 8888.

Conversion:
  throttle  (0–255, Optimus) + gear (F/N/R) → throttle_pwm (1000–2000)
  steering (-100..+100,      Optimus)         → front_pwm   (1000–2000)

Usage (inside container):
    python3 ros2_drive_bridge.py
    python3 ros2_drive_bridge.py --speed 0.5   # 50% max speed
"""

import argparse
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from optimus_interfaces.msg import DriveControl, GearControl
from backend.common.common_enums import Gear

# ── Protocol constants (must match udp_receiver.py exactly) ────────────── #
_STX1             = 0xFD
_STX2             = 0xFE
_SENDER_ID        = 0x01
_MSG_DIRECT       = 3
_SERVO_NEUTRAL    = 1500
_SERVO_MIN        = 1000
_SERVO_MAX        = 2000
_DEFAULT_SPEED    = 0.30     # 30% of max speed on start (D-pad tunes it live)
_X25_INIT         = 0xFFFF

VEHICLE_ID        = 120
DRIVE_TOPIC       = f"/vehicle_{VEHICLE_ID}/drive_control"
GEAR_TOPIC        = f"/vehicle_{VEHICLE_ID}/gear_control"
UDP_HOST          = "127.0.0.1"   # host network — main.py on host
UDP_PORT          = 8888


def _crc_acc(byte: int, crc: int) -> int:
    byte &= 0xFF
    tmp   = byte ^ (crc & 0xFF)
    tmp   = (tmp ^ (tmp << 4)) & 0xFF
    crc   = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return crc & 0xFFFF


def _crc_calc(buf: bytes) -> int:
    crc = _X25_INIT
    for b in buf:
        crc = _crc_acc(b, crc)
    return crc


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ROS2DriveBridge(Node):
    """
    Bridges Optimus ROS2 drive/gear commands to the Black Robot UDP protocol.

    drive_control fields used:
        throttle  (uint8, 0–255)   — forward speed magnitude
        steering  (int8, -100..+100) — left/right
        brakes    (uint8, 0–255)   — unused (skid-steer stops via throttle=0)

    gear_control field used:
        gear  (int8, Gear enum)    — FORWARD / NEUTRAL / REVERSE / PARKING
    """

    def __init__(self, speed_scale: float = _DEFAULT_SPEED):
        super().__init__("ros2_drive_bridge")
        self._speed  = speed_scale
        self._gear   = Gear.NEUTRAL
        self._lock   = threading.Lock()
        self._sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        cb_drive = MutuallyExclusiveCallbackGroup()
        cb_gear  = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            DriveControl, DRIVE_TOPIC,
            self._on_drive, 10, callback_group=cb_drive,
        )
        self.create_subscription(
            GearControl, GEAR_TOPIC,
            self._on_gear, 10, callback_group=cb_gear,
        )
        self.get_logger().info(
            f"Bridge started — {DRIVE_TOPIC} + {GEAR_TOPIC}"
            f" → UDP {UDP_HOST}:{UDP_PORT}  speed={speed_scale*100:.0f}%"
        )

    # ── ROS2 callbacks ──────────────────────────────────────────────────── #

    def _on_gear(self, msg: GearControl) -> None:
        with self._lock:
            try:
                self._gear = Gear(msg.gear)
            except ValueError:
                self._gear = Gear.NEUTRAL
        self.get_logger().info(f"[GEAR] → {self._gear.name}")

    def _on_drive(self, msg: DriveControl) -> None:
        with self._lock:
            gear  = self._gear
            speed = self._speed

        # ── Throttle: direction from gear, magnitude from throttle field ── #
        throttle_norm = msg.throttle / 255.0

        if gear == Gear.FORWARD:
            drive = throttle_norm * speed
        elif gear == Gear.REVERSE:
            drive = -throttle_norm * speed
        else:
            drive = 0.0             # NEUTRAL / PARKING → stop

        # ── Steering: -100..+100 → -1..+1, keep at full scale ─────────── #
        steer = _clamp(msg.steering / 100.0, -1.0, 1.0)

        # ── Convert to PWM ─────────────────────────────────────────────── #
        throttle_pwm = int(_clamp(_SERVO_NEUTRAL + drive * 500, _SERVO_MIN, _SERVO_MAX))
        steer_pwm    = int(_clamp(_SERVO_NEUTRAL + steer * 500, _SERVO_MIN, _SERVO_MAX))

        pkt = self._build_packet(throttle_pwm, steer_pwm)
        self._sock.sendto(pkt, (UDP_HOST, UDP_PORT))

    # ── Packet builder ──────────────────────────────────────────────────── #

    def _build_packet(self, throttle_pwm: int, front_pwm: int) -> bytes:
        """Build a 14-byte DIRECT_CONTROL UDP packet (matching udp_receiver.py)."""
        control_flag = 0x00          # gear_low=0 (high gear), torque_mode=0
        data = struct.pack("<BHHH", control_flag, throttle_pwm, front_pwm, _SERVO_NEUTRAL)
        # payload_len = len(data) + 2 (for CRC) = 9
        payload_len = len(data) + 2
        crc_input = struct.pack("<BB", _SENDER_ID, _MSG_DIRECT) + data
        crc = _crc_calc(crc_input)
        header = struct.pack("<BBBBB", _STX1, _STX2, payload_len, _SENDER_ID, _MSG_DIRECT)
        return header + data + struct.pack("<H", crc)


def main() -> None:
    parser = argparse.ArgumentParser(description="Optimus ROS2 → Black Robot UDP bridge")
    parser.add_argument("--speed", type=float, default=_DEFAULT_SPEED,
                        help=f"Speed scale 0.0–1.0 (default {_DEFAULT_SPEED})")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ROS2DriveBridge(speed_scale=args.speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
