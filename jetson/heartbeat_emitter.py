"""
Heartbeat Emitter — sends Optimus-compatible status packets to the GCS.

Runs as a background thread (5 Hz by default).
Reads SharedState and transmits a HEARTBEAT (MSG_TYPE=2) packet to the
Optimus GCS machine (vehicle_logic.VehicleController listens on port 5005).

Effect on vehicle_gui.py HUD:
  • CONNECTED dot  → green  (was always red without this)
  • ⚡ Voltage      → live battery voltage from CAN telemetry
  • GEAR: LOW/HIGH → mirrors shared_state.gear_low
  • THR / FNT bars → echo of last received throttle_pwm / front_pwm

Packet format — matching vehicle_logic.py HEARTBEAT_FMT = "<BHfffHHHHH":
  [STX1=0xFD][STX2=0xFE][payload_len=25][SENDER_ID=0x02][MSG_TYPE=2]  ← 5-byte base header
  [status_b:u8][voltage_raw:u16][roll:f][pitch:f][yaw:f]              ← 11 bytes body part 1
  [throttle:u16][front_steer:u16][back_steer:u16][gear_pwm:u16]       ← 8 bytes body part 2
  [crc16:u16]                                                          ← 2 bytes CRC
  Total: 30 bytes
"""

import socket
import struct
import threading
import time

from robot_config import OPTIMUS_GCS_IP, HEARTBEAT_PORT, HEARTBEAT_HZ


# ── Protocol constants (must match vehicle_logic.py exactly) ──────────────── #

_STX1            = 0xFD
_STX2            = 0xFE
_MSG_HEARTBEAT   = 2
_SENDER_ID       = 0x02          # 0x01 = Steam Deck; 0x02 = vehicle/Jetson

_BASE_FMT        = "<BBBBB"
_BODY_FMT        = "<BHfffHHHH"  # 9 fields = 23 bytes (CRC appended separately)
_PAYLOAD_LEN     = 25            # body(23) + crc(2) — matches vehicle_logic.HEARTBEAT_SIZE

_LOW_GEAR_PWM    = 1150
_HIGH_GEAR_PWM   = 1700
_SERVO_NEUTRAL   = 1500

_X25_INIT        = 0xFFFF


class HeartbeatEmitter:
    """
    Periodically sends a HEARTBEAT UDP packet to the Optimus GCS so that
    vehicle_gui.py shows live connection status, voltage and gear state.
    """

    def __init__(self, shared_state,
                 target_ip:   str = OPTIMUS_GCS_IP,
                 target_port: int = HEARTBEAT_PORT,
                 emit_hz:     int = HEARTBEAT_HZ):
        self._state       = shared_state
        self._target_ip   = target_ip
        self._target_port = target_port
        self._interval    = 1.0 / emit_hz
        self._running     = False
        self._thread: threading.Thread | None = None
        self._sock:   socket.socket    | None = None

    # ── Public API ──────────────────────────────────────────────────────────── #

    def start(self) -> None:
        """Open the UDP socket and start the background emit thread."""
        self._running = True
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread  = threading.Thread(target=self._emit_loop, daemon=True,
                                         name="HeartbeatEmitter")
        self._thread.start()
        print(f"[HB EMIT] Heartbeat → {self._target_ip}:{self._target_port} "
              f"at {int(1 / self._interval)} Hz")

    def stop(self) -> None:
        """Signal the thread to stop and close the socket."""
        self._running = False
        if self._sock:
            self._sock.close()

    # ── Internal ─────────────────────────────────────────────────────────────  #

    def _emit_loop(self) -> None:
        while self._running:
            t0 = time.monotonic()
            try:
                pkt = self._build_packet()
                self._sock.sendto(pkt, (self._target_ip, self._target_port))
            except Exception as exc:
                print(f"[HB EMIT ERROR] {exc}")
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, self._interval - elapsed))

    def _build_packet(self) -> bytes:
        """Assemble one 30-byte HEARTBEAT packet from current SharedState."""
        snap = self._state.snapshot()

        # ── Status byte ───────────────────────────────────────────────────── #
        status_b = 0x00
        if snap["gear_low"]:
            status_b |= 0x01   # bit 0 = gear_low
        # bit 1 = lights_on  → 0 (no lights on Black Robot)
        # bit 2 = imu_ok     → 0 (no IMU)
        # bit 3 = gnss_ok    → 0 (no GPS)

        # ── Body fields ───────────────────────────────────────────────────── #
        voltage_raw = int(snap["battery_voltage"] * 100)   # e.g. 48.0 V → 4800
        gear_pwm    = _LOW_GEAR_PWM if snap["gear_low"] else _HIGH_GEAR_PWM

        body = struct.pack(
            _BODY_FMT,
            status_b,
            voltage_raw,
            0.0, 0.0, 0.0,                  # roll, pitch, yaw — no IMU
            snap["throttle_pwm"],            # servo echo
            snap["front_pwm"],               # servo echo
            _SERVO_NEUTRAL,                  # back_steer unused on skid-steer
            gear_pwm,
        )

        # ── CRC-16/X.25 over [sender_id, msg_type, body] ─────────────────── #
        data_for_crc = struct.pack("<BB", _SENDER_ID, _MSG_HEARTBEAT) + body
        crc = self._crc_calculate(data_for_crc)

        # ── Assemble packet ───────────────────────────────────────────────── #
        header = struct.pack(_BASE_FMT,
                             _STX1, _STX2,
                             _PAYLOAD_LEN,   # payload_len = body(23) + crc(2) = 25
                             _SENDER_ID, _MSG_HEARTBEAT)
        return header + body + struct.pack("<H", crc)

    # ── CRC helpers (ported from vehicle_logic.py checksum.h) ──────────────── #

    @staticmethod
    def _crc_acc(byte: int, crc: int) -> int:
        byte &= 0xFF
        tmp   = byte ^ (crc & 0xFF)
        tmp   = (tmp ^ (tmp << 4)) & 0xFF
        crc   = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        return crc & 0xFFFF

    @classmethod
    def _crc_calculate(cls, buf: bytes) -> int:
        crc = _X25_INIT
        for b in buf:
            crc = cls._crc_acc(b, crc)
        return crc
