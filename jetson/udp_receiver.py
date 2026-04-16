"""
UDP Receiver — listens for Steam Deck joystick commands.

Runs as a background thread. Parses the custom UDP protocol and
writes raw joystick values into a shared state object.

Protocol (Steam Deck → Robot):
  [STX1][STX2][PayloadLen][SenderID][MsgType=3][Flag][Throttle_u16][Front_u16][Back_u16][CRC16]
"""

import struct
import socket
import threading
import time


class UDPReceiver:
    """Receives Steam Deck joystick commands over UDP."""

    # --- Protocol constants ---
    LISTEN_PORT = 8888
    MSG_DIRECT_CONTROL = 3
    DIRECT_CONTROL_FMT = "<BHH"   # 1 byte flag, 2 uint16 (throttle, front)   --- 5 bytes
    BASE_SIZE = 5                   # STX1 + STX2 + PayloadLen + SenderID + MsgType

    def __init__(self, shared_state):
        self._state = shared_state
        self._running = False
        self._thread = None
        self._last_printed = (None, None, None)  # throttle, front, gear

    # ------------------------------------------------------------------ #
    #  Public API                                                         #
    # ------------------------------------------------------------------ #

    def start(self):
        """Start the listener in a background daemon thread."""
        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        print(f"[UDP] Listening for Steam Deck on port {self.LISTEN_PORT}...")

    def stop(self):
        """Signal the listener to stop."""
        self._running = False

    # ------------------------------------------------------------------ #
    #  Internal                                                           #
    # ------------------------------------------------------------------ #

    def _listen_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)  # allow periodic check of _running flag
        sock.bind(("0.0.0.0", self.LISTEN_PORT))

        while self._running:
            try:
                data, addr = sock.recvfrom(1024)
                self._parse_packet(data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[UDP ERROR] {e}")

        sock.close()
        print("[UDP] Receiver stopped.")

    def _parse_packet(self, data):
        """Decode a Steam Deck direct-control packet."""
        if len(data) < self.BASE_SIZE + 2:
            return

        stx1, stx2, payload_len, sender_id, msg_type = struct.unpack(
            "<BBBBB", data[: self.BASE_SIZE]
        )

        if msg_type != self.MSG_DIRECT_CONTROL:
            return

        expected_len = self.BASE_SIZE + payload_len
        if len(data) != expected_len:
            return

        payload = data[self.BASE_SIZE : expected_len - 2]  # strip CRC

        try:
            if len(payload) == 5:
                control_flag, throttle_pwm, front_pwm = struct.unpack("<BHH", payload)
            elif len(payload) == 7:
                control_flag, throttle_pwm, front_pwm, _back_pwm = struct.unpack("<BHHH", payload)
            else:
                print(f"[UDP PARSE ERROR] Unknown payload length: {len(payload)}")
                return
        except struct.error as e:
            print(f"[UDP PARSE ERROR] {e}")
            return

        gear_low = bool(control_flag & 0x01)
        torque_mode = bool(control_flag & 0x02)

        # Write raw values into shared state
        with self._state.lock:
            self._state.throttle_pwm = throttle_pwm
            self._state.front_pwm = front_pwm
            self._state.gear_low = gear_low
            self._state.torque_mode = torque_mode
            self._state.last_udp_time = time.monotonic()

        # Only print when values change
        current = (throttle_pwm, front_pwm, gear_low, torque_mode)
        if current != self._last_printed:
            self._last_printed = current
            mode_str = "TORQUE" if torque_mode else "SPEED"
            print(
                f"[UDP] Mode: {mode_str} | Throttle: {throttle_pwm}  Steer: {front_pwm}  GearLow: {gear_low}"
            )
