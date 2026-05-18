"""
vehicle_logic.py — Steam Deck Vehicle Controller: Logic Layer
==============================================================
Handles:
  • UDP packet encoding / decoding / CRC (matching MCU protocol)
  • Telemetry state management (heartbeat, GNSS)
  • Gamepad input via pygame (Steam Deck native controls)
  • RTSP camera capture via OpenCV in background threads

Network topology (from main.cpp):
    Sends only to   192.168.144.170  (Steam Deck)
    MCU:            192.168.144.100  (listens 8888, sends 5005)
    Front Camera:   192.168.144.101  RTSP :554
    Back Camera:    192.168.144.102  RTSP :554
    PTZ Camera:     192.168.144.103  RTSP :554
"""

import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional

import numpy as np

# ═══════════════════════════════════════════════
#  Protocol Constants (from packet_helper.h)
# ═══════════════════════════════════════════════
STX1 = 0xFD
STX2 = 0xFE

MSG_VEHICLE_CONTROL        = 0
MSG_GNSS                   = 1
MSG_HEARTBEAT              = 2
MSG_VEHICLE_DIRECT_CONTROL = 3

MSG_TYPE_NAMES = {
    0: "VEHICLE_CONTROL",
    1: "GNSS",
    2: "HEARTBEAT",
    3: "VEHICLE_DIRECT_CONTROL",
}

SENDER_ID_STEAM_DECK = 0x01

SERVO_MIN     = 1000
SERVO_NEUTRAL = 1500
SERVO_MAX     = 2000
HIGH_GEAR_PWM = 1700
LOW_GEAR_PWM  = 1150

# ═══════════════════════════════════════════════
#  Struct Formats (Little-Endian)
# ═══════════════════════════════════════════════
BASE_FMT  = "<BBBBB"
BASE_SIZE = struct.calcsize(BASE_FMT)  # 5

HEARTBEAT_FMT  = "<BHfffHHHHH"          # MCU struct is __attribute__((packed))
HEARTBEAT_SIZE = struct.calcsize(HEARTBEAT_FMT)  # 25

GNSS_FMT  = "<BlllH"                   # MCU struct is __attribute__((packed))
GNSS_SIZE = struct.calcsize(GNSS_FMT)  # 15

DIRECT_CONTROL_FMT  = "<BHHH"
DIRECT_CONTROL_SIZE = struct.calcsize(DIRECT_CONTROL_FMT)  # 7

# ═══════════════════════════════════════════════
#  CRC-16 / X.25  (ported from checksum.h)
# ═══════════════════════════════════════════════
_X25_INIT = 0xFFFF

def _crc_acc(byte: int, crc: int) -> int:
    byte &= 0xFF
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return crc & 0xFFFF

def crc_calculate(buf: bytes) -> int:
    crc = _X25_INIT
    for b in buf:
        crc = _crc_acc(b, crc)
    return crc

# ═══════════════════════════════════════════════
#  Data Classes
# ═══════════════════════════════════════════════
@dataclass
class VehicleStatus:
    gear_low:  bool = False
    lights_on: bool = False
    imu_ok:    bool = False
    gnss_ok:   bool = False

    @staticmethod
    def from_byte(b: int) -> "VehicleStatus":
        return VehicleStatus(
            gear_low  = bool(b & 0x01),
            lights_on = bool(b & 0x02),
            imu_ok    = bool(b & 0x04),
            gnss_ok   = bool(b & 0x08),
        )


@dataclass
class HeartbeatData:
    status:      VehicleStatus = field(default_factory=VehicleStatus)
    voltage:     float = 0.0
    roll:        float = 0.0
    pitch:       float = 0.0
    yaw:         float = 0.0
    throttle:    int   = SERVO_NEUTRAL
    front_steer: int   = SERVO_NEUTRAL
    back_steer:  int   = SERVO_NEUTRAL
    gear_pwm:    int   = 0
    timestamp:   float = 0.0


@dataclass
class GNSSData:
    satellites:   int   = 0
    latitude:     float = 0.0
    longitude:    float = 0.0
    ground_speed: float = 0.0
    timestamp:    float = 0.0


class SteeringMode(Enum):
    FRONT_ONLY     = auto()
    FRONT_AND_BACK = auto()
    BACK_ONLY      = auto()


STEERING_MODE_LABELS = {
    SteeringMode.FRONT_ONLY:     "FRONT",
    SteeringMode.FRONT_AND_BACK: "FRONT+BACK",
    SteeringMode.BACK_ONLY:      "BACK",
}


@dataclass
class ControlState:
    throttle:       int           = SERVO_NEUTRAL
    front_steering: int           = SERVO_NEUTRAL
    back_steering:  int           = SERVO_NEUTRAL
    gear_low:       bool          = True
    lights_on:      bool          = False
    steering_mode:  SteeringMode  = SteeringMode.FRONT_ONLY


# ═══════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════
def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def axis_to_servo(value: float, deadzone: float = 0.10) -> int:
    """Map normalised axis (-1..+1) to PWM (1000..2000) with deadzone."""
    if abs(value) < deadzone:
        return SERVO_NEUTRAL
    sign = 1.0 if value > 0 else -1.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone) * sign
    return int(_clamp(SERVO_NEUTRAL + scaled * 500, SERVO_MIN, SERVO_MAX))


# ═══════════════════════════════════════════════
#  Packet Packing
# ═══════════════════════════════════════════════
def pack_direct_control(ctrl: ControlState) -> bytes:
    control_flag = 0
    if ctrl.gear_low:
        control_flag |= 0x01
    if ctrl.lights_on:
        control_flag |= 0x02

    payload_no_crc = struct.pack(
        DIRECT_CONTROL_FMT,
        control_flag, ctrl.throttle, ctrl.front_steering, ctrl.back_steering,
    )
    payload_len_with_crc = DIRECT_CONTROL_SIZE + 2
    data_for_crc = struct.pack("<BB", SENDER_ID_STEAM_DECK, MSG_VEHICLE_DIRECT_CONTROL) + payload_no_crc
    crc = crc_calculate(data_for_crc)
    header = struct.pack(
        BASE_FMT,
        STX1, STX2, payload_len_with_crc, SENDER_ID_STEAM_DECK, MSG_VEHICLE_DIRECT_CONTROL,
    )
    return header + payload_no_crc + struct.pack("<H", crc)


# ═══════════════════════════════════════════════
#  Packet Parsing
# ═══════════════════════════════════════════════
def _parse_heartbeat(payload: bytes) -> Optional[HeartbeatData]:
    try:
        (status_b, voltage_raw, roll, pitch, yaw,
         throttle, front, back, gear, _crc) = struct.unpack(HEARTBEAT_FMT, payload)
        return HeartbeatData(
            status=VehicleStatus.from_byte(status_b),
            voltage=voltage_raw / 100.0,
            roll=roll, pitch=pitch, yaw=yaw,
            throttle=throttle, front_steer=front, back_steer=back,
            gear_pwm=gear, timestamp=time.time(),
        )
    except struct.error:
        return None


def _parse_gnss(payload: bytes) -> Optional[GNSSData]:
    try:
        sat, lat_raw, lon_raw, speed_raw, _crc = struct.unpack(GNSS_FMT, payload)
        return GNSSData(
            satellites=sat, latitude=lat_raw / 1e7,
            longitude=lon_raw / 1e7, ground_speed=speed_raw / 1000.0,
            timestamp=time.time(),
        )
    except struct.error:
        return None


# ═══════════════════════════════════════════════
#  RTSP Camera Capture Thread
# ═══════════════════════════════════════════════
class CameraStream:
    """
    Captures RTSP frames via OpenCV in a background thread.
    Access the latest frame via .get_frame() → numpy BGR or None.

    RTSP URLs should be set in vehicle_gui.py CAMERA_URLS dict.
    Reconnects automatically on failure.
    """

    def __init__(self, name: str, url: str, max_width: int = 0,
                 rtsp_transport: str = "udp"):
        self.name = name
        self.url = url
        self.max_width = max_width        # 0 = no downscale
        self.rtsp_transport = rtsp_transport  # "udp" or "tcp"
        self.frame: Optional[np.ndarray] = None
        self.connected = False
        self.fps: float = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name=f"Cam-{self.name}"
        )
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def get_frame(self) -> Optional[np.ndarray]:
        # frame reference is swapped atomically (Python GIL); no lock/copy needed
        return self.frame

    def _capture_loop(self):
        import cv2
        import os

        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            f"rtsp_transport;{self.rtsp_transport}"
            "|fflags;nobuffer"
            "|flags;low_delay"
            "|max_delay;500000"
            "|analyzeduration;100000"
            "|probesize;100000"
        )

        while self._running:
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if self.max_width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.max_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.max_width * 0.75))

            if not cap.isOpened():
                self.connected = False
                time.sleep(2.0)
                continue

            self.connected = True
            frame_count = 0
            t_start = time.monotonic()

            while self._running:
                # read() blocks until the next frame arrives — the tight loop
                # naturally drains any internal FFMPEG buffer so self.frame
                # always holds the most recent decoded frame.
                ret, frame = cap.read()
                if not ret:
                    self.connected = False
                    break

                if self.max_width > 0 and frame.shape[1] > self.max_width:
                    scale = self.max_width / frame.shape[1]
                    new_w = self.max_width
                    new_h = int(frame.shape[0] * scale)
                    frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

                # Atomic reference swap (GIL-safe) — no lock needed
                self.frame = frame

                frame_count += 1
                elapsed = time.monotonic() - t_start
                if elapsed >= 1.0:
                    self.fps = frame_count / elapsed
                    frame_count = 0
                    t_start = time.monotonic()

            cap.release()
            time.sleep(0.5)  # Wait before reconnect


# ═══════════════════════════════════════════════
#  Vehicle Controller (UDP TX/RX)
# ═══════════════════════════════════════════════
class VehicleController:
    """
    Two background threads:
      • RX: listens on port 5005 for heartbeat + GNSS
      • TX: sends control packets to MCU port 8888 at 20 Hz
    """

    def __init__(
        self,
        car_ip: str = "192.168.120.20",
        listen_port: int = 5005,
        send_port: int = 8888,
        send_rate_hz: int = 50,  # Increased from 20Hz to 50Hz for better steering consistency
        on_heartbeat: Optional[Callable[[HeartbeatData], None]] = None,
        on_gnss: Optional[Callable[[GNSSData], None]] = None,
    ):
        self.car_ip       = car_ip
        self.listen_port  = listen_port
        self.send_port    = send_port
        self.send_interval = 1.0 / send_rate_hz

        self._on_heartbeat = on_heartbeat
        self._on_gnss      = on_gnss

        self._lock     = threading.Lock()
        self.heartbeat = HeartbeatData()
        self.gnss      = GNSSData()
        self.control   = ControlState()
        self.connected = False
        self.last_rx_time: float = 0.0

        self.packets_sent: int = 0
        self.packets_recv: int = 0
        self.crc_errors:   int = 0

        self._rx_sock: Optional[socket.socket] = None
        self._tx_sock: Optional[socket.socket] = None
        self._running = False

    def start(self):
        if self._running:
            return
        self._running = True

        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self._rx_sock.settimeout(1.0)
        self._rx_sock.bind(("0.0.0.0", self.listen_port))

        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        threading.Thread(target=self._rx_loop, daemon=True, name="VehicleRX").start()
        threading.Thread(target=self._tx_loop, daemon=True, name="VehicleTX").start()

    def stop(self):
        self._running = False
        time.sleep(0.3)
        if self._rx_sock:
            self._rx_sock.close()
        if self._tx_sock:
            self._tx_sock.close()

    def _rx_loop(self):
        _rx_debug_interval = 5.0  # print RX status every N seconds
        _rx_debug_last = time.monotonic()
        _rx_raw_count = 0
        _rx_drop_reasons: dict = {}

        while self._running:
            try:
                data, addr = self._rx_sock.recvfrom(1024)
            except socket.timeout:
                # Periodic RX health check
                now = time.monotonic()
                if now - _rx_debug_last >= _rx_debug_interval:
                    if _rx_raw_count == 0:
                        print(f"[RX] No UDP packets received on port {self.listen_port} in last {_rx_debug_interval:.0f}s")
                    else:
                        print(f"[RX] Raw={_rx_raw_count}  Parsed={self.packets_recv}  CRC_Err={self.crc_errors}  Drops={_rx_drop_reasons}")
                    _rx_raw_count = 0
                    _rx_drop_reasons = {}
                    _rx_debug_last = now
                continue
            except OSError:
                break

            _rx_raw_count += 1

            if len(data) < BASE_SIZE + 2:
                _rx_drop_reasons["too_short"] = _rx_drop_reasons.get("too_short", 0) + 1
                continue
            if data[0] != STX1 or data[1] != STX2:
                _rx_drop_reasons["bad_stx"] = _rx_drop_reasons.get("bad_stx", 0) + 1
                # Log first few bytes for diagnosis
                if _rx_drop_reasons["bad_stx"] <= 3:
                    print(f"[RX] Bad STX from {addr}: {data[:8].hex()} (len={len(data)})")
                continue
            try:
                _, _, payload_len, sender_id, msg_type = struct.unpack(BASE_FMT, data[:BASE_SIZE])
            except struct.error:
                _rx_drop_reasons["hdr_parse"] = _rx_drop_reasons.get("hdr_parse", 0) + 1
                continue

            expected = BASE_SIZE + payload_len
            if len(data) != expected:
                _rx_drop_reasons["len_mismatch"] = _rx_drop_reasons.get("len_mismatch", 0) + 1
                if _rx_drop_reasons["len_mismatch"] <= 3:
                    msg_name = MSG_TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")
                    print(f"[RX] Length mismatch: msg={msg_name} expected={expected} got={len(data)} payload_len_field={payload_len}")
                continue

            data_to_crc = data[3:expected - 2]
            received_crc = struct.unpack("<H", data[expected - 2:])[0]
            if crc_calculate(data_to_crc) != received_crc:
                self.crc_errors += 1
                _rx_drop_reasons["crc"] = _rx_drop_reasons.get("crc", 0) + 1
                if _rx_drop_reasons["crc"] <= 3:
                    msg_name = MSG_TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")
                    calc_crc = crc_calculate(data_to_crc)
                    print(f"[RX] CRC fail: msg={msg_name} calc=0x{calc_crc:04X} recv=0x{received_crc:04X}")
                continue

            payload = data[BASE_SIZE:]
            self.packets_recv += 1
            self.last_rx_time = time.time()
            msg_name = MSG_TYPE_NAMES.get(msg_type, "UNKNOWN")

            if msg_name == "HEARTBEAT" and len(payload) == HEARTBEAT_SIZE:
                hb = _parse_heartbeat(payload)
                if hb:
                    with self._lock:
                        self.heartbeat = hb
                        self.connected = True
                    if self._on_heartbeat:
                        self._on_heartbeat(hb)
                    # Log first successful heartbeat
                    if self.packets_recv <= 1:
                        print(f"[RX] First heartbeat from {addr}: V={hb.voltage:.1f}V "
                              f"IMU={'OK' if hb.status.imu_ok else 'FAIL'} "
                              f"R={hb.roll:.1f} P={hb.pitch:.1f} Y={hb.yaw:.1f}")

            elif msg_name == "GNSS" and len(payload) == GNSS_SIZE:
                gd = _parse_gnss(payload)
                if gd:
                    with self._lock:
                        self.gnss = gd
                    if self._on_gnss:
                        self._on_gnss(gd)
            else:
                _rx_drop_reasons["unknown_or_size"] = _rx_drop_reasons.get("unknown_or_size", 0) + 1
                if _rx_drop_reasons["unknown_or_size"] <= 3:
                    print(f"[RX] Unhandled: msg={msg_name} type={msg_type} payload_len={len(payload)} "
                          f"(expected HB={HEARTBEAT_SIZE} GNSS={GNSS_SIZE})")

    def _tx_loop(self):
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                ctrl = ControlState(
                    throttle=self.control.throttle,
                    front_steering=self.control.front_steering,
                    back_steering=self.control.back_steering,
                    gear_low=self.control.gear_low,
                    lights_on=self.control.lights_on,
                    steering_mode=self.control.steering_mode,
                )
                # Failsafe: if no heartbeat from MCU for 2.5s, zero controls
                if self.last_rx_time > 0 and (time.time() - self.last_rx_time > 2.5):
                    ctrl.throttle = SERVO_NEUTRAL
                    ctrl.front_steering = SERVO_NEUTRAL
                    ctrl.back_steering = SERVO_NEUTRAL

            pkt = pack_direct_control(ctrl)
            try:
                self._tx_sock.sendto(pkt, (self.car_ip, self.send_port))
                self.packets_sent += 1
            except OSError:
                pass

            elapsed = time.monotonic() - t0
            remaining = self.send_interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ── Thread-safe setters ───────────────

    def set_throttle(self, pwm: int):
        with self._lock:
            self.control.throttle = int(_clamp(pwm, SERVO_MIN, SERVO_MAX))

    def set_front_steering(self, pwm: int):
        with self._lock:
            self.control.front_steering = int(_clamp(pwm, SERVO_MIN, SERVO_MAX))

    def set_back_steering(self, pwm: int):
        with self._lock:
            self.control.back_steering = int(_clamp(pwm, SERVO_MIN, SERVO_MAX))

    def set_gear_low(self, low: bool):
        with self._lock:
            self.control.gear_low = low

    def set_lights(self, on: bool):
        with self._lock:
            self.control.lights_on = on

    def toggle_gear(self):
        with self._lock:
            self.control.gear_low = not self.control.gear_low

    def toggle_lights(self):
        with self._lock:
            self.control.lights_on = not self.control.lights_on

    def set_steering_mode(self, mode: SteeringMode):
        with self._lock:
            self.control.steering_mode = mode

    def reset_servos(self):
        with self._lock:
            self.control.throttle       = SERVO_NEUTRAL
            self.control.front_steering = SERVO_NEUTRAL
            self.control.back_steering  = SERVO_NEUTRAL

    def get_snapshot(self):
        with self._lock:
            return (
                HeartbeatData(**self.heartbeat.__dict__),
                GNSSData(**self.gnss.__dict__),
                ControlState(
                    throttle=self.control.throttle,
                    front_steering=self.control.front_steering,
                    back_steering=self.control.back_steering,
                    gear_low=self.control.gear_low,
                    lights_on=self.control.lights_on,
                    steering_mode=self.control.steering_mode,
                ),
                self.connected,
            )


# ═══════════════════════════════════════════════
#  Pygame Gamepad Manager
# ═══════════════════════════════════════════════
class GamepadManager:
    """
    Reads Razer Wolverine V2 gamepad via pygame.

    Control Mapping (Black Robot skid-steer):
    ─────────────────────────────────────────
    Right Trigger   → Forward drive   (idle=+1, full=-1 in pygame)
    Left Trigger    → Reverse drive   (idle=+1, full=-1 in pygame)
    Right Stick X   → Steer left/right
    D-Pad X (axis)  → Speed cap ±5%   (right=+1 = faster)

    LB              → Toggle gear LOW/HIGH
    B button        → Cycle camera view
    ─────────────────────────────────────────
    Note: steering mode buttons (Y/X/A) are not used on skid-steer.
    """

    # Razer Wolverine V2 axis indices (standard XInput mapping via pygame):
    AXIS_LEFT_TRIGGER  = 2   # +1.0 idle → -1.0 full press
    AXIS_RIGHT_TRIGGER = 5   # +1.0 idle → -1.0 full press
    AXIS_RIGHT_STICK_X = 3   # -1.0 = left, +1.0 = right
    AXIS_DPAD_X        = 6   # -1.0 = left, +1.0 = right

    # Button indices (Razer Wolverine V2 via XInput)
    BTN_LB = 4   # Left bumper  → toggle gear
    BTN_B  = 1   # B button     → cycle camera

    # Speed cap — adjusted by D-Pad X (matches joystick_ros_bridge.py)
    SPEED_DEFAULT = 0.30
    SPEED_STEP    = 0.05
    SPEED_MIN     = 0.05
    SPEED_MAX     = 1.00
    STEER_SCALE   = 1.00   # steer NOT capped by speed (pivot needs full torque)

    DEADZONE = 0.05

    def __init__(self, controller: VehicleController, deadzone: float = 0.10,
                 on_cycle_camera: Optional[Callable[[], None]] = None):
        self.controller = controller
        self.deadzone = deadzone
        self.on_cycle_camera = on_cycle_camera
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.joystick = None
        self.joystick_name: str = ""

    def init_pygame(self) -> bool:
        """Initialise pygame joystick subsystem and grab first gamepad."""
        import pygame
        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            print("[GamepadManager] No joystick found.")
            return False

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.joystick_name = self.joystick.get_name()
        print(f"[GamepadManager] Found: {self.joystick_name}")
        print(f"  Axes: {self.joystick.get_numaxes()}, Buttons: {self.joystick.get_numbuttons()}")
        return True

    def start(self):
        if self._running or self.joystick is None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="Gamepad")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def _poll_loop(self):
        import pygame
        clock = pygame.time.Clock()
        prev_dpad_x = 0.0
        speed_scale = self.SPEED_DEFAULT

        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    self._handle_button(event.button)

            if self.joystick:
                # ── Triggers → drive (matching joystick_ros_bridge.py) ── #
                # Triggers: +1.0 idle, -1.0 fully pressed → unit = (1 - raw) / 2
                rt = (1.0 - self.joystick.get_axis(self.AXIS_RIGHT_TRIGGER)) / 2.0
                lt = (1.0 - self.joystick.get_axis(self.AXIS_LEFT_TRIGGER))  / 2.0

                if rt > self.DEADZONE:
                    drive = rt * speed_scale
                elif lt > self.DEADZONE:
                    drive = -lt * speed_scale
                else:
                    drive = 0.0

                # ── Right Stick X → steer (full range, not speed-capped) ── #
                steer_raw = self.joystick.get_axis(self.AXIS_RIGHT_STICK_X)
                steer = steer_raw * self.STEER_SCALE if abs(steer_raw) > self.DEADZONE else 0.0

                # ── D-Pad X → speed cap adjustment (edge-detected) ── #
                dpad_x = self.joystick.get_axis(self.AXIS_DPAD_X)
                if dpad_x != prev_dpad_x:
                    if dpad_x > 0.5:
                        speed_scale = min(self.SPEED_MAX, speed_scale + self.SPEED_STEP)
                        print(f"[SPEED] ▲  {speed_scale*100:.0f}%")
                    elif dpad_x < -0.5:
                        speed_scale = max(self.SPEED_MIN, speed_scale - self.SPEED_STEP)
                        print(f"[SPEED] ▼  {speed_scale*100:.0f}%")
                    prev_dpad_x = dpad_x

                # ── Convert to PWM and write atomically ── #
                throttle_pwm = int(_clamp(SERVO_NEUTRAL + drive * 500, SERVO_MIN, SERVO_MAX))
                steer_pwm    = int(_clamp(SERVO_NEUTRAL + steer * 500, SERVO_MIN, SERVO_MAX))

                with self.controller._lock:
                    self.controller.control.throttle       = throttle_pwm
                    self.controller.control.front_steering = steer_pwm
                    self.controller.control.back_steering  = SERVO_NEUTRAL

            clock.tick(60)

    def _handle_button(self, button: int):
        if button == self.BTN_LB:
            # Left Bumper → toggle gear LOW / HIGH
            self.controller.toggle_gear()
            gear = "LOW" if self.controller.control.gear_low else "HIGH"
            print(f"[GEAR] → {gear}")

        elif button == self.BTN_B:
            # B button → cycle camera
            if self.on_cycle_camera:
                self.on_cycle_camera()

        # All other buttons intentionally ignored (no steering modes on skid-steer)


# ═══════════════════════════════════════════════
#  Quick Self-Test
# ═══════════════════════════════════════════════
if __name__ == "__main__":
    print("=== Vehicle Logic Self-Test ===")
    print(f"Base header: {BASE_SIZE}B | Heartbeat: {HEARTBEAT_SIZE}B | GNSS: {GNSS_SIZE}B | DirectCtrl: {DIRECT_CONTROL_SIZE}B")

    ctrl = ControlState(throttle=1600, front_steering=1400, gear_low=True, lights_on=True)
    pkt = pack_direct_control(ctrl)
    print(f"Packed control packet ({len(pkt)} bytes): {pkt.hex()}")

    for v in [-1.0, -0.5, 0.0, 0.05, 0.5, 1.0]:
        print(f"  axis {v:+.2f} → servo {axis_to_servo(v)}")

    print("\nRun the full app with:  python vehicle_gui.py")
