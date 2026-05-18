#!/usr/bin/env python3
"""
Black Robot — ROS2 Joystick Bridge
===================================
Subscribes to /vehicle_120/joy on the Optimus GCS (ROS_DOMAIN_ID=1)
and translates Xbox controller axes into Black Robot drive commands
sent over UDP to the robot at 50 Hz.

Run on oper@192.168.120.169:
    source /opt/ros/humble/setup.bash
    ROS_DOMAIN_ID=1 python3 joystick_ros_bridge.py [--robot-ip X.X.X.X] [--dry-run]

    --dry-run    Print commands instead of sending UDP (Step 1 mode)

Xbox Axis Layout (from Optimus controller.py / XBoxController):
    axes[2] = Left Trigger  (+1.0 idle → -1.0 full) → reverse
    axes[3] = Right Stick X                          → steer
    axes[5] = Right Trigger (+1.0 idle → -1.0 full) → forward
    axes[6] = D-Pad X (-1=left, +1=right)            → speed cap
"""

import argparse
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


# ================================================================== #
#  Configuration                                                       #
# ================================================================== #

JOY_TOPIC     = "/vehicle_120/joy"
ROBOT_IP      = "192.168.120.20"
ROBOT_PORT    = 8888
SEND_HZ       = 50           # UDP send rate
STATUS_HZ     = 2            # Console status print rate
JOY_TIMEOUT_S = 0.5          # Seconds before treating joystick as disconnected

PWM_CENTER    = 1500
PWM_RANGE     = 500
DEADZONE      = 0.05

SPEED_DEFAULT = 0.30
SPEED_STEP    = 0.05
SPEED_MIN     = 0.05
SPEED_MAX     = 1.00

STEER_SCALE   = 1.00         # Steering is NOT limited by drive speed cap

AXIS_LEFT_TRIGGER  = 2
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_TRIGGER = 5
AXIS_DPAD_X        = 6


# ================================================================== #
#  JoyState — shared data container                                   #
# ================================================================== #

class JoyState:
    """Thread-safe snapshot of the latest /joy message."""

    def __init__(self):
        self._lock = threading.Lock()
        self.axes: list = []
        self.buttons: list = []
        self.last_update: float = 0.0

    def update(self, axes: list, buttons: list) -> None:
        with self._lock:
            self.axes = list(axes)
            self.buttons = list(buttons)
            self.last_update = time.monotonic()

    def snapshot(self) -> tuple:
        with self._lock:
            return list(self.axes), list(self.buttons), self.last_update


# ================================================================== #
#  JoySubscriber — ROS2 node                                          #
# ================================================================== #

class JoySubscriber(Node):
    """Subscribes to /vehicle_120/joy and writes into JoyState."""

    def __init__(self, state: JoyState):
        super().__init__("black_robot_bridge")
        self._state = state
        self.create_subscription(Joy, JOY_TOPIC, self._on_joy, 10)
        self.get_logger().info(f"Listening on {JOY_TOPIC}")

    def _on_joy(self, msg: Joy) -> None:
        self._state.update(msg.axes, msg.buttons)


# ================================================================== #
#  CommandTranslator — axis → PWM math                                #
# ================================================================== #

class CommandTranslator:
    """
    Translates raw joy axes into Black Robot PWM values.

    Drive:   Right Trigger → forward  |  Left Trigger → reverse
    Steer:   Right Stick X → left/right
    Speed:   D-Pad X       → adjust speed cap ±5% (edge-detected)
    """

    def __init__(self, speed_scale: float = SPEED_DEFAULT):
        self._speed_scale = speed_scale
        self._prev_dpad_x = 0.0

    @property
    def speed_scale(self) -> float:
        return self._speed_scale

    def translate(self, axes: list) -> tuple:
        """Return (throttle_pwm, front_pwm) in [1000, 2000]."""
        self._update_speed_scale(axes)

        rt = self._trigger_to_unit(self._axis(axes, AXIS_RIGHT_TRIGGER, 1.0))
        lt = self._trigger_to_unit(self._axis(axes, AXIS_LEFT_TRIGGER,  1.0))

        if rt > DEADZONE:
            drive = rt * self._speed_scale
        elif lt > DEADZONE:
            drive = -lt * self._speed_scale
        else:
            drive = 0.0

        # Steer uses its own scale (independent of drive speed cap).
        # Skid-steer pivot turns need full torque differential on the ground.
        steer_raw = self._axis(axes, AXIS_RIGHT_STICK_X, 0.0)
        steer = 0.0 if abs(steer_raw) < DEADZONE else steer_raw * STEER_SCALE

        return self._to_pwm(drive), self._to_pwm(steer)

    def _update_speed_scale(self, axes: list) -> None:
        dpad_x = self._axis(axes, AXIS_DPAD_X, 0.0)
        if dpad_x != self._prev_dpad_x:
            if dpad_x > 0.5:
                self._speed_scale = min(SPEED_MAX, self._speed_scale + SPEED_STEP)
                print(f"[SPEED] ▲  {self._speed_scale*100:.0f}%")
            elif dpad_x < -0.5:
                self._speed_scale = max(SPEED_MIN, self._speed_scale - SPEED_STEP)
                print(f"[SPEED] ▼  {self._speed_scale*100:.0f}%")
        self._prev_dpad_x = dpad_x

    @staticmethod
    def _trigger_to_unit(raw: float) -> float:
        return (1.0 - raw) / 2.0

    @staticmethod
    def _to_pwm(value: float) -> int:
        return max(1000, min(2000, int(PWM_CENTER + value * PWM_RANGE)))

    @staticmethod
    def _axis(axes: list, index: int, default: float = 0.0) -> float:
        return axes[index] if index < len(axes) else default


# ================================================================== #
#  PacketBuilder — binary UDP packet                                  #
# ================================================================== #

class PacketBuilder:
    """
    Builds the Steam Deck-compatible UDP packet expected by udp_receiver.py.

    Format:
      [0xAA][0x55][PayloadLen][SenderID=0x01][MsgType=0x03]
      [control_flag:u8][throttle_pwm:u16][front_pwm:u16]
      [CRC16:u16]
    """

    STX1      = 0xAA
    STX2      = 0x55
    SENDER_ID = 0x01
    MSG_TYPE  = 0x03  # MSG_VEHICLE_DIRECT_CONTROL

    def build(self, throttle_pwm: int, front_pwm: int,
              gear_low: bool = False, torque_mode: bool = False) -> bytes:
        flag = (0x01 if gear_low else 0) | (0x02 if torque_mode else 0)
        payload = struct.pack("<BHH", flag, throttle_pwm, front_pwm)
        header  = struct.pack("<BBBBB",
                              self.STX1, self.STX2,
                              len(payload) + 2,   # payload len includes CRC
                              self.SENDER_ID, self.MSG_TYPE)
        crc = struct.pack("<H", 0x0000)           # CRC not checked by robot
        return header + payload + crc


# ================================================================== #
#  BridgeSender — 50 Hz send loop                                    #
# ================================================================== #

class BridgeSender:
    """
    Background thread: reads translated commands every 1/SEND_HZ seconds
    and either sends them over UDP or prints them (dry-run mode).

    Safety: if no joy message arrives for JOY_TIMEOUT_S, sends neutral.
    """

    def __init__(self, state: JoyState, translator: CommandTranslator,
                 robot_ip: str, dry_run: bool = False):
        self._state      = state
        self._translator = translator
        self._builder    = PacketBuilder()
        self._robot_ip   = robot_ip
        self._dry_run    = dry_run
        self._sock       = None if dry_run else socket.socket(
                               socket.AF_INET, socket.SOCK_DGRAM)
        self._running    = False
        self._thread     = None
        self._prev_line  = ""
        self._last_print = 0.0

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._sock:
            self._sock.close()

    def _loop(self) -> None:
        dt = 1.0 / SEND_HZ
        while self._running:
            t0 = time.monotonic()
            axes, _, last_update = self._state.snapshot()
            age = time.monotonic() - last_update if last_update > 0 else 999.0

            if age > JOY_TIMEOUT_S:
                t_pwm, f_pwm, status = PWM_CENTER, PWM_CENTER, "TIMEOUT"
            else:
                t_pwm, f_pwm = self._translator.translate(axes)
                status = "OK"

            self._dispatch(t_pwm, f_pwm, status)
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    def _dispatch(self, t_pwm: int, f_pwm: int, status: str) -> None:
        """Send UDP packet (or print in dry-run) and log at STATUS_HZ."""
        if not self._dry_run:
            pkt = self._builder.build(t_pwm, f_pwm)
            try:
                self._sock.sendto(pkt, (self._robot_ip, ROBOT_PORT))
            except OSError as e:
                print(f"[UDP ERROR] {e}")

        now = time.monotonic()
        if now - self._last_print >= 1.0 / STATUS_HZ:
            self._last_print = now
            mode    = "DRY-RUN" if self._dry_run else "LIVE   "
            d_pct   = (t_pwm - PWM_CENTER) / PWM_RANGE * 100
            s_pct   = (f_pwm - PWM_CENTER) / PWM_RANGE * 100
            spd_pct = self._translator.speed_scale * 100
            line = (f"[{mode}|{status}]  "
                    f"Drive:{d_pct:+6.1f}%  Steer:{s_pct:+6.1f}%  "
                    f"Speed:{spd_pct:.0f}%  "
                    f"| PWM T={t_pwm} S={f_pwm}")
            if line != self._prev_line:
                print(line)
                self._prev_line = line


# ================================================================== #
#  BlackRobotBridge — orchestrator                                    #
# ================================================================== #

class BlackRobotBridge:
    """Wires together JoySubscriber, CommandTranslator, and BridgeSender."""

    def run(self) -> None:
        args = self._parse_args()

        state      = JoyState()
        translator = CommandTranslator(speed_scale=SPEED_DEFAULT)
        sender     = BridgeSender(state, translator,
                                  robot_ip=args.robot_ip,
                                  dry_run=args.dry_run)
        rclpy.init()
        node = JoySubscriber(state)
        sender.start()

        mode_label = "DRY-RUN (print only)" if args.dry_run else f"LIVE → {args.robot_ip}:{ROBOT_PORT}"
        print("=" * 62)
        print("  BLACK ROBOT BRIDGE  —  Step 2 (UDP)")
        print(f"  Mode     : {mode_label}")
        print(f"  Topic    : {JOY_TOPIC}")
        print(f"  Speed cap: {SPEED_DEFAULT*100:.0f}%  (D-Pad ◄/► to adjust ±5%)")
        print("  Controls : RT=forward  LT=reverse  RS-X=steer")
        print("  Ctrl+C to stop")
        print("=" * 62)

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\n[BRIDGE] Stopped.")
        finally:
            sender.stop()
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass

    @staticmethod
    def _parse_args() -> argparse.Namespace:
        parser = argparse.ArgumentParser(description="Black Robot ROS2 Bridge")
        parser.add_argument("--robot-ip", default=ROBOT_IP,
                            help=f"Robot IP address (default: {ROBOT_IP})")
        parser.add_argument("--dry-run", action="store_true",
                            help="Print commands instead of sending UDP")
        return parser.parse_args()


if __name__ == "__main__":
    BlackRobotBridge().run()



# ================================================================== #
#  BridgePrinter — Step 1 output (replaces UDP)                       #
# ================================================================== #

class BridgePrinter:
    """
    Runs a background thread that reads translated commands
    and prints them at PRINT_HZ.  Replace with BridgeUDPSender in Step 2.
    """

    def __init__(self, state: JoyState, translator: CommandTranslator):
        self._state = state
        self._translator = translator
        self._running = False
        self._thread = None
        self._prev_line = ""

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _loop(self) -> None:
        dt = 1.0 / PRINT_HZ
        while self._running:
            t0 = time.monotonic()
            axes, _, last_update = self._state.snapshot()
            age = time.monotonic() - last_update if last_update > 0 else 999.0

            if age > JOY_TIMEOUT_S:
                self._emit("TIMEOUT", 1500, 1500)
            else:
                t_pwm, f_pwm = self._translator.translate(axes)
                self._emit("OK", t_pwm, f_pwm)

            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    def _emit(self, status: str, t_pwm: int, f_pwm: int) -> None:
        drive_pct = (t_pwm - PWM_CENTER) / PWM_RANGE * 100
        steer_pct = (f_pwm - PWM_CENTER) / PWM_RANGE * 100
        spd_pct   = self._translator.speed_scale * 100
        line = (
            f"[{status}]  "
            f"Drive: {drive_pct:+6.1f}%  "
            f"Steer: {steer_pct:+6.1f}%  "
            f"Speed cap: {spd_pct:.0f}%  "
            f"| PWM  throttle={t_pwm}  steer={f_pwm}"
        )
        if line != self._prev_line:
            print(line)
            self._prev_line = line


# ================================================================== #
#  BlackRobotBridge — orchestrator                                    #
# ================================================================== #

class BlackRobotBridge:
    """Wires together JoySubscriber, CommandTranslator, and BridgePrinter."""

    def run(self) -> None:
        state      = JoyState()
        translator = CommandTranslator(speed_scale=SPEED_DEFAULT)
        printer    = BridgePrinter(state, translator)

        rclpy.init()
        node = JoySubscriber(state)
        printer.start()

        print("=" * 62)
        print("  BLACK ROBOT BRIDGE  —  Step 1 (print mode, no UDP)")
        print(f"  Topic    : {JOY_TOPIC}")
        print(f"  Speed cap: {SPEED_DEFAULT*100:.0f}%  (D-Pad ◄/► to adjust ±5%)")
        print("  Controls : RT=forward  LT=reverse  RS-X=steer")
        print("  Ctrl+C to stop")
        print("=" * 62)

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\n[BRIDGE] Stopped.")
        finally:
            printer.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    BlackRobotBridge().run()
