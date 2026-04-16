"""
CAN Telemetry Reader — decodes motor controller feedback.

Runs as a background thread. Reads every CAN frame off the bus,
decodes telemetry (speed, power, temperature, faults), and writes
the values into a shared state object.

CAN IDs handled:
  0x2D0 + NodeID  — Temperature telemetry
  0x310 + NodeID  — Speed + Torque telemetry
  0x320 + NodeID  — Power + FaultLevel telemetry
  0x500 + NodeID  — Alarms / Fault reports
"""

import struct
import threading
import time


# Minimum interval between prints for each message type (seconds)
PRINT_INTERVAL = 1.0


class CANTelemetry:
    """Reads and decodes motor controller CAN telemetry."""

    def __init__(self, bus, shared_state):
        self._bus = bus
        self._state = shared_state
        self._running = False
        self._thread = None
        # Track last print time per (base_id, node_id) to avoid spam
        self._last_print = {}

    # ------------------------------------------------------------------ #
    #  Public API                                                         #
    # ------------------------------------------------------------------ #

    def start(self):
        """Start reading telemetry in a background daemon thread."""
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print("[CAN TELEM] Reader started.")

    def stop(self):
        self._running = False

    # ------------------------------------------------------------------ #
    #  Internal                                                           #
    # ------------------------------------------------------------------ #

    def _read_loop(self):
        while self._running:
            msg = self._bus.recv(timeout=1.0)
            if msg is not None:
                self._parse_frame(msg)

        print("[CAN TELEM] Reader stopped.")

    def _parse_frame(self, msg):
        """Decode a single CAN frame from a motor controller."""
        base_id = msg.arbitration_id & 0xFF0
        node_id = msg.arbitration_id & 0x00F
        node_name = self._node_name(node_id)

        try:
            if base_id == 0x2D0:
                self._parse_temperature(msg.data, node_name, node_id)

            elif base_id == 0x310:
                self._parse_speed(msg.data, node_name, node_id)

            elif base_id == 0x320:
                self._parse_power(msg.data, node_name, node_id)

            elif base_id == 0x500:
                self._parse_alarms(msg.data, node_name, node_id)

        except struct.error:
            pass

    # ---------- Parsers ------------------------------------------------ #

    def _parse_temperature(self, data, name, node_id):
        """0x2D0 — Motor & controller temperatures."""
        motor_L, motor_R, mosfet_L, mosfet_R, cpu_temp = struct.unpack(
            ">hhbbb", data[:7]
        )
        with self._state.lock:
            self._state.last_telem_time = time.monotonic()
        if self._should_print(0x2D0, node_id):
            print(
                f"[CAN {name} TEMP] MotorL: {motor_L/10.0}°C  MotorR: {motor_R/10.0}°C  "
                f"MosfetL: {mosfet_L}°C  MosfetR: {mosfet_R}°C  CPU: {cpu_temp}°C"
            )

    def _parse_speed(self, data, name, node_id):
        """0x310 — Speed (RPM) and Torque (nM) per side."""
        speed_L, speed_R, torque_L, torque_R = struct.unpack(">hhhh", data[:8])

        # Store in shared state for control loop
        with self._state.lock:
            if node_id == 2:  # front driver
                self._state.front_speed_left_rpm = speed_L
                self._state.front_speed_right_rpm = speed_R
                self._state.front_torque_left = torque_L
                self._state.front_torque_right = torque_R
            elif node_id == 4:  # rear driver
                self._state.rear_speed_left_rpm = speed_L
                self._state.rear_speed_right_rpm = speed_R
                self._state.rear_torque_left = torque_L
                self._state.rear_torque_right = torque_R
            self._state.last_telem_time = time.monotonic()

        if self._should_print(0x310, node_id):
            print(
                f"[CAN {name} SPEED] L: {speed_L} RPM  R: {speed_R} RPM  "
                f"Torque L: {torque_L} nM  R: {torque_R} nM"
            )

    def _parse_power(self, data, name, node_id):
        """0x320 — Voltage, current, fault level, motor mode."""
        total_amps, total_volts, right_amps, left_amps, fault_level, motor_mode = (
            struct.unpack(">bHbbbB", data[:7])
        )

        # Store fault level, voltage, and motor mode
        with self._state.lock:
            if node_id == 2:
                self._state.front_fault_level = fault_level
                self._state.front_motor_mode = motor_mode
            elif node_id == 4:
                self._state.rear_fault_level = fault_level
                self._state.rear_motor_mode = motor_mode
            self._state.battery_voltage = total_volts / 10.0
            self._state.last_telem_time = time.monotonic()

        # (Mode tracking print removed - it spams the console with normal regen braking events)

        if self._should_print(0x320, node_id):
            print(
                f"[CAN {name} POWER] {total_volts/10.0}V  {total_amps}A  "
                f"Fault: {fault_level}  Mode: {motor_mode:#04x}"
            )

    def _parse_alarms(self, data, name, node_id):
        """0x500 — Alarm / fault codes."""
        cpu_fault, left_fault, right_fault = struct.unpack(">HHH", data[:6])
        if cpu_fault or left_fault or right_fault:
            print(
                f"[CAN {name} ALARM] CPU: {cpu_fault:#06x}  "
                f"LeftMotor: {left_fault:#06x}  RightMotor: {right_fault:#06x}"
            )

    # ---------- Helpers ------------------------------------------------ #

    def _should_print(self, base_id, node_id):
        """Rate-limit prints to once per PRINT_INTERVAL per message type."""
        key = (base_id, node_id)
        now = time.monotonic()
        last = self._last_print.get(key, 0.0)
        if now - last >= PRINT_INTERVAL:
            self._last_print[key] = now
            return True
        return False

    @staticmethod
    def _node_name(node_id):
        if node_id == 2:
            return "FRONT"
        elif node_id == 4:
            return "REAR "
        else:
            return f"NODE_{node_id}"
