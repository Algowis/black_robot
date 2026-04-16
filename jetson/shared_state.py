"""
Shared State — thread-safe container for inter-thread data.

All fields are protected by a single lock. Threads should hold the
lock only for the duration of reads/writes (no blocking calls under lock).
"""

import threading
import time


class SharedState:
    """Holds all shared data between UDP receiver, CAN telemetry, and control loop."""

    def __init__(self):
        self.lock = threading.Lock()

        # --- Joystick inputs (written by UDP receiver) ---
        self.throttle_pwm = 1500   # center = no drive
        self.front_pwm    = 1500   # center = no steer
        self.gear_low     = False
        self.torque_mode  = False  # Toggle for Speed vs Torque mode
        self.last_udp_time = 0.0

        # --- Speed telemetry (written by CAN telemetry reader) ---
        # Front driver (NodeID 2)
        self.front_speed_left_rpm  = 0
        self.front_speed_right_rpm = 0
        self.front_torque_left     = 0
        self.front_torque_right    = 0
        self.front_fault_level     = 0
        self.front_motor_mode      = 0x05  # 0x05 = normal speed mode

        # Rear driver (NodeID 4)
        self.rear_speed_left_rpm   = 0
        self.rear_speed_right_rpm  = 0
        self.rear_torque_left      = 0
        self.rear_torque_right     = 0
        self.rear_fault_level      = 0
        self.rear_motor_mode       = 0x05  # 0x05 = normal speed mode

        # Power
        self.battery_voltage       = 0.0

        # Timing
        self.last_telem_time       = 0.0

    def snapshot(self):
        """Return a frozen copy of all values (thread-safe read)."""
        with self.lock:
            return {
                # Joystick
                "throttle_pwm":   self.throttle_pwm,
                "front_pwm":      self.front_pwm,
                "gear_low":       self.gear_low,
                "torque_mode":    self.torque_mode,
                "last_udp_time":  self.last_udp_time,
                # Front speeds
                "front_speed_left_rpm":  self.front_speed_left_rpm,
                "front_speed_right_rpm": self.front_speed_right_rpm,
                "front_fault_level":     self.front_fault_level,
                "front_motor_mode":      self.front_motor_mode,
                # Rear speeds
                "rear_speed_left_rpm":   self.rear_speed_left_rpm,
                "rear_speed_right_rpm":  self.rear_speed_right_rpm,
                "rear_fault_level":      self.rear_fault_level,
                "rear_motor_mode":       self.rear_motor_mode,
                # Power
                "battery_voltage":       self.battery_voltage,
                # Timing
                "last_telem_time":       self.last_telem_time,
            }
