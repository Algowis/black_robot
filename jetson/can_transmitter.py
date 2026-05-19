"""
CAN Transmitter — sends DriveCommand frames to motor controllers.

Provides the interface for sending drive commands over CAN.
Currently uses the OLD direct PWM-to-throttle mapping.
This will be replaced with the 6-stage steering algorithm.

DriveCommand frame (0x300 + NodeID):
  Byte 0-1: ThrottleLeft   (int16)  -1000..+1000
  Byte 2-3: ThrottleRight  (int16)  -1000..+1000
  Byte 4:   MotorMode      (uint8)  0x01 = Speed mode
  Byte 5-6: Limit          (uint16) MaxTorque in nM (when Speed mode)
  Byte 7:   Reserved       (uint8)  0
"""

import struct
import can

from robot_config import (
    MODE_SPEED_BRAKE_ON,
    MODE_SPEED_BRAKE_OFF,
    MODE_TORQUE_BRAKE_ON,
    MODE_TORQUE_BRAKE_OFF,
    TORQUE_LIMIT
)

# --- CAN IDs ---
FRONT_DRIVE_ID = 0x302    # 0x300 + NodeID 2
REAR_DRIVE_ID  = 0x304    # 0x300 + NodeID 4


class CANTransmitter:
    """Sends drive commands to motor controllers over CAN."""

    def __init__(self, bus, send_to_rear=True):
        self._bus = bus
        self._send_to_rear = send_to_rear

    # ------------------------------------------------------------------ #
    #  Public API                                                         #
    # ------------------------------------------------------------------ #

    def send_drive_command(self, throttle_left, throttle_right, torque_mode=False):
        """
        Send a DriveCommand to the motor controllers.

        Args:
            throttle_left:  int, -1000..+1000
            throttle_right: int, -1000..+1000
        """
        # Clamp to valid range
        throttle_left  = int(max(-1000, min(1000, throttle_left)))
        throttle_right = int(max(-1000, min(1000, throttle_right)))

        # Determine mode
        # Speed: 0x01 (Brake ON) is required for skid-steer precision.
        # Torque: 0x02 (Torque mode)
        if torque_mode:
            motor_mode = MODE_TORQUE_BRAKE_ON
            limit_val = 1500  # In torque mode, Limit field is max RPM
        else:
            motor_mode = MODE_SPEED_BRAKE_OFF
            limit_val = TORQUE_LIMIT  # In speed mode, Limit field is max Torque(nM)

        # Pack payload — BIG-ENDIAN (motor controller uses Motorola byte order)
        payload = struct.pack(
            ">hhBHB",
            throttle_left,
            throttle_right,
            motor_mode,
            limit_val,
            0,  # reserved
        )

        # Send to front driver
        self._send_frame(FRONT_DRIVE_ID, payload)

        # Send to rear driver (same command for 4WD skid-steer)
        if self._send_to_rear:
            self._send_frame(REAR_DRIVE_ID, payload)

    def send_stop(self):
        """Send zero throttle to all drivers."""
        self.send_drive_command(0, 0)

    # ------------------------------------------------------------------ #
    #  OLD LOGIC — direct PWM-to-throttle (to be replaced)               #
    # ------------------------------------------------------------------ #

    def send_from_pwm(self, throttle_pwm, front_pwm):
        """
        OLD mapping: convert raw PWM values to CAN throttle.
        This is the original logic from vehicle_controller.py.
        Will be replaced with the 6-stage steering algorithm.

        Args:
            throttle_pwm: int, 1000-2000 (center 1500)
            front_pwm:    int, 1000-2000 (center 1500)
        """
        left_cmd  = int(max(-1000, min(1000, (front_pwm - 1500) * 2)))
        right_cmd = int(max(-1000, min(1000, (throttle_pwm - 1500) * 2)))
        self.send_drive_command(left_cmd, right_cmd)

    # ------------------------------------------------------------------ #
    #  Internal                                                           #
    # ------------------------------------------------------------------ #

    def _send_frame(self, can_id, payload):
        """Send a single CAN frame."""
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=payload,
                is_extended_id=False,
            )
            self._bus.send(msg)
        except can.CanError as e:
            print(f"[CAN TX ERROR] ID {can_id:#x}: {e}")
