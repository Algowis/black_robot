"""
Steering Controller — Full 6-Stage Skid-Steer Algorithm.

Implements all stages from the Skid-Steer Steering Control Spec:
  Stage 1: Reference Speed (telemetry feedback)
  Stage 2: TurnAmount by Speed Zone (anti-rollover)
  Stage 3: Conflict Resolution (preserve turn, sacrifice speed)
  Stage 4: Per-Motor Targets (drive + steer mixing)
  Stage 5: Slew Rate Limiter (smooth acceleration)
  Stage 6: CAN Throttle Output (scale to ±1000)
"""

import math


# ================================================================== #
#  Robot Physical Parameters (measured)                                #
# ================================================================== #

WHEEL_RADIUS = 0.1825       # m — wheel diameter 36.5 cm
TRACK_WIDTH  = 0.585        # m — center-to-center left-right wheels
V_MAX        = 2.0          # m/s — Fixed from 13.6 to match robot_config.py

# RPM at full throttle (for telemetry conversion)
MAX_RPM = V_MAX * 60 / (2 * math.pi * WHEEL_RADIUS)  # ≈ 711 RPM


# ================================================================== #
#  Algorithm Parameters (from spec with defaults)                      #
# ================================================================== #

A_LAT_MAX        = 3.0           # m/s² — max lateral accel before rollover risk
MAX_ACCEL        = 0.16          # m/s² — Slew rate limit (Super slow: takes ~3s to reach 25%)
V_PIVOT          = 0.2           # m/s  — below this: pivot mode (no centrifugal)
V_BLEND_END      = 1.0           # m/s  — above this: full dynamic anti-rollover
DEADZONE         = 0.05          # joystick dead-band threshold
GLOBAL_LIMIT_PCT = 0.5          # Hard cap to output 250/1000 max for floor testing


class SteeringController:
    """
    Full 6-stage skid-steer steering algorithm.

    Converts joystick (drive, steer) commands into per-motor CAN throttle
    values with anti-rollover protection, conflict resolution, and smooth
    acceleration.
    """

    def __init__(self, v_max=V_MAX, track_width=TRACK_WIDTH,
                 wheel_radius=WHEEL_RADIUS, a_lat_max=A_LAT_MAX,
                 max_accel=MAX_ACCEL, v_pivot=V_PIVOT,
                 v_blend_end=V_BLEND_END, deadzone=DEADZONE,
                 global_limit_pct=GLOBAL_LIMIT_PCT,
                 loop_hz=50):

        # Physical params
        self.v_max = v_max
        self.W = track_width
        self.r = wheel_radius

        # Algorithm params
        self.a_lat_max = a_lat_max
        self.max_accel = max_accel
        self.v_pivot = v_pivot
        self.v_blend_end = v_blend_end
        self.deadzone = deadzone
        self.global_limit_pct = global_limit_pct
        self.dt = 1.0 / loop_hz

        # Slew limiter state (in m/s units)
        self._cmd_left_prev = 0.0
        self._cmd_right_prev = 0.0

    # ------------------------------------------------------------------ #
    #  Public API                                                         #
    # ------------------------------------------------------------------ #

    def compute(self, throttle_pwm, front_pwm,
                speed_left_rpm=0, speed_right_rpm=0):
        """
        Main entry point — full 6-stage algorithm.

        Args:
            throttle_pwm:    int, 1000-2000 (center 1500) — drive axis
            front_pwm:       int, 1000-2000 (center 1500) — steer axis
            speed_left_rpm:  int, current left motor RPM from telemetry
            speed_right_rpm: int, current right motor RPM from telemetry

        Returns:
            (throttle_left, throttle_right): tuple of int, each in [-1000, +1000]
        """
        # --- Input normalization + deadband ---
        drive_cmd = self._normalize(throttle_pwm)   # -1.0..+1.0
        steer_cmd = self._normalize(front_pwm)       # -1.0..+1.0

        # --- Convert telemetry RPM → m/s ---
        v_left_telem  = self._rpm_to_ms(speed_left_rpm)
        v_right_telem = self._rpm_to_ms(speed_right_rpm)

        # ============================================================ #
        #  Stage 1 — Reference Speed                                    #
        # ============================================================ #
        v_robot_actual = (v_left_telem + v_right_telem) / 2.0
        v_target_drive = drive_cmd * self.v_max

        # Option Y: conservative — use whichever is larger
        v_for_omega = max(abs(v_robot_actual), abs(v_target_drive))

        # ============================================================ #
        #  Stage 2 — TurnAmount by Speed Zone                           #
        # ============================================================ #
        turn_amount = self._compute_turn_amount(steer_cmd, v_for_omega)

        # ============================================================ #
        #  Stage 3 — Conflict Resolution                                #
        # ============================================================ #
        # Principle: preserve the turn, sacrifice linear speed.
        peak_side = abs(v_target_drive) + abs(turn_amount)
        if peak_side > self.v_max:
            # Cap drive speed to make room for the turn
            v_target_drive = math.copysign(
                self.v_max - abs(turn_amount),
                v_target_drive
            )

        # ============================================================ #
        #  Stage 4 — Per-Motor Targets                                  #
        # ============================================================ #
        # SteerCmd > 0 → right turn → left accelerates, right decelerates
        target_left  = v_target_drive + turn_amount
        target_right = v_target_drive - turn_amount

        # Hard limit maximum physical speed (to prevent motor rattle > 700 output)
        max_permitted = self.v_max * self.global_limit_pct
        peak = max(abs(target_left), abs(target_right))
        if peak > max_permitted:
            scale = max_permitted / peak
            target_left *= scale
            target_right *= scale

        # ============================================================ #
        #  Stage 5 — Slew Rate Limiter                                  #
        # ============================================================ #
        cmd_left, cmd_right = self._slew_limit(target_left, target_right)

        # ============================================================ #
        #  Stage 6 — CAN Throttle Output                                #
        # ============================================================ #
        throttle_left  = self._velocity_to_throttle(cmd_left)
        throttle_right = self._velocity_to_throttle(cmd_right)

        return throttle_left, throttle_right

    def reset(self):
        """Reset slew limiter state to zero (e.g. after safety stop)."""
        self._cmd_left_prev = 0.0
        self._cmd_right_prev = 0.0

    # ------------------------------------------------------------------ #
    #  Internal — Input Processing                                       #
    # ------------------------------------------------------------------ #

    def _normalize(self, pwm_value):
        """Convert PWM (1000-2000) to (-1.0..+1.0) with deadband."""
        cmd = (pwm_value - 1500) / 500.0
        cmd = max(-1.0, min(1.0, cmd))
        if abs(cmd) < self.deadzone:
            cmd = 0.0
        return cmd

    def _rpm_to_ms(self, rpm):
        """Convert RPM to m/s using wheel radius."""
        return rpm * 2.0 * math.pi * self.r / 60.0

    # ------------------------------------------------------------------ #
    #  Internal — Stage 2: TurnAmount                                    #
    # ------------------------------------------------------------------ #

    def _compute_turn_amount(self, steer_cmd, v_for_omega):
        """
        Compute TurnAmount based on speed zone.

        Zone A (Pivot):   V_for_omega <= V_pivot
            → Full differential, no centrifugal concern
        Zone B (Dynamic): V_for_omega >= V_blend_end
            → Anti-rollover: limit lateral acceleration
        Zone C (Blend):   V_pivot < V_for_omega < V_blend_end
            → Linear interpolation between A and B
        """
        if v_for_omega <= self.v_pivot:
            # Zone A — Pivot
            return steer_cmd * self.v_max

        elif v_for_omega >= self.v_blend_end:
            # Zone B — Dynamic (anti-rollover)
            omega_max = self.a_lat_max / v_for_omega
            return steer_cmd * (omega_max * self.W / 2.0)

        else:
            # Zone C — Blend (linear interpolation)
            ratio = (v_for_omega - self.v_pivot) / (self.v_blend_end - self.v_pivot)

            # Pivot result
            turn_pivot = steer_cmd * self.v_max

            # Dynamic result (use V_blend_end for consistency)
            omega_max_blend = self.a_lat_max / self.v_blend_end
            turn_dynamic = steer_cmd * (omega_max_blend * self.W / 2.0)

            # Interpolate
            return (1.0 - ratio) * turn_pivot + ratio * turn_dynamic

    # ------------------------------------------------------------------ #
    #  Internal — Stage 5: Slew Rate Limiter                             #
    # ------------------------------------------------------------------ #

    def _slew_limit(self, target_left, target_right):
        """
        Limit command change rate to MaxAccel m/s per second.
        Applied independently to each side.
        """
        max_delta = self.max_accel * self.dt  # m/s per cycle

        # Left
        delta_left = target_left - self._cmd_left_prev
        delta_left = max(-max_delta, min(max_delta, delta_left))
        cmd_left = self._cmd_left_prev + delta_left

        # Right
        delta_right = target_right - self._cmd_right_prev
        delta_right = max(-max_delta, min(max_delta, delta_right))
        cmd_right = self._cmd_right_prev + delta_right

        # Store for next cycle
        self._cmd_left_prev = cmd_left
        self._cmd_right_prev = cmd_right

        return cmd_left, cmd_right

    # ------------------------------------------------------------------ #
    #  Internal — Stage 6: Velocity → CAN Throttle                       #
    # ------------------------------------------------------------------ #

    def _velocity_to_throttle(self, velocity_ms):
        """Convert velocity (m/s) to CAN throttle integer [-1000, +1000]."""
        throttle = round((velocity_ms / self.v_max) * 1000)
        return int(max(-1000, min(1000, throttle)))
