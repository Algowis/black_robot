"""
Robot Configuration
Contains all CAN protocol definitions, Motor Modes, and Skid-Steer parameters
based on "CAN Command V0.1.pdf" and "skid_steer_spec.docx.pdf".
"""

# ==============================================================================
# 1. CAN Protocol IDs (Master <-> Motor Drivers)
# ==============================================================================
# Base IDs (Must add NodeID: 2 for Front, 4 for Rear)
NODE_FRONT = 2
NODE_REAR  = 4

CAN_ID_TEMP_TELEM   = 0x2D0  # Motor & controller temps (Motor -> Master)
CAN_ID_DRIVE_CMD    = 0x300  # Drive command (Master -> Motor)
CAN_ID_SPEED_TELEM  = 0x310  # Speed and Torque telemetry (Motor -> Master)
CAN_ID_POWER_TELEM  = 0x320  # Power, Voltage, Faults (Motor -> Master)
CAN_ID_CURRENT_LIM  = 0x400  # Set current limits (Master -> Motor)
CAN_ID_ALARM_RPT    = 0x500  # Detailed alarm reporting (Motor -> Master)


# ==============================================================================
# 2. Motor Operation Modes (Byte 4 of DriveCommand)
# ==============================================================================
"""
Mode Byte Construction (Bitwise):
Bits 0-1: Operating Mode (00=None, 01=Speed, 10=Torque)
Bit 2:    Disable Braking (0=Brake Enabled, 1=Brake Disabled)
Bits 3-7: Reserved (Must be 0)

Note on Limit Field in DriveCommand:
- In SPEED modes (0x01, 0x05), the Limit field sets MAXIMUM TORQUE (nM).
- In TORQUE modes (0x02, 0x06), the Limit field sets MAXIMUM SPEED (RPM).
"""

# --- SPEED MODES (Controller automatically adjusts torque to maintain set RPM) ---
MODE_SPEED_BRAKE_ON  = 0x01  # Normal mode. Active regenerative braking enabled.
MODE_SPEED_BRAKE_OFF = 0x05  # Coasting mode. Brakes disabled (stops air surging).

# --- TORQUE MODES (Controller outputs constant force, RPM floats freely) ---
MODE_TORQUE_BRAKE_ON  = 0x02 # Constant force mode. Active braking enabled.
MODE_TORQUE_BRAKE_OFF = 0x06 # Constant force mode. Braking disabled.

# --- Limits ---
TORQUE_LIMIT = 20            # Maximum torque allowed (nM)


# ==============================================================================
# 3. Skid-Steer Control Parameters
# ==============================================================================
# Physical Measurements (Need to be updated with real robot measurements)
TRACK_WIDTH_M = 0.50         # (W) Distance between left and right wheel centers
WHEEL_RADIUS_M = 0.125       # (r) Radius of the wheels

# Limits and Dynamics
V_MAX = 2.0                  # (V_max) Maximum robot speed in m/s (Throttle 1000 = V_MAX)
MAX_LATERAL_ACCEL = 3.0      # (a_lat_max) Max lateral G-force before rollover risk (m/s^2)
MAX_ACCEL = 2.0              # (MaxAccel) Max allowed acceleration rate for Slew Limiter
JOYSTICK_DEADZONE = 0.05     # Dead-band to ignore stick drift

# Blend Zones
V_PIVOT = 0.2                # Speed below which robot does pure pivot turns
V_BLEND_END = 1.0            # Speed above which full anti-rollover dynamic logic applies

# Safety Timeouts
TELEM_TIMEOUT_S = 0.2        # Stop robot if no telemetry received in this time


# ==============================================================================
# 4. Fault Levels (Byte 5 of PowerTelemetry)
# ==============================================================================
FAULT_NONE     = 0           # All good
FAULT_WARNING  = 1           # Starting to get hot
FAULT_DEGRADED = 2           # Performance reduced
FAULT_STOPPING = 3           # Initiating stop
FAULT_STOPPED  = 4           # Fully stopped (Critical)
