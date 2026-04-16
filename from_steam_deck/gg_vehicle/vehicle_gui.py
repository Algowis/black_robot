"""
vehicle_gui.py — Steam Deck Vehicle Controller GUI
====================================================
Fullscreen PyQt5 dashboard (1280×800) optimized for Steam Deck.
Camera-centric layout with telemetry HUD overlay. Gamepad-only control.

Layout:
┌──────────────────────────────────────────────────────────────────┐
│                                                    ┌──────────┐ │
│                                                    │ Back Cam  │ │
│              MAIN CAMERA (Front / Back / PTZ)      │ thumbnail │ │
│                                                    ├──────────┤ │
│                                                    │ PTZ Cam   │ │
│                                                    │ thumbnail │ │
│                                                    └──────────┘ │
├─────────────── HUD OVERLAY (bottom bar) ────────────────────────┤
│ CONN│ STEER:FRONT│ GEAR:LOW│ ⚡12.4V│ THR:1500│ GPS:8sat│ SPD │ │
└──────────────────────────────────────────────────────────────────┘

Gamepad mapping:
    Right Stick Y   → Throttle (up=forward, down=reverse)
    Left Stick X    → Steering (depends on mode)
    Y               → Steering mode: FRONT ONLY
    X               → Steering mode: FRONT + BACK
    A               → Steering mode: BACK ONLY
    B               → Cycle main camera view
    LB              → Toggle gear
    RB              → Toggle lights

Requirements:
    pip install PyQt5 pygame opencv-python numpy

Usage:
    python vehicle_gui.py
    python vehicle_gui.py --ip 192.168.144.100
    python vehicle_gui.py --windowed
"""

import os
import sys
import time
import argparse
from typing import Optional

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QLabel, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, QSize

from PyQt5.QtGui import QFont, QColor, QPainter, QPen, QBrush, QImage, QPixmap

import numpy as np

from vehicle_logic import (
    VehicleController, GamepadManager, CameraStream,
    ControlState, HeartbeatData, GNSSData, VehicleStatus,
    SteeringMode, STEERING_MODE_LABELS,
    SERVO_MIN, SERVO_NEUTRAL, SERVO_MAX,
    HIGH_GEAR_PWM, LOW_GEAR_PWM,
)

# ═══════════════════════════════════════════════
#  Camera Configuration
#  ── EDIT THESE RTSP URLs TO MATCH YOUR CAMERAS ──
# ═══════════════════════════════════════════════
CAMERA_URLS = {
    "Front": "rtsp://admin:12345@192.168.144.101/ch1/stream0",
    "Back":  "rtsp://admin:12345@192.168.144.102/ch1/stream0",
    "PTZ":   "rtsp://192.168.144.25:8554/main.264",
}

CAMERA_ORDER = ["PTZ", "Front", "Back"]

# Max capture width per camera (0 = full resolution)
# Main camera ~80% of 1280 = 1024, 640 is plenty; thumbnails ~20% = 256, 320 is fine
CAMERA_MAX_WIDTH = {
    "Front": 640,
    "Back":  640,
    "PTZ":   640,
}

# RTSP transport per camera: "udp" or "tcp"
CAMERA_TRANSPORT = {
    "Front": "tcp",
    "Back":  "tcp",
    "PTZ":   "udp",
}


# ═══════════════════════════════════════════════
#  Colors
# ═══════════════════════════════════════════════
C_BG       = "#0B0E14"
C_PANEL    = "#141820"
C_BORDER   = "#2A2F3A"
C_TEXT     = "#D4D8E0"
C_DIM      = "#6B7280"
C_ACCENT   = "#F97316"
C_GREEN    = "#22C55E"
C_RED      = "#EF4444"
C_YELLOW   = "#EAB308"
C_BLUE     = "#3B82F6"
C_CYAN     = "#06B6D4"

STYLE = f"""
QMainWindow, QWidget {{
    background-color: {C_BG};
    color: {C_TEXT};
}}
"""

# ═══════════════════════════════════════════════
#  Attitude Indicator (Artificial Horizon)
# ═══════════════════════════════════════════════
class AttitudeIndicator(QWidget):
    """Small artificial horizon showing roll and pitch visually."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(80, 80)
        self.roll = 0.0
        self.pitch = 0.0

    def set_attitude(self, roll: float, pitch: float):
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        radius = min(w, h) / 2.0 - 2

        # 1. Draw outer bezel/border
        p.setPen(QPen(QColor(C_BORDER), 2))
        p.setBrush(QColor("#000000"))
        p.drawEllipse(int(cx - radius), int(cy - radius), int(radius * 2), int(radius * 2))

        # 2. Set circular clipping path for the horizon
        from PyQt5.QtGui import QPainterPath
        clip_path = QPainterPath()
        clip_path.addEllipse(cx - radius + 1, cy - radius + 1, radius * 2 - 2, radius * 2 - 2)
        p.setClipPath(clip_path)

        # 3. Apply transformations: translate to center, apply pitch, apply roll
        p.translate(cx, cy)
        p.rotate(self.roll)
        
        # Pitch scaling: roughly 0.5 pixels per degree
        pitch_shift = max(-radius, min(radius, self.pitch * 0.5))
        p.translate(0, pitch_shift)

        # 4. Draw Sky and Ground
        # Note: We draw well 'outside' the radius so it covers the clip area when rotated
        rect_size = radius * 4
        
        # Sky (Top half)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(135, 206, 235)) # Sky Blue
        p.drawRect(int(-rect_size), int(-rect_size), int(rect_size * 2), int(rect_size))
        
        # Ground (Bottom half)
        p.setBrush(QColor(139, 69, 19)) # Dirt Brown
        p.drawRect(int(-rect_size), 0, int(rect_size * 2), int(rect_size))

        # Horizon Line
        p.setPen(QPen(Qt.white, 2))
        p.drawLine(int(-rect_size), 0, int(rect_size), 0)

        # Reset transforms to draw fixed overlay
        p.resetTransform()
        p.setClipping(False)

        # 5. Fixed horizontal neutral line (dashed)
        neutral_pen = QPen(QColor(255, 255, 255, 150), 1, Qt.DashLine)
        p.setPen(neutral_pen)
        p.drawLine(int(cx - radius + 2), int(cy), int(cx + radius - 2), int(cy))

        # 6. Fixed aircraft reference crosshair (yellow)
        c_w = radius * 0.4
        c_gap = radius * 0.15
        p.setPen(QPen(QColor(C_YELLOW), 2))
        p.drawLine(int(cx - c_w), int(cy), int(cx - c_gap), int(cy))
        p.drawLine(int(cx + c_gap), int(cy), int(cx + c_w), int(cy))
        p.drawLine(int(cx), int(cy - c_gap/2), int(cx), int(cy + c_gap/2))
        
        p.end()



# ═══════════════════════════════════════════════
#  CameraWidget — fixed-size painting, no grow
# ═══════════════════════════════════════════════
class CameraWidget(QWidget):
    """
    Custom widget that paints a camera frame scaled to fit its
    allocated area. Uses paintEvent instead of QLabel.setPixmap
    so the widget NEVER requests more space than the layout gives it.

    Key anti-growth measures:
      • SizePolicy = Ignored (accepts any size from layout)
      • sizeHint / minimumSizeHint return tiny fixed values
      • Only a single self.update() per refresh cycle (batched)
      • Pixmap is pre-scaled in paintEvent to the widget's CURRENT size
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pixmap: Optional[QPixmap] = None
        self._label_text: str = ""
        self._status_text: str = "NO SIGNAL"
        self._show_label: bool = True
        # Critical: widget passively accepts whatever size the layout assigns
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        # Prevent the widget from ever requesting a minimum size
        self.setMinimumSize(1, 1)

    # ── Size hints: always return small constants ──

    def sizeHint(self) -> QSize:
        return QSize(160, 120)

    def minimumSizeHint(self) -> QSize:
        return QSize(1, 1)

    # ── Data setters (NO self.update() — caller batches) ──

    def set_frame(self, frame: Optional[np.ndarray]):
        """Set a new BGR frame (or None for no-signal). Does NOT trigger repaint."""
        if frame is not None:
            h, w, ch = frame.shape
            bpl = ch * w
            # Convert BGR→RGB in-place view, then copy into QImage
            qimg = QImage(
                np.ascontiguousarray(frame[:, :, ::-1]).data,
                w, h, bpl, QImage.Format_RGB888,
            ).copy()  # .copy() so QImage owns the data
            self._pixmap = QPixmap.fromImage(qimg)
        else:
            self._pixmap = None

    def set_label(self, text: str):
        self._label_text = text

    def set_status(self, text: str):
        self._status_text = text

    def refresh(self):
        """Call once per frame after all setters to trigger a single repaint."""
        self.update()

    # ── Paint ──

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        if w < 2 or h < 2:
            p.end()
            return

        rect = self.rect()

        # Background
        p.fillRect(rect, QColor("#0D1117"))

        if self._pixmap and not self._pixmap.isNull():
            # Scale pixmap to fit the widget — fast nearest-neighbour for low latency
            scaled = self._pixmap.scaled(w, h, Qt.KeepAspectRatio, Qt.FastTransformation)
            x_off = (w - scaled.width()) // 2
            y_off = (h - scaled.height()) // 2
            p.drawPixmap(x_off, y_off, scaled)
        else:
            # No signal placeholder
            p.setPen(QColor(C_DIM))
            font = QFont("monospace", max(9, min(12, h // 10)))
            font.setBold(True)
            p.setFont(font)
            p.drawText(rect, Qt.AlignCenter, f"{self._label_text}\n{self._status_text}")

        # Border
        p.setPen(QPen(QColor(C_BORDER), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(0, 0, w - 1, h - 1)

        # Name overlay (top-left) — large label
        if self._show_label and self._label_text:
            font = QFont("monospace", max(16, min(28, h // 6)))
            font.setBold(True)
            p.setFont(font)
            fm = p.fontMetrics()
            tw = fm.horizontalAdvance(self._label_text) + 24
            th = fm.height() + 12
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 180))
            p.drawRoundedRect(10, 10, tw, th, 6, 6)
            p.setPen(QColor(C_ACCENT))
            p.drawText(22, 10 + fm.ascent() + 6, self._label_text)

        p.end()


# ═══════════════════════════════════════════════
#  HUD Bar Widget (bottom overlay)
# ═══════════════════════════════════════════════
class HUDBar(QWidget):
    """Translucent bottom bar showing all telemetry at a glance."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(88)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # ── Setup Layout for children (Attitude indicator) ──
        self.setLayout(QVBoxLayout())
        self.layout().setContentsMargins(0, 0, 16, 0) # Right margin
        self.layout().setSpacing(0)
        
        from PyQt5.QtWidgets import QHBoxLayout
        hbox = QHBoxLayout()
        hbox.addStretch(1) # Push to right
        
        self.attitude = AttitudeIndicator()
        hbox.addWidget(self.attitude, alignment=Qt.AlignVCenter)
        # Add space for the logo
        hbox.addSpacing(100) 
        
        self.layout().addLayout(hbox)

        self.connected = False
        self.steering_mode = SteeringMode.FRONT_ONLY
        self.gear_low = True
        self.lights_on = False
        self.voltage = 0.0
        self.throttle = 1500
        self.front_steer = 1500
        self.back_steer = 1500
        self.gear_pwm = 0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.satellites = 0
        self.latitude = 0.0
        self.longitude = 0.0
        self.speed = 0.0
        self.imu_ok = False
        self.gnss_ok = False
        self.packets_tx = 0
        self.packets_rx = 0
        self.crc_errors = 0
        self.gamepad_name = ""
        self.main_camera = "Front"

        # Load logo image
        logo_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "GG_Logo.png")
        self._logo: Optional[QPixmap] = None
        if os.path.isfile(logo_path):
            px = QPixmap(logo_path)
            if not px.isNull():
                self._logo = px.scaledToHeight(self.height() - 8, Qt.SmoothTransformation)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Background
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(11, 14, 20, 220))
        p.drawRect(0, 0, w, h)
        p.setPen(QPen(QColor(C_BORDER), 1))
        p.drawLine(0, 0, w, 0)

        # ── Row 1: Primary status ──
        y1 = 22
        x = 16

        conn_color = QColor(C_GREEN) if self.connected else QColor(C_RED)
        conn_text = "CONNECTED" if self.connected else "OFFLINE"
        x = self._draw_dot(p, x, y1, conn_color, 14)
        x = self._draw_text(p, x + 2, y1, conn_text, conn_color, 13, bold=True)
        x += 20

        mode_label = STEERING_MODE_LABELS.get(self.steering_mode, "?")
        x = self._draw_text(p, x, y1, "STEER:", QColor(C_DIM), 11)
        mode_c = QColor(C_CYAN) if self.steering_mode == SteeringMode.FRONT_AND_BACK else QColor(C_ACCENT)
        x = self._draw_text(p, x + 4, y1, mode_label, mode_c, 13, bold=True)
        x += 20

        gear_text = "LOW" if self.gear_low else "HIGH"
        gear_c = QColor(C_GREEN) if self.gear_low else QColor(C_BLUE)
        x = self._draw_text(p, x, y1, "GEAR:", QColor(C_DIM), 11)
        x = self._draw_text(p, x + 4, y1, gear_text, gear_c, 13, bold=True)
        x += 20

        light_text = "ON" if self.lights_on else "OFF"
        light_c = QColor(C_YELLOW) if self.lights_on else QColor(C_DIM)
        x = self._draw_text(p, x, y1, "LIGHT:", QColor(C_DIM), 11)
        x = self._draw_text(p, x + 4, y1, light_text, light_c, 13, bold=True)
        x += 20

        v_c = QColor(C_GREEN) if self.voltage > 11.0 else (QColor(C_YELLOW) if self.voltage > 10.0 else QColor(C_RED))
        x = self._draw_text(p, x, y1, f"{self.voltage:.1f}V", v_c, 13, bold=True)
        x += 20

        x = self._draw_text(p, x, y1, "CAM:", QColor(C_DIM), 11)
        x = self._draw_text(p, x + 4, y1, self.main_camera.upper(), QColor(C_ACCENT), 13, bold=True)

        # ── Row 2: Servos + sensors ──
        y2 = 46
        x = 16

        x = self._draw_text(p, x, y2, "THR:", QColor(C_DIM), 10)
        x = self._draw_bar(p, x + 4, y2 - 6, 80, 12, self.throttle)
        x += 12
        x = self._draw_text(p, x, y2, "FNT:", QColor(C_DIM), 10)
        x = self._draw_bar(p, x + 4, y2 - 6, 80, 12, 3000 - self.front_steer)
        x += 12
        x = self._draw_text(p, x, y2, "BCK:", QColor(C_DIM), 10)
        x = self._draw_bar(p, x + 4, y2 - 6, 80, 12, self.back_steer)
        x += 20

        imu_c = QColor(C_GREEN) if self.imu_ok else QColor(C_RED)
        x = self._draw_text(p, x, y2, "IMU:", QColor(C_DIM), 10)
        x = self._draw_text(p, x + 2, y2, "OK" if self.imu_ok else "FAIL", imu_c, 10, bold=True)
        x += 10
        # Update Attitude widget instead of drawing numbers
        self.attitude.set_attitude(self.roll, self.pitch)
        x = self._draw_text(p, x, y2, "ORIENT:", QColor(C_DIM), 10)
        x += 54 # leave gap for the widget which is absolutely positioned via layout
        x += 16
        gnss_c = QColor(C_GREEN) if self.gnss_ok else QColor(C_RED)
        x = self._draw_text(p, x, y2, f"SAT:{self.satellites}", gnss_c, 10, bold=True)

        # ── Row 3: GPS + packets ──
        y3 = 66
        x = 16
        x = self._draw_text(p, x, y3,
                            f"LAT:{self.latitude:.6f}  LON:{self.longitude:.6f}  SPD:{self.speed:.1f}m/s",
                            QColor(C_DIM), 9)
        x += 30
        x = self._draw_text(p, x, y3, f"TX:{self.packets_tx}  RX:{self.packets_rx}  ERR:{self.crc_errors}",
                            QColor(C_DIM), 9)
        x += 20
        if self.gamepad_name:
            self._draw_text(p, x, y3, f"GP: {self.gamepad_name}", QColor(C_DIM), 9)

        # Logo in bottom-right corner
        if self._logo and not self._logo.isNull():
            margin = 4
            lx = w - self._logo.width() - margin
            ly = (h - self._logo.height()) // 2
            p.drawPixmap(lx, ly, self._logo)

        p.end()

    def _draw_text(self, p, x, y, text, color, size, bold=False):
        font = QFont("monospace", size)
        font.setBold(bold)
        p.setFont(font)
        p.setPen(color)
        p.drawText(x, y, text)
        return x + p.fontMetrics().horizontalAdvance(text)

    def _draw_dot(self, p, x, y, color, size):
        font = QFont("monospace", size)
        p.setFont(font)
        p.setPen(color)
        p.drawText(x, y, "●")
        return x + p.fontMetrics().horizontalAdvance("●")

    def _draw_bar(self, p, x, y, bw, bh, value):
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C_BORDER))
        p.drawRoundedRect(x, y, bw, bh, 3, 3)
        cx = x + bw // 2
        frac = (value - 1000) / 1000.0
        pos_x = x + int(frac * bw)
        fl = min(cx, pos_x)
        fr = max(cx, pos_x)
        p.setBrush(QColor(C_BLUE))
        p.drawRoundedRect(fl, y, fr - fl, bh, 3, 3)
        p.setPen(QPen(QColor(C_DIM), 1))
        p.drawLine(cx, y, cx, y + bh)
        font = QFont("monospace", 8)
        font.setBold(True)
        p.setFont(font)
        p.setPen(QColor(C_TEXT))
        p.drawText(x + bw + 4, y + bh - 1, str(value))
        return x + bw + 36


# ═══════════════════════════════════════════════
#  Main Dashboard Window
# ═══════════════════════════════════════════════
class DashboardWindow(QMainWindow):

    def __init__(self, controller: VehicleController,
                 cameras: dict, fullscreen: bool = True):
        super().__init__()
        self.ctrl = controller
        self.cameras: dict[str, CameraStream] = cameras
        self.setWindowTitle("RC Vehicle — Steam Deck")
        self.setStyleSheet(STYLE)

        if fullscreen:
            self.showFullScreen()
        else:
            self.resize(1280, 800)

        self._main_cam_index = 0
        self._user_cam_index = 0       # tracks manual selection
        self._reverse_override = False  # True when auto-switched to rear cam

        # ── Central Widget ──
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Single fullscreen camera ──
        self._main_cam = CameraWidget()
        root.addWidget(self._main_cam, stretch=1)

        # ── HUD Bar (fixed height at bottom) ──
        self._hud = HUDBar()
        root.addWidget(self._hud)

        # ── Refresh timer ──
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(33)  # ~30 FPS

        # Start only the initially-selected camera
        self._activate_camera()

    # ── Camera cycling ──

    def cycle_camera(self):
        self._user_cam_index = (self._user_cam_index + 1) % len(CAMERA_ORDER)
        self._main_cam_index = self._user_cam_index
        self._reverse_override = False
        self._activate_camera()

    def _activate_camera(self):
        """Start the selected camera and stop all others."""
        active_name = CAMERA_ORDER[self._main_cam_index]
        for name, stream in self.cameras.items():
            try:
                if name == active_name:
                    if not getattr(stream, "_running", False):
                        stream.start()
                        print(f"[GUI] Started camera: {name}")
                else:
                    if getattr(stream, "_running", False):
                        stream.stop()
                        print(f"[GUI] Stopped camera: {name}")
            except Exception as e:
                print(f"[GUI] Camera control error for {name}: {e}")

    # ── Refresh (called at ~30 fps) ──

    def _refresh(self):
        hb, gnss, ctrl, connected = self.ctrl.get_snapshot()

        # ── Auto-switch to rear camera when reversing ──
        back_cam_idx = CAMERA_ORDER.index("Back") if "Back" in CAMERA_ORDER else -1
        if ctrl.throttle < 1450 and back_cam_idx >= 0:
            # Reversing — override to Back camera
            if not self._reverse_override:
                self._reverse_override = True
                self._main_cam_index = back_cam_idx
                self._activate_camera()
        else:
            # Not reversing — restore user's selection
            if self._reverse_override:
                self._reverse_override = False
                self._main_cam_index = self._user_cam_index
                self._activate_camera()

        # ── HUD data ──
        hb_alive = hb.timestamp > 0 and (time.time() - hb.timestamp < 3.0)
        self._hud.connected = connected and hb_alive
        self._hud.steering_mode = ctrl.steering_mode
        self._hud.gear_low = ctrl.gear_low
        self._hud.lights_on = ctrl.lights_on

        # Servo values: show LOCAL control values (what we're sending)
        # so the HUD always reflects the gamepad input, even when offline
        self._hud.throttle = ctrl.throttle
        self._hud.front_steer = ctrl.front_steering
        self._hud.back_steer = ctrl.back_steering

        # Telemetry from MCU heartbeat (only meaningful when connected)
        self._hud.voltage = hb.voltage
        self._hud.gear_pwm = hb.gear_pwm
        self._hud.roll = hb.roll
        self._hud.pitch = hb.pitch
        self._hud.yaw = hb.yaw
        self._hud.imu_ok = hb.status.imu_ok
        self._hud.gnss_ok = hb.status.gnss_ok
        self._hud.satellites = gnss.satellites
        self._hud.latitude = gnss.latitude
        self._hud.longitude = gnss.longitude
        self._hud.speed = gnss.ground_speed
        self._hud.packets_tx = self.ctrl.packets_sent
        self._hud.packets_rx = self.ctrl.packets_recv
        self._hud.crc_errors = self.ctrl.crc_errors

        main_name = CAMERA_ORDER[self._main_cam_index]
        self._hud.main_camera = main_name
        self._hud.update()

        # ── Active camera ──
        self._main_cam.set_label(main_name.upper())
        stream = self.cameras.get(main_name)
        if stream:
            frame = stream.get_frame()
            self._main_cam.set_frame(frame)
            if frame is None:
                self._main_cam.set_status("CONNECTING..." if not stream.connected else "NO SIGNAL")
        else:
            self._main_cam.set_frame(None)
            self._main_cam.set_status("NOT CONFIGURED")
        self._main_cam.refresh()

    # ── Keyboard fallback ──

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Escape:
            self.close()
        elif key == Qt.Key_B:
            self.cycle_camera()
        elif key == Qt.Key_Y:
            self.ctrl.set_steering_mode(SteeringMode.FRONT_ONLY)
        elif key == Qt.Key_X:
            self.ctrl.set_steering_mode(SteeringMode.FRONT_AND_BACK)
        elif key == Qt.Key_A:
            self.ctrl.set_steering_mode(SteeringMode.BACK_ONLY)
        elif key == Qt.Key_G:
            self.ctrl.toggle_gear()
        elif key == Qt.Key_L:
            self.ctrl.toggle_lights()
        elif key == Qt.Key_Space:
            self.ctrl.reset_servos()
        else:
            super().keyPressEvent(event)


# ═══════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="Steam Deck RC Vehicle Controller")
    parser.add_argument("--ip", default="192.168.120.20", help="Jetson IP address")
    parser.add_argument("--windowed", action="store_true", help="Run windowed")
    parser.add_argument("--no-cameras", action="store_true", help="Skip camera streams")
    args = parser.parse_args()

    controller = VehicleController(car_ip=args.ip)
    controller.start()
    print(f"[Main] Vehicle controller started → MCU at {args.ip}")

    cameras: dict[str, CameraStream] = {}
    if not args.no_cameras:
        for name, url in CAMERA_URLS.items():
            max_w = CAMERA_MAX_WIDTH.get(name, 640)
            transport = CAMERA_TRANSPORT.get(name, "udp")
            cam = CameraStream(name, url, max_width=max_w, rtsp_transport=transport)
            cameras[name] = cam
            print(f"[Main] Camera created: {name} → {url} (max_width={max_w}, transport={transport})")
    else:
        print("[Main] Cameras disabled (--no-cameras)")

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = DashboardWindow(controller, cameras, fullscreen=not args.windowed)

    gamepad: Optional[GamepadManager] = None
    try:
        gm = GamepadManager(
            controller,
            deadzone=0.10,
            on_cycle_camera=window.cycle_camera,
        )
        if gm.init_pygame():
            gm.start()
            gamepad = gm
            window._hud.gamepad_name = gm.joystick_name
            print(f"[Main] Gamepad active: {gm.joystick_name}")
        else:
            print("[Main] No gamepad found — keyboard fallback (Y/X/A/B/G/L/Space)")
    except Exception as e:
        print(f"[Main] Gamepad init error: {e}")

    window.show()
    exit_code = app.exec_()

    print("[Main] Shutting down...")
    controller.stop()
    if gamepad:
        gamepad.stop()
    for cam in cameras.values():
        cam.stop()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
