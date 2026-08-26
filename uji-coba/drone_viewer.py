"""
Drone 3D Viewer - PySide6
Membaca data BMI160 + BMP280 dari STM32F401 via serial.
Menampilkan animasi drone 3D isometrik yang mengikuti orientasi sensor.
"""

import sys
import re
import math
import time
from PySide6.QtCore import Qt, QTimer, QPointF
from PySide6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QRadialGradient,
    QLinearGradient, QPolygonF, QPainterPath
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton
)

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

IMU_RE = re.compile(
    r"\[IMU\]\s*AX:([-\d.]+)\s*AY:([-\d.]+)\s*AZ:([-\d.]+)"
    r"\s*GX:([-\d.]+)\s*GY:([-\d.]+)\s*GZ:([-\d.]+)"
)
BMP_RE = re.compile(
    r"\[BMP\]\s*T:([-\d.]+)\s*P:([-\d.]+)\s*A:([-\d.]+)"
)

# ─── Catppuccin Mocha ───
COL_BG       = QColor("#1e1e2e")
COL_BG_DARK  = QColor("#181825")
COL_SURF     = QColor("#313244")
COL_OVERLAY  = QColor("#45475a")
COL_SUBTLE   = QColor("#585b70")
COL_TEXT     = QColor("#cdd6f4")
COL_SUBTEXT  = QColor("#a6adc8")
COL_LAVENDER = QColor("#b4befe")
COL_BLUE     = QColor("#89b4fa")
COL_SAPPHIRE = QColor("#74c7ec")
COL_TEAL     = QColor("#94e2d5")
COL_GREEN    = QColor("#a6e3a1")
COL_YELLOW   = QColor("#f9e2af")
COL_PEACH    = QColor("#fab387")
COL_MAROON   = QColor("#eba0ac")
COL_RED      = QColor("#f38ba8")
COL_MAUVE    = QColor("#cba6f7")
COL_PINK     = QColor("#f5c2e7")
COL_ROSEWATER= QColor("#f5e0dc")

DEG = 180.0 / math.pi
RAD = math.pi / 180.0


def darken(c, f=0.6):
    return QColor(int(c.red()*f), int(c.green()*f), int(c.blue()*f))


def style_btn(bg, fg, hover=None):
    if hover is None:
        hover = darken(bg, 0.85)
    return f"""
        QPushButton {{
            background: {bg.name()}; color: {fg.name()};
            border: none; border-radius: 5px;
            font: bold 11px 'Consolas'; padding: 0 14px;
        }}
        QPushButton:hover {{ background: {hover.name()}; }}
        QPushButton:disabled {{ background: {COL_OVERLAY.name()}; color: {COL_SUBTLE.name()}; }}
    """


# ───────────────────── Complementary Filter ─────────────────────

class OrientationFilter:
    def __init__(self, alpha=0.96, dt=0.01):
        self.alpha = alpha
        self.dt = dt
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_time = None

    def reset(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_time = None

    def update(self, ax, ay, az, gx, gy, gz):
        now = time.monotonic()
        if self._last_time is not None:
            self.dt = now - self._last_time
        self._last_time = now
        if self.dt <= 0 or self.dt > 0.5:
            self.dt = 0.01

        acc_roll = math.atan2(ay, az) * DEG
        acc_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * DEG

        self.roll = self.alpha * (self.roll + gx * self.dt) + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * (self.pitch + gy * self.dt) + (1 - self.alpha) * acc_pitch
        self.yaw += gz * self.dt
        if self.yaw > 180:
            self.yaw -= 360
        elif self.yaw < -180:
            self.yaw += 360


# ───────────────────── Drone 3D Widget ─────────────────────

class DroneWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 350)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.altitude = 0.0
        self._prop_angle = 0.0

    def set_orientation(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def set_altitude(self, alt):
        self.altitude = alt

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        bg = QLinearGradient(0, 0, 0, h)
        bg.setColorAt(0.0, QColor("#181825"))
        bg.setColorAt(0.6, QColor("#1e1e2e"))
        bg.setColorAt(1.0, QColor("#11111b"))
        p.fillRect(self.rect(), bg)

        self._draw_ground(p, cx, cy, w, h)

        ground_y = cy + 60
        alt_px = max(-80, min(160, self.altitude * 3))

        shadow_r = max(15, 55 - int(alt_px * 0.15))
        p.setPen(Qt.NoPen)
        shadow = QRadialGradient(cx, ground_y, shadow_r)
        shadow.setColorAt(0.0, QColor(0, 0, 0, 70))
        shadow.setColorAt(1.0, QColor(0, 0, 0, 0))
        p.setBrush(QBrush(shadow))
        p.drawEllipse(QPointF(cx, ground_y), shadow_r, int(shadow_r * 0.3))

        if alt_px > 2:
            p.setPen(QPen(QColor(COL_TEAL.red(), COL_TEAL.green(),
                                 COL_TEAL.blue(), 50), 1, Qt.DashLine))
            p.drawLine(int(cx), int(ground_y), int(cx), int(ground_y - alt_px))

        drone_y = cy - 10 - alt_px
        self._draw_drone(p, cx, drone_y)

        if alt_px > 3:
            p.setPen(COL_TEAL)
            p.setFont(QFont("Consolas", 9))
            p.drawText(int(cx + 80), int(drone_y + 5), f"{self.altitude:.2f} m")

        p.end()

    def _draw_ground(self, p, cx, cy, w, h):
        ground_y = cy + 60

        for i in range(-4, 5):
            y = ground_y + i * 10
            alpha = max(8, 35 - abs(i) * 5)
            c = QColor(COL_OVERLAY)
            c.setAlpha(alpha)
            p.setPen(QPen(c, 1))
            spread = 1.0 - abs(i) * 0.06
            p.drawLine(int(cx - w * 0.45 * spread), int(y),
                       int(cx + w * 0.45 * spread), int(y))

        for i in range(-5, 6):
            x_off = i * 45
            alpha = max(5, 28 - abs(i) * 4)
            c = QColor(COL_OVERLAY)
            c.setAlpha(alpha)
            p.setPen(QPen(c, 1))
            p.drawLine(int(cx + x_off * 0.15), int(ground_y - 40),
                       int(cx + x_off), int(ground_y + 60))

    def _draw_drone(self, p, cx, cy):
        p.save()
        p.translate(cx, cy)

        pitch_rad = self.pitch * RAD
        roll_rad = self.roll * RAD

        sx = 1.0 / max(0.3, math.cos(roll_rad))
        sy = 1.0 / max(0.3, math.cos(pitch_rad))
        sx = max(0.4, min(2.5, sx))
        sy = max(0.4, min(2.5, sy))
        p.scale(sx, sy)

        p.rotate(self.yaw)

        arm_len = 72
        body_r = 20
        motor_r = 9

        arms = [
            ("FR",  45, COL_RED,    True),
            ("FL", 135, COL_BLUE,   True),
            ("BL", 225, COL_SUBTLE, False),
            ("BR", 315, COL_SUBTLE, False),
        ]

        front_y = -arm_len * 0.707
        back_y = arm_len * 0.707
        depth_order = sorted(arms, key=lambda a: (
            math.sin(a[1] * RAD) * arm_len
        ))

        for name, angle, color, is_front in depth_order:
            a = angle * RAD
            ax_ = math.cos(a) * arm_len
            ay_ = math.sin(a) * arm_len

            depth = (ay_ + arm_len) / (2 * arm_len)
            d_alpha = int(140 + depth * 115)
            d_alpha = max(90, min(255, d_alpha))
            d_scale = 0.8 + depth * 0.3

            arm_pen = QColor(COL_SUBTLE)
            arm_pen.setAlpha(d_alpha)
            p.setPen(QPen(arm_pen, int(4 * d_scale)))
            p.drawLine(0, 0, int(ax_), int(ay_))

            m_color = QColor(COL_OVERLAY)
            m_color.setAlpha(d_alpha)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(m_color))
            p.drawEllipse(QPointF(ax_, ay_), motor_r, motor_r)

            self._prop_angle += 22
            prop_r = int(26 * d_scale)
            prop_alpha = max(70, d_alpha - 40)

            p.save()
            p.translate(ax_, ay_)
            p.rotate(self._prop_angle * (1 if name in ("FR", "BL") else -1))

            pc = QColor(COL_BLUE)
            pc.setAlpha(prop_alpha)
            disc = QRadialGradient(0, 0, prop_r)
            disc.setColorAt(0.0, QColor(pc.red(), pc.green(), pc.blue(), prop_alpha))
            disc.setColorAt(0.6, QColor(pc.red(), pc.green(), pc.blue(), prop_alpha // 3))
            disc.setColorAt(1.0, QColor(pc.red(), pc.green(), pc.blue(), 0))
            p.setBrush(QBrush(disc))
            p.setPen(Qt.NoPen)
            p.drawEllipse(QPointF(0, 0), prop_r, prop_r)

            p.setPen(QPen(pc, 2))
            p.drawLine(-prop_r, 0, prop_r, 0)
            p.drawLine(0, -prop_r, 0, prop_r)

            p.restore()

        body_grad = QRadialGradient(-3, -3, body_r * 1.2)
        body_grad.setColorAt(0.0, QColor("#6c7086"))
        body_grad.setColorAt(0.5, QColor("#313244"))
        body_grad.setColorAt(1.0, QColor("#1e1e2e"))
        p.setPen(QPen(COL_SUBTLE, 2))
        p.setBrush(QBrush(body_grad))
        p.drawEllipse(QPointF(0, 0), body_r, body_r)

        p.setPen(Qt.NoPen)
        front_y_pos = -(body_r + 3)
        p.setBrush(QBrush(COL_RED))
        path_front = QPainterPath()
        path_front.moveTo(0, front_y_pos - 10)
        path_front.lineTo(-7, front_y_pos + 4)
        path_front.lineTo(7, front_y_pos + 4)
        path_front.closeSubpath()
        p.drawPath(path_front)

        p.setBrush(QBrush(COL_BLUE))
        back_y_pos = body_r + 3
        p.drawRoundedRect(-8, back_y_pos, 16, 6, 2, 2)

        led = QRadialGradient(0, 0, 5)
        led.setColorAt(0.0, COL_GREEN)
        led.setColorAt(0.4, QColor(COL_GREEN.red(), COL_GREEN.green(),
                                   COL_GREEN.blue(), 150))
        led.setColorAt(1.0, QColor(COL_GREEN.red(), COL_GREEN.green(),
                                   COL_GREEN.blue(), 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(led))
        p.drawEllipse(QPointF(0, 0), 5, 5)

        p.restore()


# ───────────────────── Info Panel ─────────────────────

class InfoPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(90)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.altitude = 0.0
        self.temp = 0.0
        self.pressure = 0.0

    def update_values(self, roll, pitch, yaw, alt, temp, press):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.altitude = alt
        self.temp = temp
        self.pressure = press
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), COL_BG_DARK)

        p.setFont(QFont("Consolas", 10, QFont.Bold))

        col_w = w // 4
        x_start = 16

        self._draw_metric(p, x_start, 8, "ROLL", f"{self.roll:+.1f}\u00b0", COL_BLUE)
        self._draw_metric(p, x_start + col_w, 8, "PITCH", f"{self.pitch:+.1f}\u00b0", COL_TEAL)
        self._draw_metric(p, x_start + col_w * 2, 8, "YAW", f"{self.yaw:+.1f}\u00b0", COL_MAUVE)
        self._draw_metric(p, x_start + col_w * 3, 8, "ALT", f"{self.altitude:+.2f} m", COL_GREEN)

        p.setFont(QFont("Consolas", 9))
        p.setPen(COL_SUBTEXT)
        p.drawText(x_start, h - 12, f"Temp: {self.temp:.1f}\u00b0C")
        p.drawText(x_start + 160, h - 12, f"Press: {self.pressure:.1f} hPa")

        bar_x = x_start + 340
        bar_w = w - bar_x - 20
        bar_h = 10
        bar_y = h - 20
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(COL_OVERLAY))
        p.drawRoundedRect(bar_x, bar_y, bar_w, bar_h, 5, 5)
        alt_fill = max(0, min(1.0, abs(self.altitude) / 50.0))
        if alt_fill > 0.01:
            bar_color = QLinearGradient(bar_x, 0, bar_x + bar_w, 0)
            bar_color.setColorAt(0.0, COL_TEAL)
            bar_color.setColorAt(1.0, COL_GREEN)
            p.setBrush(QBrush(bar_color))
            p.drawRoundedRect(bar_x, bar_y, int(bar_w * alt_fill), bar_h, 5, 5)

        p.end()

    def _draw_metric(self, p, x, y, label, value, color):
        p.setPen(COL_SUBTEXT)
        p.setFont(QFont("Consolas", 8))
        p.drawText(x, y + 10, label)
        p.setPen(color)
        p.setFont(QFont("Consolas", 14, QFont.Bold))
        p.drawText(x, y + 32, value)


# ───────────────────── Main Window ─────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone 3D Viewer — BMI160 + BMP280")
        self.setMinimumSize(620, 560)
        self.resize(720, 600)

        self._serial = None
        self._buf = b""

        self._orient = OrientationFilter(alpha=0.96, dt=0.01)
        self._alt_offset = 0.0
        self._alt_smooth = 0.0
        self._temp = 0.0
        self._press = 0.0
        self._first_alt = True

        central = QWidget()
        central.setStyleSheet(f"background:{COL_BG_DARK.name()};")
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(14, 10, 14, 12)
        root.setSpacing(8)

        top = QHBoxLayout()
        top.setSpacing(6)

        lbl = QLabel("PORT")
        lbl.setStyleSheet(f"color:{COL_SUBTEXT.name()}; font:bold 10px 'Consolas'; letter-spacing:2px;")
        top.addWidget(lbl)

        self._port_cb = QComboBox()
        self._port_cb.setMinimumWidth(130)
        self._port_cb.setFixedHeight(30)
        self._port_cb.setStyleSheet(f"""
            QComboBox {{
                background: {COL_SURF.name()}; color: {COL_TEXT.name()};
                border: 1px solid {COL_SUBTLE.name()}; border-radius: 5px;
                padding: 4px 10px; font: 11px 'Consolas';
            }}
            QComboBox::drop-down {{ border: none; width: 24px; }}
            QComboBox QAbstractItemView {{
                background: {COL_SURF.name()}; color: {COL_TEXT.name()};
                border: 1px solid {COL_SUBTLE.name()};
                selection-background-color: {COL_OVERLAY.name()};
            }}
        """)
        top.addWidget(self._port_cb)

        self._refresh_btn = QPushButton("\u21bb")
        self._refresh_btn.setFixedSize(30, 30)
        self._refresh_btn.setStyleSheet(f"""
            QPushButton {{
                background: {COL_OVERLAY.name()}; color: {COL_TEXT.name()};
                border: none; border-radius: 5px; font: 14px 'Consolas';
            }}
            QPushButton:hover {{ background: {COL_SUBTLE.name()}; }}
        """)
        self._refresh_btn.clicked.connect(self._refresh_ports)
        top.addWidget(self._refresh_btn)

        self._connect_btn = QPushButton("CONNECT")
        self._connect_btn.setFixedHeight(30)
        self._connect_btn.setStyleSheet(style_btn(COL_GREEN, COL_BG_DARK, COL_TEAL))
        self._connect_btn.clicked.connect(self._toggle_serial)
        top.addWidget(self._connect_btn)

        self._calib_btn = QPushButton("CALIBRATE")
        self._calib_btn.setFixedHeight(30)
        self._calib_btn.setStyleSheet(style_btn(COL_MAUVE, COL_BG_DARK, COL_LAVENDER))
        self._calib_btn.clicked.connect(self._calibrate)
        top.addWidget(self._calib_btn)

        self._status_lbl = QLabel("\u25cf  Disconnected")
        self._status_lbl.setStyleSheet(f"color:{COL_RED.name()}; font: bold 10px 'Consolas';")
        top.addWidget(self._status_lbl)
        top.addStretch()
        root.addLayout(top)

        sep = QLabel()
        sep.setFixedHeight(1)
        sep.setStyleSheet(f"background:{COL_SUBTLE.name()};")
        root.addWidget(sep)

        self._drone = DroneWidget()
        root.addWidget(self._drone, stretch=1)

        self._info = InfoPanel()
        root.addWidget(self._info)

        self._serial_timer = QTimer(self)
        self._serial_timer.timeout.connect(self._read_serial)
        self._serial_timer.start(16)

        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._tick)
        self._anim_timer.start(16)

        self._refresh_ports()

    def _tick(self):
        self._drone.update()

    def _calibrate(self):
        self._orient.reset()
        self._alt_offset = 0.0
        self._alt_smooth = 0.0
        self._first_alt = True
        self._drone.set_orientation(0, 0, 0)
        self._drone.set_altitude(0.0)
        self._info.update_values(0, 0, 0, 0.0, self._temp, self._press)

    def _refresh_ports(self):
        self._port_cb.clear()
        if not HAS_SERIAL:
            self._port_cb.addItem("(pyserial not installed)")
            return
        for p in serial.tools.list_ports.comports():
            self._port_cb.addItem(p.device)

    def _toggle_serial(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
            self._connect_btn.setText("CONNECT")
            self._connect_btn.setStyleSheet(style_btn(COL_GREEN, COL_BG_DARK, COL_TEAL))
            self._status_lbl.setText("\u25cf  Disconnected")
            self._status_lbl.setStyleSheet(f"color:{COL_RED.name()}; font: bold 10px 'Consolas';")
            return
        if not HAS_SERIAL:
            self._status_lbl.setText("\u25cf  pyserial missing")
            return
        port = self._port_cb.currentText()
        if not port or port.startswith("("):
            return
        try:
            self._serial = serial.Serial(port, 115200, timeout=0.02)
            self._buf = b""
            self._first_alt = True
            self._connect_btn.setText("DISCONNECT")
            self._connect_btn.setStyleSheet(style_btn(COL_RED, COL_BG_DARK, COL_MAROON))
            self._status_lbl.setText(f"\u25cf  Connected  {port}")
            self._status_lbl.setStyleSheet(f"color:{COL_GREEN.name()}; font: bold 10px 'Consolas';")
        except Exception as e:
            self._status_lbl.setText(f"\u25cf  {e}")
            self._status_lbl.setStyleSheet(f"color:{COL_RED.name()};")

    def _read_serial(self):
        if not self._serial or not self._serial.is_open:
            return
        try:
            chunk = self._serial.read(self._serial.in_waiting or 1)
        except Exception:
            self._toggle_serial()
            return
        if not chunk:
            return
        self._buf += chunk
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            self._parse_line(line.decode("ascii", errors="ignore").strip())

    def _parse_line(self, text: str):
        m = IMU_RE.search(text)
        if m:
            ax, ay, az = [float(v) for v in m.groups()[:3]]
            gx, gy, gz = [float(v) for v in m.groups()[3:]]
            self._orient.update(ax, ay, az, gx, gy, gz)
            self._drone.set_orientation(self._orient.roll, self._orient.pitch, self._orient.yaw)
            self._info.update_values(
                self._orient.roll, self._orient.pitch, self._orient.yaw,
                self._alt_smooth, self._temp, self._press
            )
            return

        m = BMP_RE.search(text)
        if m:
            self._temp = float(m.group(1))
            self._press = float(m.group(2))
            alt = float(m.group(3))
            if self._first_alt:
                self._alt_offset = alt
                self._first_alt = False
            corrected = alt - self._alt_offset
            self._alt_smooth += (corrected - self._alt_smooth) * 0.15
            self._drone.set_altitude(self._alt_smooth)
            self._info.update_values(
                self._orient.roll, self._orient.pitch, self._orient.yaw,
                self._alt_smooth, self._temp, self._press
            )


def main():
    import traceback
    try:
        app = QApplication(sys.argv)
        app.setStyle("Fusion")
        win = MainWindow()
        win.show()
        sys.exit(app.exec())
    except Exception:
        traceback.print_exc()
        input("\nTekan Enter untuk keluar...")


if __name__ == "__main__":
    main()
