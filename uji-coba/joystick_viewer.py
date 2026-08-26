"""
Joystick Viewer - PySide6
Membaca data serial dari ESP32 (format: [TX] R:xxx T:xxx Y:xxx P:xxx)
Menampilkan visualisasi posisi 2 joystick (Mode 2) + kalibrasi dari PC.
"""

import sys
import re
from PySide6.QtCore import Qt, QTimer, QPointF
from PySide6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QRadialGradient
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

DATA_RE = re.compile(r"\[TX\]\s*R:(\d+)\s*T:(\d+)\s*Y:(\d+)\s*P:(\d+)")

# ─── Catppuccin Mocha palette ───
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
COL_MAROON   = QColor("#eba0ac")
COL_RED      = QColor("#f38ba8")
COL_MAUVE    = QColor("#cba6f7")
COL_PINK     = QColor("#f5c2e7")


def darken(c, factor=0.6):
    return QColor(int(c.red()*factor), int(c.green()*factor), int(c.blue()*factor))


def style_btn(bg, fg, hover_bg=None):
    if hover_bg is None:
        hover_bg = darken(bg, 0.85)
    return f"""
        QPushButton {{
            background: {bg.name()}; color: {fg.name()};
            border: none; border-radius: 5px;
            font: bold 11px 'Consolas'; padding: 0 14px;
        }}
        QPushButton:hover {{ background: {hover_bg.name()}; }}
        QPushButton:disabled {{ background: {COL_OVERLAY.name()}; color: {COL_SUBTLE.name()}; }}
    """


# ───────────────────── Joystick Widget ─────────────────────

class JoystickWidget(QWidget):
    def __init__(self, label="Joystick", parent=None):
        super().__init__(parent)
        self.setMinimumSize(240, 240)
        self._target_x = 0.5
        self._target_y = 0.5
        self._x = 0.5
        self._y = 0.5
        self._label = label
        self._vx_label = "X"
        self._vy_label = "Y"
        self._trail_points = []
        self._max_trail = 12

    def setLabels(self, vx, vy):
        self._vx_label = vx
        self._vy_label = vy

    def set_position(self, nx: float, ny: float):
        self._target_x = max(0.0, min(1.0, nx))
        self._target_y = max(0.0, min(1.0, ny))

    def _animate(self):
        ease = 0.18
        old_x, old_y = self._x, self._y
        self._x += (self._target_x - self._x) * ease
        self._y += (self._target_y - self._y) * ease
        if abs(self._x - old_x) > 0.001 or abs(self._y - old_y) > 0.001:
            self._trail_points.append((self._x, self._y))
            if len(self._trail_points) > self._max_trail:
                self._trail_points.pop(0)
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2 + 6
        radius = min(w, h) * 0.34

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        p.fillRect(self.rect(), COL_BG)

        grad = QRadialGradient(cx, cy, radius * 1.15)
        grad.setColorAt(0.0, QColor(COL_SURF.red(), COL_SURF.green(), COL_SURF.blue(), 0))
        grad.setColorAt(0.85, QColor(COL_SURF.red(), COL_SURF.green(), COL_SURF.blue(), 0))
        grad.setColorAt(1.0, QColor(COL_SUBTLE.red(), COL_SUBTLE.green(), COL_SUBTLE.blue(), 60))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(grad))
        p.drawEllipse(QPointF(cx, cy), radius * 1.15, radius * 1.15)

        outer_grad = QRadialGradient(cx, cy, radius)
        outer_grad.setColorAt(0.0, COL_SURF)
        outer_grad.setColorAt(1.0, QColor("#252536"))
        p.setPen(QPen(COL_SUBTLE, 2.5))
        p.setBrush(QBrush(outer_grad))
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        for frac in [0.33, 0.66]:
            p.setPen(QPen(QColor(COL_OVERLAY.red(), COL_OVERLAY.green(),
                                 COL_OVERLAY.blue(), 90), 1, Qt.DotLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), radius * frac, radius * frac)

        p.setPen(QPen(QColor(COL_OVERLAY.red(), COL_OVERLAY.green(),
                             COL_OVERLAY.blue(), 120), 1))
        p.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        p.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))

        dead_r = radius * 0.07
        p.setPen(QPen(QColor(COL_RED.red(), COL_RED.green(), COL_RED.blue(), 120), 1.5, Qt.DashLine))
        p.setBrush(QBrush(QColor(COL_RED.red(), COL_RED.green(), COL_RED.blue(), 25)))
        p.drawEllipse(QPointF(cx, cy), dead_r, dead_r)

        n_trail = len(self._trail_points)
        for i, (tx, ty) in enumerate(self._trail_points):
            t_alpha = int(30 + 80 * (i / max(n_trail, 1)))
            tx_px = cx + (tx - 0.5) * 2 * radius
            ty_px = cy + (ty - 0.5) * 2 * radius
            trail_r = 3 + 2 * (i / max(n_trail, 1))
            tc = QColor(COL_LAVENDER)
            tc.setAlpha(t_alpha)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(tc))
            p.drawEllipse(QPointF(tx_px, ty_px), trail_r, trail_r)

        sx = cx + (self._x - 0.5) * 2 * radius
        sy = cy + (self._y - 0.5) * 2 * radius

        p.setPen(QPen(COL_BLUE, 2.2, Qt.SolidLine))
        p.drawLine(int(cx), int(cy), int(sx), int(sy))

        knob = radius * 0.16
        for scale, alpha in [(2.8, 25), (2.0, 40), (1.5, 70)]:
            gc = QColor(COL_BLUE)
            gc.setAlpha(alpha)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(gc))
            p.drawEllipse(QPointF(sx, sy), knob * scale, knob * scale)

        knob_grad = QRadialGradient(sx - knob * 0.25, sy - knob * 0.25, knob * 1.2)
        knob_grad.setColorAt(0.0, QColor("#e0e0f0"))
        knob_grad.setColorAt(0.5, COL_TEXT)
        knob_grad.setColorAt(1.0, COL_LAVENDER)
        p.setPen(QPen(QColor(COL_BLUE.red(), COL_BLUE.green(), COL_BLUE.blue(), 160), 1.5))
        p.setBrush(QBrush(knob_grad))
        p.drawEllipse(QPointF(sx, sy), knob, knob)

        spec_grad = QRadialGradient(sx - knob * 0.3, sy - knob * 0.35, knob * 0.5)
        spec_grad.setColorAt(0.0, QColor(255, 255, 255, 160))
        spec_grad.setColorAt(1.0, QColor(255, 255, 255, 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(spec_grad))
        p.drawEllipse(QPointF(sx - knob * 0.2, sy - knob * 0.25), knob * 0.45, knob * 0.35)

        p.setPen(COL_SUBTEXT)
        p.setFont(QFont("Consolas", 9))
        p.drawText(int(cx + radius + 6), int(cy + 4), self._vx_label)
        p.drawText(int(cx - 6), int(cy - radius - 8), self._vy_label)

        arrow_col = QColor(COL_SUBTLE)
        arrow_col.setAlpha(140)
        p.setPen(QPen(arrow_col, 1.5))
        ax = cx + radius + 2
        p.drawLine(int(ax), int(cy - 3), int(ax + 5), int(cy))
        p.drawLine(int(ax), int(cy + 3), int(ax + 5), int(cy))
        ay = cy - radius - 2
        p.drawLine(int(cx - 3), int(ay), int(cx), int(ay - 5))
        p.drawLine(int(cx + 3), int(ay), int(cx), int(ay - 5))

        p.setFont(QFont("Segoe UI", 11, QFont.Bold))
        p.setPen(COL_PINK)
        p.drawText(self.rect(), Qt.AlignHCenter | Qt.AlignTop, f"\n{self._label}")

        p.setFont(QFont("Consolas", 10, QFont.Bold))
        val_x = int(self._x * 255)
        val_y = int((1.0 - self._y) * 255)
        p.setPen(COL_GREEN)
        p.drawText(self.rect().adjusted(0, 0, 0, -8), Qt.AlignHCenter | Qt.AlignBottom,
                   f"{self._vx_label}: {val_x}    {self._vy_label}: {val_y}")

        p.end()


# ───────────────────── Main Window ─────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Joystick Viewer  —  ESP32 LoRa TX")
        self.setMinimumSize(600, 480)
        self.resize(680, 500)

        self._serial = None
        self._buf = b""

        self._calib_sampling = False
        self._calib_samples = []
        self._calib_sample_max = 60
        self._calib_center = None   # (r, t, y, p) average center values

        central = QWidget()
        central.setStyleSheet(f"background:{COL_BG_DARK.name()};")
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(14, 10, 14, 12)
        root.setSpacing(8)

        # ── top bar ──
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
        self._calib_btn.setEnabled(False)
        self._calib_btn.setStyleSheet(style_btn(COL_MAUVE, COL_BG_DARK, COL_LAVENDER))
        self._calib_btn.clicked.connect(self._start_calibration)
        top.addWidget(self._calib_btn)

        self._reset_calib_btn = QPushButton("RESET CAL")
        self._reset_calib_btn.setFixedHeight(30)
        self._reset_calib_btn.setVisible(False)
        self._reset_calib_btn.setStyleSheet(style_btn(COL_RED, COL_BG_DARK, COL_MAROON))
        self._reset_calib_btn.clicked.connect(self._reset_calibration)
        top.addWidget(self._reset_calib_btn)

        self._status_lbl = QLabel("\u25cf  Disconnected")
        self._status_lbl.setStyleSheet(f"color:{COL_RED.name()}; font: bold 10px 'Consolas';")
        top.addWidget(self._status_lbl)
        top.addStretch()
        root.addLayout(top)

        # ── separator ──
        sep = QLabel()
        sep.setFixedHeight(1)
        sep.setStyleSheet(f"background:{COL_SUBTLE.name()};")
        root.addWidget(sep)

        # ── calib status ──
        self._calib_status = QLabel("")
        self._calib_status.setAlignment(Qt.AlignCenter)
        self._calib_status.setFixedHeight(22)
        self._calib_status.setVisible(False)
        self._calib_status.setStyleSheet(f"""
            QLabel {{
                background: {COL_SURF.name()}; color: {COL_TEAL.name()};
                font: 9px 'Consolas'; border-radius: 4px;
            }}
        """)
        root.addWidget(self._calib_status)

        # ── joysticks row ──
        row = QHBoxLayout()
        row.setSpacing(20)

        self._joy_left = JoystickWidget("LEFT  (Mode 2)")
        self._joy_left.setLabels("YAW", "PITCH")
        row.addWidget(self._joy_left)

        self._joy_right = JoystickWidget("RIGHT (Mode 2)")
        self._joy_right.setLabels("ROLL", "THROTTLE")
        row.addWidget(self._joy_right)

        root.addLayout(row)

        # ── CSV readout ──
        self._csv_lbl = QLabel("R: 128    T: 128    Y: 128    P: 128")
        self._csv_lbl.setAlignment(Qt.AlignCenter)
        self._csv_lbl.setFixedHeight(36)
        self._csv_lbl.setStyleSheet(f"""
            QLabel {{
                background: {COL_SURF.name()}; color: {COL_YELLOW.name()};
                font: bold 12px 'Consolas';
                border: 1px solid {COL_SUBTLE.name()}; border-radius: 6px;
            }}
        """)
        root.addWidget(self._csv_lbl)

        # ── timers ──
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._read_serial)
        self._timer.start(16)

        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._tick_anim)
        self._anim_timer.start(16)

        self._calib_timer = QTimer(self)
        self._calib_timer.timeout.connect(self._tick_calibration)
        self._calib_timer.setInterval(50)

        self._refresh_ports()

    def _tick_anim(self):
        self._joy_left._animate()
        self._joy_right._animate()

    # ── serial ──

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
            self._calib_btn.setEnabled(False)
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
            self._connect_btn.setText("DISCONNECT")
            self._connect_btn.setStyleSheet(style_btn(COL_RED, COL_BG_DARK, COL_MAROON))
            self._status_lbl.setText(f"\u25cf  Connected  {port}")
            self._status_lbl.setStyleSheet(f"color:{COL_GREEN.name()}; font: bold 10px 'Consolas';")
            self._calib_btn.setEnabled(True)
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
        m = DATA_RE.search(text)
        if not m:
            return
        r, t, y, p = [int(v) for v in m.groups()]

        if self._calib_sampling:
            self._calib_samples.append((r, t, y, p))

        rc, tc, yc, pc = r, t, y, p
        if self._calib_center:
            cr, ct, cy_, cp = self._calib_center
            rc = max(0, min(255, 128 + (r - cr)))
            tc = max(0, min(255, 128 + (t - ct)))
            yc = max(0, min(255, 128 + (y - cy_)))
            pc = max(0, min(255, 128 + (p - cp)))

        self._joy_left.set_position(yc / 255.0, 1.0 - pc / 255.0)
        self._joy_right.set_position(rc / 255.0, 1.0 - tc / 255.0)
        self._csv_lbl.setText(f"R: {rc}    T: {tc}    Y: {yc}    P: {pc}")

    # ── calibration: klik → langsung sampling 1 detik → set center ──

    def _start_calibration(self):
        if not self._serial or not self._serial.is_open:
            return
        self._calib_sampling = True
        self._calib_samples = []
        self._calib_btn.setEnabled(False)
        self._calib_btn.setText("SAMPLING...")
        self._calib_status.setText("Kalibrasi: pegang stick di TENGAH, jangan digerakkan...")
        self._calib_status.setVisible(True)
        self._calib_timer.start()

    def _tick_calibration(self):
        if len(self._calib_samples) >= self._calib_sample_max:
            self._finish_calibration()

    def _finish_calibration(self):
        self._calib_sampling = False
        self._calib_timer.stop()

        samples = self._calib_samples
        if len(samples) < 5:
            self._calib_status.setText("Gagal: tidak cukup data. Coba lagi.")
            self._calib_btn.setEnabled(True)
            self._calib_btn.setText("CALIBRATE")
            return

        n = len(samples)
        cr = sum(s[0] for s in samples) // n
        ct = sum(s[1] for s in samples) // n
        cy = sum(s[2] for s in samples) // n
        cp = sum(s[3] for s in samples) // n
        self._calib_center = (cr, ct, cy, cp)

        self._calib_status.setText(
            f"Kalibrasi OK  |  Center: R={cr}  T={ct}  Y={cy}  P={cp}  |  Sampel: {n}"
        )
        self._calib_btn.setEnabled(True)
        self._calib_btn.setText("CALIBRATE")
        self._reset_calib_btn.setVisible(True)

    def _reset_calibration(self):
        self._calib_center = None
        self._calib_samples = []
        self._calib_status.setVisible(False)
        self._reset_calib_btn.setVisible(False)


# ───────────────────────── entry ─────────────────────────

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
