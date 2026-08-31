
"""
Drone Telemetry Viewer - PySide6
Dashboard telemetry drone bergaya Ground Control Station (GCS) profesional
dengan tema Sky Blue & White.

Data diterima dari SATU koneksi serial USB ke Remote (ESP32 + LoRa RA-02).
Remote bertindak sebagai hub: mengirim joystick ke drone via LoRa sekaligus
meneruskan telemetri balasan drone (IMU + altitude) ke laptop. Suhu (BMP280)
tidak dipakai sama sekali pada proyek ini.
  - [IMU] AX:.. AY:.. AZ:.. GX:.. GY:.. GZ:..    (BMI160 di drone)
  - [BMP] P:..  A:..                             (BMP280 di drone, tanpa suhu)
  - [TX]  R:..  T:..  Y:..  P:..                 (joystick remote ESP32)

Tab :
  - ATTITUDE   : artificial horizon + drone 3D + altitude tape + metric cards
  - RC CONTROL : dual joystick (Mode 2) + readout raw/calibrated + calibrate
"""

import sys
import re
import math
import time
from collections import deque
from PySide6.QtCore import Qt, QTimer, QPointF, QRectF, QPoint
from PySide6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QFontMetricsF,
    QRadialGradient, QLinearGradient, QPolygonF, QPainterPath,
    QAction, QPixmap, QIcon
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton, QFrame,
    QGraphicsDropShadowEffect, QFormLayout, QDoubleSpinBox,
    QStackedWidget, QMenu
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
    r"\[BMP\]\s*P:([-\d.]+)\s*A:([-\d.]+)"
)
TX_RE = re.compile(
    r"\[TX\]\s*R:(\d+)\s*T:(\d+)\s*Y:(\d+)\s*P:(\d+)"
)
SMC_RE = re.compile(
    r"\[SMC\]\s*ROLL:([-\d.]+)\s*PITCH:([-\d.]+)"
    r"\s*UR:([-\d.]+)\s*UP:([-\d.]+)\s*UY:([-\d.]+)"
)
CAL_OK_RE = re.compile(
    r"\[CAL\]\s*OK\s*CR=(\d+)\s*CT=(\d+)\s*CY=(\d+)\s*CP=(\d+)"
)

# ─────────────── Palette : Sky Blue & White ───────────────
COL_BG          = QColor("#F5F9FF")   # window bg
COL_CARD        = QColor("#FFFFFF")   # card bg
COL_CARD_ALT    = QColor("#F1F5F9")   # inner card / muted
COL_BORDER      = QColor("#E2E8F0")   # subtle border
COL_DIVIDER     = QColor("#CBD5E1")
COL_TEXT        = QColor("#0F172A")   # slate 900
COL_SUBTEXT     = QColor("#64748B")   # slate 500
COL_MUTED       = QColor("#94A3B8")
COL_ACCENT      = QColor("#2563EB")   # blue 600 - primary
COL_ACCENT_2    = QColor("#3B82F6")   # blue 500
COL_ACCENT_LT   = QColor("#7DD3FC")   # sky 300
COL_ACCENT_XLT  = QColor("#BFE3FF")   # very light sky
COL_ACCENT_DK   = QColor("#1D4ED8")   # blue 700
COL_SKY_TOP     = QColor("#BFE3FF")
COL_SKY_MID     = QColor("#5FA8F5")
COL_SKY_BOT     = QColor("#2E7FE0")
COL_GROUND_TOP  = QColor("#F1F5F9")
COL_GROUND_BOT  = QColor("#FFFFFF")
COL_OK          = QColor("#22C55E")   # green 500
COL_WARN        = QColor("#F59E0B")   # amber 500
COL_ERR         = QColor("#EF4444")   # red 500

DEG = 180.0 / math.pi
RAD = math.pi / 180.0


# ─────────────── Typography ───────────────
# Prefer modern variable/geometric sans-serif; Qt will fall back gracefully
# through the family list until it finds one installed on the system.
FONT_UI = "Segoe UI, Inter, 'Segoe UI Variable', Arial, sans-serif"
FONT_MONO = "Consolas, 'Cascadia Mono', 'JetBrains Mono', 'Fira Code', monospace"
FONT_TITLE = "Segoe UI, 'Space Grotesk', Inter, Arial, sans-serif"
# CSS-friendly (Qt stylesheet) equivalents
CSS_UI = FONT_UI
CSS_MONO = FONT_MONO
CSS_TITLE = FONT_TITLE


def qfont(family=FONT_UI, size=10, weight=QFont.Normal, letter_spacing=None):
    """Create a QFont from a CSS-style family list (comma-separated with
    optional quotes). Qt uses only the first family from setFamily, so we
    also apply setFamilies() to get real fallback behaviour."""
    families = [f.strip().strip("'\"") for f in family.split(",") if f.strip()]
    f = QFont(families[0] if families else "Segoe UI", size)
    try:
        f.setFamilies(families)
    except Exception:
        pass
    f.setWeight(weight)
    f.setStyleStrategy(QFont.PreferAntialias)
    f.setHintingPreference(QFont.PreferNoHinting)
    if letter_spacing is not None:
        f.setLetterSpacing(QFont.AbsoluteSpacing, letter_spacing)
    return f


def add_shadow(widget, blur=18, dx=0, dy=2, alpha=28):
    eff = QGraphicsDropShadowEffect(widget)
    eff.setBlurRadius(blur)
    eff.setOffset(dx, dy)
    eff.setColor(QColor(15, 23, 42, alpha))
    widget.setGraphicsEffect(eff)


# ───────────────────── Complementary Filter ─────────────────────

class OrientationFilter:
    def __init__(self, dt=0.12):
        self.dt = dt
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_time = None
        self._gz_deadband = 0.12  # dps deadband untuk meniadakan micro-drift saat diam
        self._first_run = True

    def reset(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_time = None
        self._first_run = True

    def update(self, ax, ay, az, gx, gy, gz):
        now = time.monotonic()
        if self._last_time is not None:
            self.dt = now - self._last_time
        self._last_time = now
        if self.dt <= 0 or self.dt > 0.5:
            self.dt = 0.12

        acc_total = math.sqrt(ax*ax + ay*ay + az*az)
        if acc_total > 1.0:
            acc_roll  = math.atan2(ay, az) * DEG
            acc_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * DEG
        else:
            acc_roll = self.roll
            acc_pitch = self.pitch

        # Pada sampel pertama setelah connect/reset, langsung kunci sudut accelerometer
        # tanpa delay agar drone tidak merayap/bergerak sendiri dari 0 derajat
        if self._first_run:
            self.roll = acc_roll
            self.pitch = acc_pitch
            self._first_run = False
            return

        # Adaptive complementary filter berdasarkan dt aktual
        tau = 0.8
        alpha = tau / (tau + self.dt)

        self.roll  = alpha * (self.roll  + gx * self.dt) + (1.0 - alpha) * acc_roll
        self.pitch = alpha * (self.pitch + gy * self.dt) + (1.0 - alpha) * acc_pitch

        # Deadband filter untuk Gyro Z (Yaw): abaikan noise mikro di bawah threshold
        gz_filtered = gz if abs(gz) >= self._gz_deadband else 0.0
        self.yaw += gz_filtered * self.dt

        if self.yaw > 180:
            self.yaw -= 360
        elif self.yaw < -180:
            self.yaw += 360


# ───────────────────── 3D helpers ─────────────────────

def rot_matrix(roll_deg, pitch_deg, yaw_deg):
    r = roll_deg * RAD
    p = pitch_deg * RAD
    y = yaw_deg * RAD
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr           ],
    ]


def mv(m, v):
    return (
        m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
        m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
        m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
    )


# ───────────────────── Horizon + Drone 3D Widget ─────────────────────

class HorizonDroneWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(380, 280)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.altitude = 0.0
        self._prop_angle = 0.0
        self._connected = False

    def set_orientation(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def set_altitude(self, alt):
        self.altitude = alt

    def set_connected(self, ok):
        self._connected = ok

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(0, 0, -1, -1)
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD))
        p.drawRoundedRect(rect, 12, 12)

        margin = 12
        compass_h = 34
        inner = QRectF(rect.left() + margin,
                       rect.top() + margin + compass_h + 6,
                       rect.width() - 2*margin,
                       rect.height() - 2*margin - compass_h - 6)

        self._draw_compass_tape(p, QRectF(rect.left()+margin, rect.top()+margin,
                                          rect.width()-2*margin, compass_h))
        self._draw_horizon(p, inner)
        self._draw_horizon_overlay(p, inner)
        self._draw_drone(p, inner.center().x(), inner.center().y())

        p.end()

    def _draw_compass_tape(self, p, rect):
        p.save()
        p.setClipRect(rect)
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD_ALT))
        p.drawRoundedRect(rect, 6, 6)

        cx = rect.center().x()
        px_per_deg = 3.2
        yaw = self.yaw

        p.setFont(qfont(FONT_UI, 8, QFont.DemiBold))
        for deg in range(-180, 361, 5):
            rel = ((deg - yaw + 540) % 360) - 180
            x = cx + rel * px_per_deg
            if x < rect.left() or x > rect.right():
                continue
            major = (deg % 30 == 0)
            mid = (deg % 10 == 0)
            if major:
                p.setPen(QPen(COL_TEXT, 1.5))
                p.drawLine(int(x), int(rect.top()+5), int(x), int(rect.top()+16))
                label = self._heading_label(deg % 360)
                p.setPen(COL_TEXT)
                p.drawText(QRectF(x-18, rect.top()+17, 36, 14),
                           Qt.AlignCenter, label)
            elif mid:
                p.setPen(QPen(COL_SUBTEXT, 1))
                p.drawLine(int(x), int(rect.top()+7), int(x), int(rect.top()+14))
            else:
                p.setPen(QPen(COL_MUTED, 1))
                p.drawLine(int(x), int(rect.top()+9), int(x), int(rect.top()+13))

        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(COL_ACCENT))
        tri = QPolygonF([
            QPointF(cx, rect.top()+3),
            QPointF(cx-5, rect.top()+11),
            QPointF(cx+5, rect.top()+11),
        ])
        p.drawPolygon(tri)
        p.restore()

    @staticmethod
    def _heading_label(deg):
        deg = deg % 360
        mapping = {0: "N", 90: "E", 180: "S", 270: "W",
                   45: "NE", 135: "SE", 225: "SW", 315: "NW"}
        if deg in mapping:
            return mapping[deg]
        return f"{deg:03d}"

    def _draw_horizon(self, p, rect):
        p.save()
        path = QPainterPath()
        path.addRoundedRect(rect, 10, 10)
        p.setClipPath(path)

        cx = rect.center().x()
        cy = rect.center().y()
        px_per_deg_pitch = rect.height() / 90.0
        pitch_offset = self.pitch * px_per_deg_pitch

        p.translate(cx, cy)
        p.rotate(-self.roll)

        big = max(rect.width(), rect.height()) * 2.2
        sky = QLinearGradient(0, -big, 0, pitch_offset)
        sky.setColorAt(0.0, COL_SKY_BOT)
        sky.setColorAt(0.6, COL_SKY_MID)
        sky.setColorAt(1.0, COL_SKY_TOP)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(sky))
        p.drawRect(QRectF(-big, -big, big*2, big + pitch_offset))

        ground = QLinearGradient(0, pitch_offset, 0, big)
        ground.setColorAt(0.0, COL_GROUND_TOP)
        ground.setColorAt(1.0, COL_GROUND_BOT)
        p.setBrush(QBrush(ground))
        p.drawRect(QRectF(-big, pitch_offset, big*2, big - pitch_offset))

        p.setPen(QPen(COL_ACCENT_DK, 1.6))
        p.drawLine(int(-big), int(pitch_offset), int(big), int(pitch_offset))

        p.setFont(qfont(FONT_UI, 7, QFont.DemiBold))
        skip_minor = px_per_deg_pitch < 3.0
        for pdeg in range(-40, 41, 5):
            if pdeg == 0:
                continue
            if skip_minor and pdeg % 10 != 0:
                continue
            y = pitch_offset - pdeg * px_per_deg_pitch
            if abs(y) > big * 0.6:
                continue
            major = (pdeg % 10 == 0)
            length = 46 if major else 22
            color = QColor(COL_ACCENT_DK) if pdeg > 0 else QColor(COL_SUBTEXT)
            color.setAlpha(180)
            p.setPen(QPen(color, 1.2))
            p.drawLine(int(-length/2), int(y), int(length/2), int(y))
            if major:
                p.setPen(color)
                p.drawText(QRectF(-length/2 - 28, y-7, 22, 14),
                           Qt.AlignRight | Qt.AlignVCenter, f"{pdeg:+d}")
                p.drawText(QRectF(length/2 + 6, y-7, 22, 14),
                           Qt.AlignLeft | Qt.AlignVCenter, f"{pdeg:+d}")
        p.restore()

    def _draw_horizon_overlay(self, p, rect):
        p.save()
        cx = rect.center().x()
        cy = rect.center().y()

        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(rect, 10, 10)

        arc_r = min(rect.width(), rect.height()) * 0.42
        p.translate(cx, cy)
        p.setPen(QPen(QColor(255, 255, 255, 200), 1.6))
        arc_rect = QRectF(-arc_r, -arc_r, 2*arc_r, 2*arc_r)
        p.drawArc(arc_rect, (90 - 60) * 16, 120 * 16)

        for deg in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            ang = (90 - deg) * RAD
            outer = arc_r
            inner = arc_r - (10 if deg % 30 == 0 else 6)
            x1 = math.cos(ang) * inner
            y1 = -math.sin(ang) * inner
            x2 = math.cos(ang) * outer
            y2 = -math.sin(ang) * outer
            p.setPen(QPen(QColor(255, 255, 255, 220), 1.4))
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

        p.save()
        p.rotate(self.roll)
        pointer = QPolygonF([
            QPointF(0, -arc_r + 2),
            QPointF(-6, -arc_r + 14),
            QPointF(6, -arc_r + 14),
        ])
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(COL_ACCENT))
        p.drawPolygon(pointer)
        p.restore()

        p.setBrush(QBrush(QColor(255, 255, 255, 230)))
        p.setPen(QPen(COL_ACCENT_DK, 1))
        ref = QPolygonF([
            QPointF(0, -arc_r - 4),
            QPointF(-6, -arc_r - 14),
            QPointF(6, -arc_r - 14),
        ])
        p.drawPolygon(ref)

        p.restore()

    def _chase_cam_project(self, v, sin_e, cos_e, cam_dist, focal):
        """Proyeksikan satu titik 3D (body frame: X=depan, Y=kanan, Z=atas)
        ke koordinat layar memakai kamera chase-view yang diam di belakang
        & sedikit di atas drone. Mengembalikan (sx, sy, depth, k)."""
        wx, wy, wz = v
        xc = wy
        yc = wx * sin_e + wz * cos_e
        zc = cam_dist + wx * cos_e - wz * sin_e
        if zc < 1.0:
            zc = 1.0
        k = focal / zc
        return xc * k, -yc * k, zc, k

    def _draw_drone(self, p, cx, cy):
        p.save()
        p.translate(cx, cy)

        s = max(0.55, min(1.0, min(self.width(), self.height()) / 520.0))
        arm_len  = 58 * s
        body_len = 20 * s
        body_wid = 9  * s
        motor_r  = 6.5 * s
        prop_r   = 22 * s

        # ── Kamera chase-view: diam di belakang & sedikit di atas drone,
        # SELALU mengikuti heading (yaw diabaikan di matriks rotasi) sehingga
        # yaw TIDAK memutar seluruh tampilan -- inilah yang menghilangkan
        # efek pusing "seolah dilihat dari atas berputar-putar". Hanya roll
        # & pitch drone yang membuat badan/lengan terlihat miring relatif
        # terhadap kamera yang stabil, persis seperti kamera FPV/chase-cam
        # yang menempel di ekor drone. Rotasi (mv/rot_matrix) memakai
        # variabel wx,wy,wz yang SAMA persis seperti versi sebelumnya agar
        # arah kemiringan roll/pitch tetap konsisten (hanya proyeksinya
        # yang diupgrade dari 2D datar menjadi perspektif 3D).
        CAM_ELEV_DEG = 24.0
        sin_e = math.sin(CAM_ELEV_DEG * RAD)
        cos_e = math.cos(CAM_ELEV_DEG * RAD)
        cam_dist = arm_len * 3.4
        focal = arm_len * 4.4

        def proj(v):
            return self._chase_cam_project(v, sin_e, cos_e, cam_dist, focal)

        # Matriks rotasi: -roll agar arah bank (miring) kanan/kiri sinkron dengan
        # indikator busur derajat di atas & visual horizon.
        M = rot_matrix(-self.roll, self.pitch, 0.0)   # yaw sengaja diabaikan

        d = arm_len / math.sqrt(2)
        # Koordinat body standar: +X depan, +Y kanan, +Z atas
        arms_def = [
            ("FR", ( d,  d, 0), COL_ACCENT),
            ("FL", ( d, -d, 0), COL_ACCENT),
            ("BR", (-d,  d, 0), COL_MUTED),
            ("BL", (-d, -d, 0), COL_MUTED),
        ]

        rotated = []
        for name, v, color in arms_def:
            wv = mv(M, v)
            sx, sy, zc, k = proj(wv)
            rotated.append((name, wv, sx, sy, zc, k, color))

        # gambar yang paling jauh (zc besar) duluan agar yang dekat menimpa
        rotated.sort(key=lambda a: -a[4])

        self._prop_angle = (self._prop_angle + 30) % 360
        near_zc = cam_dist - arm_len
        far_zc = cam_dist + arm_len

        # ── lengan (arm), dari pusat badan (selalu di layar (0,0)) ke tiap
        # ujung motor; ketebalan & alpha mengikuti kedalaman (depth cue) ──
        for name, wv, sx, sy, zc, k, color in rotated:
            depth_t = 1.0 - (zc - near_zc) / max(1.0, (far_zc - near_zc))
            depth_t = max(0.0, min(1.0, depth_t))
            arm_col = QColor(COL_TEXT)
            arm_col.setAlpha(int(150 + depth_t * 95))
            thick = max(1.6, 3.2 * k / (focal / cam_dist))
            p.setPen(QPen(arm_col, thick, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(0, 0), QPointF(sx, sy))

        self._draw_body(p, M, proj, body_len, body_wid)

        # ── motor + baling-baling, sebagai elips 3D (bukan lingkaran datar)
        # dengan squash & rotasi mengikuti kemiringan piringan baling-baling
        # relatif terhadap arah pandang kamera ──
        cam_fwd = (cos_e, 0.0, -sin_e)
        for name, wv, sx, sy, zc, k, color in rotated:
            depth_t = 1.0 - (zc - near_zc) / max(1.0, (far_zc - near_zc))
            depth_t = max(0.0, min(1.0, depth_t))
            scale = k / (focal / cam_dist)

            m_grad = QRadialGradient(sx - 2, sy - 2, motor_r * scale * 1.6)
            m_grad.setColorAt(0.0, QColor("#FFFFFF"))
            m_grad.setColorAt(0.6, QColor("#CBD5E1"))
            m_grad.setColorAt(1.0, QColor("#64748B"))
            p.setPen(QPen(COL_TEXT, 1))
            p.setBrush(QBrush(m_grad))
            p.drawEllipse(QPointF(sx, sy), motor_r * scale, motor_r * scale)

            # normal piringan (sumbu Z lokal drone) & tangen horizontal,
            # dipakai untuk menghitung squash + rotasi elips baling-baling
            normal_w = mv(M, (0.0, 0.0, 1.0))
            facing = abs(normal_w[0]*cam_fwd[0] + normal_w[1]*cam_fwd[1]
                         + normal_w[2]*cam_fwd[2])
            squash = max(0.24, min(1.0, facing))

            eps = 0.06
            tang_w = mv(M, (wv[0]*0 + 1.0, 0.0, 0.0))  # arah lokal X drone
            p1 = proj((wv[0] + tang_w[0]*eps, wv[1] + tang_w[1]*eps,
                       wv[2] + tang_w[2]*eps))
            p2 = proj((wv[0] - tang_w[0]*eps, wv[1] - tang_w[1]*eps,
                       wv[2] - tang_w[2]*eps))
            tdx, tdy = p1[0] - p2[0], p1[1] - p2[1]
            ang = math.degrees(math.atan2(tdy, tdx)) if (abs(tdx) + abs(tdy)) > 1e-6 else 0.0

            p.save()
            p.translate(sx, sy)
            p.rotate(ang)

            r = prop_r * scale
            disc = QRadialGradient(0, 0, r)
            base = QColor(color)
            disc.setColorAt(0.0, QColor(base.red(), base.green(), base.blue(), 90))
            disc.setColorAt(0.55, QColor(base.red(), base.green(), base.blue(), 40))
            disc.setColorAt(1.0, QColor(base.red(), base.green(), base.blue(), 0))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(disc))
            p.drawEllipse(QPointF(0, 0), r, r * squash)

            dir_sign = 1 if name in ("FR", "BL") else -1
            p.save()
            p.rotate(self._prop_angle * dir_sign)
            blade_col = QColor(base)
            blade_col.setAlpha(int(150 * scale))
            p.setPen(QPen(blade_col, 1.4))
            p.drawLine(QPointF(-r*0.9, 0), QPointF(r*0.9, 0))
            p.drawLine(QPointF(0, -r*0.9*squash), QPointF(0, r*0.9*squash))
            p.restore()

            ring = QColor(base)
            ring.setAlpha(85)
            p.setPen(QPen(ring, 1.2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(0, 0), r, r * squash)
            p.restore()

        p.restore()

    def _draw_body(self, p, M, proj, body_len, body_wid):
        """Badan/fuselage digambar sebagai poligon 3D (bukan lingkaran datar
        + segitiga yaw seperti versi lama) sehingga ikut miring & mengecil
        secara perspektif mengikuti roll/pitch, konsisten dengan lengan."""
        nose   = mv(M, ( body_len * 1.35, 0.0,        0.0))
        rightw = mv(M, ( body_len * 0.10, body_wid,   0.0))
        tail   = mv(M, (-body_len * 1.05, 0.0,        0.0))
        leftw  = mv(M, ( body_len * 0.10, -body_wid,  0.0))
        canopy = mv(M, ( body_len * 0.25, 0.0,  body_wid * 0.8))

        pts_screen = []
        depths = []
        for v in (nose, rightw, tail, leftw):
            sx, sy, zc, k = proj(v)
            pts_screen.append(QPointF(sx, sy))
            depths.append(zc)
        csx, csy, czc, cscale = proj(canopy)

        avg_depth = sum(depths) / len(depths)
        near_ref = 1.15
        far_ref = 0.75
        depth_norm = max(0.0, min(1.0, (avg_depth - far_ref) / max(1e-6, near_ref - far_ref)))

        grad = QLinearGradient(pts_screen[3], pts_screen[1])
        grad.setColorAt(0.0, QColor("#FFFFFF"))
        grad.setColorAt(0.55, QColor("#E2E8F0"))
        grad.setColorAt(1.0, QColor("#94A3B8"))
        p.setPen(QPen(COL_ACCENT_DK, 1.3))
        p.setBrush(QBrush(grad))
        p.drawPolygon(QPolygonF(pts_screen))

        # penanda hidung (nose tip) supaya arah depan drone selalu jelas
        p.setPen(QPen(COL_ACCENT_DK, 1.0))
        p.setBrush(QBrush(COL_ACCENT))
        nose_tip = pts_screen[0]
        p.drawEllipse(nose_tip, 2.4, 2.4)

        # LED status (kanopi), sedikit di depan & di atas pusat badan
        led_r = max(3.0, body_len * 0.22 * cscale / (cscale if cscale else 1))
        led = QRadialGradient(csx, csy, body_len * 0.5)
        c = COL_OK if self._connected else COL_MUTED
        led.setColorAt(0.0, c)
        led.setColorAt(0.5, QColor(c.red(), c.green(), c.blue(), 160))
        led.setColorAt(1.0, QColor(c.red(), c.green(), c.blue(), 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(led))
        p.drawEllipse(QPointF(csx, csy), body_wid * 0.6, body_wid * 0.6)


# ───────────────────── Altitude Tape ─────────────────────

class AltitudeTapeWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(110)
        self.setMinimumHeight(300)
        self.altitude = 0.0
        self.vspeed = 0.0
        self._last_alt = 0.0
        self._last_t = time.monotonic()

    def set_altitude(self, alt):
        now = time.monotonic()
        dt = now - self._last_t
        if dt > 0.05:
            inst_vs = (alt - self._last_alt) / dt
            # IIR Filter untuk Vertical Speed (m/s) agar tidak melonjak akibat noise barometer
            self.vspeed = 0.75 * self.vspeed + 0.25 * inst_vs
            if abs(self.vspeed) < 0.05:
                self.vspeed = 0.0
            self._last_alt = alt
            self._last_t = now
        self.altitude = alt
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(0, 0, -1, -1)
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD))
        p.drawRoundedRect(rect, 12, 12)

        p.setPen(COL_SUBTEXT)
        p.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.2))
        p.drawText(QRectF(rect.left(), rect.top()+8, rect.width(), 14),
                   Qt.AlignCenter, "ALTITUDE \u00B7 M")

        tape = QRectF(rect.left()+8, rect.top()+28,
                      rect.width()-16, rect.height()-28-56)
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD_ALT))
        p.drawRoundedRect(tape, 6, 6)

        p.save()
        p.setClipRect(tape)

        cy = tape.center().y()
        px_per_m = 32.0
        alt = self.altitude

        p.setFont(qfont(FONT_UI, 8))
        m_start = int(math.floor(alt - 5))
        m_end = int(math.ceil(alt + 5))
        step = 1.0
        v = m_start
        while v <= m_end + 0.01:
            y = cy - (v - alt) * px_per_m
            if tape.top() <= y <= tape.bottom():
                major = abs(v - round(v)) < 0.01
                if major:
                    p.setPen(QPen(COL_TEXT, 1.4))
                    p.drawLine(int(tape.right()-14), int(y),
                               int(tape.right()-4), int(y))
                    p.setPen(COL_TEXT)
                    p.drawText(QRectF(tape.left()+2, y-7,
                                      tape.width()-20, 14),
                               Qt.AlignRight | Qt.AlignVCenter,
                               f"{int(round(v)):+d}")
                else:
                    p.setPen(QPen(COL_MUTED, 1))
                    p.drawLine(int(tape.right()-9), int(y),
                               int(tape.right()-4), int(y))
            v += step
        p.restore()

        box_w = tape.width() - 6
        box_h = 24
        box = QRectF(tape.left()+3, cy - box_h/2, box_w, box_h)
        p.setPen(QPen(COL_ACCENT_DK, 1.2))
        p.setBrush(QBrush(COL_ACCENT))
        p.drawRoundedRect(box, 4, 4)
        p.setPen(QColor("#FFFFFF"))
        p.setFont(qfont(FONT_MONO, 11, QFont.Bold))
        p.drawText(box, Qt.AlignCenter, f"{alt:+.2f}")

        # V/Speed box: own row with two clearly separated lines
        vs_rect = QRectF(rect.left()+6, rect.bottom()-46,
                         rect.width()-12, 38)
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD_ALT))
        p.drawRoundedRect(vs_rect, 6, 6)
        p.setPen(COL_SUBTEXT)
        p.setFont(qfont(FONT_UI, 7, QFont.Bold, letter_spacing=0.8))
        p.drawText(QRectF(vs_rect.left(), vs_rect.top()+3,
                          vs_rect.width(), 12),
                   Qt.AlignCenter, "V/SPEED M/S")
        p.setPen(COL_TEXT if abs(self.vspeed) < 3 else COL_WARN)
        p.setFont(qfont(FONT_MONO, 12, QFont.Bold))
        p.drawText(QRectF(vs_rect.left(), vs_rect.top()+16,
                          vs_rect.width(), 20),
                   Qt.AlignCenter, f"{self.vspeed:+.2f}")

        p.end()


# ───────────────────── Joystick Widget ─────────────────────

class JoystickWidget(QWidget):
    """Kartu gimbal joystick RC (Mode 2) dengan geometri presisi:
      1) Header: Judul stik (kiri) + Subtitle sumbu + Mode badge (kanan)
      2) Pad: Piringan gimbal simetris di tengah dengan penanda arah sumbu
         (▲/▼/◄/►) yang elegan, anti-tabrakan dengan box footer
      3) Footer: Dua pill readout telemetri (sumbu vertikal & horizontal)
    """

    def __init__(self, title="STICK", mode="MODE 2",
                 vx_label="X", vy_label="Y", parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 290)
        self._target_x = 0.5
        self._target_y = 0.5
        self._x = 0.5
        self._y = 0.5
        self._title = title
        self._mode = mode
        self._vx_label = vx_label
        self._vy_label = vy_label
        self._trail_points = []
        self._max_trail = 12

    def set_labels(self, vx, vy):
        self._vx_label = vx
        self._vy_label = vy

    def set_position(self, nx: float, ny: float):
        self._target_x = max(0.0, min(1.0, nx))
        self._target_y = max(0.0, min(1.0, ny))

    def animate(self):
        ease = 0.22
        old_x, old_y = self._x, self._y
        self._x += (self._target_x - self._x) * ease
        self._y += (self._target_y - self._y) * ease
        if abs(self._x - old_x) > 0.001 or abs(self._y - old_y) > 0.001:
            self._trail_points.append((self._x, self._y))
            if len(self._trail_points) > self._max_trail:
                self._trail_points.pop(0)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(0, 0, -1, -1)
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD))
        p.drawRoundedRect(rect, 12, 12)

        # ── Zone 1: Header Row ──
        header_top = rect.top() + 10
        header_h = 32.0
        header_rect = QRectF(rect.left() + 16, header_top,
                             rect.width() - 32, header_h)

        # Title & Subtitle
        p.setPen(COL_TEXT)
        p.setFont(qfont(FONT_UI, 10, QFont.Bold, letter_spacing=0.8))
        p.drawText(QRectF(header_rect.left(), header_rect.top(),
                          header_rect.width() - 80, 16),
                   Qt.AlignLeft | Qt.AlignVCenter, self._title.upper())

        p.setPen(COL_SUBTEXT)
        p.setFont(qfont(FONT_UI, 7, QFont.DemiBold, letter_spacing=1.0))
        p.drawText(QRectF(header_rect.left(), header_rect.top() + 16,
                          header_rect.width() - 80, 14),
                   Qt.AlignLeft | Qt.AlignVCenter,
                   f"{self._vy_label} \u00B7 {self._vx_label}")

        # Mode Badge
        badge_font = qfont(FONT_UI, 7, QFont.Bold, letter_spacing=1.2)
        p.setFont(badge_font)
        badge_txt = self._mode.upper()
        badge_w = QFontMetricsF(badge_font).horizontalAdvance(badge_txt) + 16
        badge_rect = QRectF(header_rect.right() - badge_w,
                            header_rect.top() + 4, badge_w, 20)
        p.setPen(QPen(QColor("#BAE6FD"), 1))
        p.setBrush(QBrush(QColor("#E0F2FE")))
        p.drawRoundedRect(badge_rect, 10, 10)
        p.setPen(QColor("#0369A1"))
        p.drawText(badge_rect, Qt.AlignCenter, badge_txt)

        # ── Zone 3: Footer Readout Pills (Reserved Bottom) ──
        footer_h = 34.0
        footer_top = rect.bottom() - 12 - footer_h
        val_x = int(self._x * 255)
        val_y = int((1.0 - self._y) * 255)
        pill_gap = 10.0
        pill_w = (rect.width() - 32 - pill_gap) / 2.0
        pill_left = QRectF(rect.left() + 16, footer_top, pill_w, footer_h)
        pill_right = QRectF(pill_left.right() + pill_gap, footer_top, pill_w, footer_h)

        # ── Zone 2: Gimbal Pad Geometry (Between Header and Footer) ──
        pad_top = header_top + header_h + 12
        pad_bottom = footer_top - 16
        pad_h = pad_bottom - pad_top
        pad_w = rect.width() - 32

        cx = rect.center().x()
        cy = (pad_top + pad_bottom) / 2.0

        # Radius proporsional dengan batas aman (tidak akan pernah menabrak header/footer)
        radius = min(pad_w * 0.38, pad_h * 0.38)
        radius = max(36.0, min(80.0, radius))

        # ── Axis Indicators Around the Circle (HUD Style) ──
        p.setFont(qfont(FONT_UI, 7, QFont.Bold, letter_spacing=1.0))
        p.setPen(COL_SUBTEXT)

        # Top Axis Label (e.g. ▲ THROTTLE / ▲ PITCH)
        top_lbl = f"\u25B2  {self._vy_label.upper()}"
        p.drawText(QRectF(cx - 80, cy - radius - 16, 160, 14),
                   Qt.AlignCenter, top_lbl)

        # Bottom Axis Label (e.g. ▼ MIN)
        p.setPen(COL_MUTED)
        p.drawText(QRectF(cx - 50, cy + radius + 3, 100, 14),
                   Qt.AlignCenter, "\u25BC  MIN")

        # Left / Right Axis Ticks (◄ / ►)
        p.drawText(QRectF(cx - radius - 18, cy - 7, 14, 14),
                   Qt.AlignCenter, "\u25C4")
        p.drawText(QRectF(cx + radius + 4, cy - 7, 14, 14),
                   Qt.AlignCenter, "\u25BA")

        # ── Gimbal Outer Halo ──
        halo = QRadialGradient(cx, cy, radius * 1.08)
        halo.setColorAt(0.0, QColor(COL_ACCENT_XLT.red(), COL_ACCENT_XLT.green(),
                                    COL_ACCENT_XLT.blue(), 0))
        halo.setColorAt(0.85, QColor(COL_ACCENT_XLT.red(), COL_ACCENT_XLT.green(),
                                     COL_ACCENT_XLT.blue(), 0))
        halo.setColorAt(1.0, QColor(COL_ACCENT_2.red(), COL_ACCENT_2.green(),
                                    COL_ACCENT_2.blue(), 35))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(halo))
        p.drawEllipse(QPointF(cx, cy), radius * 1.08, radius * 1.08)

        # ── Gimbal Disc ──
        disc = QRadialGradient(cx, cy - radius * 0.2, radius * 1.3)
        disc.setColorAt(0.0, QColor("#FFFFFF"))
        disc.setColorAt(0.75, QColor("#F8FAFC"))
        disc.setColorAt(1.0, QColor("#F1F5F9"))
        p.setPen(QPen(QColor("#CBD5E1"), 1.5))
        p.setBrush(QBrush(disc))
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        # ── Concentric Reference Rings (33%, 66%) ──
        for frac in (0.33, 0.66):
            p.setPen(QPen(QColor("#E2E8F0"), 1, Qt.DotLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), radius * frac, radius * frac)

        # ── Crosshair ──
        p.setPen(QPen(QColor("#E2E8F0"), 1))
        p.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        p.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))

        # ── Center Deadzone (Subtle sky blue) ──
        dead_r = radius * 0.08
        p.setPen(QPen(QColor("#BAE6FD"), 1))
        p.setBrush(QBrush(QColor(186, 230, 253, 60)))
        p.drawEllipse(QPointF(cx, cy), dead_r, dead_r)

        # ── Motion Trail ──
        n_trail = len(self._trail_points)
        for i, (tx, ty) in enumerate(self._trail_points):
            t_alpha = int(25 + 90 * (i / max(n_trail, 1)))
            tx_px = cx + (tx - 0.5) * 2 * radius
            ty_px = cy + (ty - 0.5) * 2 * radius
            trail_r = 2.0 + 2.0 * (i / max(n_trail, 1))
            tc = QColor(COL_ACCENT_LT)
            tc.setAlpha(t_alpha)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(tc))
            p.drawEllipse(QPointF(tx_px, ty_px), trail_r, trail_r)

        # ── Stick Knob Coordinates ──
        # Batasi posisi knob agar tidak keluar dari piringan
        dx = (self._x - 0.5) * 2 * radius
        dy = (self._y - 0.5) * 2 * radius
        dist = math.sqrt(dx * dx + dy * dy)
        max_dist = radius - 3.0
        if dist > max_dist and dist > 0:
            scale = max_dist / dist
            dx *= scale
            dy *= scale
        sx = cx + dx
        sy = cy + dy

        # ── Connector Line: Center → Knob ──
        p.setPen(QPen(COL_ACCENT, 2.0, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(cx, cy), QPointF(sx, sy))

        # ── Knob Ambient Glow ──
        knob_r = max(7.5, radius * 0.13)
        glow_col = QColor(COL_ACCENT)
        glow_col.setAlpha(30)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(glow_col))
        p.drawEllipse(QPointF(sx, sy), knob_r * 2.0, knob_r * 2.0)

        # ── Knob Main Body ──
        knob_grad = QRadialGradient(sx - knob_r * 0.3, sy - knob_r * 0.35, knob_r * 1.3)
        knob_grad.setColorAt(0.0, QColor("#60A5FA"))
        knob_grad.setColorAt(0.5, COL_ACCENT)
        knob_grad.setColorAt(1.0, COL_ACCENT_DK)
        p.setPen(QPen(QColor("#FFFFFF"), 1.5))
        p.setBrush(QBrush(knob_grad))
        p.drawEllipse(QPointF(sx, sy), knob_r, knob_r)

        # ── Knob Center Pip (Precision Dot) ──
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor("#FFFFFF")))
        p.drawEllipse(QPointF(sx, sy), 2.2, 2.2)

        # ── Zone 3: Footer Readout Pills Drawing ──
        pill_defs = [
            (pill_left,  self._vy_label, val_y, COL_ACCENT),
            (pill_right, self._vx_label, val_x, COL_ACCENT_DK),
        ]

        for pill, lbl, val, col in pill_defs:
            p.setPen(QPen(QColor("#E2E8F0"), 1))
            p.setBrush(QBrush(QColor("#F8FAFC")))
            p.drawRoundedRect(pill, 8, 8)

            # Indicator Dot
            dot_x = pill.left() + 12
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(col))
            p.drawEllipse(QPointF(dot_x, pill.center().y()), 3.5, 3.5)

            # Axis Name Label
            p.setPen(COL_SUBTEXT)
            p.setFont(qfont(FONT_UI, 7.5, QFont.Bold, letter_spacing=1.0))
            p.drawText(QRectF(dot_x + 8, pill.top() + 4, pill.width() - 55, pill.height() - 8),
                       Qt.AlignLeft | Qt.AlignVCenter, lbl.upper())

            # Numerical Value
            p.setPen(COL_TEXT)
            p.setFont(qfont(FONT_MONO, 13, QFont.Bold))
            p.drawText(QRectF(pill.right() - 48, pill.top() + 4, 40, pill.height() - 8),
                       Qt.AlignRight | Qt.AlignVCenter, f"{val:>3}")

        p.end()


# ───────────────────── SMC Tuning Plot ─────────────────────

class SmcPlotWidget(QWidget):
    """Scrolling attitude and SMC-output traces from the flight controller."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(440, 390)
        self._samples = deque(maxlen=180)
        self._roll = self._pitch = 0.0
        self._ur = self._up = self._uy = 0.0

    def add_sample(self, roll, pitch, u_roll, u_pitch, u_yaw):
        self._roll, self._pitch = roll, pitch
        self._ur, self._up, self._uy = u_roll, u_pitch, u_yaw
        self._samples.append((time.monotonic(), roll, pitch, u_roll, u_pitch, u_yaw))
        self.update()

    def _status(self):
        if len(self._samples) < 20:
            return "WAITING FOR DATA", COL_MUTED
        recent = list(self._samples)[-20:]
        early = list(self._samples)[-40:-20]
        recent_amp = max(max(abs(s[1]), abs(s[2])) for s in recent)
        if not early:
            return "MONITORING", COL_WARN
        early_amp = max(max(abs(s[1]), abs(s[2])) for s in early)
        if recent_amp > max(4.0, early_amp * 1.35):
            return "DIVERGING - CHECK SIGNS", COL_ERR
        if recent_amp < max(1.0, early_amp * 0.70):
            return "STABLE / DAMPING", COL_OK
        return "OSCILLATING / HOLD", COL_WARN

    def _draw_trace(self, p, rect, values, color, scale):
        if len(self._samples) < 2:
            return
        start = self._samples[0][0]
        end = self._samples[-1][0]
        span = max(1.0, end - start)
        points = []
        for sample, value in zip(self._samples, values):
            x = rect.left() + (sample[0] - start) / span * rect.width()
            y = rect.center().y() - (value / scale) * (rect.height() * 0.42)
            points.append(QPointF(x, y))
        p.setPen(QPen(color, 1.8))
        for i in range(1, len(points)):
            p.drawLine(points[i - 1], points[i])

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect().adjusted(0, 0, -1, -1))
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD))
        p.drawRoundedRect(rect, 12, 12)

        title = QRectF(rect.left() + 16, rect.top() + 10, rect.width() - 32, 20)
        status, status_col = self._status()
        p.setPen(COL_TEXT)
        p.setFont(qfont(FONT_UI, 9, QFont.Bold, letter_spacing=1.5))
        p.drawText(title, Qt.AlignLeft | Qt.AlignVCenter, "LIVE SMC RESPONSE")
        p.setPen(status_col)
        p.drawText(title, Qt.AlignRight | Qt.AlignVCenter, status)

        top = QRectF(rect.left() + 16, rect.top() + 42, rect.width() - 32, (rect.height() - 90) * 0.48)
        bottom = QRectF(rect.left() + 16, top.bottom() + 20, rect.width() - 32, (rect.height() - 90) * 0.48)
        for plot, label, scale in ((top, "ATTITUDE (deg)", 30.0), (bottom, "SMC OUTPUT (us)", 180.0)):
            p.setPen(QPen(COL_BORDER, 1))
            p.setBrush(QBrush(COL_CARD_ALT))
            p.drawRoundedRect(plot, 6, 6)
            p.setPen(QPen(COL_DIVIDER, 1, Qt.DashLine))
            p.drawLine(QPointF(plot.left(), plot.center().y()), QPointF(plot.right(), plot.center().y()))
            p.setPen(COL_SUBTEXT)
            p.setFont(qfont(FONT_UI, 7, QFont.Bold, letter_spacing=1.0))
            p.drawText(QRectF(plot.left() + 6, plot.top() + 4, plot.width() - 12, 13), Qt.AlignLeft, label)
            p.drawText(QRectF(plot.right() - 36, plot.center().y() - 7, 30, 14), Qt.AlignRight, "0")

        samples = list(self._samples)
        self._draw_trace(p, top, [s[1] for s in samples], COL_ACCENT, 30.0)
        self._draw_trace(p, top, [s[2] for s in samples], COL_OK, 30.0)
        self._draw_trace(p, bottom, [s[3] for s in samples], COL_ACCENT, 180.0)
        self._draw_trace(p, bottom, [s[4] for s in samples], COL_OK, 180.0)
        self._draw_trace(p, bottom, [s[5] for s in samples], COL_WARN, 180.0)

        p.setFont(qfont(FONT_MONO, 8, QFont.Bold))
        p.setPen(COL_ACCENT)
        p.drawText(QRectF(top.left(), top.bottom() + 2, 95, 14), Qt.AlignLeft, f"ROLL {self._roll:+.2f}")
        p.setPen(COL_OK)
        p.drawText(QRectF(top.left() + 105, top.bottom() + 2, 100, 14), Qt.AlignLeft, f"PITCH {self._pitch:+.2f}")
        p.setPen(COL_ACCENT)
        p.drawText(QRectF(bottom.left(), bottom.bottom() + 2, 85, 14), Qt.AlignLeft, f"UR {self._ur:+.1f}")
        p.setPen(COL_OK)
        p.drawText(QRectF(bottom.left() + 90, bottom.bottom() + 2, 85, 14), Qt.AlignLeft, f"UP {self._up:+.1f}")
        p.setPen(COL_WARN)
        p.drawText(QRectF(bottom.left() + 180, bottom.bottom() + 2, 85, 14), Qt.AlignLeft, f"UY {self._uy:+.1f}")
        p.end()


# ───────────────────── Metric Card ─────────────────────

class MetricCard(QFrame):
    """Kartu metrik dengan 3 zona vertikal terpisah (label / nilai / satuan)
    yang dihitung memakai QRectF + alignment Qt, bukan baseline manual, agar
    tidak pernah tumpang tindih pada resolusi/DPI apa pun."""

    def __init__(self, label, unit, accent, parent=None):
        super().__init__(parent)
        self.setObjectName("MetricCard")
        self._label = label
        self._unit = unit
        self._accent = accent
        self._value = 0.0
        self._fmt = "{:+.1f}"
        self.setMinimumHeight(96)
        self.setStyleSheet(f"""
            #MetricCard {{
                background: {COL_CARD.name()};
                border: 1px solid {COL_BORDER.name()};
                border-left: 4px solid {accent.name()};
                border-radius: 10px;
            }}
        """)
        add_shadow(self, blur=14, dy=1, alpha=18)

    def set_format(self, fmt):
        self._fmt = fmt

    def set_value(self, v):
        self._value = v
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = self.rect().adjusted(16, 0, -14, 0)

        # Zone 1: accent dot + label (top strip)
        label_h = 22.0
        label_rect = QRectF(r.left(), r.top(), r.width(), label_h)
        dot_r = 3.0
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(self._accent))
        p.drawEllipse(QPointF(label_rect.left() + dot_r, label_rect.center().y()), dot_r, dot_r)
        p.setPen(COL_SUBTEXT)
        p.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.4))
        p.drawText(label_rect.adjusted(dot_r * 2 + 6, 0, 0, 0),
                   Qt.AlignLeft | Qt.AlignVCenter, self._label.upper())

        # Zone 2: big value (middle, generous height so glyphs never clip)
        unit_h = 20.0
        value_rect = QRectF(r.left(), r.top() + label_h,
                            r.width(), r.height() - label_h - unit_h)
        p.setPen(COL_TEXT)
        p.setFont(qfont(FONT_TITLE, 21, QFont.Bold, letter_spacing=-0.3))
        val_text = self._fmt.format(self._value)
        p.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, val_text)

        # Zone 3: unit (bottom strip, separated by a hairline)
        unit_rect = QRectF(r.left(), r.bottom() - unit_h, r.width(), unit_h)
        p.setPen(QPen(COL_BORDER, 1))
        p.drawLine(QPointF(unit_rect.left(), unit_rect.top()),
                   QPointF(unit_rect.right(), unit_rect.top()))
        p.setPen(COL_MUTED)
        p.setFont(qfont(FONT_UI, 7, QFont.DemiBold, letter_spacing=1.2))
        p.drawText(unit_rect.adjusted(0, 1, 0, 0),
                   Qt.AlignLeft | Qt.AlignVCenter, self._unit.upper())

        p.end()


# ───────────────────── Secondary Info Bar ─────────────────────

class SecondaryInfoBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("InfoBar")
        self.setFixedHeight(52)
        self.setStyleSheet(f"""
            #InfoBar {{
                background: {COL_CARD.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 10px;
            }}
        """)
        self.press = 0.0
        self.rate = 0.0
        self._last_sample_ts = 0.0

    def update_press(self, press):
        self.press = press
        self.update()

    def note_sample(self):
        now = time.monotonic()
        if self._last_sample_ts > 0:
            dt = now - self._last_sample_ts
            if dt > 0:
                inst_hz = 1.0 / dt
                self.rate = self.rate * 0.85 + inst_hz * 0.15
        self._last_sample_ts = now
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect())

        cols = [
            ("PRESSURE",  f"{self.press:.1f} hPa", COL_ACCENT_2),
            ("DATA RATE", f"{self.rate:.0f} Hz",   COL_OK),
        ]
        col_w = rect.width() / len(cols)
        for i, (lbl, val, col) in enumerate(cols):
            x = i * col_w
            if i > 0:
                p.setPen(QPen(COL_BORDER, 1))
                p.drawLine(QPointF(x, rect.top() + 10), QPointF(x, rect.bottom() - 10))

            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(col))
            p.drawEllipse(QPointF(x + 20, rect.center().y()), 4, 4)

            text_rect = QRectF(x + 32, rect.top(), col_w - 40, rect.height())
            label_rect = QRectF(text_rect.left(), rect.top() + 8, text_rect.width(), 14)
            value_rect = QRectF(text_rect.left(), rect.top() + 24, text_rect.width(), 20)

            p.setPen(COL_SUBTEXT)
            p.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.2))
            p.drawText(label_rect, Qt.AlignLeft | Qt.AlignVCenter, lbl)
            p.setPen(COL_TEXT)
            p.setFont(qfont(FONT_UI, 12, QFont.DemiBold))
            p.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, val)

        p.end()


# ───────────────────── Readout Card (RC values) ─────────────────────

class RCReadoutCard(QFrame):
    """Strip telemetri 4 channel RC dengan zona yang benar-benar terpisah:
    header (judul + badge kalibrasi) di atas, lalu 4 kolom channel yang
    masing-masing punya nama channel, nilai numerik, dan mini deflection-bar
    sendiri sehingga tidak ada teks yang bertumpukan."""

    CHANNELS = [
        ("R", "ROLL", COL_ACCENT),
        ("T", "THROTTLE", COL_ACCENT_LT),
        ("Y", "YAW", COL_ACCENT_DK),
        ("P", "PITCH", COL_OK),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("RCReadout")
        self.setFixedHeight(108)
        self.setStyleSheet(f"""
            #RCReadout {{
                background: {COL_CARD.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 10px;
            }}
        """)
        self.r = 128
        self.t = 128
        self.y = 128
        self.p = 128
        self.calibrated = False

    def set_values(self, r, t, y, p, calibrated):
        self.r, self.t, self.y, self.p = r, t, y, p
        self.calibrated = calibrated
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect())

        # ── header row ──
        header_rect = QRectF(rect.left() + 16, rect.top() + 8,
                             rect.width() - 32, 18)
        p.setPen(COL_SUBTEXT)
        p.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.6))
        p.drawText(header_rect, Qt.AlignLeft | Qt.AlignVCenter,
                   "RC CHANNELS  \u00B7  RANGE 0\u2013255")

        badge_txt = "CALIBRATED" if self.calibrated else "RAW DATA"
        badge_bg = COL_OK if self.calibrated else COL_MUTED
        badge_font = qfont(FONT_UI, 7, QFont.Bold, letter_spacing=1.6)
        badge_w = QFontMetricsF(badge_font).horizontalAdvance(badge_txt) + 20
        badge_rect = QRectF(header_rect.right() - badge_w,
                            header_rect.top() - 1, badge_w, 18)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(badge_bg))
        p.drawRoundedRect(badge_rect, 9, 9)
        p.setPen(QColor("#FFFFFF"))
        p.setFont(badge_font)
        p.drawText(badge_rect, Qt.AlignCenter, badge_txt)

        # hairline separating header from channel grid
        sep_y = header_rect.bottom() + 8
        p.setPen(QPen(COL_BORDER, 1))
        p.drawLine(QPointF(rect.left() + 16, sep_y), QPointF(rect.right() - 16, sep_y))

        # ── 4 channel columns ──
        values = [self.r, self.t, self.y, self.p]
        grid = QRectF(rect.left() + 16, sep_y + 8,
                      rect.width() - 32, rect.bottom() - (sep_y + 8) - 10)
        col_w = grid.width() / 4.0

        for i, ((code, name, col), val) in enumerate(zip(self.CHANNELS, values)):
            cx0 = grid.left() + i * col_w
            col_rect = QRectF(cx0, grid.top(), col_w - 10, grid.height())

            if i > 0:
                p.setPen(QPen(COL_BORDER, 1))
                p.drawLine(QPointF(cx0 - 5, grid.top()), QPointF(cx0 - 5, grid.bottom()))

            # name row: colored dot + "R · ROLL"
            name_rect = QRectF(col_rect.left(), col_rect.top(), col_rect.width(), 16)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(col))
            p.drawEllipse(QPointF(name_rect.left() + 3, name_rect.center().y()), 3, 3)
            p.setPen(COL_SUBTEXT)
            p.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.2))
            p.drawText(name_rect.adjusted(11, 0, 0, 0),
                       Qt.AlignLeft | Qt.AlignVCenter, f"{code} \u00B7 {name}")

            # value row
            value_rect = QRectF(col_rect.left(), name_rect.bottom() + 2,
                                col_rect.width(), 22)
            p.setPen(COL_TEXT)
            p.setFont(qfont(FONT_MONO, 15, QFont.Bold))
            p.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, f"{val:>3}")

            # deflection bar row (0-255, center mark at 128)
            bar_rect = QRectF(col_rect.left(), value_rect.bottom() + 4,
                              col_rect.width(), 6)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(COL_CARD_ALT))
            p.drawRoundedRect(bar_rect, 3, 3)

            frac = max(0.0, min(1.0, val / 255.0))
            fill_w = bar_rect.width() * frac
            fill_col = QColor(col)
            p.setBrush(QBrush(fill_col))
            p.drawRoundedRect(QRectF(bar_rect.left(), bar_rect.top(),
                                     max(4.0, fill_w), bar_rect.height()), 3, 3)

            center_x = bar_rect.left() + bar_rect.width() * 0.5
            p.setPen(QPen(COL_SUBTEXT, 1.4))
            p.drawLine(QPointF(center_x, bar_rect.top() - 2),
                      QPointF(center_x, bar_rect.bottom() + 2))

        p.end()


# ───────────────────── Status Banner (Stick Calibration) ─────────────────────

class StatusBanner(QLabel):
    """Small banner used for stick calibration hints/results."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(28)
        self.setAlignment(Qt.AlignCenter)
        self.hide()

    def show_info(self, text):
        self._apply(text, "#1E40AF", "#DBEAFE", "#93C5FD")

    def show_ok(self, text):
        self._apply(text, "#166534", "#DCFCE7", "#86EFAC")

    def show_warn(self, text):
        self._apply(text, "#92400E", "#FEF3C7", "#FCD34D")

    def show_err(self, text):
        self._apply(text, "#991B1B", "#FEE2E2", "#FCA5A5")

    def _apply(self, text, fg, bg, border):
        self.setText(text)
        self.setStyleSheet(
            f"color:{fg}; background:{bg}; border:1px solid {border};"
            f"border-radius:8px; font: bold 10px Inter, 'Segoe UI Variable', 'Segoe UI', 'SF Pro Display', system-ui, sans-serif; letter-spacing:1px;"
        )
        self.show()


# ───────────────────── Bench Test (Props-Off) Widgets ─────────────────────

class BenchMotorMixWidget(QWidget):
    """Diagram Quad-X tampak atas interaktif + visualisasi koreksi tenaga
    (thrust delta) tiap motor secara relatif. Bar hijau naik = motor menambah
    daya, bar merah turun = motor mengurangi daya. Formula mixer sinkron
    dengan writeSmcMotorMix() di main.ino."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(380, 360)
        self.roll = self.pitch = self.yaw = 0.0
        self.ur = self.up = self.uy = 0.0
        self.throttle = 0.0
        self.armed = False
        self.mix = {"M1": 0.0, "M2": 0.0, "M3": 0.0, "M4": 0.0}
        self._mix_smooth = {"M1": 0.0, "M2": 0.0, "M3": 0.0, "M4": 0.0}
        self._prop_angle = 0.0

        # Posisi relatif 4 motor Quad-X (tampak atas: +X kanan, +Y bawah)
        self._motors_meta = {
            "M1": {"pos": "FL", "pin": "PB6", "cw": True,  "dx": -1.0, "dy": -1.0, "bar_side": -1},
            "M2": {"pos": "FR", "pin": "PB7", "cw": False, "dx":  1.0, "dy": -1.0, "bar_side":  1},
            "M3": {"pos": "BR", "pin": "PB8", "cw": True,  "dx":  1.0, "dy":  1.0, "bar_side":  1},
            "M4": {"pos": "BL", "pin": "PB9", "cw": False, "dx": -1.0, "dy":  1.0, "bar_side": -1},
        }

    def set_data(self, roll, pitch, yaw, ur, up, uy, throttle, armed):
        self.roll, self.pitch, self.yaw = roll, pitch, yaw
        self.ur, self.up, self.uy = ur, up, uy
        self.throttle = throttle
        self.armed = armed
        # Mixer Quad-X (sinkron dengan writeSmcMotorMix di main.ino)
        self.mix = {
            "M1":  ur + up + uy,   # FL CW
            "M2": -ur + up - uy,   # FR CCW
            "M3": -ur - up + uy,   # BR CW
            "M4":  ur - up - uy,   # BL CCW
        }

    def animate(self):
        # Smooth easing untuk gerakan bar koreksi & rotasi baling-baling
        ease = 0.22
        for k in ("M1", "M2", "M3", "M4"):
            target = self.mix.get(k, 0.0)
            self._mix_smooth[k] += (target - self._mix_smooth[k]) * ease
        self._prop_angle = (self._prop_angle + 12.0) % 360.0
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(0, 0, -1, -1)

        # ── Kartu Utama ──
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD))
        p.drawRoundedRect(rect, 14, 14)

        # ── Zona 1: Header (36 px) ──
        header_rect = QRectF(rect.left() + 16, rect.top() + 10, rect.width() - 32, 26)
        p.setPen(COL_TEXT)
        p.setFont(qfont(FONT_UI, 10, QFont.Bold, letter_spacing=0.8))
        p.drawText(header_rect, Qt.AlignLeft | Qt.AlignVCenter, "QUAD-X MOTOR MIXING")

        # Subtitle / badge mode kanan
        state_txt = "ARMED" if self.armed else "DISARMED"
        bg_col = QColor("#DCFCE7") if self.armed else QColor("#F1F5F9")
        fg_col = QColor("#166534") if self.armed else QColor("#64748B")
        bf = qfont(FONT_UI, 7, QFont.Bold, letter_spacing=1.2)
        p.setFont(bf)
        bw = QFontMetricsF(bf).horizontalAdvance(state_txt) + 16
        pill_rect = QRectF(header_rect.right() - bw, header_rect.center().y() - 10, bw, 20)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(bg_col))
        p.drawRoundedRect(pill_rect, 10, 10)
        p.setPen(fg_col)
        p.drawText(pill_rect, Qt.AlignCenter, state_txt)

        # ── Zona 3 (Disediakan dahulu): Telemetri Kapsul Bawah (42 px) ──
        bottom_h = 40.0
        bottom_top = rect.bottom() - 12 - bottom_h
        telemetry_items = [
            ("THR", f"{self.throttle:.0f} us", COL_TEXT),
            ("uROLL", f"{self.ur:+.1f}", COL_OK if self.ur > 0 else (COL_ERR if self.ur < 0 else COL_TEXT)),
            ("uPITCH", f"{self.up:+.1f}", COL_OK if self.up < 0 else (COL_ERR if self.up > 0 else COL_TEXT)),
            ("uYAW", f"{self.uy:+.1f}", COL_WARN if abs(self.uy) > 1 else COL_TEXT),
        ]
        num_items = len(telemetry_items)
        gap = 8.0
        total_w = rect.width() - 32
        item_w = (total_w - (num_items - 1) * gap) / num_items

        for idx, (label, val_str, col) in enumerate(telemetry_items):
            ix = rect.left() + 16 + idx * (item_w + gap)
            item_rect = QRectF(ix, bottom_top, item_w, bottom_h)
            p.setPen(QPen(COL_BORDER, 1))
            p.setBrush(QBrush(COL_CARD_ALT))
            p.drawRoundedRect(item_rect, 7, 7)

            p.setPen(COL_MUTED)
            p.setFont(qfont(FONT_UI, 6, QFont.Bold, letter_spacing=1.0))
            p.drawText(QRectF(item_rect.left(), item_rect.top() + 4, item_rect.width(), 12),
                       Qt.AlignCenter, label)

            p.setPen(col)
            p.setFont(qfont(FONT_MONO, 8, QFont.Bold))
            p.drawText(QRectF(item_rect.left(), item_rect.top() + 18, item_rect.width(), 16),
                       Qt.AlignCenter, val_str)

        # ── Zona 2: Area Visual Drone 2D (Antara Header dan Footer) ──
        panel = QRectF(rect.left() + 14, header_rect.bottom() + 6,
                       rect.width() - 28, bottom_top - header_rect.bottom() - 12)
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD_ALT))
        p.drawRoundedRect(panel, 10, 10)

        cx = panel.center().x()
        cy = panel.center().y()

        # Penanda Arah Depan (FRONT ▲)
        fwd_box = QRectF(cx - 36, panel.top() + 6, 72, 18)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(COL_ACCENT_XLT))
        p.drawRoundedRect(fwd_box, 9, 9)
        p.setPen(COL_ACCENT_DK)
        p.setFont(qfont(FONT_UI, 7, QFont.Bold, letter_spacing=1.2))
        p.drawText(fwd_box, Qt.AlignCenter, "\u25B2  FRONT")

        # ── Geometri Quad-X proporsional (Safe bounds agar baling-baling tidak keluar frame) ──
        half_w = panel.width() * 0.5 - 18
        half_h = panel.height() * 0.5 - 22
        max_arm_w = (half_w - 30) / 1.42
        max_arm_h = (half_h - 26) / 1.35
        arm_d = max(28.0, min(max_arm_w, max_arm_h, 72.0))

        arm_dx = arm_d * 1.05
        arm_dy = arm_d * 0.85
        prop_r = arm_d * 0.38
        motor_r = max(9.0, prop_r * 0.42)
        body_r = arm_d * 0.32

        # 1. Lengan Karbon X (Arm)
        p.setPen(QPen(QColor("#334155"), 6, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(cx - arm_dx, cy - arm_dy), QPointF(cx + arm_dx, cy + arm_dy))
        p.drawLine(QPointF(cx + arm_dx, cy - arm_dy), QPointF(cx - arm_dx, cy + arm_dy))

        p.setPen(QPen(QColor("#64748B"), 2.5, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(cx - arm_dx, cy - arm_dy), QPointF(cx + arm_dx, cy + arm_dy))
        p.drawLine(QPointF(cx + arm_dx, cy - arm_dy), QPointF(cx - arm_dx, cy + arm_dy))

        # 2. Bodi Pusat Drone (Fuselage / FC Hub)
        body_rect = QRectF(cx - body_r, cy - body_r, body_r * 2, body_r * 2)
        body_grad = QLinearGradient(body_rect.topLeft(), body_rect.bottomRight())
        body_grad.setColorAt(0.0, QColor("#1E293B"))
        body_grad.setColorAt(1.0, QColor("#0F172A"))
        p.setPen(QPen(COL_BORDER, 1.2))
        p.setBrush(QBrush(body_grad))
        p.drawRoundedRect(body_rect, 7, 7)

        # LED status FC di tengah
        led_col = COL_OK if self.armed else COL_ACCENT
        led_grad = QRadialGradient(cx, cy, body_r * 0.32)
        led_grad.setColorAt(0.0, QColor("#FFFFFF"))
        led_grad.setColorAt(0.5, led_col)
        led_grad.setColorAt(1.0, QColor(led_col.red(), led_col.green(), led_col.blue(), 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(led_grad))
        p.drawEllipse(QPointF(cx, cy), body_r * 0.32, body_r * 0.32)

        # Label sudut roll/pitch mini di bodi
        p.setPen(QColor("#E2E8F0"))
        p.setFont(qfont(FONT_MONO, 6, QFont.Bold))
        p.drawText(QRectF(cx - body_r, cy + body_r * 0.22, body_r * 2, 10),
                   Qt.AlignCenter, f"{self.roll:+.0f}\u00B0/{self.pitch:+.0f}\u00B0")

        # 3. Empat Rumah Motor & Indikator Koreksi Tenaga (Thrust Delta Bar)
        max_delta = max([abs(v) for v in self.mix.values()] or [1.0])
        max_delta = max(max_delta, 10.0)  # Skala visual minimal 10 unit

        for name, meta in self._motors_meta.items():
            mx = cx + meta["dx"] * arm_dx
            my = cy + meta["dy"] * arm_dy
            cw = meta["cw"]
            val_raw = self.mix[name]
            val_smooth = self._mix_smooth[name]

            # Baling-baling berputar (Propeller translucent disc)
            p.save()
            p.translate(mx, my)

            disc_col = QColor(COL_ACCENT) if cw else QColor(COL_ACCENT_LT)
            p_grad = QRadialGradient(0, 0, prop_r)
            p_grad.setColorAt(0.0, QColor(disc_col.red(), disc_col.green(), disc_col.blue(), 55))
            p_grad.setColorAt(0.7, QColor(disc_col.red(), disc_col.green(), disc_col.blue(), 20))
            p_grad.setColorAt(1.0, QColor(disc_col.red(), disc_col.green(), disc_col.blue(), 0))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(p_grad))
            p.drawEllipse(QPointF(0, 0), prop_r, prop_r)

            # Garis bilah baling-baling berputar
            dir_mult = 1.0 if cw else -1.0
            p.rotate(self._prop_angle * dir_mult)
            p.setPen(QPen(QColor(disc_col.red(), disc_col.green(), disc_col.blue(), 130), 1.4, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(-prop_r * 0.85, 0), QPointF(prop_r * 0.85, 0))
            p.drawLine(QPointF(0, -prop_r * 0.85), QPointF(0, prop_r * 0.85))
            p.restore()

            # Rumah Motor (Motor Hub)
            m_grad = QRadialGradient(mx - 1.5, my - 1.5, motor_r * 1.2)
            m_grad.setColorAt(0.0, QColor("#FFFFFF"))
            m_grad.setColorAt(0.4, QColor("#CBD5E1"))
            m_grad.setColorAt(1.0, QColor("#475569"))
            p.setPen(QPen(COL_TEXT, 1.0))
            p.setBrush(QBrush(m_grad))
            p.drawEllipse(QPointF(mx, my), motor_r, motor_r)

            # Label Motor (M1, M2, M3, M4)
            p.setPen(COL_TEXT)
            p.setFont(qfont(FONT_UI, 6, QFont.Bold))
            p.drawText(QRectF(mx - motor_r, my - motor_r, motor_r * 2, motor_r * 2),
                       Qt.AlignCenter, name)

            # Badge pin & posisi di bawah/atas motor
            meta_y = my + motor_r + 2 if meta["dy"] > 0 else my - motor_r - 13
            meta_rect = QRectF(mx - 30, meta_y, 60, 11)
            p.setPen(COL_SUBTEXT)
            p.setFont(qfont(FONT_UI, 5, QFont.DemiBold))
            p.drawText(meta_rect, Qt.AlignCenter, f"{meta['pos']} · {meta['pin']}")

            # Badge CW / CCW
            rot_str = "CW \u21B7" if cw else "CCW \u21B6"
            rot_y = meta_y + 9 if meta["dy"] > 0 else meta_y - 9
            p.setPen(COL_ACCENT_DK if cw else COL_WARN)
            p.setFont(qfont(FONT_UI, 5, QFont.Bold))
            p.drawText(QRectF(mx - 30, rot_y, 60, 10), Qt.AlignCenter, rot_str)

            # ── Bar Koreksi Tenaga (Thrust Delta Bar) ──
            bar_w = 6.0
            bar_h_max = prop_r * 0.75
            bar_x = mx + meta["bar_side"] * (prop_r + 3)
            if meta["bar_side"] < 0:
                bar_x -= bar_w
            bar_y_center = my

            # Slot Background
            slot_rect = QRectF(bar_x, bar_y_center - bar_h_max, bar_w, bar_h_max * 2)
            p.setPen(QPen(COL_BORDER, 1))
            p.setBrush(QBrush(QColor("#FFFFFF")))
            p.drawRoundedRect(slot_rect, 2, 2)

            # Garis Nol Tengah
            p.setPen(QPen(COL_DIVIDER, 1))
            p.drawLine(int(bar_x), int(bar_y_center), int(bar_x + bar_w), int(bar_y_center))

            # Isi Bar (+Δ hijau ke atas, -Δ merah ke bawah)
            frac = max(-1.0, min(1.0, val_smooth / max_delta))
            if frac >= 0:
                fill_h = frac * bar_h_max
                fill_rect = QRectF(bar_x + 1, bar_y_center - fill_h, bar_w - 2, fill_h)
                fill_col = COL_OK
            else:
                fill_h = abs(frac) * bar_h_max
                fill_rect = QRectF(bar_x + 1, bar_y_center, bar_w - 2, fill_h)
                fill_col = COL_ERR

            if fill_h > 0.5:
                p.setPen(Qt.NoPen)
                p.setBrush(QBrush(fill_col))
                p.drawRoundedRect(fill_rect, 1.5, 1.5)

            # Angka nilai delta
            p.setPen(COL_OK if val_raw > 0 else (COL_ERR if val_raw < 0 else COL_MUTED))
            p.setFont(qfont(FONT_MONO, 5, QFont.Bold))
            val_rect = QRectF(bar_x - 12, bar_y_center - bar_h_max - 10, bar_w + 24, 9)
            p.drawText(val_rect, Qt.AlignCenter, f"{val_raw:+.0f}")

        p.end()


class BenchCheckCard(QFrame):
    """Kartu status verifikasi satu sumbu kontrol (props-off).
    Menggunakan struktur Qt Layout dinamis sehingga teks membungkus (word-wrap) rapi,
    badge status tampil jelas, dan kapsul telemetri tidak pernah saling menindih."""
    def __init__(self, step_no: str, title: str, instruction: str, parent=None):
        super().__init__(parent)
        self.setObjectName("BenchCheckCard")
        self._step_no = step_no
        self._title = title
        self._instruction = instruction
        self._status = "IDLE"

        self.setStyleSheet(f"""
            #BenchCheckCard {{
                background: {COL_CARD.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 12px;
            }}
        """)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(5)

        # ── Row 1: Header (Step + Title kiri, Badge kanan) ──
        h_row = QHBoxLayout()
        h_row.setContentsMargins(0, 0, 0, 0)

        self._title_lbl = QLabel(f"{step_no}  \u00B7  {title.upper()}")
        self._title_lbl.setFont(qfont(FONT_UI, 9, QFont.Bold, letter_spacing=1.0))
        self._title_lbl.setStyleSheet(f"color: {COL_TEXT.name()};")
        h_row.addWidget(self._title_lbl)

        h_row.addStretch()

        self._badge = QLabel("IDLE")
        self._badge.setFixedHeight(22)
        self._badge.setAlignment(Qt.AlignCenter)
        self._badge.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.2))
        self._badge.setStyleSheet("""
            background: #F1F5F9; color: #64748B;
            border-radius: 11px; padding: 0 12px;
        """)
        h_row.addWidget(self._badge)
        lay.addLayout(h_row)

        # ── Row 2: Instruction label ──
        self._inst_lbl = QLabel(instruction)
        self._inst_lbl.setFont(qfont(FONT_UI, 8, QFont.Normal))
        self._inst_lbl.setStyleSheet(f"color: {COL_SUBTEXT.name()};")
        self._inst_lbl.setWordWrap(True)
        lay.addWidget(self._inst_lbl)

        # ── Row 3: Readout Capsule ──
        self._readout_frame = QFrame()
        self._readout_frame.setStyleSheet(f"""
            background: {COL_CARD_ALT.name()};
            border: 1px solid {COL_BORDER.name()};
            border-radius: 6px;
        """)
        rf_lay = QHBoxLayout(self._readout_frame)
        rf_lay.setContentsMargins(10, 4, 10, 4)

        self._readout_lbl = QLabel("MENUNGGU GERAKAN...")
        self._readout_lbl.setFont(qfont(FONT_MONO, 8, QFont.Bold))
        self._readout_lbl.setStyleSheet(f"color: {COL_TEXT.name()};")
        rf_lay.addWidget(self._readout_lbl)
        lay.addWidget(self._readout_frame)

        # ── Row 4: Diagnostic Message ──
        self._diag_lbl = QLabel("Drone level / siap diuji.")
        self._diag_lbl.setFont(qfont(FONT_UI, 8, QFont.DemiBold))
        self._diag_lbl.setStyleSheet(f"color: {COL_MUTED.name()};")
        self._diag_lbl.setWordWrap(True)
        lay.addWidget(self._diag_lbl)

    def set_status(self, status: str, detail: str, val_text: str = ""):
        self._status = status
        self._diag_lbl.setText(detail)
        if val_text:
            self._readout_lbl.setText(val_text)

        if status == "PASS":
            self._badge.setText("PASS \u2713")
            self._badge.setStyleSheet("""
                background: #DCFCE7; color: #166534;
                border: 1px solid #BBF7D0;
                border-radius: 11px; padding: 0 12px;
            """)
            self._diag_lbl.setStyleSheet("color: #16A34A; font-weight: 600;")
            self.setStyleSheet(f"""
                #BenchCheckCard {{
                    background: {COL_CARD.name()};
                    border: 1.5px solid #86EFAC;
                    border-radius: 12px;
                }}
            """)
        elif status == "DANGER":
            self._badge.setText("DANGER \u2717")
            self._badge.setStyleSheet("""
                background: #FEE2E2; color: #991B1B;
                border: 1px solid #FECACA;
                border-radius: 11px; padding: 0 12px;
            """)
            self._diag_lbl.setStyleSheet("color: #DC2626; font-weight: 600;")
            self.setStyleSheet(f"""
                #BenchCheckCard {{
                    background: {COL_CARD.name()};
                    border: 1.5px solid #FCA5A5;
                    border-radius: 12px;
                }}
            """)
        else:
            self._badge.setText("IDLE")
            self._badge.setStyleSheet("""
                background: #F1F5F9; color: #64748B;
                border: 1px solid #E2E8F0;
                border-radius: 11px; padding: 0 12px;
            """)
            self._diag_lbl.setStyleSheet(f"color: {COL_SUBTEXT.name()};")
            self.setStyleSheet(f"""
                #BenchCheckCard {{
                    background: {COL_CARD.name()};
                    border: 1px solid {COL_BORDER.name()};
                    border-radius: 12px;
                }}
            """)


# ───────────────────── Main Window ─────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("QMBED")
        self.setMinimumSize(960, 760)
        self.resize(1080, 820)

        # ── serial ──
        self._serial = None
        self._buf = b""

        # ── IMU / baro state ──
        self._orient = OrientationFilter()
        self._alt_offset = 0.0
        self._alt_smooth = 0.0
        self._press = 0.0
        self._first_alt = True

        # ── joystick state ──
        # Kalibrasi dilakukan dan disimpan di ESP32. GUI hanya memicu "CAL
        # SAMPLE" lalu membaca balasan "[CAL] OK CR=.. CT=.. CY=.. CP=..".
        self._js_raw = (128, 128, 128, 128)
        self._js_calibrated = False
        self._js_calib_center_display = None  # (cr,ct,cy,cp) ADC untuk banner
        self._js_calib_sampling = False       # menunggu balasan dari ESP32
        self._armed = False

        # ── bench-test live state (SMC + IMU) ──
        self._bench_smc = (0.0, 0.0, 0.0, 0.0, 0.0)  # (roll, pitch, uRoll, uPitch, uYaw)
        self._bench_imu = (0.0, 0.0, 0.0)           # (gx, gy, gz) deg/s
        self._bench_alt_press = (0.0, 0.0)          # (alt, press)

        # ── central ──
        central = QWidget()
        central.setStyleSheet(f"background:{COL_BG.name()};")
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── header ──
        self._header = self._build_header()
        root.addWidget(self._header)

        # ── body ──
        body = QWidget()
        body_lay = QVBoxLayout(body)
        body_lay.setContentsMargins(16, 12, 16, 12)
        body_lay.setSpacing(10)
        root.addWidget(body, stretch=1)

        # ── Main Stacked Content Views (Bebas Tab Bar, Full Clean View!) ──
        self._stack = QStackedWidget()
        self._stack.addWidget(self._build_attitude_tab())  # 0
        self._stack.addWidget(self._build_rc_tab())        # 1
        self._stack.addWidget(self._build_smc_tab())       # 2
        self._stack.addWidget(self._build_bench_tab())     # 3
        body_lay.addWidget(self._stack, stretch=1)

        # ── timers ──
        self._serial_timer = QTimer(self)
        self._serial_timer.timeout.connect(self._read_serial)
        self._serial_timer.start(16)

        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._tick)
        self._anim_timer.start(16)

        self._calib_timer = QTimer(self)
        self._calib_timer.timeout.connect(self._on_calib_timeout)
        self._calib_timer.setInterval(2000)  # timeout menunggu balasan ESP32

        self._refresh_ports()

    # ────────── navigation helpers ──────────

    def _create_nav_icon(self, icon_type: str, size: int = 20) -> QIcon:
        """Buat ikon vektor kustom bertema Sky Blue & White secara matematis via QPainter."""
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.transparent)
        p = QPainter(pixmap)
        p.setRenderHint(QPainter.Antialiasing)

        center = size / 2.0
        r = size * 0.44

        if icon_type == "ATTITUDE":
            # ── Artificial Horizon / Gyro Icon ──
            p.setPen(QPen(COL_ACCENT_DK, 1.4))
            p.setBrush(QBrush(QColor("#E0F2FE")))
            p.drawEllipse(QPointF(center, center), r, r)

            ground_path = QPainterPath()
            ground_path.moveTo(center - r, center)
            ground_path.arcTo(center - r, center - r, 2 * r, 2 * r, 180, 180)
            ground_path.closeSubpath()
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor("#F1F5F9")))
            p.drawPath(ground_path)

            p.setPen(QPen(COL_ACCENT_DK, 1.4))
            p.drawLine(QPointF(center - r, center), QPointF(center + r, center))

            p.setPen(QPen(COL_ACCENT, 1.6))
            p.drawLine(QPointF(center - 3.5, center), QPointF(center + 3.5, center))
            p.drawLine(QPointF(center, center - 2.5), QPointF(center, center + 2.5))

        elif icon_type == "CONTROL":
            # ── Gimbal Stick / Crosshair Icon ──
            p.setPen(QPen(QColor("#CBD5E1"), 1.2))
            p.setBrush(QBrush(QColor("#F0F9FF")))
            p.drawEllipse(QPointF(center, center), r, r)

            p.setPen(QPen(QColor("#BAE6FD"), 1))
            p.drawLine(QPointF(center - r + 2, center), QPointF(center + r - 2, center))
            p.drawLine(QPointF(center, center - r + 2), QPointF(center, center + r - 2))

            knob_x = center + 2.2
            knob_y = center - 2.2
            knob_r = 3.2
            p.setPen(QPen(QColor("#FFFFFF"), 1.0))
            p.setBrush(QBrush(COL_ACCENT))
            p.drawEllipse(QPointF(knob_x, knob_y), knob_r, knob_r)

        elif icon_type == "SMC":
            # ── Sliding Mode / Tuning Response Wave Icon ──
            rect_box = QRectF(center - r, center - r, 2 * r, 2 * r)
            p.setPen(QPen(QColor("#E2E8F0"), 1))
            p.setBrush(QBrush(QColor("#F8FAFC")))
            p.drawRoundedRect(rect_box, 4, 4)

            p.setPen(QPen(QColor("#CBD5E1"), 1, Qt.DotLine))
            p.drawLine(QPointF(rect_box.left() + 2, center), QPointF(rect_box.right() - 2, center))

            path = QPainterPath()
            path.moveTo(rect_box.left() + 2, center + r * 0.65)
            path.cubicTo(
                rect_box.left() + r * 0.6, center - r * 0.75,
                rect_box.left() + r * 1.3, center + r * 0.35,
                rect_box.right() - 2, center
            )
            p.setPen(QPen(COL_ACCENT, 1.6, Qt.SolidLine, Qt.RoundCap))
            p.setBrush(Qt.NoBrush)
            p.drawPath(path)

        elif icon_type == "BENCH":
            # ── Quad-X Drone Motor Frame Icon ──
            d = r * 0.68
            p.setPen(QPen(QColor("#64748B"), 1.4, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(center - d, center - d), QPointF(center + d, center + d))
            p.drawLine(QPointF(center - d, center + d), QPointF(center + d, center - d))

            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(COL_ACCENT_DK))
            p.drawEllipse(QPointF(center, center), 2.2, 2.2)

            motor_r = 2.2
            for mx, my in [(-d, -d), (d, -d), (-d, d), (d, d)]:
                p.setPen(QPen(QColor("#FFFFFF"), 0.8))
                p.setBrush(QBrush(COL_ACCENT))
                p.drawEllipse(QPointF(center + mx, center + my), motor_r, motor_r)

        p.end()
        return QIcon(pixmap)

    def _show_nav_menu(self):
        menu = QMenu(self)
        menu.setStyleSheet(f"""
            QMenu {{
                background: {COL_CARD.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 8px;
                padding: 6px;
            }}
            QMenu::item {{
                background: transparent;
                color: {COL_TEXT.name()};
                padding: 8px 18px 8px 12px;
                border-radius: 5px;
                font: bold 10px Inter, 'Segoe UI Variable', 'Segoe UI', system-ui, sans-serif;
                letter-spacing: 1.2px;
            }}
            QMenu::item:selected {{
                background: {COL_ACCENT_XLT.name()};
                color: {COL_ACCENT_DK.name()};
            }}
        """)

        views = [
            ("ATTITUDE", 0, "ATTITUDE", self._create_nav_icon("ATTITUDE")),
            ("CONTROL", 1, "CONTROL", self._create_nav_icon("CONTROL")),
            ("SMC TUNING", 2, "SMC TUNING", self._create_nav_icon("SMC")),
            ("BENCH TEST", 3, "BENCH TEST", self._create_nav_icon("BENCH")),
        ]
        for title, idx, short_name, icon in views:
            action = QAction(icon, f"  {title}", self)
            action.triggered.connect(lambda checked=False, i=idx, name=short_name: self._switch_page(i, name))
            menu.addAction(action)

        menu.exec(self._nav_btn.mapToGlobal(QPoint(self._nav_btn.width() - 160, self._nav_btn.height() + 4)))

    def _switch_page(self, index: int, label_text: str):
        if hasattr(self, "_stack"):
            self._stack.setCurrentIndex(index)
        if hasattr(self, "_nav_btn"):
            self._nav_btn.setText(f"\u2630  {label_text}  \u25BE")

    # ────────── unified header ──────────

    def _build_header(self):
        header = QFrame()
        header.setObjectName("UnifiedHeader")
        header.setFixedHeight(56)
        header.setStyleSheet(f"""
            #UnifiedHeader {{
                background: {COL_CARD.name()};
                border-bottom: 1px solid {COL_BORDER.name()};
            }}
        """)
        add_shadow(header, blur=18, dy=2, alpha=20)

        lay = QHBoxLayout(header)
        lay.setContentsMargins(18, 0, 18, 0)
        lay.setSpacing(10)

        # ── Brand (Left) ──
        self._logo = QLabel("Q")
        self._logo.setFixedSize(30, 30)
        self._logo.setAlignment(Qt.AlignCenter)
        self._logo.setStyleSheet(f"""
            background: {COL_ACCENT.name()};
            color: white;
            border-radius: 7px;
            font-family: {CSS_TITLE};
            font-weight: 800;
            font-size: 15px;
        """)
        lay.addWidget(self._logo, 0, Qt.AlignVCenter)

        self._title = QLabel("QMBED")
        self._title.setStyleSheet(f"""
            color: {COL_TEXT.name()};
            font-family: {CSS_TITLE};
            font-weight: 800;
            font-size: 15px;
            letter-spacing: 2px;
        """)
        lay.addWidget(self._title, 0, Qt.AlignVCenter)

        gcs_badge = QLabel("GCS")
        gcs_badge.setStyleSheet("""
            color: #0369A1;
            background: #E0F2FE;
            border: 1px solid #BAE6FD;
            border-radius: 4px;
            padding: 1px 6px;
            font-weight: 800;
            font-size: 8px;
            letter-spacing: 1px;
        """)
        lay.addWidget(gcs_badge, 0, Qt.AlignVCenter)

        # Divider 1
        sep1 = QFrame()
        sep1.setFrameShape(QFrame.VLine)
        sep1.setStyleSheet(f"color: {COL_BORDER.name()};")
        sep1.setFixedHeight(22)
        lay.addWidget(sep1, 0, Qt.AlignVCenter)

        # ── Serial Connection Controls ──
        port_lbl = QLabel("PORT")
        port_lbl.setStyleSheet(f"""
            color: {COL_SUBTEXT.name()};
            font-family: {CSS_UI};
            font-weight: 700;
            font-size: 8.5px;
            letter-spacing: 1.2px;
        """)
        lay.addWidget(port_lbl, 0, Qt.AlignVCenter)

        self._port_cb = QComboBox()
        self._port_cb.setMinimumWidth(125)
        self._port_cb.setFixedHeight(30)
        self._port_cb.setStyleSheet(f"""
            QComboBox {{
                background: {COL_CARD_ALT.name()};
                color: {COL_TEXT.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 6px;
                padding-left: 8px;
                padding-right: 24px;
                padding-top: 0px;
                padding-bottom: 0px;
                font-family: {CSS_MONO};
                font-size: 11px;
                font-weight: 600;
            }}
            QComboBox:hover {{
                border-color: {COL_ACCENT.name()};
                background: {COL_CARD.name()};
            }}
            QComboBox:focus {{ border-color: {COL_ACCENT.name()}; }}
            QComboBox::drop-down {{
                subcontrol-origin: padding;
                subcontrol-position: center right;
                width: 18px;
                border: none;
                margin-right: 4px;
            }}
            QComboBox::down-arrow {{
                width: 0px;
                height: 0px;
                border-left: 3.5px solid transparent;
                border-right: 3.5px solid transparent;
                border-top: 4.5px solid #64748B;
            }}
            QComboBox QAbstractItemView {{
                background: {COL_CARD.name()};
                color: {COL_TEXT.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 6px;
                padding: 4px;
                selection-background-color: {COL_ACCENT_XLT.name()};
                selection-color: {COL_TEXT.name()};
                outline: none;
                font-family: {CSS_MONO};
                font-size: 11px;
            }}
        """)
        lay.addWidget(self._port_cb, 0, Qt.AlignVCenter)

        self._refresh_btn = QPushButton("\u21BB")
        self._refresh_btn.setFixedSize(30, 30)
        self._refresh_btn.setToolTip("Refresh COM ports")
        self._refresh_btn.setStyleSheet(f"""
            QPushButton {{
                background: {COL_CARD_ALT.name()};
                color: {COL_TEXT.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 6px;
                font-family: {CSS_UI};
                font-weight: 700;
                font-size: 13px;
            }}
            QPushButton:hover {{
                background: {COL_ACCENT_XLT.name()};
                border-color: {COL_ACCENT.name()};
            }}
        """)
        self._refresh_btn.clicked.connect(self._refresh_ports)
        lay.addWidget(self._refresh_btn, 0, Qt.AlignVCenter)

        self._connect_btn = QPushButton("CONNECT")
        self._connect_btn.setFixedHeight(30)
        self._connect_btn.setCursor(Qt.PointingHandCursor)
        self._apply_connect_style(False)
        self._connect_btn.clicked.connect(self._toggle_serial)
        lay.addWidget(self._connect_btn, 0, Qt.AlignVCenter)

        # Divider 2
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.VLine)
        sep2.setStyleSheet(f"color: {COL_BORDER.name()};")
        sep2.setFixedHeight(22)
        lay.addWidget(sep2, 0, Qt.AlignVCenter)

        # ── Action Buttons ──
        self._cal_imu_btn = self._make_secondary_btn("CAL IMU",
                                                    tooltip="Zero roll/pitch/yaw filter.")
        self._cal_imu_btn.clicked.connect(self._calibrate_imu)
        lay.addWidget(self._cal_imu_btn, 0, Qt.AlignVCenter)

        self._cal_stick_btn = self._make_secondary_btn(
            "CAL STICKS",
            tooltip="Hold sticks centered; sample and set center as origin."
        )
        self._cal_stick_btn.setEnabled(False)
        self._cal_stick_btn.clicked.connect(self._start_stick_calibration)
        lay.addWidget(self._cal_stick_btn, 0, Qt.AlignVCenter)

        self._arm_btn = QPushButton("ARM")
        self._arm_btn.setFixedHeight(30)
        self._arm_btn.setCursor(Qt.PointingHandCursor)
        self._arm_btn.setToolTip("Arm / Disarm drone motors.")
        self._arm_btn.setEnabled(False)
        self._apply_arm_style(False)
        self._arm_btn.clicked.connect(self._toggle_arm)
        lay.addWidget(self._arm_btn, 0, Qt.AlignVCenter)

        self._reset_stick_btn = self._make_secondary_btn(
            "RESET CAL",
            variant="danger",
            tooltip="Discard stick calibration center."
        )
        self._reset_stick_btn.setVisible(False)
        self._reset_stick_btn.clicked.connect(self._reset_stick_calibration)
        lay.addWidget(self._reset_stick_btn, 0, Qt.AlignVCenter)

        lay.addStretch(1)

        # ── Telemetry Message & Link Status ──
        self._msg = QLabel("\u2139  Awaiting connection")
        self._msg.setFixedHeight(26)
        self._msg.setMaximumWidth(300)
        self._msg.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._msg.setStyleSheet(
            f"color:#475569; background:#F1F5F9; border:1px solid #E2E8F0;"
            f"border-radius:13px; padding:0 12px;"
            f"font-family:{CSS_UI}; font-weight:600; font-size:9.5px;"
        )
        lay.addWidget(self._msg, 0, Qt.AlignVCenter)

        self._status = QLabel("\u25CF  DISCONNECTED")
        self._status.setFixedHeight(26)
        self._status.setAlignment(Qt.AlignCenter)
        self._status.setStyleSheet(
            "color:#B91C1C; background:#FEE2E2; border:1px solid #FECACA;"
            f"font-family:{CSS_UI}; font-weight:700; font-size:9.5px;"
            "letter-spacing:1.5px;"
            "padding:0 12px; border-radius:13px;"
        )
        lay.addWidget(self._status, 0, Qt.AlignVCenter)

        # Divider 3
        sep3 = QFrame()
        sep3.setFrameShape(QFrame.VLine)
        sep3.setStyleSheet(f"color: {COL_BORDER.name()};")
        sep3.setFixedHeight(22)
        lay.addWidget(sep3, 0, Qt.AlignVCenter)

        # ── Hamburger Navigation Button (Far Right) ──
        self._nav_btn = QPushButton("\u2630  ATTITUDE  \u25BE")
        self._nav_btn.setFixedHeight(30)
        self._nav_btn.setCursor(Qt.PointingHandCursor)
        self._nav_btn.setStyleSheet(f"""
            QPushButton {{
                background: {COL_CARD_ALT.name()};
                color: {COL_TEXT.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 6px;
                font: bold 10px Inter, 'Segoe UI Variable', 'Segoe UI', system-ui, sans-serif;
                letter-spacing: 1px;
                padding: 0 12px;
            }}
            QPushButton:hover {{
                background: {COL_ACCENT_XLT.name()};
                border-color: {COL_ACCENT.name()};
                color: {COL_ACCENT_DK.name()};
            }}
        """)
        self._nav_btn.clicked.connect(self._show_nav_menu)
        lay.addWidget(self._nav_btn, 0, Qt.AlignVCenter)

        return header

    def set_hint(self, text, level="info"):
        """Perbarui status pesan di header bar. Level: info / ok / warn / err."""
        self.set_message(text, level=level)

    def set_message(self, text, level="info"):
        """Tampilkan pesan status/aktivitas di pill header."""
        if not hasattr(self, "_msg"):
            return
        markers = {"info": "\u2139", "ok": "\u2713", "warn": "\u26A0", "err": "\u2717"}
        mark = markers.get(level, "\u2139")
        colors = {
            "info": ("#475569", "#F1F5F9", "#E2E8F0"),
            "ok":   ("#166534", "#DCFCE7", "#BBF7D0"),
            "warn": ("#92400E", "#FEF3C7", "#FCD34D"),
            "err":  ("#991B1B", "#FEE2E2", "#FCA5A5"),
        }
        fg, bg, bd = colors.get(level, colors["info"])
        if len(text) > 42:
            text = text[:40] + "\u2026"
        self._msg.setText(f"{mark}  {text}")
        self._msg.setStyleSheet(
            f"color:{fg}; background:{bg}; border:1px solid {bd};"
            f"border-radius:13px; padding:0 12px;"
            f"font-family:{CSS_UI}; font-weight:600; font-size:9.5px;"
        )

    def set_status(self, connected, port=None):
        if not hasattr(self, "_status"):
            return
        if connected:
            txt = f"\u25CF  CONNECTED  {port or ''}".strip()
            self._status.setStyleSheet(
                "color:#15803D; background:#DCFCE7; border:1px solid #BBF7D0;"
                f"font-family:{CSS_UI}; font-weight:700; font-size:9.5px;"
                "letter-spacing:1.5px;"
                "padding:0 12px; border-radius:13px;"
            )
        else:
            txt = "\u25CF  DISCONNECTED"
            self._status.setStyleSheet(
                "color:#B91C1C; background:#FEE2E2; border:1px solid #FECACA;"
                f"font-family:{CSS_UI}; font-weight:700; font-size:9.5px;"
                "letter-spacing:1.5px;"
                "padding:0 12px; border-radius:13px;"
            )
        self._status.setText(txt)

    def _make_secondary_btn(self, text, variant="default", tooltip=""):
        btn = QPushButton(text)
        btn.setFixedHeight(30)
        btn.setCursor(Qt.PointingHandCursor)
        if tooltip:
            btn.setToolTip(tooltip)
        if variant == "danger":
            btn.setStyleSheet(f"""
                QPushButton {{
                    background: {COL_CARD.name()};
                    color: #B91C1C;
                    border: 1px solid #FCA5A5;
                    border-radius: 6px;
                    font: bold 9.5px Inter, 'Segoe UI Variable', 'Segoe UI', system-ui, sans-serif;
                    letter-spacing: 1.2px;
                    padding: 0 10px;
                }}
                QPushButton:hover {{
                    background: #FEE2E2;
                }}
                QPushButton:disabled {{
                    background: {COL_CARD_ALT.name()};
                    color: {COL_MUTED.name()};
                    border-color: {COL_BORDER.name()};
                }}
            """)
        else:
            btn.setStyleSheet(f"""
                QPushButton {{
                    background: {COL_CARD.name()};
                    color: {COL_ACCENT.name()};
                    border: 1px solid {COL_ACCENT_LT.name()};
                    border-radius: 6px;
                    font: bold 9.5px Inter, 'Segoe UI Variable', 'Segoe UI', system-ui, sans-serif;
                    letter-spacing: 1.2px;
                    padding: 0 10px;
                }}
                QPushButton:hover {{
                    background: {COL_ACCENT_XLT.name()};
                    border-color: {COL_ACCENT.name()};
                }}
                QPushButton:disabled {{
                    background: {COL_CARD_ALT.name()};
                    color: {COL_MUTED.name()};
                    border-color: {COL_BORDER.name()};
                }}
            """)
        return btn

    def _apply_connect_style(self, connected):
        if not hasattr(self, "_connect_btn"):
            return
        if connected:
            self._connect_btn.setText("DISCONNECT")
            self._connect_btn.setStyleSheet(f"""
                QPushButton {{
                    background: #DCFCE7;
                    color: #15803D;
                    border: 1px solid #86EFAC;
                    border-radius: 6px;
                    font: bold 9.5px Inter, 'Segoe UI Variable', 'Segoe UI', system-ui, sans-serif;
                    letter-spacing: 1.2px;
                    padding: 0 12px;
                }}
                QPushButton:hover {{
                    background: #BBF7D0;
                    border-color: #4ADE80;
                }}
            """)
        else:
            self._connect_btn.setText("CONNECT")
            self._connect_btn.setStyleSheet(f"""
                QPushButton {{
                    background: {COL_ACCENT.name()};
                    color: white;
                    border: 1px solid {COL_ACCENT_DK.name()};
                    border-radius: 6px;
                    font: bold 9.5px Inter, 'Segoe UI Variable', 'Segoe UI', system-ui, sans-serif;
                    letter-spacing: 1.2px;
                    padding: 0 12px;
                }}
                QPushButton:hover {{
                    background: {COL_ACCENT_DK.name()};
                }}
            """)

    def _apply_arm_style(self, armed: bool):
        targets = [getattr(self, "_arm_btn", None), getattr(self, "_bench_arm_btn", None)]
        for btn in targets:
            if btn is None:
                continue
            if armed:
                btn.setText("DISARM")
                btn.setStyleSheet("""
                    QPushButton {
                        background: #EF4444;
                        color: white;
                        border: 1px solid #DC2626;
                        border-radius: 6px;
                        font: bold 9.5px Inter, 'Segoe UI Variable', 'Segoe UI', system-ui, sans-serif;
                        letter-spacing: 1.2px;
                        padding: 0 12px;
                    }
                    QPushButton:hover {
                        background: #DC2626;
                    }
                """)
            else:
                btn.setText("ARM")
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background: {COL_CARD.name()};
                        color: #DC2626;
                        border: 1.5px solid #EF4444;
                        border-radius: 6px;
                        font: bold 9.5px Inter, 'Segoe UI Variable', 'Segoe UI', system-ui, sans-serif;
                        letter-spacing: 1.2px;
                        padding: 0 12px;
                    }}
                    QPushButton:hover {{
                        background: #FEE2E2;
                    }}
                    QPushButton:disabled {{
                        background: {COL_CARD_ALT.name()};
                        color: {COL_MUTED.name()};
                        border-color: {COL_BORDER.name()};
                    }}
                """)

    def _toggle_arm(self):
        self._armed = not self._armed
        self._apply_arm_style(self._armed)
        if self._serial and self._serial.is_open:
            cmd = b"ARM\n" if self._armed else b"DISARM\n"
            try:
                self._serial.write(cmd)
                self._serial.flush()
            except Exception as e:
                self.set_hint(f"Serial write error: {e}", level="err")
                return

        if self._armed:
            self.set_hint("Drone ARMING / ARMED (Motors Live)", level="warn")
        else:
            self.set_hint("Drone DISARMED (Safe)", level="ok")

    # ────────── tabs ──────────

    def _build_attitude_tab(self):
        page = QWidget()
        page.setStyleSheet("background: transparent;")
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 10, 0, 0)
        lay.setSpacing(12)

        main_row = QHBoxLayout()
        main_row.setSpacing(12)
        self._horizon = HorizonDroneWidget()
        add_shadow(self._horizon, blur=22, dy=3, alpha=25)
        self._alt_tape = AltitudeTapeWidget()
        add_shadow(self._alt_tape, blur=18, dy=3, alpha=22)
        main_row.addWidget(self._horizon, stretch=1)
        main_row.addWidget(self._alt_tape)
        lay.addLayout(main_row, stretch=1)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(10)
        self._card_roll  = MetricCard("ROLL",  "degrees",  COL_ACCENT)
        self._card_pitch = MetricCard("PITCH", "degrees",  COL_ACCENT_LT)
        self._card_yaw   = MetricCard("YAW",   "degrees",  COL_ACCENT_DK)
        self._card_alt   = MetricCard("ALTITUDE", "meters", COL_OK)
        self._card_alt.set_format("{:+.2f}")
        for c in (self._card_roll, self._card_pitch, self._card_yaw, self._card_alt):
            cards_row.addWidget(c, stretch=1)
        lay.addLayout(cards_row)

        self._info_bar = SecondaryInfoBar()
        lay.addWidget(self._info_bar)

        return page

    def _build_rc_tab(self):
        page = QWidget()
        page.setStyleSheet("background: transparent;")
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 10, 0, 0)
        lay.setSpacing(12)

        self._stick_banner = StatusBanner()
        lay.addWidget(self._stick_banner)

        sticks_row = QHBoxLayout()
        sticks_row.setSpacing(12)

        self._joy_left = JoystickWidget("LEFT STICK", mode="MODE 2",
                                        vx_label="YAW", vy_label="THROTTLE")
        self._joy_right = JoystickWidget("RIGHT STICK", mode="MODE 2",
                                         vx_label="ROLL", vy_label="PITCH")
        add_shadow(self._joy_left, blur=22, dy=3, alpha=22)
        add_shadow(self._joy_right, blur=22, dy=3, alpha=22)
        sticks_row.addWidget(self._joy_left, stretch=1)
        sticks_row.addWidget(self._joy_right, stretch=1)
        lay.addLayout(sticks_row, stretch=1)

        self._rc_readout = RCReadoutCard()
        lay.addWidget(self._rc_readout)

        return page

    def _build_smc_tab(self):
        page = QWidget()
        page.setStyleSheet("background: transparent;")
        root = QHBoxLayout(page)
        root.setContentsMargins(0, 10, 0, 0)
        root.setSpacing(12)

        card = QFrame()
        card.setStyleSheet(f"""
            background: {COL_CARD.name()}; border: 1px solid {COL_BORDER.name()};
            border-radius: 10px;
        """)
        add_shadow(card, blur=18, dy=2, alpha=20)
        form = QFormLayout(card)
        form.setContentsMargins(22, 18, 22, 18)
        form.setSpacing(12)

        title = QLabel("SLIDING MODE CONTROLLER")
        title.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:800; font-size:14px; letter-spacing:2px;")
        form.addRow(title)

        self._smc_inputs = {}
        fields = [
            ("K1", "k1", 5.0, 0.1, 30.0, 0.1),
            ("K2", "k2", 2.0, 0.1, 30.0, 0.1),
            ("EPS", "eps", 8.0, 0.1, 100.0, 0.1),
            ("FORCE TO PWM", "force", 30.0, 0.1, 200.0, 0.5),
            ("MAX CORRECTION (us)", "delta", 180.0, 10.0, 500.0, 5.0),
        ]
        for label, key, value, low, high, step in fields:
            spin = QDoubleSpinBox()
            spin.setRange(low, high)
            spin.setValue(value)
            spin.setSingleStep(step)
            spin.setDecimals(2)
            spin.setSuffix(" us" if key == "delta" else "")
            spin.setStyleSheet(f"background:{COL_CARD_ALT.name()}; color:{COL_TEXT.name()}; padding:5px; border:1px solid {COL_BORDER.name()}; border-radius:5px;")
            self._smc_inputs[key] = spin

            name = QLabel(label)
            name.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:600; font-size:12px;")
            form.addRow(name, spin)

        self._smc_apply_btn = self._make_secondary_btn("APPLY SMC PARAMETERS")
        self._smc_apply_btn.clicked.connect(self._apply_smc_parameters)
        form.addRow(self._smc_apply_btn)
        root.addWidget(card, 1)

        self._smc_plot = SmcPlotWidget()
        add_shadow(self._smc_plot, blur=18, dy=2, alpha=20)
        root.addWidget(self._smc_plot, 2)

        guide = QLabel(
            "EFEK PARAMETER\n\n"
            "K1\nLebih besar: drone kembali level lebih cepat. Terlalu besar: overshoot dan osilasi.\n\n"
            "K2\nLebih besar: lebih kuat melawan gangguan. Terlalu besar: motor bergetar/chattering.\n\n"
            "EPS\nLebih besar: koreksi lebih halus, tetapi leveling lebih lambat/longgar. Lebih kecil: koreksi tajam, tetapi lebih banyak chattering.\n\n"
            "FORCE TO PWM\nMengubah torsi kendali hasil SMC menjadi koreksi PWM. Lebih besar membuat SMC lebih agresif.\n\n"
            "MAX CORRECTION\nBatas keselamatan koreksi PWM per motor. Lebih besar memberi kemampuan pemulihan lebih besar, tetapi beda tenaga motor juga lebih besar.\n\n"
            "Mulai tanpa propeller. Ubah satu parameter per kali, lalu uji kembali."
        )
        guide.setWordWrap(True)
        guide.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        guide.setStyleSheet(f"""
            background:{COL_CARD.name()}; color:{COL_SUBTEXT.name()}; border:1px solid {COL_BORDER.name()};
            border-radius:10px; padding:20px; font-size:11px; line-height:1.4;
        """)
        root.addWidget(guide, 1)
        return page

    # ────────── Bench Test (Props-Off) tab ──────────

    def _build_bench_tab(self):
        page = QWidget()
        page.setStyleSheet("background: transparent;")
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 8, 0, 0)
        lay.setSpacing(10)

        # ── White & Sky Blue Warning Banner ──
        warn = QFrame()
        warn.setStyleSheet("background: transparent; border: none;")
        wl = QHBoxLayout(warn)
        wl.setContentsMargins(14, 8, 14, 8)
        wl.setSpacing(12)

        icon_lbl = QLabel("⚠️")
        icon_lbl.setFont(qfont(FONT_UI, 12))
        icon_lbl.setAlignment(Qt.AlignCenter)
        icon_lbl.setFixedWidth(28)
        wl.addWidget(icon_lbl, 0, Qt.AlignVCenter)

        wtxt = QLabel("<b style='color:#0369A1; font-size:10px; letter-spacing:0.8px;'>PROPS-OFF BENCH TEST ONLY :</b> "
                      "<span style='color:#0F172A; font-size:10px; font-weight:500;'>Copot SEMUA propeller sebelum pengujian. "
                      "Naikkan throttle remote sedikit di atas idle (~1220 us / 20%) agar koreksi SMC aktif, "
                      "lalu gerakkan/miringkan drone dengan tangan untuk memverifikasi arah respon motor.</span>")
        wtxt.setWordWrap(True)
        wl.addWidget(wtxt, stretch=1)

        self._bench_arm_btn = QPushButton("ARM")
        self._bench_arm_btn.setFixedHeight(30)
        self._bench_arm_btn.setMinimumWidth(110)
        self._bench_arm_btn.setCursor(Qt.PointingHandCursor)
        self._apply_arm_style(False)
        self._bench_arm_btn.setEnabled(False)
        self._bench_arm_btn.clicked.connect(self._toggle_arm)
        wl.addWidget(self._bench_arm_btn)

        lay.addWidget(warn)

        # ── Main Content: 2 Columns (Quad-X Diagram + Check Cards) ──
        row = QHBoxLayout()
        row.setSpacing(12)

        # Kolom Kiri: Diagram Quad-X Motor Mix
        self._bench_mix = BenchMotorMixWidget()
        add_shadow(self._bench_mix, blur=18, dy=2, alpha=20)
        row.addWidget(self._bench_mix, 5)

        # Kolom Kanan: 3 Kartu Verifikasi Sumbu
        check_col = QVBoxLayout()
        check_col.setSpacing(8)

        self._check_roll = BenchCheckCard(
            "STEP 1", "ROLL RESPONSE",
            "Miringkan drone ke KANAN (Roll > +5\u00B0) \u2014 motor KIRI (M1, M4) harus bertambah tenaga dan motor KANAN (M2, M3) berkurang.")
        self._check_pitch = BenchCheckCard(
            "STEP 2", "PITCH RESPONSE",
            "Miringkan drone nose-UP (Pitch > +5\u00B0) \u2014 motor DEPAN (M1, M2) harus berkurang tenaga dan motor BELAKANG (M3, M4) bertambah.")
        self._check_yaw = BenchCheckCard(
            "STEP 3", "YAW DAMPING",
            "Putar drone berputar ke KANAN (GZ > +10\u00B0/s) \u2014 SMC harus meredam rotasi (uYaw bernilai negatif).")

        for c in (self._check_roll, self._check_pitch, self._check_yaw):
            add_shadow(c, blur=14, dy=1, alpha=16)
            check_col.addWidget(c)

        row.addLayout(check_col, 6)
        lay.addLayout(row, stretch=1)

        # ── Checklist bottom progress bar ──
        self._bench_steps = []
        steps = QHBoxLayout()
        steps.setSpacing(8)
        for txt in ("1. LEVEL BASELINE",
                    "2. ROLL RESPONSE",
                    "3. PITCH RESPONSE",
                    "4. YAW DAMPING"):
            chk = QPushButton("\u25CB  " + txt)
            chk.setEnabled(False)
            chk.setFixedHeight(28)
            chk.setStyleSheet(f"""
                QPushButton {{
                    background:{COL_CARD.name()}; color:{COL_SUBTEXT.name()};
                    border:1px solid {COL_BORDER.name()}; border-radius:7px;
                    font: bold 8px Inter, sans-serif; letter-spacing:1.0px;
                    padding: 0 8px;
                }}
            """)
            steps.addWidget(chk, stretch=1)
            self._bench_steps.append(chk)
        lay.addLayout(steps)

        return page

    def _update_bench(self):
        # Gunakan roll/pitch ONBOARD drone (dari paket SMC) karena itulah yang
        # benar-benar dipakai controller, bukan filter ulang di GUI.
        roll = self._bench_smc[0]
        pitch = self._bench_smc[1]
        ur, up, uy = self._bench_smc[2], self._bench_smc[3], self._bench_smc[4]
        gz = self._bench_imu[2]

        # Konversi throttle stik mentah (0-255) ke PWM us (1000-2000 us)
        t_raw = self._js_raw[1] if len(self._js_raw) > 1 else 0
        throttle = 1000.0 + (t_raw / 255.0) * 1000.0 if self._armed else 1000.0

        self._bench_mix.set_data(roll, pitch, self._orient.yaw,
                                 ur, up, uy, throttle, self._armed)

        # ── Roll check ──
        if abs(roll) < 3.0:
            self._check_roll.set_status("IDLE",
                "Drone level. Miringkan drone ke kanan atau kiri untuk menguji.",
                f"ROLL {roll:+.1f}\u00B0   \u00B7   uROLL {ur:+.1f}")
        else:
            # roll>0 (kanan) => koreksi harus positive (naikkan kiri): ur>0
            if (roll > 0 and ur > 0) or (roll < 0 and ur < 0):
                self._check_roll.set_status("PASS",
                    "Koreksi BENAR: Motor sisi yang turun dinaikkan untuk melawan kemiringan.",
                    f"ROLL {roll:+.1f}\u00B0   \u00B7   uROLL {ur:+.1f}")
            else:
                self._check_roll.set_status("DANGER",
                    "Koreksi TERBALIK: Motor justru memperparah kemiringan! Tanda uRoll/mixer salah.",
                    f"ROLL {roll:+.1f}\u00B0   \u00B7   uROLL {ur:+.1f}")

        # ── Pitch check ──
        if abs(pitch) < 3.0:
            self._check_pitch.set_status("IDLE",
                "Drone level. Miringkan drone depan naik atau turun untuk menguji.",
                f"PITCH {pitch:+.1f}\u00B0   \u00B7   uPITCH {up:+.1f}")
        else:
            # pitch>0 (nose-up) => koreksi harus negative (turunkan depan): up<0
            if (pitch > 0 and up < 0) or (pitch < 0 and up > 0):
                self._check_pitch.set_status("PASS",
                    "Koreksi BENAR: Depan yang terangkat dikoreksi turun kembali level.",
                    f"PITCH {pitch:+.1f}\u00B0   \u00B7   uPITCH {up:+.1f}")
            else:
                self._check_pitch.set_status("DANGER",
                    "Koreksi TERBALIK: Depan yang terangkat malah dinaikkan! Tanda uPitch salah.",
                    f"PITCH {pitch:+.1f}\u00B0   \u00B7   uPITCH {up:+.1f}")

        # ── Yaw check ──
        if abs(gz) < 8.0:
            self._check_yaw.set_status("IDLE",
                "Drone diam. Putar drone berputar ke kanan atau kiri untuk menguji.",
                f"GZ {gz:+.1f}\u00B0/s   \u00B7   uYAW {uy:+.1f}")
        else:
            # rotasi kanan (gz>0) => damping berarti uy<0
            if (gz > 0 and uy < 0) or (gz < 0 and uy > 0):
                self._check_yaw.set_status("PASS",
                    "Koreksi BENAR: SMC meredam laju putaran yaw secara aktif.",
                    f"GZ {gz:+.1f}\u00B0/s   \u00B7   uYAW {uy:+.1f}")
            else:
                self._check_yaw.set_status("DANGER",
                    "Koreksi TERBALIK: SMC justru mempercepat rotasi yaw! Tanda uYaw salah.",
                    f"GZ {gz:+.1f}\u00B0/s   \u00B7   uYAW {uy:+.1f}")

        # ── checklist auto-update ──
        steps = self._bench_steps
        if len(steps) >= 4:
            level = abs(roll) < 2.5 and abs(pitch) < 2.5 and abs(gz) < 2.5
            self._style_bench_step(steps[0], level)
            roll_ok = self._check_roll._status == "PASS"
            pitch_ok = self._check_pitch._status == "PASS"
            yaw_ok = self._check_yaw._status == "PASS"
            self._style_bench_step(steps[1], roll_ok)
            self._style_bench_step(steps[2], pitch_ok)
            self._style_bench_step(steps[3], yaw_ok)

    def _style_bench_step(self, btn, ok):
        idx = self._bench_steps.index(btn)
        base = ["1. LEVEL BASELINE", "2. ROLL RESPONSE",
                "3. PITCH RESPONSE", "4. YAW DAMPING"][idx]
        if ok:
            btn.setText("\u2713  " + base)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background:{COL_OK.name()}; color:white;
                    border:none; border-radius:7px;
                    font: bold 8px Inter, sans-serif; letter-spacing:1.0px;
                    padding: 0 8px;
                }}
            """)
        else:
            btn.setText("\u25CB  " + base)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background:{COL_CARD.name()}; color:{COL_SUBTEXT.name()};
                    border:1px solid {COL_BORDER.name()}; border-radius:7px;
                    font: bold 8px Inter, sans-serif; letter-spacing:1.0px;
                    padding: 0 8px;
                }}
            """)

    def _apply_smc_parameters(self):
        if not self._serial or not self._serial.is_open:
            self.set_hint("Connect to remote before applying SMC parameters.", level="warn")
            return
        values = self._smc_inputs
        command = "SMC {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}\n".format(
            values["k1"].value(), values["k2"].value(), values["eps"].value(),
            values["force"].value(), values["delta"].value()
        )
        try:
            self._serial.write(command.encode("ascii"))
            self._serial.flush()
            self.set_hint("SMC parameters sent to remote and drone.", level="ok")
        except Exception as e:
            self.set_hint(f"SMC parameter write error: {e}", level="err")

    # ────────── lifecycle ──────────

    def _tick(self):
        self._horizon.update()
        self._joy_left.animate()
        self._joy_right.animate()
        if hasattr(self, "_bench_mix"):
            self._bench_mix.animate()
            self._update_bench()

    # ────────── IMU calibration ──────────

    def _calibrate_imu(self):
        self._orient.reset()
        self._alt_offset = 0.0
        self._alt_smooth = 0.0
        self._first_alt = True
        self._horizon.set_orientation(0, 0, 0)
        self._horizon.set_altitude(0.0)
        self._alt_tape.set_altitude(0.0)
        self._card_roll.set_value(0)
        self._card_pitch.set_value(0)
        self._card_yaw.set_value(0)
        self._card_alt.set_value(0)
        self.set_hint("IMU calibrated \u2713", level="ok")

    # ────────── Stick calibration ──────────
    # ESP32 melakukan semua sampling & penyimpanan center. GUI hanya:
    #   1) mengirim "CAL SAMPLE" saat tombol ditekan
    #   2) menunggu balasan "[CAL] OK CR=.. CT=.. CY=.. CP=.."
    #   3) menampilkan hasil & mengaktifkan tombol RESET.

    def _start_stick_calibration(self):
        if not self._serial or not self._serial.is_open:
            return
        self._js_calib_sampling = True
        self._js_calibrated = False
        self._cal_stick_btn.setEnabled(False)
        self._cal_stick_btn.setText("SAMPLING\u2026")
        self._stick_banner.show_info(
            "SAMPLING \u2013 hold both sticks perfectly centered, do not move."
        )
        try:
            self._serial.write(b"CAL SAMPLE\n")
            self._serial.flush()
        except Exception as e:
            self.set_hint(f"Gagal kirim kalibrasi ke remote: {e}", level="err")
            self._on_calib_timeout()
            return
        self._calib_timer.start()

    def _on_calib_timeout(self):
        # Tidak menerima balasan ESP32 (atau gagal tulis): kembalikan UI.
        self._js_calib_sampling = False
        self._calib_timer.stop()
        self._cal_stick_btn.setText("CALIBRATE STICKS")
        self._cal_stick_btn.setEnabled(True)
        if not self._js_calibrated:
            self._stick_banner.show_err(
                "NO REPLY FROM REMOTE \u2013 check serial link, try again."
            )

    def _on_cal_ok(self, cr, ct, cy, cp, n_samples=None):
        # Balasan dari ESP32: sampling selesai & center sudah di-update di ESP32.
        self._js_calib_sampling = False
        self._calib_timer.stop()
        self._js_calibrated = True
        self._js_calib_center_display = (cr, ct, cy, cp)
        self._cal_stick_btn.setText("CALIBRATE STICKS")
        self._cal_stick_btn.setEnabled(True)
        self._reset_stick_btn.setVisible(True)
        tag = f"  \u2022  {n_samples} samples" if n_samples else ""
        self._stick_banner.show_ok(
            f"CALIBRATED  \u2022  Center R={cr}  T={ct}  Y={cy}  P={cp}{tag}"
        )

    def _reset_stick_calibration(self):
        # RESET: ESP32 re-sample center (sama dengan kalibrasi ulang). Karena
        # kalibrasi sekarang terpusat di ESP32, "reset" = kalibrasi ulang.
        self._stick_banner.show_warn(
            "RE-CALIBRATING \u2013 hold sticks centered, single sample."
        )
        self._start_stick_calibration()

    # ────────── ports / serial ──────────

    def _refresh_ports(self):
        self._port_cb.clear()
        if not HAS_SERIAL:
            self._port_cb.addItem("(pyserial not installed)")
            return
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            self._port_cb.addItem("(no ports found)")
        for p in ports:
            self._port_cb.addItem(p.device)

    def _toggle_serial(self):
        if self._serial and self._serial.is_open:
            if self._armed:
                try:
                    self._serial.write(b"DISARM\n")
                    self._serial.flush()
                except Exception:
                    pass
            self._serial.close()
            self._serial = None
            self._armed = False
            self._apply_arm_style(False)
            self._apply_connect_style(False)
            self.set_status(False)
            self._horizon.set_connected(False)
            self._cal_stick_btn.setEnabled(False)
            self._arm_btn.setEnabled(False)
            if hasattr(self, "_bench_arm_btn"):
                self._bench_arm_btn.setEnabled(False)
                self._bench_arm_btn.setText("ARM")
                self._apply_arm_style(False)
            self.set_hint("Disconnected", level="info")
            return
        if not HAS_SERIAL:
            self.set_hint("pyserial not installed", level="err")
            return
        port = self._port_cb.currentText()
        if not port or port.startswith("("):
            self.set_hint("Select a valid port", level="warn")
            return
        try:
            self._serial = serial.Serial(port, 115200, timeout=0.02)
            self._buf = b""
            self._first_alt = True
            self._apply_connect_style(True)
            self.set_status(True, port)
            self._horizon.set_connected(True)
            self._cal_stick_btn.setEnabled(True)
            self._arm_btn.setEnabled(True)
            if hasattr(self, "_bench_arm_btn"):
                self._bench_arm_btn.setEnabled(True)
            self.set_hint(f"Streaming from {port}", level="ok")
        except Exception as e:
            self.set_hint(f"Error: {e}", level="err")
            self.set_status(False)

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

    # ────────── data processing ──────────

    def _push_att_metrics(self):
        self._card_roll.set_value(self._orient.roll)
        self._card_pitch.set_value(self._orient.pitch)
        self._card_yaw.set_value(self._orient.yaw)
        self._card_alt.set_value(self._alt_smooth)
        self._horizon.set_orientation(self._orient.roll,
                                      self._orient.pitch,
                                      self._orient.yaw)
        self._alt_tape.set_altitude(self._alt_smooth)
        self._horizon.set_altitude(self._alt_smooth)
        self._info_bar.update_press(self._press)

    def _push_rc_values(self):
        # ESP32 sudah mengirim nilai 0-255 yang sudah terkalibrasi penuh
        # (netral = 128). Terapkan langsung tanpa offset tambahan (tidak ada
        # double-offsetting lagi).
        r, t, y, p = self._js_raw

        # Left stick: X = YAW, Y = THROTTLE
        # Right stick: X = ROLL, Y = PITCH
        # Normalize 0..255 -> 0..1; joy widget Y=0 at top so invert
        self._joy_left.set_position(y / 255.0, 1.0 - t / 255.0)
        self._joy_right.set_position(r / 255.0, 1.0 - p / 255.0)
        self._rc_readout.set_values(r, t, y, p, self._js_calibrated)

    def _parse_line(self, text: str):
        m = IMU_RE.search(text)
        if m:
            ax, ay, az = [float(v) for v in m.groups()[:3]]
            gx, gy, gz = [float(v) for v in m.groups()[3:]]
            self._orient.update(ax, ay, az, gx, gy, gz)
            self._bench_imu = (gx, gy, gz)
            self._info_bar.note_sample()
            self._push_att_metrics()
            return

        m = BMP_RE.search(text)
        if m:
            self._press = float(m.group(1))
            alt = float(m.group(2))
            if self._first_alt:
                self._alt_offset = alt
                self._alt_smooth = 0.0
                self._first_alt = False
            corrected = alt - self._alt_offset
            # Digital IIR Low-Pass Filter: y[k] = 0.80 * y[k-1] + 0.20 * x[k]
            self._alt_smooth = 0.80 * self._alt_smooth + 0.20 * corrected
            self._push_att_metrics()
            return

        m = TX_RE.search(text)
        if m:
            r, t, y, p = [int(v) for v in m.groups()]
            self._js_raw = (r, t, y, p)
            self._push_rc_values()
            return

        m = SMC_RE.search(text)
        if m:
            roll, pitch, u_roll, u_pitch, u_yaw = [float(v) for v in m.groups()]
            self._bench_smc = (roll, pitch, u_roll, u_pitch, u_yaw)
            if hasattr(self, "_smc_plot"):
                self._smc_plot.add_sample(roll, pitch, u_roll, u_pitch, u_yaw)
            return

        m = CAL_OK_RE.search(text)
        if m and self._js_calib_sampling:
            cr, ct, cy, cp = [int(v) for v in m.groups()]
            self._on_cal_ok(cr, ct, cy, cp, n_samples=None)
            return


def main():
    import traceback
    try:
        app = QApplication(sys.argv)
        app.setStyle("Fusion")
        app.setFont(qfont(FONT_UI, 10))
        win = MainWindow()
        win.show()
        sys.exit(app.exec())
    except Exception:
        traceback.print_exc()
        input("\nTekan Enter untuk keluar.")


if __name__ == "__main__":
    main()
