import sys
import os
import json
import re
import math
import time
import csv
from datetime import datetime
from collections import deque
from PySide6.QtCore import Qt, QTimer, QPointF, QRectF, QPoint, QUrl
from PySide6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QFontMetricsF,
    QRadialGradient, QLinearGradient, QPolygonF, QPainterPath,
    QAction, QPixmap, QIcon, QDesktopServices
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton, QFrame,
    QGraphicsDropShadowEffect,
    QStackedWidget, QMenu, QFormLayout, QDoubleSpinBox, QScrollArea
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
    r"\[TX\]\s*R:([-+\d.]+)(?:deg)?\s*T:(\d+)(?:us)?\s*Y:([-+\d.]+)(?:dps)?\s*P:([-+\d.]+)(?:deg)?(?:\s*ARM:(\d+))?(?:\s*RAW_T:(\d+))?"
)
SMC_RE = re.compile(
    r"\[SMC\]\s*ROLL:([-\d.]+)\s*PITCH:([-\d.]+)"
    r"(?:\s*YAW:([-\d.]+))?"
    r"\s*UR:([-\d.]+)\s*UP:([-\d.]+)\s*UY:([-\d.]+)"
)
BAT_RE = re.compile(
    r"\[BAT\]\s*V:([-\d.]+)"
)
CAL_OK_RE = re.compile(
    r"\[CAL\]\s*OK\s*CR=(\d+)\s*CT=(\d+)\s*CY=(\d+)\s*CP=(\d+)"
)
PID_OK_RE = re.compile(
    r"\[PID\]\s*OK"
)
FLAGS_RE = re.compile(
    r"\[FLAGS\]\s*G:(\d)\s*B:(\d)\s*P:(\d)\s*V:(\d)\s*BS:(\d)\s*FS:(\d)"
)

PID_CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pid_config.json")
RECORDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "records")

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
            # Data gap > 1 detik dianggap koneksi terputus / drone restart.
            # Reset filter agar tidak drift dari state lama yang basi.
            if self.dt > 1.0:
                self._first_run = True
                self.yaw = 0.0
        self._last_time = now
        if self.dt <= 0 or self.dt > 0.5:
            self.dt = 0.12

        acc_total = math.sqrt(ax*ax + ay*ay + az*az)
        if acc_total > 1.0:
            acc_roll  = math.atan2(ay, az) * DEG
            acc_pitch = math.atan2(ax, math.sqrt(ay*ay + az*az)) * DEG
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
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self._has_target = False
        self.altitude = 0.0
        self._prop_angle = 0.0
        self._connected = False

    def set_orientation(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def set_target_orientation(self, target_roll, target_pitch):
        self.target_roll = target_roll
        self.target_pitch = target_pitch
        self._has_target = True

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

        # Actual Roll Pointer
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

        # Target Roll Pointer from Pilot Remote (Amber Cue)
        if self._has_target and abs(self.target_roll) > 0.3:
            p.save()
            p.rotate(self.target_roll)
            tgt_pointer = QPolygonF([
                QPointF(0, -arc_r + 3),
                QPointF(-4, -arc_r + 12),
                QPointF(4, -arc_r + 12),
            ])
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor("#F59E0B"))) # Amber target marker
            p.drawPolygon(tgt_pointer)
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


# ───────────────────── PID Realtime Plot Widget ─────────────────────

class PidPlotWidget(QWidget):
    """Visualisasi kurva respon real-time PID presisi tinggi:
    1) Subplot 1: Roll Setpoint vs Actual Roll (dengan selisih galat/error)
    2) Subplot 2: Pitch Setpoint vs Actual Pitch (dengan selisih galat/error)
    3) Subplot 3: Output Koreksi PWM (uRoll, uPitch, uYaw)
    Lengkap dengan grid skala sumbu Y, grid waktu sumbu X, dan badge nilai digital live."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 440)
        self._samples = deque(maxlen=250)
        self._cur_target_roll = 0.0
        self._cur_actual_roll = 0.0
        self._cur_target_pitch = 0.0
        self._cur_actual_pitch = 0.0
        self._cur_u_roll = 0.0
        self._cur_u_pitch = 0.0
        self._cur_u_yaw = 0.0
        self._recording = False
        self._record_time_str = ""

    def set_recording_status(self, recording: bool, time_str: str = ""):
        self._recording = recording
        self._record_time_str = time_str
        self.update()

    def add_sample(self, target_roll, actual_roll, target_pitch, actual_pitch, u_roll, u_pitch, u_yaw):
        self._cur_target_roll = target_roll
        self._cur_actual_roll = actual_roll
        self._cur_target_pitch = target_pitch
        self._cur_actual_pitch = actual_pitch
        self._cur_u_roll = u_roll
        self._cur_u_pitch = u_pitch
        self._cur_u_yaw = u_yaw
        self._samples.append((
            time.monotonic(),
            target_roll, actual_roll,
            target_pitch, actual_pitch,
            u_roll, u_pitch, u_yaw
        ))
        self.update()

    def _draw_subplot_grid(self, p, plot_rect, y_ticks, y_scale, y_unit):
        # Background canvas
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(QColor("#F8FAFC")))
        p.drawRoundedRect(plot_rect, 6, 6)

        p.save()
        p.setClipRect(plot_rect)

        # Horizontal Gridlines & Zero Line
        cy = plot_rect.center().y()
        h_half = plot_rect.height() * 0.44

        for val in y_ticks:
            y_pos = cy - (val / y_scale) * h_half
            if val == 0:
                # Zero Line: solid & prominent
                p.setPen(QPen(QColor("#64748B"), 1.2))
                p.drawLine(QPointF(plot_rect.left(), y_pos), QPointF(plot_rect.right(), y_pos))
            else:
                # Gridlines: subtle dotted
                p.setPen(QPen(QColor("#CBD5E1"), 1, Qt.DotLine))
                p.drawLine(QPointF(plot_rect.left(), y_pos), QPointF(plot_rect.right(), y_pos))

        # Vertical Time Gridlines (every 1.0 second over 5s window)
        now = time.monotonic()
        time_span = 5.0
        for sec in range(1, int(time_span) + 1):
            x_frac = 1.0 - (sec / time_span)
            x_pos = plot_rect.left() + x_frac * plot_rect.width()
            p.setPen(QPen(QColor("#E2E8F0"), 1, Qt.DashLine))
            p.drawLine(QPointF(x_pos, plot_rect.top()), QPointF(x_pos, plot_rect.bottom()))

        p.restore()

        # Y-Axis Labels (Left of plot_rect)
        p.setFont(qfont(FONT_MONO, 7, QFont.Bold))
        for val in y_ticks:
            y_pos = cy - (val / y_scale) * h_half
            val_str = f"{val:+d}{y_unit}" if val != 0 else f"0{y_unit}"
            label_rect = QRectF(plot_rect.left() - 44, y_pos - 7, 40, 14)
            p.setPen(COL_TEXT if val == 0 else COL_SUBTEXT)
            p.drawText(label_rect, Qt.AlignRight | Qt.AlignVCenter, val_str)

        # X-Axis Time Labels (Bottom of plot_rect)
        p.setFont(qfont(FONT_UI, 6.5, QFont.DemiBold))
        p.setPen(COL_MUTED)
        for sec in range(0, int(time_span) + 1):
            x_frac = 1.0 - (sec / time_span)
            x_pos = plot_rect.left() + x_frac * plot_rect.width()
            txt = "NOW" if sec == 0 else f"-{sec}s"
            t_rect = QRectF(x_pos - 18, plot_rect.bottom() + 3, 36, 12)
            p.drawText(t_rect, Qt.AlignCenter, txt)

    def _draw_series(self, p, plot_rect, idx, color, width, y_scale, style=Qt.SolidLine):
        if len(self._samples) < 2:
            return
        now = time.monotonic()
        time_span = 5.0
        t_start = now - time_span

        cy = plot_rect.center().y()
        h_half = plot_rect.height() * 0.44

        points = []
        for s in self._samples:
            t = s[0]
            if t < t_start - 0.2:
                continue
            val = s[idx]
            x_frac = max(0.0, min(1.0, (t - t_start) / time_span))
            x = plot_rect.left() + x_frac * plot_rect.width()
            y = cy - (val / y_scale) * h_half
            y = max(plot_rect.top() + 2, min(plot_rect.bottom() - 2, y))
            points.append(QPointF(x, y))

        if len(points) < 2:
            return

        p.save()
        p.setClipRect(plot_rect)
        p.setPen(QPen(color, width, style, Qt.RoundCap, Qt.RoundJoin))
        for i in range(1, len(points)):
            p.drawLine(points[i - 1], points[i])
        p.restore()

    def _draw_badges(self, p, right_x, top_y, badges):
        """Draw small legend pills aligned to the right."""
        cur_right = right_x
        p.setFont(qfont(FONT_MONO, 7, QFont.Bold))
        for text, col, is_dashed in reversed(badges):
            fm = QFontMetricsF(p.font())
            tw = fm.horizontalAdvance(text)
            pill_w = tw + 18
            pill_h = 16
            cur_right -= pill_w
            pill_rect = QRectF(cur_right, top_y, pill_w, pill_h)

            p.setPen(QPen(col, 1))
            p.setBrush(QBrush(QColor(col.red(), col.green(), col.blue(), 18)))
            p.drawRoundedRect(pill_rect, 4, 4)

            # Indicator line/dash
            p.setPen(QPen(col, 1.8, Qt.DashLine if is_dashed else Qt.SolidLine))
            p.drawLine(QPointF(pill_rect.left() + 4, pill_rect.center().y()),
                       QPointF(pill_rect.left() + 12, pill_rect.center().y()))

            p.setPen(col)
            p.drawText(QRectF(pill_rect.left() + 15, pill_rect.top(), tw + 2, pill_h),
                       Qt.AlignLeft | Qt.AlignVCenter, text)
            cur_right -= 6

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(0, 0, -1, -1)

        # Outer Background Card
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(QBrush(COL_CARD))
        p.drawRoundedRect(rect, 12, 12)

        # Main Header
        header_rect = QRectF(rect.left() + 16, rect.top() + 10, rect.width() - 32, 22)
        p.setPen(COL_TEXT)
        p.setFont(qfont(FONT_UI, 10, QFont.Bold, letter_spacing=1.2))
        p.drawText(header_rect, Qt.AlignLeft | Qt.AlignVCenter, "REAL-TIME PID TRACKING & RESPONS")

        # Live Recording Indicator on Plot Header
        if self._recording:
            rec_badge = f"\u25CF REC {self._record_time_str}".strip() if self._record_time_str else "\u25CF REC"
            p.setFont(qfont(FONT_MONO, 7.5, QFont.Bold))
            rec_fm = QFontMetricsF(p.font())
            rec_w = rec_fm.horizontalAdvance(rec_badge) + 16
            rec_rect = QRectF(header_rect.right() - rec_w, header_rect.top() + 1, rec_w, 20)
            p.setPen(QPen(QColor("#EF4444"), 1))
            p.setBrush(QBrush(QColor(239, 68, 68, 25)))
            p.drawRoundedRect(rec_rect, 4, 4)
            p.setPen(QColor("#DC2626"))
            p.drawText(rec_rect, Qt.AlignCenter, rec_badge)

        # Subplot Layout: 3 Rows
        top_y = header_rect.bottom() + 8
        avail_h = rect.bottom() - top_y - 12
        row_h = (avail_h - 20) / 3.0

        # ──────────────── SUBPLOT 1: ROLL (Setpoint vs Actual) ────────────────
        row1_rect = QRectF(rect.left() + 14, top_y, rect.width() - 28, row_h)
        err_roll = self._cur_target_roll - self._cur_actual_roll

        # Row 1 Header & Badges
        p.setFont(qfont(FONT_UI, 8, QFont.Bold))
        p.setPen(COL_ACCENT_DK)
        p.drawText(QRectF(row1_rect.left() + 44, row1_rect.top(), 220, 16),
                   Qt.AlignLeft | Qt.AlignVCenter, "1. ROLL TRACKING (\u00B0)")

        badges_roll = [
            (f"SETPOINT: {self._cur_target_roll:+.1f}\u00B0", QColor("#F59E0B"), True),
            (f"ACTUAL: {self._cur_actual_roll:+.1f}\u00B0", QColor("#2563EB"), False),
            (f"ERROR: {err_roll:+.1f}\u00B0", COL_OK if abs(err_roll) < 2.0 else (COL_WARN if abs(err_roll) < 6.0 else COL_ERR), False),
        ]
        self._draw_badges(p, row1_rect.right(), row1_rect.top() + 1, badges_roll)

        plot1_rect = QRectF(row1_rect.left() + 44, row1_rect.top() + 18, row1_rect.width() - 52, row1_rect.height() - 34)
        self._draw_subplot_grid(p, plot1_rect, [20, 10, 0, -10, -20], 25.0, "\u00B0")
        self._draw_series(p, plot1_rect, 1, QColor("#F59E0B"), 1.8, 25.0, Qt.DashLine)  # Target Roll
        self._draw_series(p, plot1_rect, 2, QColor("#2563EB"), 2.2, 25.0, Qt.SolidLine) # Actual Roll

        # ──────────────── SUBPLOT 2: PITCH (Setpoint vs Actual) ────────────────
        row2_y = row1_rect.bottom() + 10
        row2_rect = QRectF(rect.left() + 14, row2_y, rect.width() - 28, row_h)
        err_pitch = self._cur_target_pitch - self._cur_actual_pitch

        # Row 2 Header & Badges
        p.setFont(qfont(FONT_UI, 8, QFont.Bold))
        p.setPen(COL_ACCENT_DK)
        p.drawText(QRectF(row2_rect.left() + 44, row2_rect.top(), 220, 16),
                   Qt.AlignLeft | Qt.AlignVCenter, "2. PITCH TRACKING (\u00B0)")

        badges_pitch = [
            (f"SETPOINT: {self._cur_target_pitch:+.1f}\u00B0", QColor("#F59E0B"), True),
            (f"ACTUAL: {self._cur_actual_pitch:+.1f}\u00B0", QColor("#16A34A"), False),
            (f"ERROR: {err_pitch:+.1f}\u00B0", COL_OK if abs(err_pitch) < 2.0 else (COL_WARN if abs(err_pitch) < 6.0 else COL_ERR), False),
        ]
        self._draw_badges(p, row2_rect.right(), row2_rect.top() + 1, badges_pitch)

        plot2_rect = QRectF(row2_rect.left() + 44, row2_rect.top() + 18, row2_rect.width() - 52, row2_rect.height() - 34)
        self._draw_subplot_grid(p, plot2_rect, [20, 10, 0, -10, -20], 25.0, "\u00B0")
        self._draw_series(p, plot2_rect, 3, QColor("#F59E0B"), 1.8, 25.0, Qt.DashLine)  # Target Pitch
        self._draw_series(p, plot2_rect, 4, QColor("#16A34A"), 2.2, 25.0, Qt.SolidLine) # Actual Pitch

        # ──────────────── SUBPLOT 3: PWM OUTPUT CORRECTIONS ────────────────
        row3_y = row2_rect.bottom() + 10
        row3_rect = QRectF(rect.left() + 14, row3_y, rect.width() - 28, row_h)

        # Row 3 Header & Badges
        p.setFont(qfont(FONT_UI, 8, QFont.Bold))
        p.setPen(COL_ACCENT_DK)
        p.drawText(QRectF(row3_rect.left() + 44, row3_rect.top(), 220, 16),
                   Qt.AlignLeft | Qt.AlignVCenter, "3. PID OUTPUT KOREKSI (\u03BCs)")

        badges_pwm = [
            (f"uROLL: {self._cur_u_roll:+.0f}\u03BCs", QColor("#2563EB"), False),
            (f"uPITCH: {self._cur_u_pitch:+.0f}\u03BCs", QColor("#16A34A"), False),
            (f"uYAW: {self._cur_u_yaw:+.0f}\u03BCs", QColor("#8B5CF6"), False),
        ]
        self._draw_badges(p, row3_rect.right(), row3_rect.top() + 1, badges_pwm)

        plot3_rect = QRectF(row3_rect.left() + 44, row3_rect.top() + 18, row3_rect.width() - 52, row3_rect.height() - 34)
        self._draw_subplot_grid(p, plot3_rect, [150, 75, 0, -75, -150], 200.0, "")
        self._draw_series(p, plot3_rect, 5, QColor("#2563EB"), 2.0, 200.0, Qt.SolidLine) # uRoll
        self._draw_series(p, plot3_rect, 6, QColor("#16A34A"), 2.0, 200.0, Qt.SolidLine) # uPitch
        self._draw_series(p, plot3_rect, 7, QColor("#8B5CF6"), 1.8, 200.0, Qt.DashLine)  # uYaw


# ───────────────────── Metric Card ─────────────────────

class MetricCard(QFrame):
    """Kartu metrik dengan 3 zona vertikal terpisah (label + target badge / nilai / satuan + error)
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
        self._has_target = False
        self._target_val = 0.0
        self._target_unit = "°"
        self._target_label = "CMD"
        self._show_error = False
        self.setMinimumHeight(100)
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

    def set_target(self, target_val, target_unit="°", label="CMD", show_error=False):
        self._has_target = True
        self._target_val = target_val
        self._target_unit = target_unit
        self._target_label = label
        self._show_error = show_error
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = self.rect().adjusted(16, 0, -14, 0)

        # Zone 1: accent dot + label (left) & target badge (right)
        label_h = 24.0
        label_rect = QRectF(r.left(), r.top() + 2, r.width(), label_h)
        dot_r = 3.0
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(self._accent))
        p.drawEllipse(QPointF(label_rect.left() + dot_r, label_rect.center().y()), dot_r, dot_r)
        p.setPen(COL_SUBTEXT)
        p.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.4))
        p.drawText(label_rect.adjusted(dot_r * 2 + 6, 0, 0, 0),
                   Qt.AlignLeft | Qt.AlignVCenter, self._label.upper())

        # Target badge on top-right (from Remote LoRa TX)
        if self._has_target:
            if isinstance(self._target_val, float):
                tgt_str = f"{self._target_label}: {self._target_val:+.1f}{self._target_unit}"
            elif isinstance(self._target_val, int):
                tgt_str = f"{self._target_label}: {self._target_val}{self._target_unit}"
            else:
                tgt_str = f"{self._target_label}: {self._target_val}"

            badge_font = qfont(FONT_UI, 7, QFont.Bold, letter_spacing=0.5)
            badge_fm = QFontMetricsF(badge_font)
            bw = badge_fm.horizontalAdvance(tgt_str) + 12
            bh = 17.0
            bx = r.right() - bw
            by = label_rect.center().y() - bh / 2.0
            badge_rect = QRectF(bx, by, bw, bh)

            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor("#EFF6FF")))  # Soft light blue bg
            p.drawRoundedRect(badge_rect, 4, 4)
            p.setPen(QColor("#1D4ED8"))  # Deep blue text
            p.setFont(badge_font)
            p.drawText(badge_rect, Qt.AlignCenter, tgt_str)

        # Zone 2: big value (middle, generous height so glyphs never clip)
        unit_h = 22.0
        value_rect = QRectF(r.left(), r.top() + label_h,
                            r.width(), r.height() - label_h - unit_h)
        p.setPen(COL_TEXT)
        p.setFont(qfont(FONT_TITLE, 21, QFont.Bold, letter_spacing=-0.3))
        val_text = self._fmt.format(self._value)
        p.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, val_text)

        # Zone 3: unit on left + error on right (bottom strip, separated by a hairline)
        unit_rect = QRectF(r.left(), r.bottom() - unit_h, r.width(), unit_h)
        p.setPen(QPen(COL_BORDER, 1))
        p.drawLine(QPointF(unit_rect.left(), unit_rect.top()),
                   QPointF(unit_rect.right(), unit_rect.top()))
        p.setPen(COL_MUTED)
        p.setFont(qfont(FONT_UI, 7, QFont.DemiBold, letter_spacing=1.2))
        p.drawText(unit_rect.adjusted(0, 1, 0, 0),
                   Qt.AlignLeft | Qt.AlignVCenter, self._unit.upper())

        # Error display on bottom-right if enabled
        if self._has_target and self._show_error and isinstance(self._target_val, (int, float)):
            err = self._value - float(self._target_val)
            err_str = f"ERR: {err:+.1f}°"
            err_col = COL_OK if abs(err) < 2.0 else (COL_WARN if abs(err) < 6.0 else COL_ERR)
            p.setPen(err_col)
            p.setFont(qfont(FONT_UI, 7, QFont.Bold, letter_spacing=0.6))
            p.drawText(unit_rect.adjusted(0, 1, 0, 0),
                       Qt.AlignRight | Qt.AlignVCenter, err_str)

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
        self.vbat = 11.1
        self.press = 0.0
        self.rate = 0.0
        self._last_sample_ts = 0.0
        self._gyro_calib = True  # dari [FLAGS] downlink drone
        self._flags_received = False

    def update_vbat(self, vbat):
        if vbat > 3.0:
            self.vbat = vbat
        self.update()

    def update_press(self, press):
        self.press = press
        self.update()

    def update_flags(self, flags: dict):
        """Terima dict flags dari _parse_line: gyro_calib, bmi_ok, dst.
        Saat ini kolom yang ditampilkan hanya GYRO CALIB (paling relevan
        untuk debug drift IMU); flag lain disimpan untuk ekstensi UI."""
        self._gyro_calib = bool(flags.get("gyro_calib", True))
        self._flags_received = True
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

        vbat_col = COL_OK if self.vbat >= 10.5 else (COL_WARN if self.vbat >= 9.9 else COL_ERR)
        if not self._flags_received:
            gyro_txt, gyro_col = "WAIT...", COL_MUTED
        elif self._gyro_calib:
            gyro_txt, gyro_col = "OK", COL_OK
        else:
            gyro_txt, gyro_col = "FAIL", COL_ERR
        cols = [
            ("BATTERY (3S)", f"{self.vbat:.2f} V", vbat_col),
            ("PRESSURE",     f"{self.press:.1f} hPa", COL_ACCENT_2),
            ("GYRO CALIB",   gyro_txt, gyro_col),
            ("DATA RATE",    f"{self.rate:.0f} Hz",   COL_OK),
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
        self.targets = (0.0, 1200, 0.0, 0.0)
        self.calibrated = False

    def set_values(self, r, t, y, p, calibrated, targets=None):
        self.r, self.t, self.y, self.p = r, t, y, p
        self.calibrated = calibrated
        if targets is not None:
            self.targets = targets
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

            # value row with physical units from LoraTx
            value_rect = QRectF(col_rect.left(), name_rect.bottom() + 2,
                                col_rect.width(), 22)
            p.setPen(COL_TEXT)
            p.setFont(qfont(FONT_MONO, 14, QFont.Bold))
            p.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, f"{val:>3}")

            # Physical Target String (Degrees, PWM, Rate)
            target_str = ""
            if i == 0:  # Roll
                target_str = f"{self.targets[0]:+.1f}°"
            elif i == 1: # Throttle
                target_str = f"{int(self.targets[1])}us"
            elif i == 2: # Yaw
                target_str = f"{self.targets[2]:+.1f}°/s"
            elif i == 3: # Pitch
                target_str = f"{self.targets[3]:+.1f}°"

            p.setPen(QColor(col))
            p.setFont(qfont(FONT_UI, 9, QFont.Bold))
            p.drawText(value_rect, Qt.AlignRight | Qt.AlignVCenter, target_str)

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


# ───────────────────── Motor RPM Telemetry Card ─────────────────────

class MotorRpmCard(QFrame):
    """Strip telemetri & estimasi RPM 4 Motor Quad-X di Tab Attitude.
    Menampilkan header dengan status ARM/DISARM, serta 4 kolom motor (M1 FL, M2 FR, M3 BR, M4 BL)
    dengan nilai RPM real-time, PWM microseconds, persentase daya, dan progress bar dinamis."""

    MOTORS = [
        ("M1", "FL \u00B7 CW",  COL_ACCENT),      # PB6
        ("M2", "FR \u00B7 CCW", COL_ACCENT_LT),   # PB7
        ("M3", "BR \u00B7 CW",  COL_ACCENT_DK),   # PB8
        ("M4", "BL \u00B7 CCW", COL_OK),          # PB9
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("MotorRpmCard")
        self.setFixedHeight(108)
        self.setStyleSheet(f"""
            #MotorRpmCard {{
                background: {COL_CARD.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 10px;
            }}
        """)
        add_shadow(self, blur=14, dy=1, alpha=18)
        self.armed = False
        self.vbat = 11.1
        self.pwm = [1000, 1000, 1000, 1000]
        self.rpm = [0, 0, 0, 0]

    def set_data(self, ur, up, uy, throttle_pwm, armed, vbat=11.1):
        self.armed = armed
        self.vbat = vbat if vbat > 3.0 else 11.1
        base_pwm = throttle_pwm if armed else 1000.0

        if armed:
            m1 = max(1000.0, min(2000.0, base_pwm + ur + up - uy))
            m2 = max(1000.0, min(2000.0, base_pwm - ur + up + uy))
            m3 = max(1000.0, min(2000.0, base_pwm - ur - up - uy))
            m4 = max(1000.0, min(2000.0, base_pwm + ur - up + uy))
        else:
            m1 = m2 = m3 = m4 = 1000.0

        self.pwm = [int(m1), int(m2), int(m3), int(m4)]
        # A2212 1400KV * VBat * 0.75 loaded prop efficiency
        max_rpm = 1400.0 * self.vbat * 0.75
        self.rpm = [
            int(((p - 1000.0) / 1000.0) * max_rpm) if armed and p > 1020 else 0
            for p in self.pwm
        ]
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect())

        # ── Header Row ──
        header_rect = QRectF(rect.left() + 16, rect.top() + 8, rect.width() - 32, 18)
        p.setPen(COL_SUBTEXT)
        p.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.6))
        p.drawText(header_rect, Qt.AlignLeft | Qt.AlignVCenter,
                   f"MOTOR REALTIME RPM \u00B7 A2212 1400KV (3S {self.vbat:.2f}V)")

        badge_txt = "ARMED" if self.armed else "DISARMED"
        badge_bg = COL_OK if self.armed else COL_MUTED
        badge_font = qfont(FONT_UI, 7, QFont.Bold, letter_spacing=1.6)
        badge_w = QFontMetricsF(badge_font).horizontalAdvance(badge_txt) + 20
        badge_rect = QRectF(header_rect.right() - badge_w, header_rect.top() - 1, badge_w, 18)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(badge_bg))
        p.drawRoundedRect(badge_rect, 9, 9)
        p.setPen(QColor("#FFFFFF"))
        p.setFont(badge_font)
        p.drawText(badge_rect, Qt.AlignCenter, badge_txt)

        # Hairline separator
        sep_y = header_rect.bottom() + 8
        p.setPen(QPen(COL_BORDER, 1))
        p.drawLine(QPointF(rect.left() + 16, sep_y), QPointF(rect.right() - 16, sep_y))

        # ── 4 Motor Columns ──
        grid = QRectF(rect.left() + 16, sep_y + 8,
                      rect.width() - 32, rect.bottom() - (sep_y + 8) - 10)
        col_w = grid.width() / 4.0

        for i, ((code, name, col), pwm_val, rpm_val) in enumerate(zip(self.MOTORS, self.pwm, self.rpm)):
            cx0 = grid.left() + i * col_w
            col_rect = QRectF(cx0, grid.top(), col_w - 10, grid.height())

            if i > 0:
                p.setPen(QPen(COL_BORDER, 1))
                p.drawLine(QPointF(cx0 - 5, grid.top()), QPointF(cx0 - 5, grid.bottom()))

            # Motor Name: Colored dot + "M1 · FL (CW)"
            name_rect = QRectF(col_rect.left(), col_rect.top(), col_rect.width(), 16)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(col))
            p.drawEllipse(QPointF(name_rect.left() + 3, name_rect.center().y()), 3, 3)
            p.setPen(COL_SUBTEXT)
            p.setFont(qfont(FONT_UI, 8, QFont.Bold, letter_spacing=1.2))
            p.drawText(name_rect.adjusted(11, 0, 0, 0),
                       Qt.AlignLeft | Qt.AlignVCenter, f"{code} \u00B7 {name}")

            # Main RPM Value + PWM subtext
            value_rect = QRectF(col_rect.left(), name_rect.bottom() + 2, col_rect.width(), 22)
            p.setPen(COL_TEXT if self.armed and rpm_val > 0 else COL_MUTED)
            p.setFont(qfont(FONT_MONO, 14, QFont.Bold))
            rpm_str = f"{rpm_val:,} RPM" if self.armed else "0 RPM"
            p.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, rpm_str)

            # Sub-info: PWM us + %
            pct = int(((pwm_val - 1000) / 1000.0) * 100) if self.armed else 0
            pct_str = f"{pwm_val} \u00B5s ({pct}%)"
            sub_font = qfont(FONT_UI, 7, QFont.DemiBold)
            p.setFont(sub_font)
            sub_w = QFontMetricsF(sub_font).horizontalAdvance(pct_str)
            sub_rect = QRectF(col_rect.right() - sub_w, value_rect.top() + 4, sub_w, 14)
            p.setPen(COL_SUBTEXT)
            p.drawText(sub_rect, Qt.AlignRight | Qt.AlignVCenter, pct_str)

            # Bar RPM / Output (0 - 100%)
            bar_rect = QRectF(col_rect.left(), value_rect.bottom() + 4, col_rect.width(), 6)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(COL_CARD_ALT))
            p.drawRoundedRect(bar_rect, 3, 3)

            frac = max(0.0, min(1.0, (pwm_val - 1000) / 1000.0)) if self.armed else 0.0
            if frac > 0:
                fill_w = bar_rect.width() * frac
                fill_col = COL_WARN if frac > 0.85 else QColor(col)
                p.setBrush(QBrush(fill_col))
                p.drawRoundedRect(QRectF(bar_rect.left(), bar_rect.top(),
                                         max(4.0, fill_w), bar_rect.height()), 3, 3)

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
        # Mixer Quad-X (sinkron dengan writeSmcMotorMix di motors.cpp)
        self.mix = {
            "M1":  ur + up - uy,   # FL CW
            "M2": -ur + up + uy,   # FR CCW
            "M3": -ur - up - uy,   # BR CW
            "M4":  ur - up + uy,   # BL CCW
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
        self._vbat = 11.1
        self._first_alt = True

        # ── Drone health flags (dari [FLAGS] tag downlink) ──
        # Default optimistic supaya tidak flicker warning saat baru konek
        # sebelum paket pertama tiba.
        self._flags = {
            "gyro_calib": True,   # gyroCalibValid
            "bmi_ok":     True,
            "bmp_ok":     True,
            "vbat_ok":    True,
            "batt_stage": 0,      # 0=OK 1=WARN 2=LIMIT 3=CRITICAL
            "fs_stage":   0,      # 0=OK 2=DESCENT 3=LAND
        }
        self._flags_received = False  # true setelah paket [FLAGS] pertama

        # ── joystick state ──
        # Kalibrasi dilakukan dan disimpan di ESP32. GUI hanya memicu "CAL
        # SAMPLE" lalu membaca balasan "[CAL] OK CR=.. CT=.. CY=.. CP=..".
        self._js_raw = (128, 128, 128, 128)
        self._js_target = (0.0, 1200, 0.0, 0.0)  # (roll_deg, throttle_pwm, yaw_dps, pitch_deg)
        self._js_calibrated = False
        self._js_calib_center_display = None  # (cr,ct,cy,cp) ADC untuk banner
        self._js_calib_sampling = False       # menunggu balasan dari ESP32
        self._armed = False

        # ── throttle integrator state (spring-centered stick) ──
        self._throttle_smoothed = 1200.0
        self._throttle_last_ms = time.monotonic() * 1000.0

        # ── bench-test live state (SMC + IMU) ──
        self._bench_smc = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # (roll, pitch, yaw, uRoll, uPitch, uYaw)
        self._bench_imu = (0.0, 0.0, 0.0)           # (gx, gy, gz) deg/s
        self._bench_alt_press = (0.0, 0.0)          # (alt, press)

        # ── Telemetry & PID CSV Recorder state ──
        self._is_recording = False
        self._record_file = None
        self._record_writer = None
        self._record_filepath = ""
        self._record_start_time = 0.0
        self._record_sample_count = 0
        self._record_timer = QTimer(self)
        self._record_timer.timeout.connect(self._update_record_ui)

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
        self._stack.addWidget(self._build_bench_tab())     # 2
        self._stack.addWidget(self._build_pid_tab())       # 3
        body_lay.addWidget(self._stack, stretch=1)

        # Muat konfigurasi parameter PID terakhir yang tersimpan
        self._load_pid_config()

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

        elif icon_type == "PID":
            # ── PID Tuning / Waveform Icon ──
            p.setPen(QPen(COL_ACCENT_DK, 1.2))
            p.setBrush(QBrush(QColor("#E0F2FE")))
            p.drawRoundedRect(QRectF(center - r, center - r, 2 * r, 2 * r), 4, 4)

            p.setPen(QPen(COL_DIVIDER, 1, Qt.DashLine))
            p.drawLine(QPointF(center - r + 2, center), QPointF(center + r - 2, center))

            path = QPainterPath()
            path.moveTo(center - r + 3, center + 2)
            path.cubicTo(center - r * 0.3, center - r * 0.7,
                         center + r * 0.3, center + r * 0.7,
                         center + r - 3, center - 2)
            p.setPen(QPen(COL_ACCENT, 1.5))
            p.drawPath(path)

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
            ("BENCH TEST", 2, "BENCH TEST", self._create_nav_icon("BENCH")),
            ("PID TUNING", 3, "PID TUNING", self._create_nav_icon("PID")),
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
            esc_arm = self._pid_inputs["esc_arm_spin_pwm"].value() if hasattr(self, "_pid_inputs") and "esc_arm_spin_pwm" in self._pid_inputs else 1200.0
            self._throttle_smoothed = esc_arm
            self._throttle_last_ms = time.monotonic() * 1000.0
            self.set_hint("Drone ARMING / ARMED (Motors Live)", level="warn")
        else:
            esc_min = self._pid_inputs["esc_min_pwm"].value() if hasattr(self, "_pid_inputs") and "esc_min_pwm" in self._pid_inputs else 1000.0
            self._throttle_smoothed = esc_min
            self.set_hint("Drone DISARMED (Safe)", level="ok")

    def _update_gui_throttle(self):
        # Preview collective only. Firmware receives the self-centering stick
        # command and applies its own BMI160 vertical damping.
        esc_min = self._pid_inputs["esc_min_pwm"].value() if hasattr(self, "_pid_inputs") and "esc_min_pwm" in self._pid_inputs else 1000.0
        esc_arm = self._pid_inputs["esc_arm_spin_pwm"].value() if hasattr(self, "_pid_inputs") and "esc_arm_spin_pwm" in self._pid_inputs else 1200.0
        esc_max = self._pid_inputs["esc_max_pwm"].value() if hasattr(self, "_pid_inputs") and "esc_max_pwm" in self._pid_inputs else 1300.0

        if not self._armed:
            self._throttle_smoothed = esc_min
            return self._throttle_smoothed

        t_raw = self._js_raw[1] if len(self._js_raw) > 1 else 128
        if t_raw > 128 + 6:
            stick_norm = (t_raw - 134.0) / 121.0
            stick_norm = max(0.0, min(1.0, stick_norm))
            target = esc_arm + stick_norm * (esc_max - esc_arm)
        elif t_raw < 128 - 6:
            stick_down_norm = (122.0 - t_raw) / 122.0
            stick_down_norm = max(0.0, min(1.0, stick_down_norm))
            target = esc_arm - stick_down_norm * (esc_arm - esc_min)
        else:
            target = esc_arm

        # Ramp halus + clamp aman
        self._throttle_smoothed = 0.90 * self._throttle_smoothed + 0.10 * target
        self._throttle_smoothed = max(esc_min, min(esc_max, self._throttle_smoothed))

        return self._throttle_smoothed

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

        self._motor_rpm_card = MotorRpmCard()
        lay.addWidget(self._motor_rpm_card)

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

        self._joy_left = JoystickWidget("LEFT STICK", mode="CUSTOM",
                                        vx_label="ROLL", vy_label="THROTTLE")
        self._joy_right = JoystickWidget("RIGHT STICK", mode="CUSTOM",
                                         vx_label="YAW", vy_label="PITCH")
        add_shadow(self._joy_left, blur=22, dy=3, alpha=22)
        add_shadow(self._joy_right, blur=22, dy=3, alpha=22)
        sticks_row.addWidget(self._joy_left, stretch=1)
        sticks_row.addWidget(self._joy_right, stretch=1)
        lay.addLayout(sticks_row, stretch=1)

        self._rc_readout = RCReadoutCard()
        lay.addWidget(self._rc_readout)

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
            "Miringkan drone ke KANAN (Roll > +5\u00B0) \u2014 motor KANAN (M2, M3) harus bertambah tenaga dan motor KIRI (M1, M4) berkurang (uRoll bernilai negatif).")
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
        # Gunakan roll/pitch/yaw ONBOARD drone (dari paket SMC) karena itulah yang
        # benar-benar dipakai controller, bukan filter ulang di GUI.
        roll = self._bench_smc[0]
        pitch = self._bench_smc[1]
        yaw = self._bench_smc[2]
        ur, up, uy = self._bench_smc[3], self._bench_smc[4], self._bench_smc[5]
        gz = self._bench_imu[2]

        # Ramping throttle dari stik berpegas
        throttle = self._update_gui_throttle()

        self._bench_mix.set_data(roll, pitch, yaw,
                                 ur, up, uy, throttle, self._armed)

        # ── Roll check ──
        if abs(roll) < 3.0:
            self._check_roll.set_status("IDLE",
                "Drone level. Miringkan drone ke kanan atau kiri untuk menguji.",
                f"ROLL {roll:+.1f}\u00B0   \u00B7   uROLL {ur:+.1f}")
        else:
            # roll>0 (kanan) => koreksi harus negatif (naikkan kanan M2,M3 / turunkan kiri M1,M4): ur<0
            if (roll > 0 and ur < 0) or (roll < 0 and ur > 0):
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

    # ────────── PID Tuning tab ──────────

    def _build_pid_tab(self):
        page = QWidget()
        page.setStyleSheet("background: transparent;")
        root = QHBoxLayout(page)
        root.setContentsMargins(0, 10, 0, 0)
        root.setSpacing(14)

        # ── Kolom Kiri: Form Parameter Tuning PID ──
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("background: transparent;")

        form_card = QFrame()
        form_card.setObjectName("PidFormCard")
        form_card.setStyleSheet(f"""
            #PidFormCard {{
                background: {COL_CARD.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 12px;
            }}
        """)
        add_shadow(form_card, blur=18, dy=2, alpha=20)

        form_vbox = QVBoxLayout(form_card)
        form_vbox.setContentsMargins(18, 16, 18, 16)
        form_vbox.setSpacing(12)

        # Header Title
        title_lbl = QLabel("CASCADE PID (ROLL, PITCH & YAW)")
        title_lbl.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:800; font-size:13px; letter-spacing:1.5px;")
        form_vbox.addWidget(title_lbl)

        sub_lbl = QLabel("Tuning inner/outer loop Roll & Pitch serta inner rate PID Yaw")
        sub_lbl.setStyleSheet(f"color:{COL_SUBTEXT.name()}; font-size:10px;")
        form_vbox.addWidget(sub_lbl)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {COL_BORDER.name()};")
        form_vbox.addWidget(sep)

        self._pid_inputs = {}

        # 1. Outer Loop (Angle PID)
        angle_group = QLabel("1. OUTER LOOP — SUDUT SIKAP ROLL & PITCH (ANGLE)")
        angle_group.setStyleSheet(f"color:{COL_ACCENT_DK.name()}; font-weight:700; font-size:11px; letter-spacing:1.0px; margin-top:4px;")
        form_vbox.addWidget(angle_group)

        angle_form = QFormLayout()
        angle_form.setSpacing(8)
        angle_fields = [
            ("Angle Kp", "angle_kp", 5.00000, 0.0, 30.0, 0.00100, 5),
            ("Angle Ki", "angle_ki", 0.05000, 0.0, 5.0, 0.00010, 5),
            ("Angle Kd", "angle_kd", 0.12000, 0.0, 5.0, 0.00010, 5),
        ]
        for label, key, val, mn, mx, step, dec in angle_fields:
            spin = self._make_pid_spinbox(val, mn, mx, step, dec)
            self._pid_inputs[key] = spin
            lbl_w = QLabel(f"{label}:")
            lbl_w.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:600; font-size:11px;")
            angle_form.addRow(lbl_w, spin)
        form_vbox.addLayout(angle_form)

        # 2. Inner Loop (Rate PID)
        rate_group = QLabel("2. INNER LOOP — KECEPATAN SUDUT ROLL & PITCH (RATE)")
        rate_group.setStyleSheet(f"color:{COL_ACCENT_DK.name()}; font-weight:700; font-size:11px; letter-spacing:1.0px; margin-top:6px;")
        form_vbox.addWidget(rate_group)

        rate_form = QFormLayout()
        rate_form.setSpacing(8)
        rate_fields = [
            ("Rate Kp", "rate_kp", 1.60000, 0.0, 10.0, 0.00100, 5),
            ("Rate Ki", "rate_ki", 0.30000, 0.0, 5.0, 0.00100, 5),
            ("Rate Kd", "rate_kd", 0.04500, 0.0, 1.0, 0.00005, 5),
        ]
        for label, key, val, mn, mx, step, dec in rate_fields:
            spin = self._make_pid_spinbox(val, mn, mx, step, dec)
            self._pid_inputs[key] = spin
            lbl_w = QLabel(f"{label}:")
            lbl_w.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:600; font-size:11px;")
            rate_form.addRow(lbl_w, spin)
        form_vbox.addLayout(rate_form)

        # 3. Yaw Inner Loop (Rate PID)
        yaw_group = QLabel("3. YAW RATE PID (KECEPATAN PUTAR)")
        yaw_group.setStyleSheet(f"color:{COL_ACCENT_DK.name()}; font-weight:700; font-size:11px; letter-spacing:1.0px; margin-top:6px;")
        form_vbox.addWidget(yaw_group)

        yaw_form = QFormLayout()
        yaw_form.setSpacing(8)
        yaw_fields = [
            ("Yaw Kp", "yaw_kp", 2.00000, 0.0, 10.0, 0.00100, 5),
            ("Yaw Ki", "yaw_ki", 0.15000, 0.0, 5.0, 0.00100, 5),
            ("Yaw Kd", "yaw_kd", 0.00000, 0.0, 1.0, 0.00005, 5),
        ]
        for label, key, val, mn, mx, step, dec in yaw_fields:
            spin = self._make_pid_spinbox(val, mn, mx, step, dec)
            self._pid_inputs[key] = spin
            lbl_w = QLabel(f"{label}:")
            lbl_w.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:600; font-size:11px;")
            yaw_form.addRow(lbl_w, spin)
        form_vbox.addLayout(yaw_form)

        # 4. ESC PWM Limits
        esc_group = QLabel("4. BATASAN ESC PWM (MIN, IDLE & MAX OUTPUT)")
        esc_group.setStyleSheet(f"color:{COL_ACCENT_DK.name()}; font-weight:700; font-size:11px; letter-spacing:1.0px; margin-top:6px;")
        form_vbox.addWidget(esc_group)

        esc_form = QFormLayout()
        esc_form.setSpacing(8)
        esc_fields = [
            ("Min ESC (Stop / Disarm)", "esc_min_pwm", 1000.0, 900.0, 1300.0, 10.0, 0, " \u03BCs"),
            ("Idle / Arm Spin (20%)", "esc_arm_spin_pwm", 1200.0, 1000.0, 1500.0, 10.0, 0, " \u03BCs"),
            ("Max ESC PWM (100%)", "esc_max_pwm", 1300.0, 1100.0, 2000.0, 10.0, 0, " \u03BCs"),
        ]
        for label, key, val, mn, mx, step, dec, suffix in esc_fields:
            spin = self._make_pid_spinbox(val, mn, mx, step, dec, suffix=suffix)
            self._pid_inputs[key] = spin
            lbl_w = QLabel(f"{label}:")
            lbl_w.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:600; font-size:11px;")
            esc_form.addRow(lbl_w, spin)
        form_vbox.addLayout(esc_form)

        # 5. Vertical Hover
        hover_group = QLabel("5. VERTICAL HOVER THROTTLE")
        hover_group.setStyleSheet(f"color:{COL_ACCENT_DK.name()}; font-weight:700; font-size:11px; letter-spacing:1.0px; margin-top:6px;")
        form_vbox.addWidget(hover_group)

        hover_form = QFormLayout()
        hover_form.setSpacing(8)
        hover_spin = self._make_pid_spinbox(1260.0, 1000.0, 2000.0, 5.0, 0, suffix=" us")
        self._pid_inputs["hover_throttle_pwm"] = hover_spin
        hover_label = QLabel("Hover Throttle (Stick Center):")
        hover_label.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:600; font-size:11px;")
        hover_form.addRow(hover_label, hover_spin)
        form_vbox.addLayout(hover_form)

        # 6. Limits & Safety
        limit_group = QLabel("6. BATASAN SAFETY & DEFLEKSI")
        limit_group.setStyleSheet(f"color:{COL_ACCENT_DK.name()}; font-weight:700; font-size:11px; letter-spacing:1.0px; margin-top:6px;")
        form_vbox.addWidget(limit_group)

        limit_form = QFormLayout()
        limit_form.setSpacing(8)
        limit_fields = [
            ("Max Delta PWM", "max_delta_pwm", 300.0, 50.0, 500.0, 10.0, 0, " \u03BCs"),
            ("Max Angle", "max_angle", 25.0, 5.0, 45.0, 1.0, 1, " \u00B0"),
        ]
        for label, key, val, mn, mx, step, dec, suffix in limit_fields:
            spin = self._make_pid_spinbox(val, mn, mx, step, dec, suffix=suffix)
            self._pid_inputs[key] = spin
            lbl_w = QLabel(f"{label}:")
            lbl_w.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:600; font-size:11px;")
            limit_form.addRow(lbl_w, spin)
        form_vbox.addLayout(limit_form)

        # Action Buttons
        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_row.setContentsMargins(0, 8, 0, 0)

        self._pid_apply_btn = QPushButton("\u2713  APPLY PID PARAMETERS")
        self._pid_apply_btn.setFixedHeight(34)
        self._pid_apply_btn.setCursor(Qt.PointingHandCursor)
        self._pid_apply_btn.setStyleSheet(f"""
            QPushButton {{
                background: {COL_ACCENT.name()}; color: white;
                border: none; border-radius: 7px;
                font: bold 10px Inter, sans-serif; letter-spacing: 1.0px;
            }}
            QPushButton:hover {{
                background: {COL_ACCENT_DK.name()};
            }}
        """)
        self._pid_apply_btn.clicked.connect(self._apply_pid_parameters)
        btn_row.addWidget(self._pid_apply_btn, 2)

        self._pid_reset_btn = QPushButton("\u21BA  DEFAULT")
        self._pid_reset_btn.setFixedHeight(34)
        self._pid_reset_btn.setCursor(Qt.PointingHandCursor)
        self._pid_reset_btn.setStyleSheet(f"""
            QPushButton {{
                background: {COL_CARD_ALT.name()}; color: {COL_TEXT.name()};
                border: 1px solid {COL_BORDER.name()}; border-radius: 7px;
                font: bold 10px Inter, sans-serif; letter-spacing: 0.8px;
            }}
            QPushButton:hover {{
                background: #E2E8F0;
            }}
        """)
        self._pid_reset_btn.clicked.connect(self._restore_pid_defaults)
        btn_row.addWidget(self._pid_reset_btn, 1)

        form_vbox.addLayout(btn_row)

        scroll.setWidget(form_card)
        root.addWidget(scroll, 4)

        # ── Kolom Kanan: Real-Time Waveform Plot + Telemetry CSV Recorder + Guide ──
        right_col = QVBoxLayout()
        right_col.setSpacing(10)

        # ── Card Kontrol Recording Telemetri & PID ke CSV ──
        rec_card = QFrame()
        rec_card.setObjectName("PidRecordCard")
        rec_card.setStyleSheet(f"""
            #PidRecordCard {{
                background: {COL_CARD.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 12px;
            }}
        """)
        add_shadow(rec_card, blur=14, dy=1, alpha=16)

        rec_layout = QVBoxLayout(rec_card)
        rec_layout.setContentsMargins(14, 10, 14, 10)
        rec_layout.setSpacing(8)

        rec_top_row = QHBoxLayout()
        rec_top_row.setSpacing(10)

        rec_title_box = QVBoxLayout()
        rec_title_box.setSpacing(2)

        rec_title_lbl = QLabel("RECORD TELEMETRI ATTITUDE & PID KE CSV")
        rec_title_lbl.setStyleSheet(f"color: {COL_TEXT.name()}; font-weight: 800; font-size: 11px; letter-spacing: 1.2px;")
        rec_title_box.addWidget(rec_title_lbl)

        rec_sub_lbl = QLabel(" ")
        rec_sub_lbl.setStyleSheet(f"color: {COL_SUBTEXT.name()}; font-size: 9.5px;")
        rec_title_box.addWidget(rec_sub_lbl)

        rec_top_row.addLayout(rec_title_box, 1)

        self._rec_btn = QPushButton("⏺  START RECORDING")
        self._rec_btn.setFixedHeight(32)
        self._rec_btn.setCursor(Qt.PointingHandCursor)
        self._rec_btn.setStyleSheet(f"""
            QPushButton {{
                background: {COL_ACCENT.name()}; color: white;
                border: none; border-radius: 7px;
                padding: 0 16px;
                font: bold 10px Inter, sans-serif; letter-spacing: 0.8px;
            }}
            QPushButton:hover {{
                background: {COL_ACCENT_DK.name()};
            }}
        """)
        self._rec_btn.clicked.connect(self._toggle_recording)
        rec_top_row.addWidget(self._rec_btn)

        self._rec_open_btn = QPushButton("📁  BUKA FOLDER")
        self._rec_open_btn.setFixedHeight(32)
        self._rec_open_btn.setCursor(Qt.PointingHandCursor)
        self._rec_open_btn.setStyleSheet(f"""
            QPushButton {{
                background: {COL_CARD_ALT.name()}; color: {COL_TEXT.name()};
                border: 1px solid {COL_BORDER.name()}; border-radius: 7px;
                padding: 0 12px;
                font: bold 10px Inter, sans-serif; letter-spacing: 0.6px;
            }}
            QPushButton:hover {{
                background: #E2E8F0;
            }}
        """)
        self._rec_open_btn.clicked.connect(self._open_records_folder)
        rec_top_row.addWidget(self._rec_open_btn)

        rec_layout.addLayout(rec_top_row)

        self._rec_status_lbl = QLabel("● IDLE — Siap merekam telemetri penerbangan")
        self._rec_status_lbl.setStyleSheet(f"""
            color: {COL_SUBTEXT.name()};
            font-size: 10px;
            font-family: {CSS_MONO};
            font-weight: 600;
            padding: 5px 10px;
            background: {COL_CARD_ALT.name()};
            border-radius: 6px;
            border: 1px solid {COL_BORDER.name()};
        """)
        rec_layout.addWidget(self._rec_status_lbl)

        right_col.addWidget(rec_card, 0)

        self._pid_plot = PidPlotWidget()
        add_shadow(self._pid_plot, blur=18, dy=2, alpha=20)
        right_col.addWidget(self._pid_plot, 5)

        guide_card = QFrame()
        guide_card.setStyleSheet(f"""
            background: {COL_CARD.name()};
            border: 1px solid {COL_BORDER.name()};
            border-radius: 12px;
            padding: 14px;
        """)
        add_shadow(guide_card, blur=14, dy=1, alpha=16)

        guide_lay = QVBoxLayout(guide_card)
        guide_lay.setContentsMargins(10, 8, 10, 8)
        guide_lay.setSpacing(6)

        g_title = QLabel("PANDUAN TUNING PID (ROLL, PITCH & YAW)")
        g_title.setStyleSheet(f"color:{COL_TEXT.name()}; font-weight:800; font-size:11px; letter-spacing:1.2px;")
        guide_lay.addWidget(g_title)

        g_text = QLabel(
            "\u2022 Outer Angle Kp: Mengatur kecepatan respons drone kembali tegak saat stik Roll/Pitch dilepas. Jika berosilasi lambat (wobble), turunkan.\n"
            "\u2022 Outer Angle Ki & Kd: Ki mengoreksi offset kemiringan konstan. Kd meredam overshoot saat mendekati target sudut datar (0\u00B0).\n"
            "\u2022 Inner Rate Kp & Kd: Rate Kp memberi kekakuan terhadap hembusan angin. Rate Kd meredam osilasi frekuensi tinggi dan getaran motor.\n"
            "\u2022 Inner Rate Ki: Menahan drone dari drift sikap ketika pusat massa (baterai/frame) tidak seimbang sempurna.\n"
            "\u2022 Batasan ESC PWM: Min PWM (1000\u03BCs Stop/Disarm), Arm Spin (1200\u03BCs Idle saat Armed 20%), dan Max PWM (1300\u03BCs Batas Tenaga 100%) dapat disetel langsung dari GUI tanpa hardcode program.\n"
            "\u2022 Lakukan uji tanpa baling-baling (props-off) di tab BENCH TEST terlebih dahulu sebelum uji terbang hover perdana!"
        )
        g_text.setWordWrap(True)
        g_text.setStyleSheet(f"color:{COL_SUBTEXT.name()}; font-size:10px; line-height:1.4;")
        guide_lay.addWidget(g_text)

        right_col.addWidget(guide_card, 2)
        root.addLayout(right_col, 5)

        return page

    def _make_pid_spinbox(self, value, min_val, max_val, step, decimals, suffix=""):
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        if suffix:
            spin.setSuffix(suffix)
        spin.setStyleSheet(f"""
            QDoubleSpinBox {{
                background: {COL_CARD_ALT.name()};
                color: {COL_TEXT.name()};
                border: 1px solid {COL_BORDER.name()};
                border-radius: 6px;
                padding: 4px 8px;
                font-family: {CSS_MONO};
                font-size: 11px;
                font-weight: bold;
            }}
            QDoubleSpinBox:focus {{
                border: 1.5px solid {COL_ACCENT.name()};
            }}
        """)
        return spin

    def _save_pid_config(self):
        if not hasattr(self, "_pid_inputs") or not self._pid_inputs:
            return False
        try:
            data = {k: spin.value() for k, spin in self._pid_inputs.items()}
            with open(PID_CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4)
            return True
        except Exception as e:
            print(f"[WARN] Gagal menyimpan konfigurasi PID ke {PID_CONFIG_FILE}: {e}")
            return False

    def _load_pid_config(self):
        if not hasattr(self, "_pid_inputs") or not self._pid_inputs:
            return
        if not os.path.exists(PID_CONFIG_FILE):
            return
        try:
            with open(PID_CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                for k, v in data.items():
                    if k in self._pid_inputs and isinstance(v, (int, float)):
                        self._pid_inputs[k].setValue(float(v))
        except Exception as e:
            print(f"[WARN] Gagal memuat konfigurasi PID dari {PID_CONFIG_FILE}: {e}")

    def _apply_pid_parameters(self):
        # Simpan konfigurasi ke file json saat Apply diklik
        saved = self._save_pid_config()

        if not self._serial or not self._serial.is_open:
            if saved:
                self.set_hint("Parameter PID disimpan ke config (Hubungkan serial untuk kirim ke Drone)", level="ok")
            else:
                self.set_hint("Hubungkan ke Remote ESP32 sebelum mengirim parameter PID!", level="warn")
            return
        inputs = self._pid_inputs
        esc_min = inputs['esc_min_pwm'].value() if 'esc_min_pwm' in inputs else 1000.0
        esc_arm = inputs['esc_arm_spin_pwm'].value() if 'esc_arm_spin_pwm' in inputs else 1200.0
        esc_max = inputs['esc_max_pwm'].value() if 'esc_max_pwm' in inputs else 1300.0
        hover = inputs['hover_throttle_pwm'].value() if 'hover_throttle_pwm' in inputs else 1260.0
        hover = max(esc_arm, min(hover, esc_max))
        if 'hover_throttle_pwm' in inputs:
            inputs['hover_throttle_pwm'].setValue(hover)
        # Format: PID <angleKp> <angleKi> <angleKd> <rateKp> <rateKi> <rateKd> <yawKp> <yawKi> <yawKd> <maxAngle> <maxYawRate> <maxDeltaPwm> <escMinPwm> <escArmSpinPwm> <escMaxPwm> <hoverThrottlePwm>
        cmd = (
            f"PID {inputs['angle_kp'].value():.5f} {inputs['angle_ki'].value():.5f} {inputs['angle_kd'].value():.5f} "
            f"{inputs['rate_kp'].value():.5f} {inputs['rate_ki'].value():.5f} {inputs['rate_kd'].value():.5f} "
            f"{inputs['yaw_kp'].value():.5f} {inputs['yaw_ki'].value():.5f} {inputs['yaw_kd'].value():.5f} "
            f"{inputs['max_angle'].value():.1f} 150.0 {inputs['max_delta_pwm'].value():.1f} "
            f"{esc_min:.1f} {esc_arm:.1f} {esc_max:.1f} {hover:.1f}\n"
        )
        try:
            self._serial.write(cmd.encode("ascii"))
            self._serial.flush()
            self.set_hint("Parameter PID tersimpan & berhasil dikirim ke Remote & Drone via LoRa \u2713", level="ok")
        except Exception as e:
            self.set_hint(f"Parameter PID tersimpan, namun gagal kirim via serial: {e}", level="err")

    def _restore_pid_defaults(self):
        defaults = {
            "angle_kp": 5.00000, "angle_ki": 0.05000, "angle_kd": 0.12000,
            "rate_kp": 1.60000, "rate_ki": 0.30000, "rate_kd": 0.04500,
            "yaw_kp": 2.00000, "yaw_ki": 0.15000, "yaw_kd": 0.00000,
            "esc_min_pwm": 1000.0, "esc_arm_spin_pwm": 1200.0, "esc_max_pwm": 1300.0,
            "hover_throttle_pwm": 1260.0,
            "max_angle": 25.0, "max_delta_pwm": 300.0
        }
        for k, v in defaults.items():
            if k in self._pid_inputs:
                self._pid_inputs[k].setValue(v)
        self.set_hint("Parameter PID & Limit ESC PWM dikembalikan ke nilai default.", level="ok")

    # ────────── CSV Recording (Telemetry & PID) ──────────

    def _toggle_recording(self):
        if getattr(self, "_is_recording", False):
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        try:
            os.makedirs(RECORDS_DIR, exist_ok=True)
            timestamp_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._record_filepath = os.path.join(RECORDS_DIR, f"pid_record_{timestamp_tag}.csv")
            self._record_file = open(self._record_filepath, "w", newline="", encoding="utf-8")
            self._record_writer = csv.writer(self._record_file)

            headers = [
                "timestamp", "elapsed_sec",
                "roll_actual_deg", "roll_target_deg", "roll_error_deg",
                "pitch_actual_deg", "pitch_target_deg", "pitch_error_deg",
                "yaw_actual_deg", "yaw_target_dps",
                "u_roll", "u_pitch", "u_yaw",
                "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
                "altitude_m", "vbat_v", "armed",
                "angle_kp", "angle_ki", "angle_kd",
                "rate_kp", "rate_ki", "rate_kd",
                "yaw_kp", "yaw_ki", "yaw_kd"
            ]
            self._record_writer.writerow(headers)
            self._record_file.flush()

            self._is_recording = True
            self._record_start_time = time.time()
            self._record_sample_count = 0
            self._record_timer.start(200)

            if hasattr(self, "_rec_btn"):
                self._rec_btn.setText("⏹  STOP RECORDING")
                self._rec_btn.setStyleSheet(f"""
                    QPushButton {{
                        background: {COL_ERR.name()}; color: white;
                        border: none; border-radius: 7px;
                        padding: 0 16px;
                        font: bold 10px Inter, sans-serif; letter-spacing: 0.8px;
                    }}
                    QPushButton:hover {{
                        background: #DC2626;
                    }}
                """)

            fname = os.path.basename(self._record_filepath)
            if hasattr(self, "_rec_status_lbl"):
                self._rec_status_lbl.setText(f"● RECORDING [00:00:00] · 0 baris · {fname}")
                self._rec_status_lbl.setStyleSheet(f"""
                    color: {COL_ERR.name()};
                    font-size: 10px;
                    font-family: {CSS_MONO};
                    font-weight: 700;
                    padding: 5px 10px;
                    background: #FEF2F2;
                    border-radius: 6px;
                    border: 1px solid #FECACA;
                """)

            if hasattr(self, "_pid_plot"):
                self._pid_plot.set_recording_status(True, "00:00:00")

            self.set_hint(f"Mulai merekam telemetri ke records/{fname}...", level="ok")
        except Exception as e:
            self._is_recording = False
            self.set_hint(f"Gagal memulai recording: {e}", level="err")

    def _record_csv_sample(self, tgt_roll, roll, tgt_pitch, pitch, yaw, u_roll, u_pitch, u_yaw):
        if not getattr(self, "_is_recording", False) or not self._record_writer:
            return
        try:
            now_ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            elapsed = time.time() - self._record_start_time
            err_roll = tgt_roll - roll
            err_pitch = tgt_pitch - pitch
            tgt_yaw = self._js_target[2] if hasattr(self, "_js_target") else 0.0

            gx, gy, gz = self._bench_imu if hasattr(self, "_bench_imu") else (0.0, 0.0, 0.0)
            alt = self._alt_smooth if hasattr(self, "_alt_smooth") else 0.0
            vbat = self._vbat if hasattr(self, "_vbat") else 0.0
            armed = 1 if getattr(self, "_armed", False) else 0

            p = getattr(self, "_pid_inputs", {})
            a_kp = p['angle_kp'].value() if 'angle_kp' in p else 0.0
            a_ki = p['angle_ki'].value() if 'angle_ki' in p else 0.0
            a_kd = p['angle_kd'].value() if 'angle_kd' in p else 0.0
            r_kp = p['rate_kp'].value() if 'rate_kp' in p else 0.0
            r_ki = p['rate_ki'].value() if 'rate_ki' in p else 0.0
            r_kd = p['rate_kd'].value() if 'rate_kd' in p else 0.0
            y_kp = p['yaw_kp'].value() if 'yaw_kp' in p else 0.0
            y_ki = p['yaw_ki'].value() if 'yaw_ki' in p else 0.0
            y_kd = p['yaw_kd'].value() if 'yaw_kd' in p else 0.0

            row = [
                now_ts,
                f"{elapsed:.3f}",
                f"{roll:.2f}", f"{tgt_roll:.2f}", f"{err_roll:.2f}",
                f"{pitch:.2f}", f"{tgt_pitch:.2f}", f"{err_pitch:.2f}",
                f"{yaw:.2f}", f"{tgt_yaw:.2f}",
                f"{u_roll:.2f}", f"{u_pitch:.2f}", f"{u_yaw:.2f}",
                f"{gx:.2f}", f"{gy:.2f}", f"{gz:.2f}",
                f"{alt:.2f}", f"{vbat:.2f}", armed,
                f"{a_kp:.5f}", f"{a_ki:.5f}", f"{a_kd:.5f}",
                f"{r_kp:.5f}", f"{r_ki:.5f}", f"{r_kd:.5f}",
                f"{y_kp:.5f}", f"{y_ki:.5f}", f"{y_kd:.5f}"
            ]
            self._record_writer.writerow(row)
            self._record_sample_count += 1
            if self._record_sample_count % 10 == 0:
                self._record_file.flush()
        except Exception as e:
            print(f"[WARN] Error menulis sample CSV: {e}")

    def _stop_recording(self):
        if not getattr(self, "_is_recording", False):
            return
        self._is_recording = False
        self._record_timer.stop()

        if self._record_file:
            try:
                self._record_file.flush()
                self._record_file.close()
            except Exception:
                pass
            self._record_file = None
            self._record_writer = None

        if hasattr(self, "_pid_plot"):
            self._pid_plot.set_recording_status(False)

        if hasattr(self, "_rec_btn"):
            self._rec_btn.setText("⏺  START RECORDING")
            self._rec_btn.setStyleSheet(f"""
                QPushButton {{
                    background: {COL_ACCENT.name()}; color: white;
                    border: none; border-radius: 7px;
                    padding: 0 16px;
                    font: bold 10px Inter, sans-serif; letter-spacing: 0.8px;
                }}
                QPushButton:hover {{
                    background: {COL_ACCENT_DK.name()};
                }}
            """)

        duration_sec = int(time.time() - self._record_start_time)
        m, s = divmod(duration_sec, 60)
        h, m = divmod(m, 60)
        dur_str = f"{h:02d}:{m:02d}:{s:02d}"
        fname = os.path.basename(self._record_filepath)

        if hasattr(self, "_rec_status_lbl"):
            self._rec_status_lbl.setText(f"\u2713 TERSIMPAN — Total {self._record_sample_count:,} baris ({dur_str}) di {fname}")
            self._rec_status_lbl.setStyleSheet(f"""
                color: {COL_OK.name()};
                font-size: 10px;
                font-family: {CSS_MONO};
                font-weight: 700;
                padding: 5px 10px;
                background: #F0FDF4;
                border-radius: 6px;
                border: 1px solid #BBF7D0;
            """)

        self.set_hint(f"Perekaman selesai! {self._record_sample_count:,} baris tersimpan ke records/{fname}", level="ok")

    def _update_record_ui(self):
        if not getattr(self, "_is_recording", False):
            return
        elapsed_int = int(time.time() - self._record_start_time)
        m, s = divmod(elapsed_int, 60)
        h, m = divmod(m, 60)
        dur_str = f"{h:02d}:{m:02d}:{s:02d}"
        fname = os.path.basename(self._record_filepath)

        if hasattr(self, "_rec_status_lbl"):
            self._rec_status_lbl.setText(f"● RECORDING [{dur_str}] · {self._record_sample_count:,} baris data · {fname}")

        if hasattr(self, "_pid_plot"):
            self._pid_plot.set_recording_status(True, dur_str)

    def _open_records_folder(self):
        try:
            os.makedirs(RECORDS_DIR, exist_ok=True)
            QDesktopServices.openUrl(QUrl.fromLocalFile(os.path.abspath(RECORDS_DIR)))
            self.set_hint(f"Membuka folder rekaman: {RECORDS_DIR}", level="ok")
        except Exception as e:
            self.set_hint(f"Gagal membuka folder: {e}", level="err")

    def closeEvent(self, event):
        if getattr(self, "_is_recording", False):
            self._stop_recording()
        self._save_pid_config()
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        super().closeEvent(event)

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
            # Reset filter orientasi agar tidak drift dari sesi sebelumnya
            self._orient.reset()
            # Reset attitude snapshot dari drone (biar horizon start di 0
            # bukan nilai stale dari koneksi lama)
            self._bench_smc = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
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
        # Roll, pitch & yaw DIAMBIL LANGSUNG dari drone (tag [SMC], berisi output
        # fusi complementary filter & integrasi onboard 200Hz).
        # Ini menghindari double-integrate di GUI yang menyebabkan drift & aliasing.
        roll_fw  = self._bench_smc[0]
        pitch_fw = self._bench_smc[1]
        yaw_fw   = self._bench_smc[2]

        tgt_r, tgt_t, tgt_y, tgt_p = self._js_target

        self._card_roll.set_value(roll_fw)
        self._card_roll.set_target(tgt_r, target_unit="°", label="CMD", show_error=True)

        self._card_pitch.set_value(pitch_fw)
        self._card_pitch.set_target(tgt_p, target_unit="°", label="CMD", show_error=True)

        self._card_yaw.set_value(yaw_fw)
        self._card_yaw.set_target(tgt_y, target_unit="°/s", label="RATE")

        self._card_alt.set_value(self._alt_smooth)
        self._card_alt.set_target(tgt_t, target_unit="us", label="THR")

        self._horizon.set_orientation(roll_fw, pitch_fw, yaw_fw)
        self._horizon.set_target_orientation(tgt_r, tgt_p)
        self._alt_tape.set_altitude(self._alt_smooth)
        self._horizon.set_altitude(self._alt_smooth)
        self._info_bar.update_press(self._press)
        self._info_bar.update_vbat(self._vbat)

        throttle_pwm = tgt_t if tgt_t >= 1000 else self._update_gui_throttle()
        ur, up, uy = self._bench_smc[3], self._bench_smc[4], self._bench_smc[5]
        if hasattr(self, "_motor_rpm_card"):
            self._motor_rpm_card.set_data(ur, up, uy, throttle_pwm, self._armed, vbat=self._vbat)

    def _push_rc_values(self):
        # ESP32 sudah mengirim nilai yang sudah terkalibrasi penuh.
        r, t, y, p = self._js_raw

        # Left stick: X = ROLL, Y = THROTTLE
        # Right stick: X = YAW, Y = PITCH
        # Normalize 0..255 -> 0..1; joy widget Y=0 at top so invert
        self._joy_left.set_position(r / 255.0, 1.0 - t / 255.0)
        self._joy_right.set_position(y / 255.0, 1.0 - p / 255.0)
        self._rc_readout.set_values(r, t, y, p, self._js_calibrated, targets=self._js_target)

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
                self._alt_smooth = alt
                self._first_alt = False
            else:
                # Filter IIR ringan agar responsif terhadap ketinggian asli dari drone
                self._alt_smooth = 0.80 * self._alt_smooth + 0.20 * alt
            self._push_att_metrics()
            return

        m = TX_RE.search(text)
        if m:
            g = m.groups()
            r_str, t_str, y_str, p_str = g[0], g[1], g[2], g[3]
            arm_str = g[4] if len(g) > 4 else None
            raw_t_str = g[5] if len(g) > 5 else None

            r_val = float(r_str)
            t_val = int(t_str)
            y_val = float(y_str)
            p_val = float(p_str)

            # Deteksi format satuan fisik (derajat/PWM dari LoraTx baru) atau format raw 0..255
            if "deg" in text or "us" in text or t_val >= 900 or ("." in r_str or "." in p_str):
                tgt_r = r_val
                tgt_t = t_val
                tgt_y = y_val
                tgt_p = p_val
                raw_r = int(round(128.0 + (tgt_r / 25.0) * 127.0))
                raw_p = int(round(128.0 - (tgt_p / 25.0) * 127.0))  # Maju (tgt_p < 0) -> raw_p naik (> 128)
                raw_y = int(round(128.0 + (tgt_y / 150.0) * 127.0))

                if raw_t_str is not None:
                    raw_t = int(raw_t_str)
                else:
                    # Dual-Zone Spring Stick: PWM 1200us = netral (128)
                    idle_pwm = 1200
                    min_pwm = 1000
                    max_pwm = 1300
                    if tgt_t >= idle_pwm:
                        if max_pwm > idle_pwm:
                            raw_t = int(round(128.0 + ((tgt_t - idle_pwm) / float(max_pwm - idle_pwm)) * 127.0))
                        else:
                            raw_t = 128
                    else:
                        if idle_pwm > min_pwm:
                            raw_t = int(round(128.0 - ((idle_pwm - tgt_t) / float(idle_pwm - min_pwm)) * 128.0))
                        else:
                            raw_t = 128
            else:
                raw_r, raw_t, raw_y, raw_p = int(r_val), int(t_val), int(y_val), int(p_val)
                tgt_r = ((raw_r - 128) / 127.0) * 25.0 if abs(raw_r - 128) > 4 else 0.0
                tgt_p = -((raw_p - 128) / 127.0) * 25.0 if abs(raw_p - 128) > 4 else 0.0  # Maju -> Nose Down
                tgt_y = ((raw_y - 128) / 127.0) * 150.0 if abs(raw_y - 128) > 4 else 0.0
                tgt_t = int(1000 + (raw_t / 255.0) * 1000)

            raw_r = max(0, min(255, raw_r))
            raw_t = max(0, min(255, raw_t))
            raw_y = max(0, min(255, raw_y))
            raw_p = max(0, min(255, raw_p))

            self._js_raw = (raw_r, raw_t, raw_y, raw_p)
            self._js_target = (tgt_r, tgt_t, tgt_y, tgt_p)

            if arm_str is not None:
                self._armed = (arm_str == "1")
                if hasattr(self, "_apply_arm_style"):
                    self._apply_arm_style(self._armed)

            self._push_rc_values()
            self._push_att_metrics()
            return

        m = SMC_RE.search(text)
        if m:
            g = m.groups()
            roll = float(g[0])
            pitch = float(g[1])
            if g[2] is not None:
                yaw = float(g[2])
                u_roll = float(g[3])
                u_pitch = float(g[4])
                u_yaw = float(g[5])
            else:
                yaw = self._orient.yaw
                u_roll = float(g[3])
                u_pitch = float(g[4])
                u_yaw = float(g[5])

            self._bench_smc = (roll, pitch, yaw, u_roll, u_pitch, u_yaw)
            self._orient.yaw = yaw
            max_ang = 25.0
            if hasattr(self, "_pid_inputs") and "max_angle" in self._pid_inputs:
                max_ang = self._pid_inputs["max_angle"].value()
            r, t, y, p = self._js_raw
            tgt_roll = ((r - 128) / 127.0) * max_ang if abs(r - 128) > 4 else 0.0
            tgt_pitch = ((p - 128) / 127.0) * max_ang if abs(p - 128) > 4 else 0.0

            if hasattr(self, "_pid_plot"):
                self._pid_plot.add_sample(tgt_roll, roll, tgt_pitch, pitch, u_roll, u_pitch, u_yaw)

            if getattr(self, "_is_recording", False):
                self._record_csv_sample(tgt_roll, roll, tgt_pitch, pitch, yaw, u_roll, u_pitch, u_yaw)

            self._push_att_metrics()
            return

        m = BAT_RE.search(text)
        if m:
            self._vbat = float(m.group(1))
            self._push_att_metrics()
            return

        m = PID_OK_RE.search(text)
        if m:
            self.set_hint("Parameter PID berhasil diterapkan ke Drone \u2713", level="ok")
            return

        m = CAL_OK_RE.search(text)
        if m and self._js_calib_sampling:
            cr, ct, cy, cp = [int(v) for v in m.groups()]
            self._on_cal_ok(cr, ct, cy, cp, n_samples=None)
            return

        # ── [FLAGS] Drone health status (dari downlink byte flags) ──
        m = FLAGS_RE.search(text)
        if m:
            g, b, p, v, bs, fs = [int(x) for x in m.groups()]
            prev_gyro = self._flags.get("gyro_calib", True)
            self._flags = {
                "gyro_calib": bool(g),
                "bmi_ok":     bool(b),
                "bmp_ok":     bool(p),
                "vbat_ok":    bool(v),
                "batt_stage": bs,
                "fs_stage":   fs,
            }
            self._flags_received = True

            # Notifikasi user saat status kalibrasi gyro berubah
            if prev_gyro and not bool(g):
                self.set_hint(
                    "\u26A0 GYRO NOT CALIBRATED - Reset drone di permukaan diam!",
                    level="err"
                )
            elif not prev_gyro and bool(g):
                self.set_hint("\u2713 Gyro calibration recovered (ZUPT)", level="ok")

            # Refresh info bar (bila widget-nya sudah dibuat)
            if hasattr(self, "_info_bar") and hasattr(self._info_bar, "update_flags"):
                self._info_bar.update_flags(self._flags)
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
