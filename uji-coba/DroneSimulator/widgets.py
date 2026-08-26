import math

from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPolygonF, QLinearGradient
from PySide6.QtWidgets import QWidget

SKY_TOP = QColor(82, 153, 226)
SKY_BOTTOM = QColor(196, 229, 252)
GROUND_TOP = QColor(86, 148, 66)
GROUND_BOTTOM = QColor(46, 96, 46)
GRID_COLOR = QColor(40, 80, 40, 110)
DRONE_BODY = QColor(40, 44, 52)
DRONE_ARM = QColor(120, 128, 140)
MOTOR_COLOR = QColor(70, 76, 88)
ROTOR_COLOR = QColor(230, 230, 235, 190)
ROTOR_LINE = QColor(60, 120, 220, 230)
TRAIL_COLOR = QColor(255, 210, 60, 180)
SHADOW_COLOR = QColor(0, 0, 0, 70)


class SimView(QWidget):
    ARM_LEN = 1.0
    ROTOR_R = 0.5
    CAM_ELEV = math.radians(42)
    F = 520.0

    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        self.rotor_phase = 0.0
        self.zoom = 1.0
        self.cam_az = math.pi / 2.0
        self.setMinimumSize(480, 360)
        self.setAutoFillBackground(False)

    def advance_cam(self, dt):
        target = math.pi / 2.0 - self.model.yaw
        diff = (target - self.cam_az + math.pi) % (2.0 * math.pi) - math.pi
        k = 1.0 - math.exp(-2.5 * dt)
        self.cam_az += diff * k

    # ------------------------------------------------------------------ math
    def _cam(self, p):
        x, y, z = p
        tx, ty = self.model.x, self.model.y
        rx, ry = x - tx, y - ty
        # Chase cam: kamera mengejar posisi di belakang drone dengan lag halus.
        # Saat drone berputar, kamera agak tertinggal sehingga putarannya terlihat,
        # lalu kamera menyesuaikan kembali di belakangnya.
        a = self.cam_az
        ca, sa = math.cos(a), math.sin(a)
        xa = rx * ca - ry * sa
        ya = rx * sa + ry * ca
        za = z
        el = self.CAM_ELEV - math.pi / 2.0
        ce, se = math.cos(el), math.sin(el)
        ye = ya * ce - za * se
        ze = ya * se + za * ce
        return xa, ye, ze

    def _cam_dist(self):
        return (11.0 + max(self.model.z, 0.0) * 1.8) * self.zoom

    def project(self, p):
        xa, ye, ze = self._cam(p)
        d = self._cam_dist()
        denom = d - ze
        if denom < 0.15:
            denom = 0.15
        cx = self.width() / 2.0
        cy = self.height() * 0.55
        return QPointF(cx + (xa * self.F) / denom, cy - (ye * self.F) / denom)

    def wheelEvent(self, event):
        steps = event.angleDelta().y() / 120.0
        self.zoom = max(0.3, min(4.0, self.zoom * (1.15 ** steps)))
        self.update()
        event.accept()

    # --------------------------------------------------------------- painting
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        horizon_y = self.project(
            (self.model.x + math.cos(self.model.yaw) * 300.0,
             self.model.y + math.sin(self.model.yaw) * 300.0, 0.0)
        ).y()

        sky = QLinearGradient(0, 0, 0, max(horizon_y, 1))
        sky.setColorAt(0.0, SKY_TOP)
        sky.setColorAt(1.0, SKY_BOTTOM)
        painter.fillRect(0, 0, w, max(0, int(horizon_y)), sky)

        ground = QLinearGradient(0, horizon_y, 0, h)
        ground.setColorAt(0.0, GROUND_TOP)
        ground.setColorAt(1.0, GROUND_BOTTOM)
        painter.fillRect(0, max(0, int(horizon_y)), w, h - max(0, int(horizon_y)), ground)

        self._draw_grid(painter)
        self._draw_home(painter)
        self._draw_trail(painter)
        self._draw_shadow(painter)
        self._draw_drone(painter)
        self._draw_labels(painter)
        self._draw_hud(painter)

        painter.setPen(QPen(QColor(42, 47, 56), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(1, 1, w - 2, h - 2, 12, 12)

    def _draw_home(self, painter):
        p = self.project((0.0, 0.0, 0.0))
        if not self._visible(p):
            return
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(255, 255, 255, 150), 2))
        painter.drawEllipse(p, 14, 14)
        painter.drawLine(QPointF(p.x() - 18, p.y()), QPointF(p.x() + 18, p.y()))
        painter.drawLine(QPointF(p.x(), p.y() - 18), QPointF(p.x(), p.y() + 18))
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(p.x() + 20, p.y() + 4, "HOME")

    def _project_or_none(self, p):
        xa, ye, ze = self._cam(p)
        d = self._cam_dist()
        denom = d - ze
        if denom < 0.15:
            return None
        cx = self.width() / 2.0
        cy = self.height() * 0.55
        return QPointF(cx + (xa * self.F) / denom, cy - (ye * self.F) / denom)

    def _unproject_ground(self, sx, sy):
        d = self._cam_dist()
        el = self.CAM_ELEV - math.pi / 2.0
        t = math.tan(el)
        cx = self.width() / 2.0
        cy = self.height() * 0.55
        denom = d / (1.0 + (cy - sy) * t / self.F)
        if denom <= 0.15:
            return None
        xa = (sx - cx) * denom / self.F
        ye = (cy - sy) * denom / self.F
        ya = ye / math.cos(el)
        ca, sa = math.cos(self.cam_az), math.sin(self.cam_az)
        rx = xa * ca + ya * sa
        ry = -xa * sa + ya * ca
        return (self.model.x + rx, self.model.y + ry)

    def _visible_ground_extent(self):
        w, h = self.width(), self.height()
        m = self.model
        samples = []
        for y in (0.0, h * 0.25, h * 0.5, h * 0.75, h):
            samples.append((0.0, y))
            samples.append((float(w), y))
        for x in (0.0, w * 0.25, w * 0.5, w * 0.75, w):
            samples.append((x, 0.0))
            samples.append((x, float(h)))
        pts = []
        for sx, sy in samples:
            g = self._unproject_ground(sx, sy)
            if g is not None:
                pts.append(g)
        if not pts:
            return None
        xs = [p[0] for p in pts] + [m.x]
        ys = [p[1] for p in pts] + [m.y]
        return min(xs), max(xs), min(ys), max(ys)

    def _draw_grid(self, painter):
        ext = self._visible_ground_extent()
        if ext is None:
            return
        x0, x1, y0, y1 = ext
        span = max(x1 - x0, y1 - y0)
        if span < 1e-6:
            return
        steps = [1.5, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0]
        step = next((s for s in steps if span / s <= 48), steps[-1])
        major = max(1, int(round(5.0 / step)))

        def draw_line(ax, ay, bx, by, is_major):
            painter.setPen(QPen(
                QColor(52, 96, 52, 150) if is_major else GRID_COLOR,
                2 if is_major else 1,
            ))
            for f in range(4):
                p1 = self._project_or_none((ax + (bx - ax) * f / 4.0,
                                            ay + (by - ay) * f / 4.0, 0.0))
                p2 = self._project_or_none((ax + (bx - ax) * (f + 1) / 4.0,
                                            ay + (by - ay) * (f + 1) / 4.0, 0.0))
                if p1 is not None and p2 is not None:
                    painter.drawLine(p1, p2)

        i = 0
        v = math.floor(x0 / step) * step
        while v <= x1 + 1e-9:
            draw_line(v, y0, v, y1, i % major == 0)
            v += step
            i += 1
        i = 0
        v = math.floor(y0 / step) * step
        while v <= y1 + 1e-9:
            draw_line(x0, v, x1, v, i % major == 0)
            v += step
            i += 1

    def _draw_trail(self, painter):
        if len(self.model.trail) < 2:
            return
        painter.setPen(QPen(TRAIL_COLOR, 2.5, Qt.SolidLine, Qt.RoundCap))
        pts = [self.project(pt) for pt in self.model.trail]
        painter.drawPolyline(QPolygonF(pts))

    def _draw_shadow(self, painter):
        painter.setPen(Qt.NoPen)
        painter.setBrush(SHADOW_COLOR)
        shadow = self.project((self.model.x, self.model.y, 0.0))
        r = (26.0 - min(self.model.z * 2.0, 18.0)) * math.sqrt(self.zoom)
        if r > 2:
            painter.drawEllipse(shadow, r, r * 0.45)

    def _arm_quad(self, painter, p0, p1, width):
        dx, dy = p1.x() - p0.x(), p1.y() - p0.y()
        L = math.hypot(dx, dy)
        if L < 0.5:
            return
        ux, uy = dx / L, dy / L
        px, py = -uy, ux
        w2 = width / 2.0
        painter.drawPolygon(QPolygonF([
            QPointF(p0.x() + px * w2, p0.y() + py * w2),
            QPointF(p0.x() - px * w2, p0.y() - py * w2),
            QPointF(p1.x() - px * w2, p1.y() - py * w2),
            QPointF(p1.x() + px * w2, p1.y() - py * w2),
        ]))

    def _draw_blades(self, painter, pt, a, r, pen):
        painter.setPen(pen)
        for k in range(2):
            aa = a + k * math.pi
            painter.drawLine(
                QPointF(pt.x() + math.cos(aa) * r * 0.95, pt.y() + math.sin(aa) * r * 0.80),
                QPointF(pt.x() - math.cos(aa) * r * 0.20, pt.y() - math.sin(aa) * r * 0.15),
            )

    def _draw_drone(self, painter):
        m = self.model
        s = math.sqrt(self.zoom)

        gnd = self.project((m.x, m.y, 0.0))
        if m.armed and m.rotor_speed > 0.05 and self._visible(gnd):
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(255, 255, 255, 22))
            painter.drawEllipse(gnd, 34 * s, 34 * s * 0.42)

        center = self.project((m.x, m.y, m.z + 0.15))
        painter.setPen(QPen(QColor(255, 255, 255, 90), 1, Qt.DashLine))
        painter.drawLine(center, gnd)

        motors = [
            (self.ARM_LEN, 0.0, 0.15),
            (-self.ARM_LEN, 0.0, 0.15),
            (0.0, self.ARM_LEN, 0.15),
            (0.0, -self.ARM_LEN, 0.15),
        ]
        motor_pts = [self.project(self._world(pt, m.x, m.y, m.z)) for pt in motors]
        arm_px = math.hypot(motor_pts[0].x() - center.x(), motor_pts[0].y() - center.y())
        rotor_r = max(7.0, arm_px * 0.62)

        # rotor discs (blur) di belakang lengan
        self.rotor_phase += 0.6 + m.rotor_speed * 2.6
        for pt in motor_pts:
            painter.setPen(Qt.NoPen)
            if m.rotor_speed > 0.05:
                painter.setBrush(QColor(235, 240, 245, 34 + int(26 * m.rotor_speed)))
            else:
                painter.setBrush(QColor(235, 240, 245, 24))
            painter.drawEllipse(pt, rotor_r, rotor_r * 0.80)

        # lengan (outline gelap + isian)
        painter.setPen(QPen(QColor(30, 34, 40), 2))
        painter.setBrush(QColor(96, 106, 120))
        for pt in motor_pts:
            self._arm_quad(painter, center, pt, max(3.0, arm_px * 0.28))
        painter.setBrush(QColor(142, 154, 170))
        painter.setPen(Qt.NoPen)
        for pt in motor_pts:
            self._arm_quad(painter, center, pt, max(2.0, arm_px * 0.15))

        # motor (housing + highlight)
        mr = max(2.6, arm_px * 0.16)
        for pt in motor_pts:
            painter.setPen(QPen(QColor(30, 34, 40), 2))
            painter.setBrush(QColor(70, 78, 90))
            painter.drawEllipse(pt, mr, mr)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(122, 134, 150))
            painter.drawEllipse(QPointF(pt.x() - mr * 0.25, pt.y() - mr * 0.25), mr * 0.4, mr * 0.4)

        # bilah baling-baling (di atas motor)
        if m.rotor_speed > 0.05:
            a = self.rotor_phase
            for pt in motor_pts:
                self._draw_blades(
                    painter, pt, a, rotor_r,
                    QPen(QColor(240, 244, 250, 210), max(1.6, 2.4 * s), Qt.SolidLine, Qt.RoundCap),
                )
                self._draw_blades(
                    painter, pt, a + math.pi / 4, rotor_r,
                    QPen(QColor(240, 244, 250, 60), 1.2 * s, Qt.SolidLine, Qt.RoundCap),
                )

        # badan drone
        br = max(3.5, arm_px * 0.42)
        painter.setPen(QPen(QColor(30, 34, 40), 2.5))
        painter.setBrush(QColor(58, 64, 74))
        painter.drawEllipse(center, br, br)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(92, 102, 116))
        painter.drawEllipse(QPointF(center.x() - br * 0.2, center.y() - br * 0.2), br * 0.45, br * 0.45)
        painter.setBrush(QColor(214, 224, 236))
        painter.drawEllipse(center, br * 0.28, br * 0.28)

        # LED depan (merah = DISARMED, hijau = ARMED) sebagai penanda arah hadap
        nose = self._world((self.ARM_LEN * 1.4, 0.0, 0.15), m.x, m.y, m.z)
        nose_p = self.project(nose)
        dx, dy = nose_p.x() - center.x(), nose_p.y() - center.y()
        L = math.hypot(dx, dy)
        if L > 2:
            ux, uy = dx / L, dy / L
            led = QPointF(center.x() + ux * br * 0.8, center.y() + uy * br * 0.8)
            painter.setPen(QPen(QColor(30, 34, 40), 1.5))
            painter.setBrush(QColor(80, 255, 120) if m.armed else QColor(255, 80, 80))
            painter.drawEllipse(led, 3.0 * s, 3.0 * s)

    def _world(self, local, wx, wy, wz):
        x, y, z = local
        yaw, pitch, roll = self.model.yaw, self.model.pitch, self.model.roll

        x, y = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
        y, z = y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)
        x, z = x * math.cos(pitch) + z * math.sin(pitch), -x * math.sin(pitch) + z * math.cos(pitch)

        return (wx + x, wy + y, wz + z)

    def _visible(self, p):
        return -200 < p.x() < self.width() + 200 and -200 < p.y() < self.height() + 200

    def _panel(self, painter, rect, alpha=120, radius=7):
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, alpha))
        painter.drawRoundedRect(rect, radius, radius)

    def _draw_labels(self, painter):
        m = self.model
        painter.setFont(QFont("Consolas", 9))
        text = f"X:{m.x:+5.1f}  Y:{m.y:+5.1f}  Z:{m.z:5.1f}m"
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        self._panel(painter, QRectF(6, self.height() - 28, tw + 14, 20), 130, 5)
        painter.setPen(QColor(255, 255, 255, 235))
        painter.drawText(QRectF(13, self.height() - 26, tw, 16),
                         Qt.AlignLeft | Qt.AlignVCenter, text)

    # ------------------------------------------------------------------- HUD
    def _draw_hud(self, painter):
        w, h = self.width(), self.height()
        m = self.model

        self._draw_status(painter, w, h)                 # kiri-atas
        self._draw_heading(painter, w, h)                # tengah-atas
        self._draw_tape(painter, m.z, "ALT m", QColor(140, 220, 255), w - 26, y0=60, max_val=30)
        self._draw_tape(painter, m.speed(), "SPD m/s", QColor(255, 220, 120), w - 26, y0=150, max_val=20)
        self._draw_attitude(painter, w - 176, h - 88, box=64)   # kanan-bawah
        self._draw_compass(painter, w - 92, h - 88)             # kanan-bawah (arah)
        self._draw_throttle(painter, m.throttle_pct, w, h)     # bawah-tengah
        if m.connected and not m.has_frames:
            self._draw_warning(painter, w, h)
        else:
            self._draw_hint(painter, w, h)

    def _draw_attitude(self, painter, cx, cy, box=72.0):
        roll = self.model.roll_deg
        pitch = self.model.pitch_deg
        clip = QRectF(cx - box / 2, cy - box / 2, box, box)

        painter.save()
        painter.setClipRect(clip)
        painter.translate(cx, cy)
        painter.rotate(roll)

        painter.setPen(QPen(QColor(255, 255, 255, 220), 1))
        painter.setBrush(QColor(255, 255, 255, 200))
        painter.drawRect(-box, -box, box * 2, box)
        painter.setPen(QPen(QColor(80, 80, 90, 220), 1))
        painter.setBrush(QColor(70, 90, 70, 200))
        painter.drawRect(-box, 0, box * 2, box)

        step = 8.0
        shift = pitch * (box / 2) / 15.0
        for i in range(-3, 4):
            y = i * step + shift
            if i == 0:
                continue
            width = box * 0.35 if abs(i) == 1 else (box * 0.22 if abs(i) == 2 else box * 0.14)
            x = -width / 2
            painter.setPen(QPen(QColor(255, 255, 255, 230), 2))
            painter.drawLine(x, y, x + width, y)

        painter.restore()

        painter.setPen(QPen(QColor(255, 60, 60, 240), 2))
        painter.drawLine(cx - box / 2, cy, cx + box / 2, cy)

        painter.setPen(QPen(QColor(255, 255, 255, 230), 2))
        painter.drawRect(int(cx - box / 2), int(cy - box / 2), int(box), int(box))

        label = f"R {self.model.roll_deg:+3.0f}  P {self.model.pitch_deg:+3.0f}"
        painter.setFont(QFont("Consolas", 9))
        fm = painter.fontMetrics()
        lw = fm.horizontalAdvance("R +00  P +00")
        lh = fm.height()
        rx = cx - lw / 2.0 - 7
        ry = cy - box / 2.0 - lh - 8
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 135))
        painter.drawRoundedRect(QRectF(rx, ry, lw + 14, lh + 4), 5, 5)
        painter.setPen(QPen(QColor(255, 255, 255, 60), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(QRectF(rx, ry, lw + 14, lh + 4), 5, 5)
        painter.setPen(QColor(255, 255, 255, 235))
        painter.drawText(QRectF(rx, ry + 1, lw + 14, lh + 2), Qt.AlignCenter, label)
        painter.restore()

    def _draw_compass(self, painter, cx, cy, r=36.0):
        heading = self.model.heading_deg()
        rad = math.radians

        self._panel(painter, QRectF(cx - r, cy - r, 2 * r, 2 * r), 140, r)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(255, 255, 255, 150), 2))
        painter.drawEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))

        for deg in range(0, 360, 15):
            a = rad(deg - heading)
            outer = r - 3
            inner = r - 9 if deg % 45 == 0 else r - 6
            painter.setPen(QPen(QColor(255, 255, 255, 190), 2 if deg % 45 == 0 else 1))
            painter.drawLine(
                QPointF(cx + math.sin(a) * outer, cy - math.cos(a) * outer),
                QPointF(cx + math.sin(a) * inner, cy - math.cos(a) * inner),
            )

        letters = {0: "N", 90: "E", 180: "S", 270: "W"}
        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        for deg, ch in letters.items():
            a = rad(deg - heading)
            px = cx + math.sin(a) * (r - 15)
            py = cy - math.cos(a) * (r - 15)
            painter.setPen(QColor(255, 214, 64) if deg == 0 else QColor(255, 255, 255, 225))
            painter.drawText(QRectF(px - 8, py - 7, 16, 14), Qt.AlignCenter, ch)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 214, 64))
        painter.drawPolygon(QPolygonF([
            QPointF(cx, cy - r - 6),
            QPointF(cx - 5, cy - r + 2),
            QPointF(cx + 5, cy - r + 2),
        ]))

        text = f"{heading:03.0f}"
        painter.setFont(QFont("Consolas", 9))
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        self._panel(painter, QRectF(cx - 20, cy + r - 2, 40, 18), 130, 4)
        painter.setPen(QColor(255, 255, 255, 235))
        painter.drawText(QRectF(cx - 20, cy + r, 40, 14), Qt.AlignCenter, text)

    def _draw_heading(self, painter, w, h):
        heading = self.model.heading_deg()
        painter.setFont(QFont("Consolas", 10))
        text = f"H:{heading:03.0f}"
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        x0 = w / 2.0 - (tw + 16) / 2.0
        self._panel(painter, QRectF(x0, 8, tw + 16, 24), 130, 6)
        painter.setPen(QColor(255, 255, 255, 235))
        painter.drawText(QRectF(x0 + 8, 10, tw, 20),
                         Qt.AlignLeft | Qt.AlignVCenter, text)

    def _draw_tape(self, painter, value, label, color, x, y0=60, max_val=30.0):
        bar_h = 70
        self._panel(painter, QRectF(x - 2, y0 - 2, 26, bar_h + 4), 150, 5)
        frac = max(0.0, min(value / max_val, 1.0))
        fill_h = int(bar_h * frac)
        if fill_h > 0:
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawRoundedRect(QRectF(x, y0 + bar_h - fill_h, 22, fill_h), 3, 3)
        py = y0 + bar_h - fill_h
        painter.setPen(QPen(QColor(255, 255, 255, 230), 2))
        painter.drawLine(int(x - 7), int(py), int(x), int(py))
        painter.setFont(QFont("Consolas", 8))
        fm = painter.fontMetrics()
        text_v = f"{value:.1f}"
        wl = max(fm.horizontalAdvance(label), fm.horizontalAdvance(text_v)) + 10
        self._panel(painter, QRectF(x - wl - 4, y0 - 34, wl, 34), 120, 5)
        painter.setPen(QColor(255, 255, 255, 235))
        painter.drawText(QRectF(x - wl + 2, y0 - 30, wl - 6, 14), Qt.AlignLeft, label)
        painter.drawText(QRectF(x - wl + 2, y0 - 18, wl - 6, 14), Qt.AlignLeft, text_v)

    def _draw_throttle(self, painter, throttle, w, h):
        cx = w / 2.0
        y = h - 30
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 150))
        painter.drawRoundedRect(QRectF(cx - 51, y - 2, 102, 16), 4, 4)
        frac = max(0.0, min(throttle / 100.0, 1.0))
        painter.setBrush(QColor(60, 200, 120))
        if frac > 0.02:
            painter.drawRoundedRect(QRectF(cx - 50, y, max(6, int(100 * frac)), 12), 3, 3)
        painter.setPen(QPen(QColor(255, 255, 255, 200), 1))
        painter.drawLine(int(cx - 50 + 50), y, int(cx - 50 + 50), y + 12)
        painter.setFont(QFont("Consolas", 9))
        text = f"THR {throttle:3.0f}%"
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        self._panel(painter, QRectF(cx - 50, y - 26, tw + 14, 20), 120, 5)
        painter.setPen(QColor(255, 255, 255, 235))
        painter.drawText(QRectF(cx - 44, y - 24, tw, 16),
                         Qt.AlignLeft | Qt.AlignVCenter, text)

    def _draw_status(self, painter, w, h):
        m = self.model
        x0, y0 = 10, 10
        pw, ph = 48, 82

        self._panel(painter, QRectF(x0, y0, pw, ph), 150, 10)
        painter.setPen(QPen(QColor(255, 255, 255, 40), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(QRectF(x0 + 0.5, y0 + 0.5, pw - 1, ph - 1), 10, 10)

        # --- ARM (lingkaran)
        c = QColor(80, 230, 110) if m.armed else QColor(255, 90, 90)
        painter.setPen(QPen(QColor(20, 22, 28), 2))
        painter.setBrush(c)
        painter.drawEllipse(QPointF(x0 + 24, y0 + 22), 7, 7)

        # --- SIGNAL (3 bar)
        cy = y0 + 48
        bar_col = QColor(90, 220, 130) if m.connected else QColor(110, 115, 125)
        for i, bh in enumerate((5, 9, 13)):
            bx = x0 + 16 + i * 6
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(70, 75, 85))
            painter.drawRect(QRectF(bx, cy + 2 - 3, 3, 3))
            painter.setBrush(bar_col)
            painter.drawRect(QRectF(bx, cy + 2 - bh, 3, bh))

        # --- BATT (ikon baterai)
        cy = y0 + 74
        bcol = QColor(80, 230, 110)
        if m.battery < 30:
            bcol = QColor(255, 190, 60)
        if m.battery < 10:
            bcol = QColor(255, 80, 80)
        bx = x0 + 13
        bw, bh = 22, 11
        by = cy - bh / 2.0
        painter.setBrush(QColor(40, 44, 54))
        painter.setPen(QPen(QColor(190, 200, 215, 220), 1))
        painter.drawRoundedRect(QRectF(bx, by, bw, bh), 2, 2)
        fill = int(bw * m.battery / 100.0)
        if fill > 1:
            painter.setPen(Qt.NoPen)
            painter.setBrush(bcol)
            painter.drawRect(QRectF(bx + 1.5, by + 1.5, fill - 3, bh - 3))
        painter.setPen(QPen(QColor(190, 200, 215, 220), 1))
        painter.drawLine(QPointF(bx + bw + 1, by + 3), QPointF(bx + bw + 1, by + bh - 3))

    def _draw_warning(self, painter, w, h):
        painter.setFont(QFont("Consolas", 11))
        fm = painter.fontMetrics()
        t1 = "TERHUBUNG TAPI TIDAK ADA DATA CH"
        t2 = "Cek baud 115200 & firmware JoystickTest"
        ww = max(fm.horizontalAdvance(t1), fm.horizontalAdvance(t2)) + 26
        x0 = w / 2.0 - ww / 2.0
        self._panel(painter, QRectF(x0, 120, ww, 44), 165, 8)
        painter.setPen(QColor(255, 90, 90))
        painter.drawText(QRectF(x0, 126, ww, 16), Qt.AlignCenter, t1)
        painter.setPen(QColor(255, 200, 120))
        painter.drawText(QRectF(x0, 142, ww, 16), Qt.AlignCenter, t2)

    def _draw_hint(self, painter, w, h):
        m = self.model
        hint = None
        if not m.armed:
            hint = "DISARMED - tekan SPASI atau tombol ARM"
        elif m.z < 0.05:
            hint = "Dorong THROTTLE (+) ke atas untuk terbang"
        elif abs(m.ch1) < 5 and abs(m.ch3) < 5 and abs(m.ch4) < 5:
            hint = "Gerakkan stick untuk bermanuver"
        if not hint:
            return
        painter.setFont(QFont("Consolas", 10))
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(hint)
        x0 = w / 2.0 - (tw + 16) / 2.0
        self._panel(painter, QRectF(x0, h - 96, tw + 16, 24), 130, 6)
        painter.setPen(QColor(255, 255, 255, 235))
        painter.drawText(QRectF(x0 + 8, h - 94, tw, 20),
                         Qt.AlignLeft | Qt.AlignVCenter, hint)
