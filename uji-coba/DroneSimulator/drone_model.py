import math
from collections import deque

GRAVITY = 9.81
MAX_TILT = math.radians(30)
YAW_RATE_MAX = 2.5
HOVER_THROTTLE = 50.0     # posisi throttle netral (hover)
HOVER_BAND = 6.0          # % sekitar hover = tahan ketinggian
VZ_MAX = 3.0              # kecepatan vertikal maksimum (m/s)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class DroneModel:
    def __init__(self):
        self.trail = deque(maxlen=240)
        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.throttle_pct = 0.0
        self.rotor_speed = 0.0
        self.battery = 100.0
        self.armed = False
        self.connected = False
        self.has_frames = False
        self.time = 0.0
        self.last_trail_t = 0.0
        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.ch4 = 0
        self.trail.clear()

    def set_channels(self, yaw_pct, throttle_pct, roll_pct, pitch_pct):
        # yaw_pct/roll_pct/pitch_pct dalam -100..+100 (0 = netral),
        # throttle_pct dalam 0..100 (0 = bawah, 100 = atas).
        self.ch1 = yaw_pct
        self.ch2 = throttle_pct
        self.ch3 = roll_pct
        self.ch4 = pitch_pct

    def update(self, dt):
        if dt <= 0:
            return
        self.time += dt

        if not self.armed:
            self.throttle_pct = 0.0
            self.rotor_speed = 0.0
            self.vx *= 0.85
            self.vy *= 0.85
            self.vz = 0.0
            if self.z > 0.0:
                self.z = max(0.0, self.z - 2.0 * dt)
            self.pitch *= 0.85
            self.roll *= 0.85
            return

        self.throttle_pct = clamp(self.ch2, 0.0, 100.0)
        self.rotor_speed = clamp(0.1 + self.throttle_pct / 100.0 * 0.9, 0.0, 1.0)

        # Kontrol vertikal ala flight controller (altitude hold):
        # throttle di sekitar 50% -> tahan ketinggian, di atas -> naik,
        # di bawah -> turun. Kecepatan vertikal mengikuti target secara halus.
        offset = self.throttle_pct - HOVER_THROTTLE
        if abs(offset) < HOVER_BAND:
            vz_target = 0.0
        else:
            mag = (abs(offset) - HOVER_BAND) / (100.0 - HOVER_THROTTLE - HOVER_BAND)
            vz_target = clamp(mag, 0.0, 1.0) * math.copysign(1.0, offset) * VZ_MAX
        kv = 1.0 - math.exp(-5.5 * dt)
        self.vz += (vz_target - self.vz) * kv

        if self.z <= 0.0 and self.vz <= 0.0:
            self.z = 0.0
            self.vz = 0.0
        else:
            self.z += self.vz * dt
            if self.z < 0.0:
                self.z = 0.0
                self.vz = 0.0

        target_roll = clamp(self.ch3 / 100.0, -1.0, 1.0) * MAX_TILT
        target_pitch = -clamp(self.ch4 / 100.0, -1.0, 1.0) * MAX_TILT
        k = 1.0 - math.exp(-3.0 * dt)
        self.roll += (target_roll - self.roll) * k
        self.pitch += (target_pitch - self.pitch) * k

        self.yaw += (self.ch1 / 100.0) * YAW_RATE_MAX * dt

        cosz = math.cos(self.pitch) * math.cos(self.roll)
        if abs(cosz) < 1e-6:
            cosz = 0.0
        accel_fwd = -GRAVITY * math.tan(self.pitch) * cosz
        accel_right = GRAVITY * math.tan(self.roll) * cosz

        yaw = self.yaw
        self.vx += (accel_fwd * math.cos(yaw) - accel_right * math.sin(yaw)) * dt
        self.vy += (accel_fwd * math.sin(yaw) + accel_right * math.cos(yaw)) * dt
        drag = 1.0 - 0.35 * dt
        self.vx *= drag
        self.vy *= drag

        if self.z > 0.01:
            self.x += self.vx * dt
            self.y += self.vy * dt

        self.battery = max(
            0.0,
            self.battery - 0.06 * dt * (0.5 + self.throttle_pct / 200.0),
        )

        if self.time - self.last_trail_t > 0.1 and self.z > 0.05:
            self.trail.append((self.x, self.y, self.z))
            self.last_trail_t = self.time

    def speed(self):
        return math.hypot(self.vx, self.vy)

    def heading_deg(self):
        return math.degrees(self.yaw) % 360.0

    @property
    def pitch_deg(self):
        return math.degrees(self.pitch)

    @property
    def roll_deg(self):
        return math.degrees(self.roll)

    @property
    def vertical_deg(self):
        return math.degrees(math.atan2(self.vz, self.speed())) if self.speed() > 0.01 else 0.0
