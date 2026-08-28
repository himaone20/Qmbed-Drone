"""Simple Sensor Viewer - GUI tampil orientasi (Roll/Pitch/Yaw) + altitude.

Pasangan firmware : uji-coba/SimpleSensorRead/SimpleSensorRead.ino
Format serial     : SENS ROLL:<f> PITCH:<f> YAW:<f> P:<f> ALT:<f>
Perintah dikirim  :
    'r\\n' -> reset baseline altitude (altitude -> 0 m)
    'y\\n' -> reset yaw = 0

Dependencies:
    pip install pyserial PySide6

Menjalankan:
    python simple_sensor_viewer.py
"""

from __future__ import annotations

import re
import sys

import serial
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

SENS_RE = re.compile(
    r"SENS\s+ROLL:(\S+)\s+PITCH:(\S+)\s+YAW:(\S+)"
    r"\s+P:(\S+)\s+ALT:(\S+)"
)


class SerialReader(QThread):
    """Thread pembaca serial. Parse baris SENS -> emit sample dict."""

    sample = Signal(dict)
    line = Signal(str)
    connection_changed = Signal(bool, str)
    failed = Signal(str)

    def __init__(self, port: str, baud: int = 115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = False
        self._ser: serial.Serial | None = None

    def stop(self):
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass

    def send_command(self, data: bytes) -> bool:
        if self._ser is None or not self._ser.is_open:
            return False
        try:
            self._ser.write(data)
            self._ser.flush()
            return True
        except Exception:
            return False

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as exc:
            self.failed.emit(f"Gagal buka {self.port}: {exc}")
            return

        self.connection_changed.emit(True, self.port)
        self._running = True

        try:
            while self._running and self._ser.is_open:
                raw = self._ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", "replace").strip()
                if not text:
                    continue
                self.line.emit(text)

                m = SENS_RE.search(text)
                if m:
                    try:
                        data = {
                            "roll":  float(m.group(1)),
                            "pitch": float(m.group(2)),
                            "yaw":   float(m.group(3)),
                            "p":     float(m.group(4)),
                            "alt":   float(m.group(5)),
                        }
                        self.sample.emit(data)
                    except ValueError:
                        pass
        except serial.SerialException as exc:
            if self._running:
                self.failed.emit(str(exc))
        finally:
            try:
                if self._ser is not None:
                    self._ser.close()
            except Exception:
                pass
            self.connection_changed.emit(False, self.port)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simple Sensor Viewer")
        self.setMinimumWidth(360)

        self.reader: SerialReader | None = None

        mono = QFont("Consolas")
        mono.setStyleHint(QFont.Monospace)
        mono.setPointSize(14)

        # ── Baris koneksi ────────────────────────────────────────────────
        self.combo_port = QComboBox()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_connect = QPushButton("Connect")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect.clicked.connect(self.toggle_connect)

        row_conn = QHBoxLayout()
        row_conn.addWidget(QLabel("Port:"))
        row_conn.addWidget(self.combo_port, 1)
        row_conn.addWidget(self.btn_refresh)
        row_conn.addWidget(self.btn_connect)

        self.lbl_status = QLabel("Status: Disconnected")

        # ── Grup Orientation ────────────────────────────────────────────
        self.lbl_roll  = QLabel("+0.00")
        self.lbl_pitch = QLabel("+0.00")
        self.lbl_yaw   = QLabel("+0.00")
        for w in (self.lbl_roll, self.lbl_pitch, self.lbl_yaw):
            w.setFont(mono)
            w.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        grp_ori = QGroupBox("Orientation (deg)")
        g1 = QGridLayout(grp_ori)
        g1.addWidget(QLabel("Roll"),  0, 0); g1.addWidget(self.lbl_roll,  0, 1)
        g1.addWidget(QLabel("Pitch"), 1, 0); g1.addWidget(self.lbl_pitch, 1, 1)
        g1.addWidget(QLabel("Yaw"),   2, 0); g1.addWidget(self.lbl_yaw,   2, 1)
        g1.setColumnStretch(1, 1)

        # ── Grup Barometer ───────────────────────────────────────────────
        self.lbl_p   = QLabel("0.00 hPa")
        self.lbl_alt = QLabel("+0.00 m")
        for w in (self.lbl_p, self.lbl_alt):
            w.setFont(mono)
            w.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        grp_baro = QGroupBox("Barometer")
        g2 = QGridLayout(grp_baro)
        g2.addWidget(QLabel("Pressure"), 0, 0); g2.addWidget(self.lbl_p,   0, 1)
        g2.addWidget(QLabel("Altitude"), 1, 0); g2.addWidget(self.lbl_alt, 1, 1)
        g2.setColumnStretch(1, 1)

        # ── Tombol perintah ──────────────────────────────────────────────
        self.btn_reset_alt = QPushButton("Reset baseline altitude")
        self.btn_reset_yaw = QPushButton("Reset yaw = 0")
        self.btn_reset_alt.setEnabled(False)
        self.btn_reset_yaw.setEnabled(False)
        self.btn_reset_alt.clicked.connect(self.reset_baseline)
        self.btn_reset_yaw.clicked.connect(self.reset_yaw)

        # ── Layout utama ─────────────────────────────────────────────────
        root = QVBoxLayout(self)
        root.addLayout(row_conn)
        root.addWidget(self.lbl_status)
        root.addWidget(grp_ori)
        root.addWidget(grp_baro)
        root.addWidget(self.btn_reset_alt)
        root.addWidget(self.btn_reset_yaw)
        root.addStretch(1)

        self.refresh_ports()

    # ── Helpers ──────────────────────────────────────────────────────────
    def refresh_ports(self):
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_port.addItem(f"{p.device} - {p.description}", p.device)
        if not ports:
            self.combo_port.addItem("(tidak ada port)", "")

    def toggle_connect(self):
        if self.reader is None:
            device = self.combo_port.currentData()
            if not device:
                self.lbl_status.setText("Status: pilih COM port dulu")
                return
            self.reader = SerialReader(device, 115200, self)
            self.reader.sample.connect(self.on_sample)
            self.reader.connection_changed.connect(self.on_connection_changed)
            self.reader.failed.connect(self.on_failed)
            self.reader.start()
            self.btn_connect.setText("Connecting...")
            self.btn_connect.setEnabled(False)
        else:
            self.reader.stop()
            self.reader.wait(1000)
            self.reader = None

    def reset_baseline(self):
        if self.reader is not None:
            ok = self.reader.send_command(b"r\n")
            self.lbl_status.setText(
                "Status: perintah reset baseline dikirim" if ok
                else "Status: gagal kirim perintah"
            )

    def reset_yaw(self):
        if self.reader is not None:
            ok = self.reader.send_command(b"y\n")
            self.lbl_status.setText(
                "Status: perintah reset yaw dikirim" if ok
                else "Status: gagal kirim perintah"
            )

    def _set_controls_connected(self, connected: bool):
        self.btn_reset_alt.setEnabled(connected)
        self.btn_reset_yaw.setEnabled(connected)
        self.combo_port.setEnabled(not connected)
        self.btn_refresh.setEnabled(not connected)

    # ── Slots ────────────────────────────────────────────────────────────
    def on_sample(self, data: dict):
        self.lbl_roll.setText(f"{data['roll']:+.2f}")
        self.lbl_pitch.setText(f"{data['pitch']:+.2f}")
        self.lbl_yaw.setText(f"{data['yaw']:+.2f}")
        self.lbl_p.setText(f"{data['p']:.2f} hPa")
        self.lbl_alt.setText(f"{data['alt']:+.2f} m")

    def on_connection_changed(self, connected: bool, port: str):
        if connected:
            self.lbl_status.setText(f"Status: Connected ke {port}")
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setEnabled(True)
            self._set_controls_connected(True)
        else:
            self.lbl_status.setText("Status: Disconnected")
            self.btn_connect.setText("Connect")
            self.btn_connect.setEnabled(True)
            self._set_controls_connected(False)
            self.reader = None

    def on_failed(self, msg: str):
        self.lbl_status.setText(f"Status: ERROR - {msg}")
        self.btn_connect.setText("Connect")
        self.btn_connect.setEnabled(True)
        self._set_controls_connected(False)
        self.reader = None

    def closeEvent(self, event):
        if self.reader is not None:
            self.reader.stop()
            self.reader.wait(1000)
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
