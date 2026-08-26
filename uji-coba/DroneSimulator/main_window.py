import serial.tools.list_ports
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from drone_model import DroneModel
from serial_reader import SerialReader
from style import APP_STYLESHEET
from widgets import SimView


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SIMULATOR DRONE MANGKRAK")
        self.resize(1150, 800)

        self.model = DroneModel()
        self.reader = None
        self.latest = (0, 0, 0, 0)
        self.frames = 0
        self.frames_baseline = 0
        self.last_stale_check = 0
        self._stale_logged = False
        self.invert = [False, False, False, False]

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)

        self._refresh_ports()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        self.view = SimView(self.model)
        self.view.setMinimumWidth(620)
        root.addWidget(self.view, 1)

        side = QVBoxLayout()
        side.setSpacing(8)

        title = QLabel("SIMULATOR DRONE MANGKRAK")
        title.setObjectName("appTitle")
        sub = QLabel("Kontrol drone • JOY: Mode 2 (ROLL/THR & YAW/PITCH)")
        sub.setObjectName("appSub")
        side.addWidget(title)
        side.addWidget(sub)

        side.addWidget(self._build_serial_box())
        side.addWidget(self._build_channels_box())
        side.addWidget(self._build_telemetry_box())
        side.addWidget(self._build_log_box(), 1)
        side.addWidget(self._build_help_box())
        root.addLayout(side)

        self.statusBar().showMessage("Belum terhubung")

    def _build_serial_box(self):
        box = QGroupBox("Koneksi Remote")
        lay = QGridLayout(box)

        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "9600", "57600"])
        self.baud_combo.setCurrentText("115200")

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn = QPushButton("Hubungkan")
        self.connect_btn.setObjectName("connectBtn")
        self.connect_btn.clicked.connect(self._toggle_connect)

        self.status_label = QLabel("● OFFLINE")
        self.status_label.setStyleSheet("color:#e05b5b; font-weight:bold;")

        self.arm_btn = QPushButton("ARM / DISARM  [SPASI]")
        self.arm_btn.setObjectName("armBtn")
        self.arm_btn.setProperty("armed", False)
        self.arm_btn.setCheckable(True)
        self.arm_btn.setMinimumHeight(36)
        self.arm_btn.clicked.connect(self._toggle_armed)

        lay.addWidget(QLabel("Port:"), 0, 0)
        lay.addWidget(self.port_combo, 0, 1, 1, 2)
        lay.addWidget(QLabel("Baud:"), 1, 0)
        lay.addWidget(self.baud_combo, 1, 1)
        lay.addWidget(self.refresh_btn, 1, 2)
        lay.addWidget(self.connect_btn, 2, 0, 1, 2)
        lay.addWidget(self.status_label, 2, 2)
        lay.addWidget(self.arm_btn, 3, 0, 1, 3)
        return box

    def _build_channels_box(self):
        box = QGroupBox("Channel (dari joystick test)")
        grid = QGridLayout(box)
        self.ch_bars = {}
        self.ch_invert = {}
        names = [
            ("CH1", "ROLL"),
            ("CH2", "THROTTLE"),
            ("CH3", "YAW"),
            ("CH4", "PITCH"),
        ]
        tips = {
            "CH1": "ROLL  (J1 X / GPIO32)",
            "CH2": "THROTTLE (J1 Y / GPIO33)",
            "CH3": "YAW  (J2 X / GPIO34)",
            "CH4": "PITCH (J2 Y / GPIO35)",
        }
        for row, (key, label) in enumerate(names):
            lbl = QLabel(label)
            lbl.setToolTip(tips[key])
            bar = QProgressBar()
            bar.setObjectName(key.lower())
            bar.setToolTip(tips[key])
            bar.setRange(0, 255)
            bar.setValue(128)
            bar.setTextVisible(False)
            val = QLabel("0")
            val.setFixedWidth(42)
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            inv = QCheckBox("Balik")
            idx = int(key[2]) - 1
            inv.toggled.connect(lambda checked, i=idx: self._set_invert(i, checked))
            grid.addWidget(lbl, row, 0)
            grid.addWidget(bar, row, 1)
            grid.addWidget(val, row, 2)
            grid.addWidget(inv, row, 3)
            self.ch_bars[key] = (bar, val)
            self.ch_invert[key] = inv
        return box

    def _build_telemetry_box(self):
        box = QGroupBox("Telemetri")
        grid = QGridLayout(box)
        self.tel = {}
        rows = [
            ("alt", "Altitude", "0.0 m"),
            ("spd", "Kecepatan", "0.0 m/s"),
            ("head", "Heading", "000 deg"),
            ("pitch", "Pitch", "0.0 deg"),
            ("roll", "Roll", "0.0 deg"),
            ("thr", "Throttle", "0 %"),
            ("batt", "Baterai", "100 %"),
        ]
        for row, (key, label, init) in enumerate(rows):
            grid.addWidget(QLabel(label + ":"), row, 0)
            v = QLabel(init)
            v.setObjectName("telValue")
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(v, row, 1)
            self.tel[key] = v
        return box

    def _build_log_box(self):
        box = QGroupBox("Log")
        lay = QVBoxLayout(box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(110)
        lay.addWidget(self.log_view)
        return box

    def _build_help_box(self):
        box = QGroupBox("Kontrol")
        lay = QVBoxLayout(box)
        lbl = QLabel(
            "SPASI = ARM/DISARM\n"
            "R = Reset drone\n"
            "ESC = Keluar\n"
            "Scroll = Zoom\n\n"
            "Arah terbalik? ubah\n"
            "INVERT_*/SWAP_* di .ino."
        )
        lay.addWidget(lbl)
        return box

    # ------------------------------------------------------------ serial
    def _refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if current:
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    def _toggle_connect(self):
        if self.reader is not None and self.reader.isRunning():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.currentText()
        if not port:
            self._log("Pilih port serial dulu.")
            return
        baud = int(self.baud_combo.currentText())
        self.reader = SerialReader(port, baud, parent=self)
        self.reader.channels.connect(self._on_channels)
        self.reader.line.connect(self._on_line)
        self.reader.connection_changed.connect(self._on_connection)
        self.reader.failed.connect(self._on_failed)
        self.reader.start()
        self.connect_btn.setText("Putuskan")
        self._log(f"Menghubungkan ke {port} @ {baud}...")

    def _disconnect(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.wait(1000)
            self.reader = None
        self.connect_btn.setText("Hubungkan")
        self.status_label.setText("● OFFLINE")
        self.status_label.setStyleSheet("color:#e05b5b; font-weight:bold;")
        self.model.connected = False
        self._log("Koneksi diputus.")

    def _on_channels(self, ch1, ch2, ch3, ch4):
        # ch1=ROLL(0-255) ch2=THROTTLE(0-255) ch3=YAW(0-255) ch4=PITCH(0-255)
        vals = [ch1, ch2, ch3, ch4]
        for i in range(4):
            if self.invert[i]:
                vals[i] = 255 - vals[i]
        r, t, y, p = vals
        yaw_pct = (y - 128) / 127.0 * 100.0
        roll_pct = (r - 128) / 127.0 * 100.0
        pitch_pct = (p - 128) / 127.0 * 100.0
        throttle_pct = t / 255.0 * 100.0
        self.latest = (r, t, y, p)
        self.model.set_channels(yaw_pct, throttle_pct, roll_pct, pitch_pct)
        self.model.has_frames = True
        self.frames += 1
        self._update_channel_bars()

    def _update_channel_bars(self):
        vals = {"CH1": self.latest[0], "CH2": self.latest[1],
                "CH3": self.latest[2], "CH4": self.latest[3]}
        for key, (bar, val) in self.ch_bars.items():
            bar.setValue(vals[key])
            val.setText(str(vals[key]))

    def _set_invert(self, index, checked):
        self.invert[index] = checked
        self._log(f"CH{index + 1} arah dibalik: {'YA' if checked else 'TIDAK'}")

    def _toggle_armed(self):
        self._set_armed(not self.model.armed)

    def _set_armed(self, armed):
        self.model.armed = armed
        self.arm_btn.setChecked(armed)
        self.arm_btn.setProperty("armed", armed)
        self._repolish(self.arm_btn)
        if armed:
            self.arm_btn.setText("ARMED  [SPASI]")
            self._log("ARMED - dorong throttle ke atas untuk terbang.")
        else:
            self.arm_btn.setText("ARM / DISARM  [SPASI]")
            self._log("DISARMED")

    def _repolish(self, widget):
        widget.style().unpolish(widget)
        widget.style().polish(widget)
        widget.update()

    def _on_line(self, text):
        self._log(text)

    def _on_connection(self, connected, port):
        self.model.connected = connected
        if connected:
            self.status_label.setText(f"● ONLINE ({port})")
            self.status_label.setStyleSheet("color:#7ac76a; font-weight:bold;")
            self._log(f"Terhubung ke {port}. SPASI untuk ARM.")
        else:
            self.status_label.setText("● OFFLINE")
            self.status_label.setStyleSheet("color:#e05b5b; font-weight:bold;")

    def _on_failed(self, msg):
        self._log(f"ERROR: {msg}")
        self._disconnect()

    # ------------------------------------------------------------- loop
    def _tick(self):
        dt = 1.0 / 60.0
        self.model.update(dt)
        self.view.advance_cam(dt)

        if self.model.connected:
            now = int(self.model.time)
            if now - self.last_stale_check >= 2:
                self.last_stale_check = now
                if self.frames > self.frames_baseline:
                    self._stale_logged = False
                elif not self._stale_logged:
                    self._stale_logged = True
                    self._log("PERINGATAN: data CH berhenti masuk. Cek kabel/koneksi remote.")
                self.frames_baseline = self.frames

        m = self.model
        self.tel["alt"].setText(f"{m.z:5.1f} m")
        self.tel["spd"].setText(f"{m.speed():5.1f} m/s")
        self.tel["head"].setText(f"{m.heading_deg():03.0f} deg")
        self.tel["pitch"].setText(f"{m.pitch_deg:+5.1f} deg")
        self.tel["roll"].setText(f"{m.roll_deg:+5.1f} deg")
        self.tel["thr"].setText(f"{m.throttle_pct:3.0f} %")
        self.tel["batt"].setText(f"{m.battery:3.0f} %")
        self.view.update()

    # -------------------------------------------------------------- keys
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self._set_armed(not self.model.armed)
        elif event.key() == Qt.Key_R:
            self.model.reset()
            self._log("Drone di-reset.")
        elif event.key() == Qt.Key_Escape:
            self._disconnect()
            self.close()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self._disconnect()
        super().closeEvent(event)

    def _log(self, text):
        self.log_view.append(text)
        sb = self.log_view.verticalScrollBar()
        sb.setValue(sb.maximum())


def run():
    import sys

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
