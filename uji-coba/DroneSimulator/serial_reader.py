import re

import serial
from PySide6.QtCore import QThread, Signal

# Format firmware JoystickTest v2:
#   JOY LX:<raw> LY:<raw> RX:<raw> RY:<raw> R:<cmd> T:<cmd> Y:<cmd> P:<cmd>
# ch1=R(ROLL) ch2=T(THROTTLE) ch3=Y(YAW) ch4=P(PITCH), semua 0..255
CHANNEL_PATTERN = re.compile(
    r"R:(\d+)\s+T:(\d+)\s+Y:(\d+)\s+P:(\d+)"
)


class SerialReader(QThread):
    channels = Signal(int, int, int, int)
    line = Signal(str)
    connection_changed = Signal(bool, str)
    failed = Signal(str)

    def __init__(self, port: str, baud: int = 115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = False
        self._ser = None

    def stop(self):
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass

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
                m = CHANNEL_PATTERN.search(text)
                if m:
                    self.channels.emit(
                        int(m.group(1)),
                        int(m.group(2)),
                        int(m.group(3)),
                        int(m.group(4)),
                    )
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
