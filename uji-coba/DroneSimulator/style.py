APP_STYLESHEET = """
* {
    font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
    font-size: 13px;
}
QMainWindow {
    background: #14161b;
}
QWidget#central {
    background: #14161b;
}
QGroupBox {
    background: #1d2026;
    border: 1px solid #2a2f38;
    border-radius: 10px;
    margin-top: 14px;
    padding-top: 8px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 2px 8px;
    color: #7fd4ff;
    background: #1d2026;
    border-radius: 4px;
}
QLabel {
    color: #cdd3de;
    background: transparent;
}
QPushButton {
    background: #2a2f3a;
    color: #e6e9ef;
    border: 1px solid #3a4150;
    border-radius: 7px;
    padding: 7px 12px;
    font-weight: 600;
}
QPushButton:hover { background: #343b49; border-color: #4a5568; }
QPushButton:pressed { background: #232833; }
QPushButton:disabled { color: #6b7280; background: #23262d; }
QPushButton#connectBtn {
    background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #1e7fb8, stop:1 #145f8c);
    border: 1px solid #2b95d6;
    color: #ffffff;
}
QPushButton#connectBtn:hover {
    background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #268ec9, stop:1 #17699b);
}
QPushButton#armBtn[armed="true"] {
    background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #2d8f4e, stop:1 #1f6a39);
    border: 1px solid #3fae63;
    color: #ffffff;
}
QPushButton#armBtn[armed="false"] {
    background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #a0443f, stop:1 #772f2c);
    border: 1px solid #c25b55;
    color: #ffffff;
}
QComboBox {
    background: #22262e;
    color: #e6e9ef;
    border: 1px solid #3a4150;
    border-radius: 7px;
    padding: 5px 8px;
}
QComboBox:hover { border-color: #4a5568; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox::down-arrow {
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid #8a94a6;
    margin-right: 8px;
}
QComboBox QAbstractItemView {
    background: #22262e;
    color: #e6e9ef;
    border: 1px solid #3a4150;
    selection-background-color: #2b95d6;
    selection-color: #ffffff;
}
QProgressBar {
    background: #262b34;
    border: 1px solid #343b46;
    border-radius: 5px;
    min-height: 10px;
    max-height: 12px;
    text-align: center;
}
QProgressBar::chunk { border-radius: 4px; }
QProgressBar#ch1::chunk {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #0fb6d0, stop:1 #1fd0e8);
}
QProgressBar#ch2::chunk {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #21b36b, stop:1 #37e08a);
}
QProgressBar#ch3::chunk {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #e08a1e, stop:1 #f0a94a);
}
QProgressBar#ch4::chunk {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #9c6be0, stop:1 #b98af0);
}
QCheckBox {
    color: #cdd3de;
    spacing: 6px;
}
QCheckBox::indicator {
    width: 15px;
    height: 15px;
    border: 1px solid #4a5568;
    border-radius: 4px;
    background: #22262e;
}
QCheckBox::indicator:hover { border-color: #7fd4ff; }
QCheckBox::indicator:checked {
    background: #2b95d6;
    border-color: #2b95d6;
}
QTextEdit {
    background: #12141a;
    color: #a8c4d8;
    border: 1px solid #2a2f38;
    border-radius: 7px;
    padding: 6px;
    font-family: Consolas, "Courier New", monospace;
    font-size: 12px;
    selection-background-color: #2b95d6;
}
QLabel#telValue {
    font-family: Consolas, "Courier New", monospace;
    font-size: 13px;
    font-weight: 700;
    color: #7fd4ff;
}
QLabel#appTitle {
    font-size: 17px;
    font-weight: 800;
    color: #eaf2ff;
}
QLabel#appSub {
    font-size: 11px;
    color: #7a8595;
}
QScrollBar:vertical {
    background: #1a1d23;
    width: 10px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #3a4150;
    border-radius: 5px;
    min-height: 24px;
}
QScrollBar::handle:vertical:hover { background: #4a5568; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }
QStatusBar {
    background: #14161b;
    color: #7a8595;
}
QStatusBar::item { border: none; }
"""
