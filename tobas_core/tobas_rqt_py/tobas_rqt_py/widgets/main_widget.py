import os
import signal
from typing import override
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtGui import QIcon, QCloseEvent


class MainWidget(QWidget):

    def __init__(self, title: str, icon_path: str, widget: QWidget) -> None:
        super().__init__()

        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(icon_path))

        rows = QVBoxLayout()
        self.setLayout(rows)
        self._widget = widget
        rows.addWidget(self._widget)

    @override
    def closeEvent(self, _: QCloseEvent) -> None:
        self._widget.close()

        # クローズ時にプロセスごと落とすことで確実に終了させる
        os.kill(os.getpid(), signal.SIGINT)
