#!/usr/bin/python

import sys
from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout

from rviz2_py import VisualizationFrame


class MainWidget(QWidget):

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.setWindowTitle("Test Visualization Frame")

        self._frame = VisualizationFrame()
        self._frame.setHelpPath("")
        self._frame.setSplashPath("")
        self._frame.initialize()

        rows = QVBoxLayout()
        self.setLayout(rows)
        rows.addWidget(self._frame)


def main(args=None) -> None:
    app = QApplication(sys.argv)

    main_widget = MainWidget()
    main_widget.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
