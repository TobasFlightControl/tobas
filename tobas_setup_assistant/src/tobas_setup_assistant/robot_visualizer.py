from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .frame_tree import FrameTreeWidget
from .rviz import RvizWidget


class RobotVisualizerWidget(QWidget):
    HEIGHT = 350

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        self._frame_tree = FrameTreeWidget(main)
        self._cols.addWidget(self._frame_tree)

        self._rviz = RvizWidget(main)
        self._cols.addWidget(self._rviz)

        self.setFixedHeight(self.HEIGHT)
        self.setVisible(False)

    def define_connections(self) -> None:
        self._frame_tree.define_connections()
        self._rviz.define_connections()
        self._main.urdf_parser.robot_model_updated.connect(self._visualize)

    def highlight_link(self, link_name: str) -> None:
        return self._rviz.highlight_link(link_name)

    @pyqtSlot()
    def _visualize(self) -> None:
        self.setVisible(True)
