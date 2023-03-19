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

        self.frame_tree = FrameTreeWidget(main)
        self._cols.addWidget(self.frame_tree)

        self.rviz = RvizWidget(main)
        self._cols.addWidget(self.rviz)

        self.setFixedHeight(self.HEIGHT)
        self.setVisible(False)

    def define_connections(self) -> None:
        self.frame_tree.define_connections()
        self.rviz.define_connections()
        self._main.urdf_parser.robot_model_updated.connect(self._visualize)

    @pyqtSlot()
    def _visualize(self) -> None:
        self.setVisible(True)
