from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget
from tobas_rqt_tools.utils import place_center


class StartWidget(Widget):
    NAME = "Start"

    TITLE_PSIZE = 50
    SUBTITLE_PSIZE = 20

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(parent=main)
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        title = QLabel("Tobas", self)
        title.setFont(QFont("Default", pointSize=self.TITLE_PSIZE, weight=QFont.Bold))
        place_center(title, rows)

        sub_title = QLabel("— The Flight Controller for All Drones —", self)  # Em dash: Ctrl+Shift+u -> 2014 -> Enter
        sub_title.setFont(QFont("Default", pointSize=self.SUBTITLE_PSIZE, weight=QFont.Bold))
        place_center(sub_title, rows)

        # TODO: ニュース，リリースノート，関連リンクなど

        rows.addStretch()

    def define_connections(self) -> None:
        pass
