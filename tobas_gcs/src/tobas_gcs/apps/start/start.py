from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from typing import override
from PyQt5.QtWidgets import QLabel, QVBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.utils import place_center
from tobas_tools_py.drone import Drone

from ..base import BaseAppWidget


class StartWidget(BaseAppWidget):
    NAME = "Start"

    TITLE_PSIZE = 50
    SUBTITLE_PSIZE = 20

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        title = QLabel("Tobas", self)
        title.setFont(QFont("Default", pointSize=self.TITLE_PSIZE, weight=QFont.Weight.Bold))
        place_center(title, rows)

        sub_title = QLabel("— The Flight Controller for All Drones —", self)  # Em dash: Ctrl+Shift+u -> 2014 -> Enter
        sub_title.setFont(QFont("Default", pointSize=self.SUBTITLE_PSIZE, weight=QFont.Weight.Bold))
        place_center(sub_title, rows)

        # TODO: ニュース，リリースノート，関連リンクなど

        rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass
