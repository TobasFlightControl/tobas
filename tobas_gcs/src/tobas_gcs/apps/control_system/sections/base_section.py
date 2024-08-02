from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

from abc import abstractmethod
from PyQt5.QtWidgets import QLabel, QVBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.widgets import Widget
from tobas_tools_py.drone import Drone

from ....common import TITLE_PSIZE, TO_DO


class BaseControlSystemSectionWidget(Widget):
    LABEL = TO_DO

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel(self.LABEL)
        label.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Weight.Bold))
        self._rows.addWidget(label)

    @abstractmethod
    def update_internal_data_structures(self) -> None:
        raise NotImplementedError()
