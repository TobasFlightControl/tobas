from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_tools_py.drone import Drone

from ..base import BaseAppWidget
from .map_widget import MapWidget


class MissionPlannerWidget(BaseAppWidget):
    NAME = "Mission Planner"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._map = MapWidget()
        self._map.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        rows.addWidget(self._map)

        rows.addStretch()

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        pass
